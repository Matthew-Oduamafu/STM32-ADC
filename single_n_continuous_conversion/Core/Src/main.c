/*
 * main.c — STM32F407 Discovery
 * Bare-metal ADC: Single + Continuous conversion
 * Signal source: LDR in voltage divider on PA1 (ADC1 Channel 1)
 *
 * Hardware wiring:
 *   3.3V ──── LDR ──── PA1 ──── 10kΩ ──── GND
 *                       ^
 *                  ADC input (0 – 3.3 V)
 *
 * Registers used (all offsets from RM0090 section 13.13):
 *   ADC1_SR   +0x00  status (EOC flag)
 *   ADC1_CR1  +0x04  resolution, scan, EOCIE(End Of Conversion Interrupt Enabled)
 *   ADC1_CR2  +0x08  CONT, ADON, SWSTART, DMA bits
 *   ADC1_SMPR2+0x10  sample time for CH0..CH9
 *   ADC1_SQR3 +0x34  SQ1 — first (only) channel in sequence
 *   ADC1_DR   +0x4C  12-bit result (right-aligned)
 *
 * GPIO / RCC registers (RM0090 sections 6, 7, 8):
 *   RCC_AHB1ENR  — enable GPIOA and ADC1 clocks
 *   RCC_APB2ENR  — enable ADC1 peripheral clock
 *   GPIOA_MODER  — set PA1 to Analog mode (11)
 */

#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

/* ── Base addresses ─────────────────────────────────────────── */
#define PERIPH_BASE       0x40000000UL
#define AHB1_BASE        (PERIPH_BASE + 0x00020000UL)
#define APB2_BASE        (PERIPH_BASE + 0x00010000UL)

#define GPIOA_BASE       (AHB1_BASE  + 0x0000UL)
#define RCC_BASE         (AHB1_BASE  + 0x3800UL)
#define ADC1_BASE        (APB2_BASE  + 0x2000UL)
#define ADC_COMMON_BASE  (APB2_BASE  + 0x2300UL)   /* ADC_CCR lives here */

/* ── RCC registers ──────────────────────────────────────────── */
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

/* RCC_AHB1ENR bits */
#define RCC_AHB1ENR_GPIOAEN   (1UL << 0)   /* bit 0 */

/* RCC_APB2ENR bits */
#define RCC_APB2ENR_ADC1EN    (1UL << 8)   /* bit 8 */

/* ── GPIOA registers ────────────────────────────────────────── */
#define GPIOA_MODER  (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))

/*
 * MODER: 2 bits per pin.  PA1 = bits [3:2]
 * 00 Input  01 Output  10 Alternate  11 Analog
 */
#define GPIO_MODER_ANALOG_PA1  (0x3UL << (1 * 2))   /* 11 at bits [3:2] */
#define GPIO_MODER_MASK_PA1    (0x3UL << (1 * 2))

/* ── ADC1 registers (RM0090 §13.13) ────────────────────────── */
#define ADC1_SR    (*(volatile uint32_t *)(ADC1_BASE + 0x00UL))
#define ADC1_CR1   (*(volatile uint32_t *)(ADC1_BASE + 0x04UL))
#define ADC1_CR2   (*(volatile uint32_t *)(ADC1_BASE + 0x08UL))
#define ADC1_SMPR2 (*(volatile uint32_t *)(ADC1_BASE + 0x10UL))
#define ADC1_SQR1  (*(volatile uint32_t *)(ADC1_BASE + 0x2CUL))
#define ADC1_SQR3  (*(volatile uint32_t *)(ADC1_BASE + 0x34UL))
#define ADC1_DR    (*(volatile uint32_t *)(ADC1_BASE + 0x4CUL))

/* ADC common control register (ADC_CCR) — shared prescaler */
#define ADC_CCR    (*(volatile uint32_t *)(ADC_COMMON_BASE + 0x04UL))

/* ── ADC_CR1 bits ───────────────────────────────────────────── */
/*
 * RES[1:0] at bits [25:24]
 * 00 = 12-bit  (15 ADCCLK cycles)
 * 01 = 10-bit
 * 10 =  8-bit
 * 11 =  6-bit
 */
#define ADC_CR1_RES_12BIT     (0x0UL << 24)

/* Bit 8: SCAN — scan mode (not needed for single channel, set 0) */
#define ADC_CR1_SCAN          (1UL << 8)

/* ── ADC_CR2 bits ───────────────────────────────────────────── */
#define ADC_CR2_ADON          (1UL << 0)   /* ADC ON/OFF */
#define ADC_CR2_CONT          (1UL << 1)   /* Continuous conversion mode */
#define ADC_CR2_DMA           (1UL << 8)   /* DMA mode enable */
#define ADC_CR2_DDS           (1UL << 9)   /* DMA disable selection (keep DMA alive) */
#define ADC_CR2_EOCS          (1UL << 10)  /* EOC set after each conversion */
#define ADC_CR2_SWSTART       (1UL << 30)  /* Software start */

/* EXTEN[1:0] at bits [29:28] — 00 = trigger disabled (software only) */
#define ADC_CR2_EXTEN_NONE    (0x0UL << 28)

/* ── ADC_SR bits ────────────────────────────────────────────── */
#define ADC_SR_EOC            (1UL << 1)   /* End of conversion */
#define ADC_SR_OVR            (1UL << 5)   /* Overrun */

/* ── ADC_CCR bits ───────────────────────────────────────────── */
/*
 * ADCPRE[1:0] at bits [17:16] — APB2 clock prescaler for ADC clock
 * 00 = PCLK2/2   01 = PCLK2/4   10 = PCLK2/6   11 = PCLK2/8
 * ADC max clock = 36 MHz. APB2 = 84 MHz → divide by 4 → 21 MHz.
 */
#define ADC_CCR_ADCPRE_DIV4   (0x1UL << 16)

/* ── ADC_SMPR2: sample time for CH1 ────────────────────────── */
/*
 * SMPR2 covers channels 0–9, 3 bits each.
 * CH1 is at bits [5:3].
 * 111 = 480 cycles — slow but maximises accuracy for high-impedance LDR divider.
 * (LDR + 10kΩ divider impedance can be several kΩ; long sample time lets the
 *  internal capacitor fully charge.)
 */
#define ADC_SMPR2_CH1_480CYC  (0x7UL << (1 * 3))   /* 111 at bits [5:3] */
#define ADC_SMPR2_CH1_MASK    (0x7UL << (1 * 3))

/* ── ADC_SQR1: sequence length ──────────────────────────────── */
/*
 * L[3:0] at bits [23:20].  0000 = 1 conversion.
 */
#define ADC_SQR1_L_1CONV      (0x0UL << 20)

/* ── ADC_SQR3: first conversion in regular sequence ─────────── */
/*
 * SQ1[4:0] at bits [4:0].  Channel number 1 → 00001.
 */
#define ADC_SQR3_SQ1_CH1      (1UL << 0)

/* ── Result storage ─────────────────────────────────────────── */
volatile uint16_t adc_result;       /* updated every conversion in continuous mode */
volatile uint32_t conversion_count; /* how many conversions have completed */

/* ── Simple busy-delay ──────────────────────────────────────── */
static void delay(volatile uint32_t count)
{
    while (count--) { __asm__("nop"); }
}

/* ── Map 12-bit ADC result to a light level 0–100 ─────────── */
static uint8_t adc_to_light_percent(uint16_t raw)
{
    /*
     * In the voltage divider:  V_PA1 = 3.3V * R_pulldown / (LDR + R_pulldown)
     * LDR resistance drops in bright light → V_PA1 rises → raw ADC value rises.
     * So raw=0 → very dark, raw=4095 → very bright.
     * Simple linear mapping:
     */
	return (uint8_t)((raw * 100UL) / 4095UL);
}

/* ─────────────────────────────────────────────────────────────
 * Single conversion — blocking poll
 *
 * Starts one conversion via SWSTART, waits for EOC, returns result.
 * CONT=0 so ADC stops after one sample.
 * ───────────────────────────────────────────────────────────── */
static uint16_t adc_single_read(void)
{
    /* Clear any stale EOC */
    ADC1_SR &= ~ADC_SR_EOC;

    /* Start conversion */
    ADC1_CR2 |= ADC_CR2_SWSTART;

    /* Poll EOC — set by hardware when conversion is complete */
    while (!(ADC1_SR & ADC_SR_EOC)) { }

    /* Reading DR automatically clears EOC */
    return (uint16_t)(ADC1_DR & 0x0FFFUL);
}

/* ─────────────────────────────────────────────────────────────
 * Continuous conversion — non-blocking read
 *
 * ADC is already running (CONT=1, SWSTART fired once in init).
 * Just read DR whenever you want the latest value.
 * EOC is set after each conversion when EOCS=1.
 * ───────────────────────────────────────────────────────────── */
static uint16_t adc_continuous_read_latest(void)
{
    /* Optional: wait for current conversion to complete */
//    while (!(ADC1_SR & ADC_SR_EOC)) { }
    return (uint16_t)(ADC1_DR & 0x0FFFUL);
}

/* ─────────────────────────────────────────────────────────────
 * Switch ADC from continuous mode → single conversion mode
 * ───────────────────────────────────────────────────────────── */
static void adc_set_single_mode(void)
{
    /* Must turn ADC off before changing CONT */
    ADC1_CR2 &= ~ADC_CR2_ADON;
    ADC1_CR2 &= ~ADC_CR2_CONT;    /* CONT=0: single conversion */
    ADC1_CR2 |=  ADC_CR2_ADON;    /* Turn ADC back on */
    delay(10000);                  /* Stabilisation */
}

/* ─────────────────────────────────────────────────────────────
 * Peripheral initialisation
 * ───────────────────────────────────────────────────────────── */
static void adc_gpio_init(void)
{
    /* 1. Enable GPIOA clock (AHB1ENR bit 0) */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC_AHB1ENR; /* dummy read — ensure write completes before GPIO access */

    /*
     * 2. Set PA1 to Analog mode (MODER bits [3:2] = 11).
     *    Analog mode disables the digital input Schmitt trigger,
     *    reducing noise and preventing partial-voltage damage on the pin.
     */
    GPIOA_MODER &= ~GPIO_MODER_MASK_PA1;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA1;
}

static void adc_peripheral_init(void)
{
    /* 1. Enable ADC1 clock on APB2 (APB2ENR bit 8) */
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC_APB2ENR;  /* read-back delay */

    /*
     * 2. Set ADC clock prescaler in ADC_CCR.
     *    APB2 = 84 MHz.  Divide by 4 → ADCCLK = 21 MHz (max 36 MHz).
     *    ADC_CCR is the common register shared by ADC1/2/3.
     */
    ADC_CCR &= ~(0x3UL << 16);       /* clear ADCPRE */
    ADC_CCR |=  ADC_CCR_ADCPRE_DIV4;

    /*
     * 3. CR1: 12-bit resolution, scan mode off (single channel).
     *    RES[1:0]=00 → 12-bit.
     */
    ADC1_CR1  = ADC_CR1_RES_12BIT;   /* SCAN=0, no interrupts */

    /*
     * 4. CR2: software trigger only (EXTEN=00), right alignment,
     *    EOCS=1 so EOC is set after each individual conversion
     *    (needed for both single and continuous polling).
     *    CONT and ADON left 0 here — set by mode-switch functions.
     */
    ADC1_CR2  = ADC_CR2_EXTEN_NONE
              | ADC_CR2_EOCS;

    /*
     * 5. SMPR2: CH1 sample time = 480 cycles.
     *    Long sample time accommodates the LDR voltage divider's
     *    output impedance (up to ~10 kΩ).
     *    Total conversion time = sample_time + 12 cycles (12-bit)
     *                          = 492 cycles @ 21 MHz ≈ 23.4 µs
     */
    ADC1_SMPR2 &= ~ADC_SMPR2_CH1_MASK;
    ADC1_SMPR2 |=  ADC_SMPR2_CH1_480CYC;

    /*
     * 6. SQR1: sequence length = 1 conversion (L[3:0] = 0000).
     */
    ADC1_SQR1  = ADC_SQR1_L_1CONV;

    /*
     * 7. SQR3: first (only) conversion = Channel 1 (PA1).
     *    SQ1[4:0] = 00001.
     */
    ADC1_SQR3  = ADC_SQR3_SQ1_CH1;

    /*
     * 8. Power on ADC and wait for stabilisation.
     *    RM0090 does not give a hard stabilisation time for F407
     *    (unlike F1 which needs tSTAB).  A short delay is good practice.
     */
    ADC1_CR2  |= ADC_CR2_ADON;
    delay(10000);                /* ~120 µs @ 168 MHz — more than enough */
}

static void adc_init_continuous_mode(void)
{
    /* Full re-init with CONT=1 from the beginning */

    /* Turn ADC fully off and reset CR2 */
    ADC1_CR2 = 0;
    delay(100000);

    /* CR1: 12-bit, no scan */
    ADC1_CR1 = ADC_CR1_RES_12BIT;

    /*
     * CR2: CONT=1, software trigger, right-align.
     * No EOCS this time — in continuous mode we just read DR directly,
     * we don't need EOC to fire after every conversion.
     */
    ADC1_CR2 = ADC_CR2_CONT
             | ADC_CR2_EXTEN_NONE;

    /* Sample time and sequence — same as before */
    ADC1_SMPR2 &= ~ADC_SMPR2_CH1_MASK;
    ADC1_SMPR2 |=  ADC_SMPR2_CH1_480CYC;
    ADC1_SQR1   =  ADC_SQR1_L_1CONV;
    ADC1_SQR3   =  ADC_SQR3_SQ1_CH1;

    /* Power on */
    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);

    /* Clear stale flags */
    ADC1_SR = 0;

    /* Fire once — runs forever from here */
    ADC1_CR2 |= ADC_CR2_SWSTART;
    delay(10000);
}

/* ─────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────── */
int main(void)
{
    /* Initialise GPIO and ADC peripheral */
    adc_gpio_init();
    adc_peripheral_init();

    /* ── PART 1: Single conversion mode ─────────────────────── */
    /*
     * CONT=0 (default after init).
     * Each call to adc_single_read() triggers one conversion,
     * polls EOC, then returns. ADC goes idle between calls.
     * Useful when you only need a sample on demand.
     */
    adc_set_single_mode();

    uint16_t single_samples[8];
    for (int i = 0; i < 8; i++)
    {
        single_samples[i] = adc_single_read();
        delay(500000);   /* ~60 ms between samples — watch in debugger */
    }
    (void)single_samples;

    /* single_samples[] now holds 8 on-demand LDR readings.
     * Observe in STM32CubeIDE Live Expressions or Expressions view. */

    /* ── PART 2: Continuous conversion mode ─────────────────── */
    /*
     * CONT=1.  After the first SWSTART, the ADC restarts immediately
     * after each conversion and runs forever.
     * We just read DR whenever we want the latest value.
     *
     * Cover/uncover the LDR while watching adc_result in the
     * debugger — it should sweep the full 0–4095 range.
     */
    adc_init_continuous_mode();   /* clean re-init, not a mode switch */

	conversion_count = 0;

	while (1)
	{
		adc_result = adc_continuous_read_latest();   /* just reads DR */
		conversion_count++;

		uint8_t light_level = adc_to_light_percent(adc_result);
		printf("Light intensity: %u%%\n", (unsigned int)light_level);

		delay(2000000);
	}
}
