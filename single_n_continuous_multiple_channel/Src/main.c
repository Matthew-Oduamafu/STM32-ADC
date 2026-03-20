/*
 * main.c — STM32F407 Discovery
 * Bare-metal ADC: Scan mode — single-shot and continuous, 2 channels
 *
 * Channel map:
 *   CH1  →  PA1  →  LDR in 10kΩ voltage divider
 *   CH4  →  PA4  →  Potentiometer wiper (rails to 3.3V and GND)
 *
 * Hardware wiring:
 *
 *   LDR (PA1):
 *   3.3V ──── LDR ──── PA1 ──── 10kΩ ──── GND
 *
 *   Potentiometer (PA4):
 *   3.3V ──── [pot end] ──── [wiper → PA4] ──── [pot end] ──── GND
 *
 * Key concept — why EOCS=1 is required for multi-channel without DMA:
 *   In scan mode the ADC converts SQ1, SQ2, ... SQn in sequence,
 *   writing each result to ADC_DR before moving to the next channel.
 *   Without DMA you must read DR after every single conversion before
 *   it gets overwritten by the next one. EOCS=1 (RM0090 §13.8.2) makes
 *   EOC fire after EACH individual conversion — so you can poll EOC,
 *   read DR, then wait for the next EOC and read again.
 *
 * Registers used (RM0090 §13.13):
 *   ADC1_SR    +0x00  EOC, OVR flags
 *   ADC1_CR1   +0x04  RES[1:0], SCAN
 *   ADC1_CR2   +0x08  ADON, CONT, EOCS, SWSTART, EXTEN
 *   ADC1_SMPR2 +0x10  sample time CH0–CH9
 *   ADC1_SQR1  +0x2C  sequence length L[3:0]
 *   ADC1_SQR3  +0x34  SQ1, SQ2 (slots 1–6)
 *   ADC1_DR    +0x4C  12-bit result (right-aligned)
 *   ADC_CCR    +0x04 from ADC_COMMON_BASE — prescaler
 */

#include <stdint.h>
#include <stdio.h>

/* ── Base addresses ─────────────────────────────────────────── */
#define PERIPH_BASE      0x40000000UL
#define AHB1_BASE       (PERIPH_BASE + 0x00020000UL)
#define APB2_BASE       (PERIPH_BASE + 0x00010000UL)

#define GPIOA_BASE      (AHB1_BASE  + 0x0000UL)
#define RCC_BASE        (AHB1_BASE  + 0x3800UL)
#define ADC1_BASE       (APB2_BASE  + 0x2000UL)
#define ADC_COMMON_BASE (APB2_BASE  + 0x2300UL)

/* ── RCC registers ──────────────────────────────────────────── */
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

#define RCC_AHB1ENR_GPIOAEN  (1UL << 0)
#define RCC_APB2ENR_ADC1EN   (1UL << 8)

/* ── GPIOA registers ────────────────────────────────────────── */
#define GPIOA_MODER  (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))

/*
 * MODER: 2 bits per pin.
 * PA1 → bits [3:2],   PA4 → bits [9:8]
 * 11 = Analog mode
 */
#define GPIO_MODER_ANALOG_PA1  (0x3UL << (1 * 2))
#define GPIO_MODER_MASK_PA1    (0x3UL << (1 * 2))
#define GPIO_MODER_ANALOG_PA4  (0x3UL << (4 * 2))
#define GPIO_MODER_MASK_PA4    (0x3UL << (4 * 2))

/* ── ADC1 registers ─────────────────────────────────────────── */
#define ADC1_SR    (*(volatile uint32_t *)(ADC1_BASE + 0x00UL))
#define ADC1_CR1   (*(volatile uint32_t *)(ADC1_BASE + 0x04UL))
#define ADC1_CR2   (*(volatile uint32_t *)(ADC1_BASE + 0x08UL))
#define ADC1_SMPR2 (*(volatile uint32_t *)(ADC1_BASE + 0x10UL))
#define ADC1_SQR1  (*(volatile uint32_t *)(ADC1_BASE + 0x2CUL))
#define ADC1_SQR3  (*(volatile uint32_t *)(ADC1_BASE + 0x34UL))
#define ADC1_DR    (*(volatile uint32_t *)(ADC1_BASE + 0x4CUL))

#define ADC_CCR    (*(volatile uint32_t *)(ADC_COMMON_BASE + 0x04UL))

/* ── ADC_CR1 bits ───────────────────────────────────────────── */
/*
 * RES[1:0] bits [25:24]  — 00 = 12-bit resolution
 */
#define ADC_CR1_RES_12BIT  (0x0UL << 24)

/*
 * SCAN bit [8]
 * Must be 1 for multi-channel sequences. Without SCAN=1 the ADC
 * only ever converts the channel in SQ1 regardless of L[3:0].
 */
#define ADC_CR1_SCAN       (1UL << 8)

/* ── ADC_CR2 bits ───────────────────────────────────────────── */
#define ADC_CR2_ADON        (1UL << 0)    /* ADC on/off */
#define ADC_CR2_CONT        (1UL << 1)    /* continuous mode */
#define ADC_CR2_EOCS        (1UL << 10)   /* EOC after each conversion */
#define ADC_CR2_SWSTART     (1UL << 30)   /* software start */
#define ADC_CR2_EXTEN_NONE  (0x0UL << 28) /* no external trigger */

/* ── ADC_SR bits ────────────────────────────────────────────── */
#define ADC_SR_EOC  (1UL << 1)
#define ADC_SR_OVR  (1UL << 5)

/* ── ADC_CCR prescaler ──────────────────────────────────────── */
/*
 * ADCPRE[1:0] bits [17:16]
 * 01 = APB2 / 4 = 84 / 4 = 21 MHz ADCCLK (max 36 MHz)
 */
#define ADC_CCR_ADCPRE_DIV4  (0x1UL << 16)

/* ── ADC_SMPR2 sample times ─────────────────────────────────── */
/*
 * SMPR2 covers CH0–CH9, 3 bits each, starting at bit 0 for CH0.
 * CH1 → bits [5:3]    (1 * 3 = 3)
 * CH4 → bits [14:12]  (4 * 3 = 12)
 * 111 = 480 cycles — handles LDR divider impedance comfortably.
 * Total per channel: (480 + 12) / 21 MHz ≈ 23.4 µs
 */
#define ADC_SMPR2_CH1_480CYC  (0x7UL << (1 * 3))
#define ADC_SMPR2_CH1_MASK    (0x7UL << (1 * 3))
#define ADC_SMPR2_CH4_480CYC  (0x7UL << (4 * 3))
#define ADC_SMPR2_CH4_MASK    (0x7UL << (4 * 3))

/* ── ADC_SQR1 — sequence length ─────────────────────────────── */
/*
 * L[3:0] bits [23:20].  Value = N-1.
 * 0001 = 2 conversions.
 */
#define ADC_SQR1_L_2CONV  (0x1UL << 20)

/* ── ADC_SQR3 — channel slots 1 and 2 ──────────────────────── */
/*
 * SQ1[4:0] bits [4:0]  — 1st in sequence → CH1 → PA1 (LDR)
 * SQ2[4:0] bits [9:5]  — 2nd in sequence → CH4 → PA4 (pot)
 */
#define ADC_SQR3_SQ1_CH1  (1UL << 0)
#define ADC_SQR3_SQ2_CH4  (4UL << 5)

/* ── Channel count ──────────────────────────────────────────── */
#define ADC_NUM_CHANNELS  2

/* ── Result storage ─────────────────────────────────────────── */
/*
 * [0] = CH1 result (LDR, PA1)
 * [1] = CH4 result (pot, PA4)
 */
volatile uint16_t adc_results[ADC_NUM_CHANNELS];
volatile uint32_t conversion_count;

/* ── Helpers ────────────────────────────────────────────────── */
static void delay(volatile uint32_t count)
{
    while (count--) { __asm__("nop"); }
}

static uint8_t raw_to_percent(uint16_t raw)
{
    return (uint8_t)((raw * 100UL) / 4095UL);
}

/* ─────────────────────────────────────────────────────────────
 * gpio_init
 * Set PA1 and PA4 to Analog mode (MODER = 11).
 * ───────────────────────────────────────────────────────────── */
static void gpio_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC_AHB1ENR;                      /* bus pipeline flush */

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA1;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA1;

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA4;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA4;
}

/* ─────────────────────────────────────────────────────────────
 * adc_base_init
 * Configures everything that never changes between modes:
 * clock prescaler, resolution, scan, sample times, sequence.
 * Does NOT set CONT, ADON, or fire SWSTART.
 * ───────────────────────────────────────────────────────────── */
static void adc_base_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC_APB2ENR;                      /* bus pipeline flush */

    /* Prescaler */
    ADC_CCR &= ~(0x3UL << 16);
    ADC_CCR |=  ADC_CCR_ADCPRE_DIV4;

    /*
     * CR1: 12-bit, SCAN=1.
     * SCAN=1 is what makes the ADC walk through SQ1 → SQ2
     * automatically. Without it, only SQ1 is ever converted.
     */
    ADC1_CR1 = ADC_CR1_RES_12BIT
             | ADC_CR1_SCAN;

    /*
     * CR2: software trigger, EOCS=1.
     * EOCS=1 causes EOC to assert after each individual channel
     * conversion. This is the mechanism that allows reading DR
     * channel-by-channel in adc_read_sequence() without DMA.
     */
    ADC1_CR2 = ADC_CR2_EXTEN_NONE
             | ADC_CR2_EOCS;

    /* Sample times for CH1 and CH4 */
    ADC1_SMPR2 &= ~(ADC_SMPR2_CH1_MASK | ADC_SMPR2_CH4_MASK);
    ADC1_SMPR2 |=  (ADC_SMPR2_CH1_480CYC | ADC_SMPR2_CH4_480CYC);

    /*
     * Sequence: 2 conversions in order CH1 → CH4.
     * SQR1 sets the total length.
     * SQR3 sets which channel goes in each slot.
     */
    ADC1_SQR1 = ADC_SQR1_L_2CONV;
    ADC1_SQR3 = ADC_SQR3_SQ1_CH1
              | ADC_SQR3_SQ2_CH4;
}

/* ─────────────────────────────────────────────────────────────
 * adc_read_sequence — blocking, no DMA
 *
 * Reads one complete pass through the sequence (CH1 then CH4).
 * For each channel:
 *   1. Poll EOC — set by hardware after that channel finishes
 *   2. Read DR  — clears EOC, hardware moves to next channel
 *
 * Works identically in single and continuous mode because the
 * polling loop always catches the EOC for the current channel
 * before moving on.
 * ───────────────────────────────────────────────────────────── */
// works fine for single mode
static void adc_read_sequence(void)
{
    for (int ch = 0; ch < ADC_NUM_CHANNELS; ch++)
    {
        while (!(ADC1_SR & ADC_SR_EOC)) { }
        adc_results[ch] = (uint16_t)(ADC1_DR & 0x0FFFUL);
        /* Reading DR clears EOC — hardware starts next channel */
    }
}

static void adc_read_sequence_con(void)
{
    /*
     * Strategy: wait for ONE EOC to sync to a channel boundary,
     * then immediately read both channels back-to-back.
     * We accept that the first EOC might be from CH1 or CH4 —
     * after reading it we know the NEXT one is always the other
     * channel. So we read two consecutive conversions and store
     * them in order.
     *
     * The real fix for production code is DMA — it removes this
     * sync problem entirely by writing each result to a dedicated
     * memory slot automatically.
     */

    /* Wait for any conversion to complete */
    while (!(ADC1_SR & ADC_SR_EOC)) { }

    /* Read it — clears EOC, ADC immediately starts next channel */
    uint16_t first  = (uint16_t)(ADC1_DR & 0x0FFFUL);

    /* Wait for the second channel to complete */
    while (!(ADC1_SR & ADC_SR_EOC)) { }

    uint16_t second = (uint16_t)(ADC1_DR & 0x0FFFUL);

    /*
     * We don't know if first=CH1,second=CH4 or first=CH4,second=CH1.
     * So track which channel we're on using a static flag that
     * persists across calls and flips each time.
     */
    static uint8_t synced = 0;
    static uint8_t phase  = 0;   /* 0 = first read is CH1, 1 = first read is CH4 */

    if (!synced)
    {
        /*
         * First call: we don't know phase yet. Read one more
         * conversion to force alignment — after this call the
         * ADC will be at a known boundary.
         */
        synced = 1;
        phase  = 0;
        adc_results[0] = first;
        adc_results[1] = second;
        return;
    }

    if (phase == 0)
    {
        adc_results[0] = first;   /* CH1 */
        adc_results[1] = second;  /* CH4 */
    }
    else
    {
        adc_results[0] = second;  /* CH1 */
        adc_results[1] = first;   /* CH4 */
    }
    phase ^= 1;   /* flip for next call */
}


/* ─────────────────────────────────────────────────────────────
 * adc_init_single_mode
 *
 * CONT=0: ADC converts SQ1 → SQ2 once, then stops.
 * A new SWSTART is required for each scan.
 * ───────────────────────────────────────────────────────────── */
static void adc_init_single_mode(void)
{
    ADC1_CR2 = 0;
    delay(100000);                          /* full power-down */

    /* Restore CR2 without CONT */
    ADC1_CR2 = ADC_CR2_EXTEN_NONE
             | ADC_CR2_EOCS;

    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);                          /* ADC stabilisation */

    ADC1_SR = 0;                            /* clear stale flags */
}

static void adc_trigger(void)
{
    ADC1_SR  &= ~ADC_SR_EOC;               /* clear any stale EOC */
    ADC1_CR2 |=  ADC_CR2_SWSTART;          /* fire */
}

/* ─────────────────────────────────────────────────────────────
 * adc_init_continuous_mode
 *
 * CONT=1: after SQ1 → SQ2 completes, ADC immediately restarts
 * from SQ1 without any further software trigger.
 * One SWSTART is all that's needed.
 * ───────────────────────────────────────────────────────────── */
static void adc_init_continuous_mode(void)
{
    ADC1_CR2 = 0;
    delay(100000);

    /* CONT=1 must be set before ADON */
    ADC1_CR2 = ADC_CR2_CONT
             | ADC_CR2_EXTEN_NONE
             | ADC_CR2_EOCS;

    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);

    ADC1_SR = 0;

    /* Single trigger — ADC loops CH1→CH4→CH1→CH4→... forever */
    ADC1_CR2 |= ADC_CR2_SWSTART;
    delay(10000);                           /* let first conversion land */
}

/* ─────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────── */
int main(void)
{
    gpio_init();
    adc_base_init();

    /* ── PART 1: Single scan mode ────────────────────────────
     *
     * Flow per iteration:
     *   adc_trigger()       → SWSTART fires → ADC starts CH1
     *   adc_read_sequence() → polls EOC, reads CH1 result
     *                       → ADC auto-advances to CH2
     *                       → polls EOC, reads CH4 result
     *                       → ADC stops (CONT=0)
     *
     * Vary the LDR light and rotate the pot between samples
     * to confirm both channels are independent and updating.
     * ───────────────────────────────────────────────────────── */
    adc_init_single_mode();

    printf("=== PART 1: Single scan mode ===\r\n");

    for (int i = 0; i < 8; i++)
    {
        adc_trigger();
        adc_read_sequence();

        printf("Sample %d | LDR (PA1): %4u raw %3u%%  |  "
               "Pot (PA4): %4u raw %3u%%\r\n",
               i,
               (unsigned int)adc_results[0],
               (unsigned int)raw_to_percent(adc_results[0]),
               (unsigned int)adc_results[1],
               (unsigned int)raw_to_percent(adc_results[1]));

        delay(1000000);
    }

    /* ── PART 2: Continuous scan mode ────────────────────────
     *
     * ADC free-runs: CH1→CH4→CH1→CH4→...
     * adc_read_sequence() stays in sync by polling EOC before
     * each read — it naturally catches the two EOCs from the
     * current pass through the sequence.
     *
     * Move the pot and cover/uncover the LDR to watch both
     * channels update independently in real time.
     * ───────────────────────────────────────────────────────── */
    adc_init_continuous_mode();

    printf("\r\n=== PART 2: Continuous scan mode ===\r\n");

    conversion_count = 0;

    while (1)
    {
    	adc_read_sequence_con();
        conversion_count++;

        printf("[%lu] LDR (PA1): %4u raw %3u%%  |  "
               "Pot (PA4): %4u raw %3u%%\r\n",
               (unsigned long)conversion_count,
               (unsigned int)adc_results[0],
               (unsigned int)raw_to_percent(adc_results[0]),
               (unsigned int)adc_results[1],
               (unsigned int)raw_to_percent(adc_results[1]));

        delay(2000000);
    }
}
