/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Bare-metal ADC — single channel, single mode, EOC interrupt
 *
 * Signal source: LDR in 10kΩ voltage divider on PA1 (ADC1 Channel 1)
 *
 *   3.3V ──── LDR ──── PA1 ──── 10kΩ ──── GND
 *
 * How it works:
 *   1. main() calls adc_start() — fires SWSTART
 *   2. ADC converts CH1, sets EOC flag
 *   3. EOC flag → EOCIE enabled → ADC fires IRQ 18
 *   4. ADC_IRQHandler: reads ADC_DR (clears EOC), stores result,
 *      sets conversion_done flag
 *   5. main() sees flag, prints, waits, triggers again
 *   CPU is completely free during conversion (~23 µs)
 *
 * Key difference from polling (project 1):
 *   Polling:   CPU sits in while(!(ADC_SR & EOC)) — blocked
 *   Interrupt: CPU continues, ISR fires when hardware is done
 *
 * ADC IRQ (RM0090 Table 62):
 *   ADC1/2/3 global interrupt → IRQ 18
 *   NVIC_ISER0 bit 18  (IRQ 18 < 32 → ISER0)
 *   NVIC_IPR4  byte 2  (18 / 4 = IPR4, 18 % 4 = byte 2, bits [23:16])
 *
 * NVIC base (PM0214 §4.3.11): 0xE000E100
 *   NVIC_ISER0 = base + 0x000
 *   NVIC_IPR4  = base + 0x300 + (0x04 * 4) = base + 0x310
 *
 * Registers used (RM0090 §13.13):
 *   ADC1_SR    +0x00  EOC, OVR flags
 *   ADC1_CR1   +0x04  EOCIE (bit 5), RES (bits 25:24)
 *   ADC1_CR2   +0x08  ADON, CONT, EOCS, SWSTART
 *   ADC1_SMPR2 +0x10  sample time CH1
 *   ADC1_SQR1  +0x2C  sequence length
 *   ADC1_SQR3  +0x34  SQ1 channel select
 *   ADC1_DR    +0x4C  12-bit result
 ******************************************************************************
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

/*
 * NVIC base (PM0214 §4.3.11):
 * "The base address of the main NVIC register block is 0xE000E100."
 */
#define NVIC_BASE       0xE000E100UL

/* ── RCC ────────────────────────────────────────────────────── */
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

#define RCC_AHB1ENR_GPIOAEN  (1UL << 0)
#define RCC_APB2ENR_ADC1EN   (1UL << 8)

/* ── GPIOA ──────────────────────────────────────────────────── */
#define GPIOA_MODER  (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))

/* PA1 bits [3:2] — 11 = Analog mode */
#define GPIO_MODER_ANALOG_PA1  (0x3UL << (1 * 2))
#define GPIO_MODER_MASK_PA1    (0x3UL << (1 * 2))

/* ── ADC1 ───────────────────────────────────────────────────── */
#define ADC1_SR    (*(volatile uint32_t *)(ADC1_BASE + 0x00UL))
#define ADC1_CR1   (*(volatile uint32_t *)(ADC1_BASE + 0x04UL))
#define ADC1_CR2   (*(volatile uint32_t *)(ADC1_BASE + 0x08UL))
#define ADC1_SMPR2 (*(volatile uint32_t *)(ADC1_BASE + 0x10UL))
#define ADC1_SQR1  (*(volatile uint32_t *)(ADC1_BASE + 0x2CUL))
#define ADC1_SQR3  (*(volatile uint32_t *)(ADC1_BASE + 0x34UL))
#define ADC1_DR    (*(volatile uint32_t *)(ADC1_BASE + 0x4CUL))
#define ADC_CCR    (*(volatile uint32_t *)(ADC_COMMON_BASE + 0x04UL))

/* ADC_CR1 bits */
#define ADC_CR1_RES_12BIT  (0x0UL << 24)  /* 12-bit resolution */
/*
 * EOCIE bit [5]: End Of Conversion Interrupt Enable.
 * When set, the ADC fires IRQ 18 every time the EOC flag is set.
 * EOC is set by hardware when a regular channel conversion completes.
 * This is the ONLY bit that differs from the polling version —
 * everything else about the ADC configuration is identical.
 */
#define ADC_CR1_EOCIE      (1UL << 5)

/* ADC_CR2 bits */
#define ADC_CR2_ADON        (1UL << 0)    /* ADC on/off */
#define ADC_CR2_CONT        (1UL << 1)    /* continuous — NOT set (single mode) */
/*
 * EOCS bit [10]: EOC selection.
 * 1 = EOC fires after each individual conversion.
 * Required here so the ISR knows the conversion is done for
 * this single-channel case. Also enables overrun detection.
 */
#define ADC_CR2_EOCS        (1UL << 10)
#define ADC_CR2_EXTEN_NONE  (0x0UL << 28) /* software trigger */
#define ADC_CR2_SWSTART     (1UL << 30)   /* start conversion */

/* ADC_SR bits */
#define ADC_SR_EOC   (1UL << 1)
#define ADC_SR_OVR   (1UL << 5)

/* ADC_CCR: APB2 84MHz / 4 = 21MHz ADCCLK */
#define ADC_CCR_ADCPRE_DIV4  (0x1UL << 16)

/* ADC_SMPR2: CH1 bits [5:3] — 111 = 480 cycles */
#define ADC_SMPR2_CH1_480CYC  (0x7UL << (1 * 3))
#define ADC_SMPR2_CH1_MASK    (0x7UL << (1 * 3))

/* ADC_SQR1: L[3:0] bits [23:20] — 0000 = 1 conversion */
#define ADC_SQR1_L_1CONV  (0x0UL << 20)

/* ADC_SQR3: SQ1[4:0] bits [4:0] — 00001 = CH1 (PA1) */
#define ADC_SQR3_SQ1_CH1  (1UL << 0)

/* ── NVIC registers (PM0214 §4.3) ──────────────────────────── */
/*
 * ADC IRQ = 18 (RM0090 Table 62, vector address 0x0000 0088)
 *
 * NVIC_ISER0 (covers IRQ 0–31):
 *   base + 0x000  →  0xE000E100
 *   bit 18 = IRQ 18 enable
 *
 * NVIC_IPR4 (covers IRQ 16–19):
 *   base + 0x300 + (0x04 * 4) = base + 0x310  →  0xE000E410
 *   IRQ 18 is byte 2 of IPR4  (18 % 4 = 2)
 *   Priority field = bits [23:16] — only bits [23:20] implemented
 *   We write 0xA0 to byte 2 → priority level 10
 *   Shift: 0xA0UL << (2 * 8) = 0xA0UL << 16
 */
#define NVIC_ISER0  (*(volatile uint32_t *)(NVIC_BASE + 0x000UL))
#define NVIC_IPR4   (*(volatile uint32_t *)(NVIC_BASE + 0x310UL))

#define NVIC_ISER0_ADC        (1UL << 18)
#define NVIC_IPR4_ADC_PRI     (0xA0UL << 16)   /* byte 2, priority 10 */
#define NVIC_IPR4_ADC_MASK    (0xFFUL << 16)

/* ── Shared state ───────────────────────────────────────────── */
/*
 * adc_result:       written by ISR, read by main()
 * conversion_done:  flag set by ISR, cleared by main()
 * Both volatile — modified in interrupt context.
 */
volatile uint16_t adc_result;
volatile uint8_t  conversion_done;

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
 * gpio_init — PA1 to Analog mode
 * ───────────────────────────────────────────────────────────── */
static void gpio_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC_AHB1ENR;                      /* bus pipeline flush */

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA1;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA1;
}

/* ─────────────────────────────────────────────────────────────
 * nvic_init
 *
 * Priority set before enable — prevents a spurious fire with
 * garbage priority if the ADC raises an interrupt before we
 * configure the priority field (PM0214 §4.3.7).
 * ───────────────────────────────────────────────────────────── */
static void nvic_init(void)
{
    /* Step 1: set priority for IRQ 18 in IPR4 byte 2 */
    NVIC_IPR4 &= ~NVIC_IPR4_ADC_MASK;
    NVIC_IPR4 |=  NVIC_IPR4_ADC_PRI;

    /* Step 2: enable IRQ 18 in ISER0 */
    NVIC_ISER0 = NVIC_ISER0_ADC;
}

/* ─────────────────────────────────────────────────────────────
 * adc_init
 *
 * Single mode, single channel CH1 (PA1), EOC interrupt enabled.
 *
 * The only register difference from polling project 1:
 *   ADC_CR1 now has EOCIE=1 — everything else is identical.
 * ───────────────────────────────────────────────────────────── */
static void adc_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC_APB2ENR;

    /* Prescaler: 84 MHz APB2 / 4 = 21 MHz ADCCLK */
    ADC_CCR &= ~(0x3UL << 16);
    ADC_CCR |=  ADC_CCR_ADCPRE_DIV4;

    /*
     * CR1: 12-bit resolution, EOCIE=1.
     * EOCIE tells the ADC peripheral to assert its interrupt line
     * when the EOC flag is set. That signal goes to the NVIC,
     * which (with IRQ 18 enabled) calls ADC_IRQHandler.
     * SCAN=0: single channel, no scan needed.
     */
    ADC1_CR1 = ADC_CR1_RES_12BIT
             | ADC_CR1_EOCIE;

    /*
     * CR2: CONT=0 (single), EOCS=1, software trigger.
     * EOCS=1 ensures EOC fires after the individual conversion,
     * which also enables overrun detection automatically.
     */
    ADC1_CR2 = ADC_CR2_EOCS
             | ADC_CR2_EXTEN_NONE;
    /* CONT=0, DMA=0 — both left clear */

    /* Sample time: 480 cycles for CH1 (high-impedance LDR divider) */
    ADC1_SMPR2 &= ~ADC_SMPR2_CH1_MASK;
    ADC1_SMPR2 |=  ADC_SMPR2_CH1_480CYC;

    /* Sequence: 1 conversion, CH1 */
    ADC1_SQR1 = ADC_SQR1_L_1CONV;
    ADC1_SQR3 = ADC_SQR3_SQ1_CH1;

    /* Power on ADC, wait for analog circuits to stabilise */
    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);

    ADC1_SR = 0;    /* clear any stale flags */
}

/* ─────────────────────────────────────────────────────────────
 * adc_start
 *
 * Clears the done flag, then fires SWSTART.
 * Flag cleared BEFORE trigger — if cleared after, there is a
 * theoretical race where the ISR fires between SWSTART and the
 * clear, setting done=1 which we then overwrite with 0.
 * ───────────────────────────────────────────────────────────── */
static void adc_start(void)
{
    conversion_done = 0;
    ADC1_CR2 |= ADC_CR2_SWSTART;
}

/* ─────────────────────────────────────────────────────────────
 * ADC_IRQHandler
 *
 * Called by hardware when the ADC raises IRQ 18.
 * Shared by ADC1, ADC2, and ADC3 — check instance via EOC flag.
 *
 * The vector name must match startup_stm32f407vgtx.s exactly.
 *
 * Responsibilities:
 *   1. Check EOC is set — confirms this is a regular conversion
 *      complete, not another ADC event on the shared IRQ line.
 *   2. Read ADC1_DR — this automatically clears the EOC flag.
 *      If EOC is not cleared before returning, the ISR fires again
 *      immediately (same as leaving TCIF set in DMA projects).
 *   3. Set conversion_done — signals main() that result is valid.
 *
 * What NOT to do here:
 *   - No printf (not ISR-safe)
 *   - No delay
 *   - No adc_start() — triggering the next conversion from inside
 *     the ISR works but couples the ISR to application logic.
 *     Better to re-trigger from main() after processing the result.
 * ───────────────────────────────────────────────────────────── */
void ADC_IRQHandler(void)
{
    if (ADC1_SR & ADC_SR_EOC)
    {
        /*
         * Reading ADC1_DR clears EOC automatically.
         * Must happen before returning — leaving EOC set with
         * EOCIE=1 causes the ISR to immediately re-enter.
         */
        adc_result = (uint16_t)(ADC1_DR & 0x0FFFUL);

        /* Signal main() — result is valid */
        conversion_done = 1;
    }

    /* Clear overrun flag if set — prevents stalled state */
    if (ADC1_SR & ADC_SR_OVR)
    {
        ADC1_SR &= ~ADC_SR_OVR;
    }
}

/* ─────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────── */
int main(void)
{
    gpio_init();
    adc_init();
    nvic_init();    /* enable IRQ 18 after ADC is fully configured */

    uint32_t count = 0;

    /* Fire the first conversion */
    adc_start();

    while (1)
    {
        /*
         * Wait for ISR to set conversion_done.
         * The CPU is free here — in an RTOS you would sleep the
         * task and wake it from the ISR instead of spinning.
         */
        while (!conversion_done) { }

        /*
         * Snapshot result. ADC is idle (CONT=0) so adc_result
         * cannot change until we call adc_start() again.
         */
        uint16_t raw     = adc_result;
        uint8_t  percent = raw_to_percent(raw);

        printf("[%lu] LDR (PA1 CH1): %4u raw  %3u%%\r\n",
               (unsigned long)count,
               (unsigned int)raw,
               (unsigned int)percent);

        count++;
        delay(2000000);     /* ~240 ms between samples */

        /* Trigger the next single conversion */
        adc_start();
    }
}
