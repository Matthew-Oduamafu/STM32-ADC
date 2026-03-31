/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Bare-metal ADC — multi-channel, single mode, EOC interrupt
 *
 * Channels:
 *   CH1  →  PA1  →  LDR in 10kΩ voltage divider
 *   CH4  →  PA4  →  Potentiometer wiper
 *
 * Hardware wiring:
 *   LDR:  3.3V ── LDR ── PA1 ── 10kΩ ── GND
 *   Pot:  3.3V ── [end] ── [wiper → PA4] ── [end] ── GND
 *
 * How it works:
 *   1. main() calls adc_start() — fires SWSTART
 *   2. ADC converts CH1 → sets EOC → fires IRQ 18
 *   3. ISR: reads DR (clears EOC), stores result in adc_results[0],
 *            increments channel index, ADC auto-advances to CH4
 *   4. ADC converts CH4 → sets EOC → fires IRQ 18 again
 *   5. ISR: reads DR, stores adc_results[1], sets scan_complete
 *   6. main() sees scan_complete, prints both results, re-triggers
 *
 * Key mechanism (RM0090 §13.12):
 *   SCAN=1 + EOCS=1 + EOCIE=1 →
 *     EOC fires after EACH individual channel conversion in the sequence.
 *     Each EOC fires IRQ 18. ISR runs once per channel.
 *     Reading ADC_DR clears EOC and hardware advances to the next channel.
 *
 * This is the non-DMA equivalent of the DMA single-mode project.
 * Instead of DMA writing DR to memory automatically, the ISR does it.
 * Suitable for 2–4 slow channels where DMA setup overhead isn't justified.
 *
 * ADC IRQ (RM0090 Table 62):
 *   ADC1/2/3 global interrupt → IRQ 18
 *   NVIC_ISER0 bit 18
 *   NVIC_IPR4  byte 2 bits [23:16]  (18/4=4, 18%4=2)
 *
 * NVIC base (PM0214 §4.3.11): 0xE000E100
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

#define NVIC_BASE       0xE000E100UL

/* ── RCC ────────────────────────────────────────────────────── */
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

#define RCC_AHB1ENR_GPIOAEN  (1UL << 0)
#define RCC_APB2ENR_ADC1EN   (1UL << 8)

/* ── GPIOA ──────────────────────────────────────────────────── */
#define GPIOA_MODER  (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))

/* PA1 bits [3:2], PA4 bits [9:8] — 11 = Analog mode */
#define GPIO_MODER_ANALOG_PA1  (0x3UL << (1 * 2))
#define GPIO_MODER_MASK_PA1    (0x3UL << (1 * 2))
#define GPIO_MODER_ANALOG_PA4  (0x3UL << (4 * 2))
#define GPIO_MODER_MASK_PA4    (0x3UL << (4 * 2))

/* ── ADC1 ───────────────────────────────────────────────────── */
#define ADC1_SR    (*(volatile uint32_t *)(ADC1_BASE + 0x00UL))
#define ADC1_CR1   (*(volatile uint32_t *)(ADC1_BASE + 0x04UL))
#define ADC1_CR2   (*(volatile uint32_t *)(ADC1_BASE + 0x08UL))
#define ADC1_SMPR2 (*(volatile uint32_t *)(ADC1_BASE + 0x10UL))
#define ADC1_SQR1  (*(volatile uint32_t *)(ADC1_BASE + 0x2CUL))
#define ADC1_SQR3  (*(volatile uint32_t *)(ADC1_BASE + 0x34UL))
#define ADC1_DR    (*(volatile uint32_t *)(ADC1_BASE + 0x4CUL))
#define ADC_CCR    (*(volatile uint32_t *)(ADC_COMMON_BASE + 0x04UL))

/* ADC_CR1 */
#define ADC_CR1_RES_12BIT  (0x0UL << 24)
/*
 * SCAN bit [8]: enables automatic stepping through SQ1→SQ2→...→SQL.
 * Without SCAN=1 the ADC only ever converts SQ1.
 */
#define ADC_CR1_SCAN       (1UL << 8)
/*
 * EOCIE bit [5]: fires IRQ 18 every time EOC is set.
 * With EOCS=1, EOC is set after EACH individual channel.
 * So the ISR fires once per channel — twice for a 2-channel sequence.
 * RM0090 §13.12: "At the end of each regular channel conversion
 * if the EOCS bit is set to 1."
 */
#define ADC_CR1_EOCIE      (1UL << 5)

/* ADC_CR2 */
#define ADC_CR2_ADON        (1UL << 0)
/*
 * EOCS bit [10]: EOC fires after each individual conversion.
 * Combined with EOCIE=1, this is the mechanism that gives us
 * one ISR call per channel in scan mode.
 * Also enables overrun detection automatically.
 */
#define ADC_CR2_EOCS        (1UL << 10)
#define ADC_CR2_EXTEN_NONE  (0x0UL << 28)
#define ADC_CR2_SWSTART     (1UL << 30)
/* CONT=0, DMA=0 — single mode, no DMA */

/* ADC_SR */
#define ADC_SR_EOC  (1UL << 1)
#define ADC_SR_OVR  (1UL << 5)

/* ADC_CCR: APB2 84MHz / 4 = 21MHz */
#define ADC_CCR_ADCPRE_DIV4  (0x1UL << 16)

/* ADC_SMPR2: 480 cycles for CH1 [5:3] and CH4 [14:12] */
#define ADC_SMPR2_CH1_480CYC  (0x7UL << (1 * 3))
#define ADC_SMPR2_CH1_MASK    (0x7UL << (1 * 3))
#define ADC_SMPR2_CH4_480CYC  (0x7UL << (4 * 3))
#define ADC_SMPR2_CH4_MASK    (0x7UL << (4 * 3))

/* ADC_SQR1: L[3:0] bits [23:20] — 0001 = 2 conversions */
#define ADC_SQR1_L_2CONV  (0x1UL << 20)

/* ADC_SQR3: SQ1=CH1 bits[4:0], SQ2=CH4 bits[9:5] */
#define ADC_SQR3_SQ1_CH1  (1UL << 0)
#define ADC_SQR3_SQ2_CH4  (4UL << 5)

/* ── NVIC ───────────────────────────────────────────────────── */
/*
 * IRQ 18 — same as project 6 (same shared ADC IRQ line).
 * NVIC_ISER0 bit 18.
 * NVIC_IPR4 byte 2 bits [23:16].
 */
#define NVIC_ISER0  (*(volatile uint32_t *)(NVIC_BASE + 0x000UL))
#define NVIC_IPR4   (*(volatile uint32_t *)(NVIC_BASE + 0x310UL))

#define NVIC_ISER0_ADC      (1UL << 18)
#define NVIC_IPR4_ADC_PRI   (0xA0UL << 16)
#define NVIC_IPR4_ADC_MASK  (0xFFUL << 16)

/* ── Number of channels ─────────────────────────────────────── */
#define ADC_NUM_CHANNELS  2

/* ── Shared state ───────────────────────────────────────────── */
/*
 * adc_results[]:    filled by ISR one channel per call
 * channel_index:    tracks which channel the ISR is reading
 * scan_complete:    set by ISR when all channels read
 * All volatile — written in interrupt context, read in main.
 */
volatile uint16_t adc_results[ADC_NUM_CHANNELS];
volatile uint8_t  channel_index;
volatile uint8_t  scan_complete;

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
 * gpio_init — PA1 and PA4 to Analog mode
 * ───────────────────────────────────────────────────────────── */
static void gpio_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC_AHB1ENR;

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA1;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA1;

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA4;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA4;
}

/* ─────────────────────────────────────────────────────────────
 * nvic_init — enable IRQ 18, set priority
 * ───────────────────────────────────────────────────────────── */
static void nvic_init(void)
{
    NVIC_IPR4  &= ~NVIC_IPR4_ADC_MASK;
    NVIC_IPR4  |=  NVIC_IPR4_ADC_PRI;
    NVIC_ISER0  =  NVIC_ISER0_ADC;
}

/* ─────────────────────────────────────────────────────────────
 * adc_init
 *
 * Scan mode, 2 channels (CH1 then CH4), EOC interrupt per channel.
 *
 * vs project 6 (single channel EOC interrupt):
 *   Added:   SCAN=1 in CR1
 *   Changed: L=0001 (2 conversions) in SQR1
 *   Added:   SQ2=CH4 in SQR3
 *   EOCIE and EOCS are identical
 * ───────────────────────────────────────────────────────────── */
static void adc_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC_APB2ENR;

    ADC_CCR &= ~(0x3UL << 16);
    ADC_CCR |=  ADC_CCR_ADCPRE_DIV4;

    /*
     * CR1: 12-bit, SCAN=1, EOCIE=1.
     * SCAN=1: ADC walks CH1→CH4 automatically after each conversion.
     * EOCIE=1: fires IRQ 18 after each channel (because EOCS=1).
     */
    ADC1_CR1 = ADC_CR1_RES_12BIT
             | ADC_CR1_SCAN
             | ADC_CR1_EOCIE;

    /*
     * CR2: EOCS=1, software trigger, CONT=0.
     * EOCS=1 makes EOC assert after each individual conversion —
     * without it EOC only fires at the end of the full sequence
     * and EOCIE would only interrupt once, missing CH1's result.
     */
    ADC1_CR2 = ADC_CR2_EOCS
             | ADC_CR2_EXTEN_NONE;

    /* 480 cycles sample time for both channels */
    ADC1_SMPR2 &= ~(ADC_SMPR2_CH1_MASK | ADC_SMPR2_CH4_MASK);
    ADC1_SMPR2 |=  (ADC_SMPR2_CH1_480CYC | ADC_SMPR2_CH4_480CYC);

    /* Sequence: 2 conversions — CH1 first, CH4 second */
    ADC1_SQR1 = ADC_SQR1_L_2CONV;
    ADC1_SQR3 = ADC_SQR3_SQ1_CH1
              | ADC_SQR3_SQ2_CH4;

    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);

    ADC1_SR = 0;
}

/* ─────────────────────────────────────────────────────────────
 * adc_start
 *
 * Resets ISR state, then fires SWSTART.
 * State reset BEFORE trigger to avoid the race condition where
 * the ISR fires between SWSTART and the state clear.
 * ───────────────────────────────────────────────────────────── */
static void adc_start(void)
{
    channel_index = 0;
    scan_complete = 0;
    ADC1_CR2 |= ADC_CR2_SWSTART;
}

/* ─────────────────────────────────────────────────────────────
 * ADC_IRQHandler
 *
 * Fires once per channel conversion (EOCS=1 + EOCIE=1).
 * For a 2-channel sequence this ISR runs twice per scan:
 *   Call 1: EOC after CH1 → read DR → store adc_results[0]
 *   Call 2: EOC after CH4 → read DR → store adc_results[1]
 *              → set scan_complete
 *
 * Reading ADC1_DR clears EOC. This is mandatory —
 * if EOC is left set with EOCIE=1 the ISR re-enters instantly.
 *
 * The ADC automatically advances to the next channel in the
 * sequence after each conversion (SCAN=1). No software trigger
 * is needed between channels — only between full sequences.
 * ───────────────────────────────────────────────────────────── */
void ADC_IRQHandler(void)
{
    if (ADC1_SR & ADC_SR_EOC)
    {
        /*
         * Read DR — clears EOC, hardware advances to next channel.
         * Store in the slot matching the current channel position.
         */
        if (channel_index < ADC_NUM_CHANNELS)
        {
            adc_results[channel_index] = (uint16_t)(ADC1_DR & 0x0FFFUL);
            channel_index++;
        }

        /*
         * All channels in the sequence have been converted.
         * Signal main() — do NOT call adc_start() here.
         * Re-triggering from inside the ISR works but tightly
         * couples ISR to application timing. main() controls when
         * the next scan happens.
         */
        if (channel_index >= ADC_NUM_CHANNELS)
        {
            scan_complete = 1;
        }
    }

    /* Clear overrun flag — prevents the ADC from stalling */
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
    nvic_init();

    uint32_t count = 0;

    /* Fire the first scan */
    adc_start();

    while (1)
    {
        /*
         * Wait for ISR to collect both channel results.
         * The ISR fires twice — once for CH1, once for CH4.
         * Only after the second call does scan_complete go high.
         */
        while (!scan_complete) { }

        /*
         * Snapshot results. ADC is idle (CONT=0, sequence done)
         * so adc_results[] is stable until next adc_start().
         */
        uint16_t ldr_raw = adc_results[0];
        uint16_t pot_raw = adc_results[1];

        printf("[%lu] LDR (PA1 CH1): %4u raw %3u%%  |  "
               "Pot (PA4 CH4): %4u raw %3u%%\r\n",
               (unsigned long)count,
               (unsigned int)ldr_raw,
               (unsigned int)raw_to_percent(ldr_raw),
               (unsigned int)pot_raw,
               (unsigned int)raw_to_percent(pot_raw));

        count++;
        delay(2000000);

        /* Trigger the next scan */
        adc_start();
    }
}
