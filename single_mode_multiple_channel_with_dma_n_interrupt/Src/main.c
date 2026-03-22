/*
 * main.c — STM32F407 Discovery
 * Bare-metal ADC: multi-channel single mode + DMA + DMA TC interrupt
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
 *   1. main() calls adc_start_single_scan()
 *      → toggles ADC DMA bit (resets ADC's internal DMA request)
 *      → arms DMA stream (NDTR=2, EN=1)
 *      → fires SWSTART
 *   2. ADC converts CH1 → DMA writes adc_results[0] → NDTR--
 *   3. ADC converts CH4 → DMA writes adc_results[1] → NDTR-- = 0
 *   4. DMA raises TCIF0 → fires IRQ 56 (DMA2 Stream0)
 *   5. ISR: clears TCIF0, sets scan_complete = 1
 *   6. main() sees scan_complete, reads results, prints, triggers again
 *
 *   After a single-mode sequence completes with DDS=0, the ADC
 *   internally stops issuing DMA requests (RM0090 §13.8.1).
 *   The DMA bit in ADC_CR2 must be toggled (cleared then set) before
 *   each new SWSTART to reset the ADC's internal DMA request logic.
 *   Without this, only the first scan ever completes.
 *
 * NVIC (PM0214 §4.3, RM0090 Table 62):
 *   DMA2 Stream0 → IRQ 56
 *   NVIC_ISER1 bit 24  (56 - 32 = 24)
 *   NVIC_IPR14 byte 0 bits [7:4]  (56 / 4 = 14, 56 % 4 = 0)
 *
 * DMA assignment (RM0090 Table 44):
 *   ADC1 → DMA2, Stream 0, Channel 0
 */

#include <stdint.h>
#include <stdio.h>

/* ── Base addresses ─────────────────────────────────────────── */
#define PERIPH_BASE      0x40000000UL
#define AHB1_BASE       (PERIPH_BASE + 0x00020000UL)
#define APB2_BASE       (PERIPH_BASE + 0x00010000UL)

#define GPIOA_BASE      (AHB1_BASE  + 0x0000UL)
#define RCC_BASE        (AHB1_BASE  + 0x3800UL)
#define DMA2_BASE       (AHB1_BASE  + 0x6400UL)
#define ADC1_BASE       (APB2_BASE  + 0x2000UL)
#define ADC_COMMON_BASE (APB2_BASE  + 0x2300UL)

/*
 * NVIC base address (PM0214 §4.3.11):
 * "The base address of the main NVIC register block is 0xE000E100."
 */
#define NVIC_BASE       0xE000E100UL

/* ── RCC ────────────────────────────────────────────────────── */
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x44UL))

#define RCC_AHB1ENR_GPIOAEN  (1UL << 0)
#define RCC_AHB1ENR_DMA2EN   (1UL << 22)
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
#define ADC_CR1_SCAN       (1UL << 8)
#define ADC_CR1_RES_12BIT  (0x0UL << 24)

/* ADC_CR2 */
#define ADC_CR2_ADON        (1UL << 0)
/*
 * DMA bit [8]: enables DMA requests from ADC after each conversion.
 * Must be toggled (cleared then set) before each new single-mode
 * scan to reset the ADC's internal DMA request mechanism.
 */
#define ADC_CR2_DMA         (1UL << 8)
#define ADC_CR2_EXTEN_NONE  (0x0UL << 28)
#define ADC_CR2_SWSTART     (1UL << 30)
/* CONT=0, DDS=0, EOCS=0 — all left clear for single mode */

/* ADC_CCR: APB2 84MHz / 4 = 21MHz ADCCLK */
#define ADC_CCR_ADCPRE_DIV4  (0x1UL << 16)

/* ADC_SMPR2: 480 cycles, CH1 bits[5:3], CH4 bits[14:12] */
#define ADC_SMPR2_CH1_480CYC  (0x7UL << (1 * 3))
#define ADC_SMPR2_CH1_MASK    (0x7UL << (1 * 3))
#define ADC_SMPR2_CH4_480CYC  (0x7UL << (4 * 3))
#define ADC_SMPR2_CH4_MASK    (0x7UL << (4 * 3))

/* ADC_SQR1: L[3:0] bits[23:20] — 0001 = 2 conversions */
#define ADC_SQR1_L_2CONV  (0x1UL << 20)

/* ADC_SQR3: SQ1=CH1 bits[4:0], SQ2=CH4 bits[9:5] */
#define ADC_SQR3_SQ1_CH1  (1UL << 0)
#define ADC_SQR3_SQ2_CH4  (4UL << 5)

/* ── DMA2 Stream 0 registers ────────────────────────────────── */
#define DMA2_LISR   (*(volatile uint32_t *)(DMA2_BASE + 0x00UL))
#define DMA2_LIFCR  (*(volatile uint32_t *)(DMA2_BASE + 0x08UL))
#define DMA2_S0CR   (*(volatile uint32_t *)(DMA2_BASE + 0x10UL))
#define DMA2_S0NDTR (*(volatile uint32_t *)(DMA2_BASE + 0x14UL))
#define DMA2_S0PAR  (*(volatile uint32_t *)(DMA2_BASE + 0x18UL))
#define DMA2_S0M0AR (*(volatile uint32_t *)(DMA2_BASE + 0x1CUL))

/* DMA_SxCR bits */
#define DMA_SxCR_EN        (1UL << 0)
/*
 * TCIE bit [4]: Transfer Complete Interrupt Enable.
 * When NDTR reaches 0, DMA sets TCIF0 and (with TCIE=1)
 * asserts IRQ 56 — DMA2_Stream0_IRQHandler is called.
 */
#define DMA_SxCR_TCIE      (1UL << 4)
#define DMA_SxCR_DIR_P2M   (0x0UL << 6)
#define DMA_SxCR_MINC      (1UL << 10)
#define DMA_SxCR_PSIZE_16  (0x1UL << 11)
#define DMA_SxCR_MSIZE_16  (0x1UL << 13)
#define DMA_SxCR_PL_HIGH   (0x2UL << 16)
#define DMA_SxCR_CHSEL_CH0 (0x0UL << 25)
/* CIRC=0: normal mode, stream stops after NDTR=0 */

/* DMA LISR / LIFCR flags for Stream 0 */
#define DMA_LISR_TCIF0     (1UL << 5)
#define DMA_LISR_TEIF0     (1UL << 3)
#define DMA_LIFCR_CLEAR_S0 (0x3DUL)

/* ── NVIC registers (PM0214 §4.3.11) ───────────────────────── */
/*
 * IRQ 56 (DMA2 Stream0):
 *
 * NVIC_ISER1 (covers IRQ 32–63):
 *   base + 0x004  →  0xE000E104
 *   bit 24 = IRQ 56 enable  (56 - 32 = 24)
 *
 * NVIC_IPR14 (covers IRQ 56–59):
 *   base + 0x300 + (0x04 * 14) = base + 0x338  →  0xE000E438
 *   IRQ 56 is byte 0 of IPR14  (56 % 4 = 0)
 *   Priority field = bits [7:4] of that byte (PM0214 §4.3.7)
 *   We write 0xA0 → upper nibble = 0xA = priority level 10
 */
#define NVIC_ISER1  (*(volatile uint32_t *)(NVIC_BASE + 0x004UL))
#define NVIC_IPR14  (*(volatile uint32_t *)(NVIC_BASE + 0x338UL))

#define NVIC_ISER1_DMA2_S0       (1UL << 24)
#define NVIC_IPR14_DMA2_S0_PRI   (0xA0UL << 0)
#define NVIC_IPR14_DMA2_S0_MASK  (0xFFUL << 0)

/* ── Number of channels ─────────────────────────────────────── */
#define ADC_NUM_CHANNELS  2

/* ── Shared state ───────────────────────────────────────────── */
/*
 * adc_results[]: DMA writes here directly from ADC1_DR.
 * [0] = CH1 (PA1, LDR),  [1] = CH4 (PA4, pot)
 *
 * scan_complete: set by ISR, cleared by adc_start_single_scan().
 * Both volatile — modified outside normal CPU execution flow.
 */
volatile uint16_t adc_results[ADC_NUM_CHANNELS];
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
 * nvic_init
 *
 * Set priority before enabling — if the interrupt fires with
 * an uninitialised priority field the NVIC uses garbage.
 * ───────────────────────────────────────────────────────────── */
static void nvic_init(void)
{
    /* Priority first */
    NVIC_IPR14 &= ~NVIC_IPR14_DMA2_S0_MASK;
    NVIC_IPR14 |=  NVIC_IPR14_DMA2_S0_PRI;

    /* Enable IRQ 56 */
    NVIC_ISER1 = NVIC_ISER1_DMA2_S0;
}

/* ─────────────────────────────────────────────────────────────
 * dma_configure
 *
 * One-time setup of everything that never changes between shots:
 * PAR, M0AR, and CR (including TCIE=1).
 * NDTR and EN are set fresh in dma_rearm() before every scan.
 * ───────────────────────────────────────────────────────────── */
static void dma_configure(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC_AHB1ENR;

    DMA2_S0CR &= ~DMA_SxCR_EN;
    while (DMA2_S0CR & DMA_SxCR_EN) { }

    DMA2_LIFCR = DMA_LIFCR_CLEAR_S0;

    DMA2_S0PAR  = (uint32_t)(&ADC1_DR);
    DMA2_S0M0AR = (uint32_t)(adc_results);

    DMA2_S0CR = DMA_SxCR_CHSEL_CH0
              | DMA_SxCR_PL_HIGH
              | DMA_SxCR_MSIZE_16
              | DMA_SxCR_PSIZE_16
              | DMA_SxCR_MINC
              | DMA_SxCR_DIR_P2M
              | DMA_SxCR_TCIE;
    /* CIRC=0, EN=0 — both left clear */
}

/* ─────────────────────────────────────────────────────────────
 * dma_rearm
 *
 * Resets the stream for a fresh single-shot transfer.
 * Must clear all flags before re-enabling (RM0090 §10.3.17).
 * Must write NDTR while EN=0.
 * ───────────────────────────────────────────────────────────── */
static void dma_rearm(void)
{
    DMA2_S0CR &= ~DMA_SxCR_EN;
    while (DMA2_S0CR & DMA_SxCR_EN) { }

    DMA2_LIFCR  = DMA_LIFCR_CLEAR_S0;
    DMA2_S0NDTR = ADC_NUM_CHANNELS;
    DMA2_S0CR  |= DMA_SxCR_EN;
}

/* ─────────────────────────────────────────────────────────────
 * adc_init
 * ───────────────────────────────────────────────────────────── */
static void adc_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC_APB2ENR;

    ADC_CCR &= ~(0x3UL << 16);
    ADC_CCR |=  ADC_CCR_ADCPRE_DIV4;

    ADC1_CR1 = ADC_CR1_RES_12BIT
             | ADC_CR1_SCAN;

    /* CONT=0, DDS=0 — single mode, DMA stops cleanly after sequence */
    ADC1_CR2 = ADC_CR2_DMA
             | ADC_CR2_EXTEN_NONE;

    ADC1_SMPR2 &= ~(ADC_SMPR2_CH1_MASK | ADC_SMPR2_CH4_MASK);
    ADC1_SMPR2 |=  (ADC_SMPR2_CH1_480CYC | ADC_SMPR2_CH4_480CYC);

    ADC1_SQR1 = ADC_SQR1_L_2CONV;
    ADC1_SQR3 = ADC_SQR3_SQ1_CH1
              | ADC_SQR3_SQ2_CH4;

    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);

    ADC1_SR = 0;
}

/* ─────────────────────────────────────────────────────────────
 * adc_start_single_scan
 *
 * Starts one complete scan: CH1 then CH4.
 * Three steps, in this exact order:
 *
 *   1. Clear scan_complete BEFORE arming — not after. If cleared
 *      after SWSTART there is a race where the DMA could complete
 *      and set the flag before we clear it, losing the signal.
 *
 *   2. Toggle the ADC DMA bit (clear then set).
 *      After a single-mode sequence completes with DDS=0, the ADC
 *      internally disables its DMA request line (RM0090 §13.8.1).
 *      The bit is still set in ADC_CR2 but the internal mechanism
 *      has stopped. Toggling it resets that internal logic so the
 *      ADC will fire DMA requests again for the next sequence.
 *      Without this, only the first scan ever produces DMA requests
 *      and every subsequent scan hangs waiting for scan_complete.
 *
 *   3. Arm DMA, then fire SWSTART.
 *      DMA must be armed before the first conversion completes.
 * ───────────────────────────────────────────────────────────── */
static void adc_start_single_scan(void)
{
    /* Step 1: clear completion flag before arming */
    scan_complete = 0;

    /* Step 2: reset ADC's internal DMA request mechanism */
    ADC1_CR2 &= ~ADC_CR2_DMA;
    ADC1_CR2 |=  ADC_CR2_DMA;

    /* Step 3: arm DMA stream, then trigger */
    dma_rearm();
    ADC1_CR2 |= ADC_CR2_SWSTART;
}

/* ─────────────────────────────────────────────────────────────
 * DMA2_Stream0_IRQHandler
 *
 * Called by hardware when DMA2 Stream0 transfer completes.
 * Vector name must match startup_stm32f407vgtx.s exactly.
 *
 * Mandatory: clear TCIF0 in LIFCR before returning.
 *   If left set while TCIE=1, the ISR re-enters immediately
 *   on return — an infinite interrupt storm.
 * ───────────────────────────────────────────────────────────── */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2_LISR & DMA_LISR_TCIF0)
    {
        /* Clear TC flag — mandatory before returning */
        DMA2_LIFCR = DMA_LISR_TCIF0;

        /* Signal main() — both results are in adc_results[] */
        scan_complete = 1;
    }

    /* Clear transfer error flag if set */
    if (DMA2_LISR & DMA_LISR_TEIF0)
    {
        DMA2_LIFCR = DMA_LISR_TEIF0;
    }
}

/* ─────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────── */
int main(void)
{
    gpio_init();
    dma_configure();
    adc_init();
    nvic_init();

    uint32_t count = 0;

    /* Fire the first scan */
    adc_start_single_scan();

    while (1)
    {
        /*
         * Wait for ISR to set scan_complete.
         * The CPU spins here — in an RTOS you would sleep the
         * task and wake it from the ISR instead.
         */
        while (!scan_complete) { }

        /*
         * Snapshot results. DMA stream is stopped (EN cleared by
         * hardware after NDTR=0) so the buffer is stable here.
         */
        uint16_t ldr_raw = adc_results[0];
        uint16_t pot_raw = adc_results[1];

        printf("[%lu] LDR (PA1): %4u raw %3u%%  |  "
               "Pot (PA4): %4u raw %3u%%\r\n",
               (unsigned long)count,
               (unsigned int)ldr_raw,
               (unsigned int)raw_to_percent(ldr_raw),
               (unsigned int)pot_raw,
               (unsigned int)raw_to_percent(pot_raw));

        count++;
        delay(2000000);

        /* Trigger the next scan */
        adc_start_single_scan();
    }
}
