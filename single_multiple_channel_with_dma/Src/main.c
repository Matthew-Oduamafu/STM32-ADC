/*
 * main.c — STM32F407 Discovery
 * Bare-metal ADC: multi-channel single mode + DMA
 *
 * Channels:
 *   CH1  →  PA1  →  LDR in 10kΩ voltage divider
 *   CH4  →  PA4  →  Potentiometer wiper
 *
 * Hardware wiring:
 *   LDR:  3.3V ── LDR ── PA1 ── 10kΩ ── GND
 *   Pot:  3.3V ── [end] ── [wiper → PA4] ── [end] ── GND
 *
 * How single mode + DMA differs from continuous mode + DMA:
 *
 *   Continuous (previous project):
 *     CONT=1, DDS=1 — ADC restarts automatically after every
 *     sequence. DMA runs forever in circular mode. CPU just reads
 *     the buffer whenever it wants.
 *
 *   Single (this project):
 *     CONT=0, DDS=0 — ADC converts the sequence ONCE per SWSTART,
 *     then stops. DMA transfers the results and stops too (NDTR=0).
 *     To get a new batch: re-arm DMA, then fire SWSTART again.
 *     CPU controls exactly when sampling happens.
 *
 * One conversion cycle:
 *   SWSTART → ADC converts CH1 → DMA: ADC_DR → adc_results[0]
 *          → ADC converts CH4 → DMA: ADC_DR → adc_results[1]
 *          → ADC stops, DMA stops (NDTR=0)
 *          → TC flag set in DMA2_LISR
 *          → re-arm and repeat
 *
 * Why re-arm DMA between shots (RM0090 §10.3.17 + §13.8.1):
 *   After NDTR reaches 0 in normal (non-circular) mode, the DMA
 *   stream is automatically disabled (EN clears). The RM states:
 *   "DMA bits are not cleared by hardware, however they must have
 *   been cleared and set to the wanted mode by software before new
 *   DMA requests can be generated." So before each new SWSTART,
 *   you must: clear TC flag, reset NDTR, re-enable stream.
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
 * CONT bit [1] = 0 → single mode (sequence runs once per SWSTART)
 * We define it only for clarity — it is deliberately not set.
 */
#define ADC_CR2_CONT        (1UL << 1)
/*
 * DMA  bit [8]: ADC fires a DMA request after each conversion.
 * DDS  bit [9]: = 0 in single mode.
 *   When DDS=0, no new DMA request is issued after the last
 *   transfer — exactly what we want. ADC and DMA both stop cleanly
 *   after one full sequence. (RM0090 §13.13.3 Bit 9 description)
 */
#define ADC_CR2_DMA         (1UL << 8)
#define ADC_CR2_EXTEN_NONE  (0x0UL << 28)
#define ADC_CR2_SWSTART     (1UL << 30)

/* ADC_SR */
#define ADC_SR_OVR  (1UL << 5)

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
/* Stream 0: offset term = 0x18 * 0 = 0 */
#define DMA2_LISR   (*(volatile uint32_t *)(DMA2_BASE + 0x00UL))
#define DMA2_LIFCR  (*(volatile uint32_t *)(DMA2_BASE + 0x08UL))
#define DMA2_S0CR   (*(volatile uint32_t *)(DMA2_BASE + 0x10UL))
#define DMA2_S0NDTR (*(volatile uint32_t *)(DMA2_BASE + 0x14UL))
#define DMA2_S0PAR  (*(volatile uint32_t *)(DMA2_BASE + 0x18UL))
#define DMA2_S0M0AR (*(volatile uint32_t *)(DMA2_BASE + 0x1CUL))

/* ── DMA_SxCR bits (RM0090 §10.5.5) ────────────────────────── */
#define DMA_SxCR_EN        (1UL << 0)
#define DMA_SxCR_DIR_P2M   (0x0UL << 6)   /* peripheral-to-memory */
/*
 * CIRC bit [8] = 0 → normal mode (no circular reload).
 * After NDTR reaches 0, the stream stops. This is correct for
 * single-shot — we re-arm manually before the next trigger.
 * We define it only for documentation — it is not set.
 */
#define DMA_SxCR_CIRC      (1UL << 8)
#define DMA_SxCR_MINC      (1UL << 10)    /* increment memory ptr */
#define DMA_SxCR_PSIZE_16  (0x1UL << 11)  /* 16-bit peripheral */
#define DMA_SxCR_MSIZE_16  (0x1UL << 13)  /* 16-bit memory */
#define DMA_SxCR_PL_HIGH   (0x2UL << 16)  /* high priority */
#define DMA_SxCR_CHSEL_CH0 (0x0UL << 25)  /* channel 0 = ADC1 */

/*
 * LISR / LIFCR flags for Stream 0 (RM0090 §10.5.1, §10.5.3)
 * Stream 0 flags sit at the low bits of the LISR register.
 *
 * LISR bit 5: TCIF0  — transfer complete
 * LISR bit 3: TEIF0  — transfer error
 *
 * LIFCR: writing 1 clears the corresponding LISR flag.
 * Full clear mask for Stream 0:
 *   CTCIF0 bit5, CHTIF0 bit4, CTEIF0 bit3, CDMEIF0 bit2, CFEIF0 bit0
 */
#define DMA_LISR_TCIF0      (1UL << 5)
#define DMA_LISR_TEIF0      (1UL << 3)
#define DMA_LIFCR_CLEAR_S0  (0x3DUL)

/* ── Number of channels ─────────────────────────────────────── */
#define ADC_NUM_CHANNELS  2

/* ── Result buffer ──────────────────────────────────────────── */
/*
 * DMA writes directly here from ADC1_DR.
 * [0] = CH1 result (PA1, LDR)
 * [1] = CH4 result (PA4, pot)
 * volatile: value changes outside CPU control — never cache it.
 */
volatile uint16_t adc_results[ADC_NUM_CHANNELS];

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
    (void)RCC_AHB1ENR;                      /* bus pipeline flush */

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA1;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA1;

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA4;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA4;
}

/* ─────────────────────────────────────────────────────────────
 * dma_configure
 *
 * Sets up everything about Stream 0 that never changes between
 * shots: PAR, M0AR, CR. Does NOT set NDTR or EN — those are
 * set fresh in dma_rearm() before every new trigger.
 *
 * Key difference from continuous mode:
 *   CIRC is NOT set → normal mode, stream stops after NDTR=0.
 *   DMA_SxCR_EN is NOT set here — set in dma_rearm().
 * ───────────────────────────────────────────────────────────── */
static void dma_configure(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC_AHB1ENR;

    /* Disable stream, wait for hardware confirmation */
    DMA2_S0CR &= ~DMA_SxCR_EN;
    while (DMA2_S0CR & DMA_SxCR_EN) { }

    /* Clear all Stream 0 flags before configuring */
    DMA2_LIFCR = DMA_LIFCR_CLEAR_S0;

    /* Peripheral address — fixed, never changes */
    DMA2_S0PAR = (uint32_t)(&ADC1_DR);

    /* Memory address — start of result array */
    DMA2_S0M0AR = (uint32_t)(adc_results);

    /*
     * Control register — no CIRC, no EN yet.
     * CHSEL=000: Channel 0 (ADC1, Table 44)
     * PL=10:     High priority
     * MSIZE=01:  16-bit memory (uint16_t)
     * PSIZE=01:  16-bit peripheral (ADC_DR)
     * MINC=1:    advance through adc_results[0] then [1]
     * PINC=0:    peripheral address stays at ADC_DR (default)
     * CIRC=0:    normal mode — stops after 2 transfers
     * DIR=00:    peripheral-to-memory
     */
    DMA2_S0CR = DMA_SxCR_CHSEL_CH0
              | DMA_SxCR_PL_HIGH
              | DMA_SxCR_MSIZE_16
              | DMA_SxCR_PSIZE_16
              | DMA_SxCR_MINC
              | DMA_SxCR_DIR_P2M;
    /* CIRC=0 and EN=0 — both left clear intentionally */
}

/* ─────────────────────────────────────────────────────────────
 * dma_rearm
 *
 * Called before every SWSTART to reset the stream for a new
 * single-shot transfer.
 *
 * Why this is needed (RM0090 §10.3.17 + §13.8.1):
 *   After the stream completes (NDTR=0), EN is cleared by hardware.
 *   NDTR stays at 0. A new transfer cannot start until EN is set
 *   again — and NDTR must be reloaded before EN is set, because
 *   writing NDTR is forbidden while EN=1.
 *   The TC flag must also be cleared before re-enabling:
 *   "All stream dedicated bits in the status register must be
 *   cleared before the stream can be re-enabled."
 * ───────────────────────────────────────────────────────────── */
static void dma_rearm(void)
{
	/* Ensure stream is disabled (should already be, but be safe) */
    DMA2_S0CR &= ~DMA_SxCR_EN;
    while (DMA2_S0CR & DMA_SxCR_EN) { }

    /* Clear transfer complete and any other flags from last shot */
    DMA2_LIFCR = DMA_LIFCR_CLEAR_S0;

    /* Reload transfer count — must be written while EN=0 */
    DMA2_S0NDTR = ADC_NUM_CHANNELS;

    /* Re-enable — stream is now armed for the next 2 transfers */
    DMA2_S0CR |= DMA_SxCR_EN;

    /*
     * Re-arm ADC DMA request logic.
     * When DDS=0, the ADC clears its internal DMA enable after
     * the last transfer of a single-mode sequence. Toggling the
     * DMA bit in CR2 resets it so the next SWSTART generates
     * DMA requests again.
     */
    ADC1_CR2 &= ~ADC_CR2_DMA;
    ADC1_CR2 |=  ADC_CR2_DMA;
}

/* ─────────────────────────────────────────────────────────────
 * dma_transfer_complete
 *
 * Returns 1 when DMA has finished transferring both channel
 * results into adc_results[]. Used for polling in the main loop.
 *
 * TCIF0 is bit 5 of DMA2_LISR (Stream 0 transfer complete).
 * It is set by hardware when NDTR reaches 0.
 * ───────────────────────────────────────────────────────────── */
static uint8_t dma_transfer_complete(void)
{
    return (DMA2_LISR & DMA_LISR_TCIF0) ? 1 : 0;
}

/* ─────────────────────────────────────────────────────────────
 * adc_init
 *
 * Configures ADC1 for single scan of CH1 then CH4.
 * CONT=0: sequence runs once per SWSTART then stops.
 * DMA=1:  fire DMA request after each conversion.
 * DDS=0:  no further DMA requests after last transfer —
 *         matches normal (non-circular) DMA mode.
 * ───────────────────────────────────────────────────────────── */
static void adc_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC_APB2ENR;

    ADC_CCR &= ~(0x3UL << 16);
    ADC_CCR |=  ADC_CCR_ADCPRE_DIV4;

    /* CR1: 12-bit, SCAN=1 (required for multi-channel) */
    ADC1_CR1 = ADC_CR1_RES_12BIT
             | ADC_CR1_SCAN;

    /*
     * CR2: DMA=1, DDS=0, CONT=0, software trigger only.
     *
     * CONT=0: ADC converts once and stops. This is the defining
     *         difference from the continuous mode project.
     *
     * DDS=0:  after the last transfer the ADC stops generating
     *         DMA requests. This matches the DMA stream stopping
     *         at NDTR=0 — both ADC and DMA halt together cleanly.
     *
     * EOCS:   not set — CPU never reads ADC_DR directly.
     *         DMA handles all data movement as in continuous mode.
     */
    ADC1_CR2 = ADC_CR2_DMA
             | ADC_CR2_EXTEN_NONE;
    /* CONT=0, DDS=0, EOCS=0 — all left clear intentionally */

    /* Sample time: 480 cycles for both channels */
    ADC1_SMPR2 &= ~(ADC_SMPR2_CH1_MASK | ADC_SMPR2_CH4_MASK);
    ADC1_SMPR2 |=  (ADC_SMPR2_CH1_480CYC | ADC_SMPR2_CH4_480CYC);

    /* Sequence: 2 conversions, CH1 then CH4 */
    ADC1_SQR1 = ADC_SQR1_L_2CONV;
    ADC1_SQR3 = ADC_SQR3_SQ1_CH1
              | ADC_SQR3_SQ2_CH4;

    /* Power on and wait for analog circuits to stabilise */
    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);

    ADC1_SR = 0;    /* clear any stale flags */
}

/* ─────────────────────────────────────────────────────────────
 * adc_start_single_scan
 *
 * Arms DMA, then fires SWSTART.
 * DMA must be armed first so it is ready the moment the first
 * conversion completes — same ordering rule as continuous mode.
 * ───────────────────────────────────────────────────────────── */
static void adc_start_single_scan(void)
{
    ADC1_SR = 0;                    /* clear stale ADC flags */
    dma_rearm();
    ADC1_CR2 |= ADC_CR2_SWSTART;
}

/* ─────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────── */
int main(void)
{
    gpio_init();
    dma_configure();
    adc_init();

    uint32_t count = 0;

    while (1)
    {
        /*
         * Trigger one complete scan: CH1 then CH4.
         * DMA transfers both results into adc_results[].
         * ADC stops. DMA stream stops (NDTR=0, EN cleared).
         */
        adc_start_single_scan();

        /*
         * Wait for DMA transfer complete flag (TCIF0).
         * This is set by hardware when NDTR reaches 0 — i.e.
         * both channel results are safely in adc_results[].
         *
         * We poll here rather than using an interrupt to keep
         * the code simple. In a real application you would
         * handle TCIF0 in a DMA interrupt handler instead.
         *
         * Timeout guard: if DMA never completes (misconfigured
         * stream, clock not enabled, etc.) this prevents an
         * infinite hang.
         */
        volatile uint32_t timeout = 1000000;
        while (!dma_transfer_complete() && timeout) { timeout--; }

        if (timeout == 0)
        {
            /*
             * DMA did not complete — likely a configuration error.
             * Check: DMA2 clock enabled? NDTR set before EN?
             *        ADC1 clock enabled? PA1/PA4 in Analog mode?
             */
            printf("DMA timeout — check configuration\r\n");
            delay(2000000);
            continue;
        }

        /*
         * Both results are now valid in adc_results[].
         * Read them into locals before printing — the DMA
         * cannot overwrite them while the stream is stopped,
         * but taking a snapshot is good practice.
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

        /*
         * Delay before next scan.
         * Unlike continuous mode, nothing happens between scans —
         * the ADC is completely idle during this delay.
         * Remove the delay to scan as fast as the loop allows.
         */
        delay(2000000);
    }
}
