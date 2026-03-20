/*
 * main.c — STM32F407 Discovery
 * Bare-metal ADC: continuous multi-channel scan + DMA
 *
 * Channels:
 *   CH1  →  PA1  →  LDR in 10kΩ voltage divider
 *   CH4  →  PA4  →  Potentiometer wiper
 *
 * Hardware wiring:
 *   LDR:  3.3V ── LDR ── PA1 ── 10kΩ ── GND
 *   Pot:  3.3V ── [end] ── [wiper → PA4] ── [end] ── GND
 *
 * How it works end-to-end:
 *   1. ADC1 converts CH1 (PA1) → fires DMA request
 *   2. DMA2 Stream0 reads ADC1_DR → writes adc_results[0] → NDTR--
 *   3. ADC1 converts CH4 (PA4) → fires DMA request
 *   4. DMA2 Stream0 reads ADC1_DR → writes adc_results[1] → NDTR--
 *   5. NDTR=0: circular mode reloads NDTR=2, memory pointer resets
 *   6. ADC restarts CH1 — repeat forever, zero CPU involvement
 *   CPU reads adc_results[] whenever it wants fresh values.
 *
 * DMA assignment (RM0090 Table 44):
 *   ADC1  →  DMA2, Stream 0, Channel 0
 *
 * Register offsets (RM0090 §10.5):
 *   DMA_LISR    +0x00   low interrupt status (streams 0-3)
 *   DMA_LIFCR   +0x08   low interrupt flag clear
 *   DMA_SxCR    +0x10 + (0x18 × stream)
 *   DMA_SxNDTR  +0x14 + (0x18 × stream)
 *   DMA_SxPAR   +0x18 + (0x18 × stream)
 *   DMA_SxM0AR  +0x1C + (0x18 × stream)
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

/* PA1 bits [3:2], PA4 bits [9:8] — 11 = Analog */
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
#define ADC_CR2_CONT        (1UL << 1)
/*
 * DMA  bit [8]: ADC fires a DMA request after each conversion.
 * DDS  bit [9]: keep DMA requests going even after NDTR reaches 0.
 *               Without DDS=1 the DMA stops after one full pass
 *               and never restarts — the circular loop breaks.
 */
#define ADC_CR2_DMA         (1UL << 8)
#define ADC_CR2_DDS         (1UL << 9)
#define ADC_CR2_EXTEN_NONE  (0x0UL << 28)
#define ADC_CR2_SWSTART     (1UL << 30)

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
/* Stream 0 offset term: 0x18 × 0 = 0 */
#define DMA2_LISR   (*(volatile uint32_t *)(DMA2_BASE + 0x00UL))
#define DMA2_LIFCR  (*(volatile uint32_t *)(DMA2_BASE + 0x08UL))
#define DMA2_S0CR   (*(volatile uint32_t *)(DMA2_BASE + 0x10UL))
#define DMA2_S0NDTR (*(volatile uint32_t *)(DMA2_BASE + 0x14UL))
#define DMA2_S0PAR  (*(volatile uint32_t *)(DMA2_BASE + 0x18UL))
#define DMA2_S0M0AR (*(volatile uint32_t *)(DMA2_BASE + 0x1CUL))

/* ── DMA_SxCR bits (RM0090 §10.5.5) ────────────────────────── */
#define DMA_SxCR_EN        (1UL << 0)
/*
 * DIR[1:0] bits [7:6] — 00 = peripheral-to-memory
 */
#define DMA_SxCR_DIR_P2M   (0x0UL << 6)
#define DMA_SxCR_CIRC      (1UL << 8)    /* circular mode */
/*
 * PINC bit [9] — 0: peripheral address fixed (ADC_DR never moves)
 * MINC bit [10] — 1: memory address increments after each transfer
 */
#define DMA_SxCR_MINC      (1UL << 10)
/*
 * PSIZE[1:0] bits [12:11] — 01 = 16-bit (ADC_DR result is 16-bit)
 * MSIZE[1:0] bits [14:13] — 01 = 16-bit (adc_results is uint16_t)
 */
#define DMA_SxCR_PSIZE_16  (0x1UL << 11)
#define DMA_SxCR_MSIZE_16  (0x1UL << 13)
/*
 * PL[1:0] bits [17:16] — 10 = High priority
 */
#define DMA_SxCR_PL_HIGH   (0x2UL << 16)
/*
 * CHSEL[2:0] bits [27:25] — 000 = Channel 0
 * ADC1 → DMA2 Stream0 Channel0 (RM0090 Table 44)
 */
#define DMA_SxCR_CHSEL_CH0 (0x0UL << 25)

/*
 * DMA_LIFCR: clear all flags for Stream 0
 * CTCIF0=bit5, CHTIF0=bit4, CTEIF0=bit3, CDMEIF0=bit2, CFEIF0=bit0
 */
#define DMA_LIFCR_CLEAR_S0  (0x3DUL)

/* ── Number of channels ─────────────────────────────────────── */
#define ADC_NUM_CHANNELS  2

/* ── Result buffer ──────────────────────────────────────────── */
/*
 * DMA writes directly here from ADC1_DR.
 * volatile: value changes outside CPU control — never optimise away.
 * [0] = CH1 (PA1, LDR)   [1] = CH4 (PA4, pot)
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
    (void)RCC_AHB1ENR;

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA1;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA1;

    GPIOA_MODER &= ~GPIO_MODER_MASK_PA4;
    GPIOA_MODER |=  GPIO_MODER_ANALOG_PA4;
}

/* ─────────────────────────────────────────────────────────────
 * dma_init
 *
 * RM0090 §10.3.17 stream configuration procedure:
 *  1. Disable stream, wait for EN=0
 *  2. Clear all interrupt flags in LIFCR
 *  3. PAR  = &ADC1_DR  (fixed peripheral source)
 *  4. M0AR = &adc_results[0]  (destination array)
 *  5. NDTR = ADC_NUM_CHANNELS
 *  6. CR   = channel, direction, sizes, MINC, CIRC
 *  EN is not set here — set in start() after ADC is ready
 * ───────────────────────────────────────────────────────────── */
static void dma_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC_AHB1ENR;

    /* Step 1: disable stream */
    DMA2_S0CR &= ~DMA_SxCR_EN;
    while (DMA2_S0CR & DMA_SxCR_EN) { }

    /* Step 2: clear all Stream 0 flags */
    DMA2_LIFCR = DMA_LIFCR_CLEAR_S0;

    /* Step 3: peripheral address = ADC1_DR */
    DMA2_S0PAR = (uint32_t)(&ADC1_DR);

    /* Step 4: memory address = start of result array */
    DMA2_S0M0AR = (uint32_t)(adc_results);

    /* Step 5: transfer count */
    DMA2_S0NDTR = ADC_NUM_CHANNELS;

    /*
     * Step 6: stream control
     *   CHSEL=000  Channel 0  (ADC1)
     *   PL=10      High priority
     *   MSIZE=01   16-bit memory
     *   PSIZE=01   16-bit peripheral
     *   MINC=1     increment memory pointer
     *   PINC=0     peripheral pointer fixed (default, not set)
     *   CIRC=1     circular — NDTR reloads after every full pass
     *   DIR=00     peripheral-to-memory
     */
    DMA2_S0CR = DMA_SxCR_CHSEL_CH0
              | DMA_SxCR_PL_HIGH
              | DMA_SxCR_MSIZE_16
              | DMA_SxCR_PSIZE_16
              | DMA_SxCR_MINC
              | DMA_SxCR_CIRC
              | DMA_SxCR_DIR_P2M;
}

/* ─────────────────────────────────────────────────────────────
 * adc_init — ADC1 continuous scan, DMA enabled
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
     * CR2: CONT + DMA + DDS
     * EOCS is intentionally NOT set — CPU never reads ADC_DR.
     * DMA handles every transfer; EOC polling is not needed.
     */
    ADC1_CR2 = ADC_CR2_CONT
             | ADC_CR2_DMA
             | ADC_CR2_DDS
             | ADC_CR2_EXTEN_NONE;

    ADC1_SMPR2 &= ~(ADC_SMPR2_CH1_MASK | ADC_SMPR2_CH4_MASK);
    ADC1_SMPR2 |=  (ADC_SMPR2_CH1_480CYC | ADC_SMPR2_CH4_480CYC);

    ADC1_SQR1 = ADC_SQR1_L_2CONV;
    ADC1_SQR3 = ADC_SQR3_SQ1_CH1
              | ADC_SQR3_SQ2_CH4;

    ADC1_CR2 |= ADC_CR2_ADON;
    delay(100000);               /* ADC analog stabilisation */

    ADC1_SR = 0;
}

/* ─────────────────────────────────────────────────────────────
 * start
 *
 * Enable DMA first, then trigger ADC.
 * DMA must be armed before the first conversion completes —
 * if ADC fires a DMA request with no active stream, that
 * first transfer is lost and the channel alignment breaks.
 * ───────────────────────────────────────────────────────────── */
static void start(void)
{
    DMA2_S0CR  |= DMA_SxCR_EN;        /* arm DMA stream */
    ADC1_CR2   |= ADC_CR2_SWSTART;    /* one trigger, runs forever */
}

/* ─────────────────────────────────────────────────────────────
 * main
 * ───────────────────────────────────────────────────────────── */
int main(void)
{
    gpio_init();
    dma_init();
    adc_init();
    start();

    /*
     * From here the ADC and DMA run entirely in hardware.
     *
     * ADC cycle (continuous, forever):
     *   CH1 → ADC_DR → DMA → adc_results[0]
     *   CH4 → ADC_DR → DMA → adc_results[1]
     *   NDTR reloads → repeat
     *
     * The CPU just reads the array. No polling, no EOC wait,
     * no sync logic needed.
     *
     * To verify DMA is running:
     *   Watch DMA2_S0NDTR in Live Expressions — it should
     *   briefly show 1 and 0 between its steady value of 2.
     *   If it never changes from 2, DMA is not running.
     */

    uint32_t count = 0;

    while (1)
    {
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
    }
}
