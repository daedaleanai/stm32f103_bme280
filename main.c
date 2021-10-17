/*
 *  At read registers over SPI1 and print to UART TX1 (921600 8N1)
 *
 */
#include "stm32f103_md.h"

#include "bme280.h"
#include "clock.h"
#include "gpio2.h"
#include "serial.h"
#include "spi.h"
/*
    STM32F013CB (LQFP48/LQFP48) Pin Assignments:

    Pin   Function     DIR   Electrical     Connected to
    ---   ---------    ---   ----------- ---------------------------------------

    PA4   SPI1 NSS     out   AF_PP 50MHz    bme280 CSB  active low
    PA5   SPI1 SCK     out   AF_PP 50MHz    bme280 SPI sck
    PA6   SPI1 MISO    in    PullUp         bme280 SPI miso (sdo)
    PA7   SPI1 MOSI    out   AF_PP 50MHz    bme280 SPI mosi (sdi)

    PA9   USART1 TX    out   AF_PP 10MHz
    PA10  USART1 RX    in    PullUp

    PA13  SWDIO        in/out               ST-Link programmer
    PA14  SWCLK        in/out               ST-Link programmer


    PC13  LED0         out   OUT_OD 2MHz    On-board yellow LED

*/

struct SPI_Type *const BME_SPI = &SPI1;

enum {
    BME_CSB_PIN = PA4,  // NSS on SPI1
    SPI1_SCK_PIN = PA5,
    SPI1_MISO_PIN = PA6,
    SPI1_MOSI_PIN = PA7,

    USART1_TX_PIN = PA9,
    USART1_RX_PIN = PA10,
    LED0_PIN = PC13,
};

/* clang-format off */
static struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, Mode_IN}, // reset
    {PBAll, Mode_IN}, // reset
    {PCAll, Mode_IN}, // reset
    {SPI1_MOSI_PIN | SPI1_SCK_PIN | BME_CSB_PIN, Mode_AF_PP_50MHz},
    {SPI1_MISO_PIN, Mode_IPU},
    {USART1_TX_PIN, Mode_AF_PP_50MHz},
    {USART1_RX_PIN, Mode_IPU},
    {LED0_PIN, Mode_Out_OD_2MHz},
    {0, 0}, // sentinel
};
/* clang-format on */

void spi1_slave_select(int slave_func) {
    (void)slave_func;
}
void spi2_slave_select(int slave_func) {
    (void)slave_func;
}

// blocking write single register
static uint16_t bme_writereg(struct SPI_Type *spi, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    spi_xmit_enq(spi, 0, 2, buf);
    spi_wait(spi);
    int dum;
    return spi_xmit_deq(spi, &dum, NULL, NULL);
}

// blocking read single register
static uint16_t bme_readreg(struct SPI_Type *spi, uint8_t reg, uint8_t *val) {
    reg |= BME280_READREG;  // flag for reading
    uint8_t buf[2] = {reg, 0};
    const int n = sizeof buf;
    spi_xmit_enq(spi, 0, n, buf);
    spi_wait(spi);
    int dum;
    uint16_t r = spi_xmit_deq(spi, &dum, NULL, NULL);
    *val = buf[n - 1];
    return r;
}

static inline void led0_on(void) {
    digitalLo(LED0_PIN);
}
static inline void led0_off(void) {
    digitalHi(LED0_PIN);
}
static inline void led0_toggle(void) {
    digitalToggle(LED0_PIN);
}

void TIM3_IRQ_Handler(void) {
    if ((TIM3.SR & TIM_SR_UIF) == 0)
        return;

    led0_toggle();

    TIM3.SR &= ~TIM_SR_UIF;
}

static struct Ringbuffer usart1tx;
static struct LinearisationParameters linparm;

/* clang-format off */
enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
    enum IRQn_Type irq;
    uint8_t        group, sub;
} irqprios[] = {
    {SysTick_IRQn,       0, 0},
	{DMA1_Channel2_IRQn, 1, 0}, // spi1 done xmit
    {DMA1_Channel4_IRQn, 1, 0}, // spi2 done xmit
    {TIM3_IRQn,          2, 0},
    {USART1_IRQn,        3, 0},
    {None_IRQn, 0xff, 0xff},
};
/* clang-format on */

int main(void) {
    uint8_t rf = (RCC.CSR >> 24) & 0xfc;
    RCC.CSR |= RCC_CSR_RMVF;  // Set RMVF bit to clear the reset flags

    SysTick_Config(1U << 24);  // tick at 72Mhz/2^24 = 4.2915 HZ

    NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
        NVIC_SetPriority(
                irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
    }

    RCC.APB2ENR |=
            RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_SPI1EN;
    RCC.APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC.AHBENR |= RCC_AHBENR_DMA1EN;

    delay(10);  // let all clocks and peripherals start up

    for (const struct gpio_config_t *p = pin_cfgs; p->pins; ++p) {
        gpioConfig(p->pins, p->mode);
    }

    gpioLock(PAAll);
    gpioLock(PBAll);
    gpioLock(PCAll);

    __enable_irq();

    led0_off();

    spi_init(BME_SPI, 4);  // APB2Clk / (1<<3) = 4.5MHz: 14..19us per xmit)

    serial_init(&USART1, 921600, &usart1tx);

    serial_printf(&USART1, "SWREV:%s\n", __REVISION__);
    serial_printf(&USART1, "CPUID:%08lx\n", SCB.CPUID);
    serial_printf(&USART1, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
    serial_printf(
            &USART1, "RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "",
            rf & 0x20 ? " IWDG" : "", rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
    serial_wait(&USART1);

    // enable 1Hz TIM3 to trigger
    TIM3.DIER |= TIM_DIER_UIE;
    TIM3.PSC = 7200 - 1;  // 72MHz / 7200 = 10Khz
    TIM3.ARR = 10000 - 1;  // 10KHz/10000 = 1Hz
    TIM3.CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn);

    {
        uint8_t v;
        uint16_t r = bme_readreg(BME_SPI, 0xd0, &v);
        if ((r != 0) || v != 0x60)
            serial_printf(&USART1, "bme ID expected 0x60, error: 0x%x (%d)\n", v, r);
    }

    {
        uint8_t buf88[1 + BME280_CALIBTP_LEN] = {BME280_CALIBTP_REG | BME280_READREG};
        spi_xmit_enq(BME_SPI, 1, sizeof buf88, buf88);
        spi_wait(BME_SPI);
        int dum = 0;
        uint16_t r = spi_xmit_deq(BME_SPI, &dum, NULL, NULL);
        if ((r != 0) || (dum != 1))
            serial_printf(&USART1, "BME read 1: %d %x\n", r, dum);

        uint8_t bufe1[1 + BME280_CALIBH_LEN] = {BME280_CALIBH_REG | BME280_READREG};
        spi_xmit_enq(BME_SPI, 2, sizeof bufe1, bufe1);
        spi_wait(BME_SPI);
        dum = 0;
        r = spi_xmit_deq(BME_SPI, &dum, NULL, NULL);
        if ((r != 0) || (dum != 2))
            serial_printf(&USART1, "BME read 2: %d %x\n", r, dum);

        bme_decodeLinearisationParameters(&linparm, buf88 + 1, bufe1 + 1);
    }

    serial_printf(&USART1, "T:");
    for (size_t i = 1; i < 4; ++i)
        serial_printf(&USART1, " %ld", linparm.T[i]);
    serial_printf(&USART1, "\n");

    serial_printf(&USART1, "P:");
    for (size_t i = 1; i < 10; ++i)
        serial_printf(&USART1, " %ld", linparm.P[i]);
    serial_printf(&USART1, "\n");

    serial_printf(&USART1, "H:");
    for (size_t i = 1; i < 7; ++i)
        serial_printf(&USART1, " %ld", linparm.H[i]);
    serial_printf(&USART1, "\n");

#if 1
    bme_writereg(BME_SPI, BME280_REG_CONFIG, BME280_CONFIG_TSB10|BME280_CONFIG_FLT16); 
    bme_writereg(BME_SPI, BME280_REG_CTRLHUM, BME280_CTRLHUM_H16); 
    bme_writereg(BME_SPI, BME280_REG_CTRLMEAS, BME280_CTRLMEAS_P16|BME280_CTRLMEAS_T16|BME280_CTRLMEAS_NORMAL);
#else
    bme_writereg(BME_SPI, 0x72, 0b00000100);  // humidity oversampling x8
    bme_writereg(BME_SPI, 0x74, 0b10010011);  // tmp, pressure 8x, normal mode
#endif

    // while (IWDG.SR != 0)
    // 	__NOP();

    // IWDG.KR  = 0x5555; // enable watchdog config
    // IWDG.PR  = 0;      // prescaler /4 -> 10kHz
    // IWDG.RLR = 0xFFF;  // count to 4096 -> 409.6ms timeout
    // IWDG.KR  = 0xcccc; // start watchdog countdown

    int dum = 3;
    for (uint64_t count = 0;; ++count) {
        //		__enable_irq();

        serial_wait(&USART1);  // empty tx buf so all the printfs below wont block
                               // (512 bytes buffer)

        //		__WFI(); // wait for interrupt to change the state of any of the
        // subsystems
        //		__disable_irq();

        uint8_t buf[1 + BME280_DATA_LEN] = {BME280_DATA_REG | BME280_READREG};
        spi_xmit_enq(BME_SPI, dum + 1, sizeof buf, buf);
        spi_wait(BME_SPI);

        uint16_t r = spi_xmit_deq(BME_SPI, &dum, NULL, NULL);
        serial_printf(&USART1, "BME read: %d %x\n", r, dum);

        for (size_t i = 0; i < sizeof buf; ++i) {
            serial_printf(&USART1, " %02x", buf[i]);
        }

        uint32_t t_mdegc, p_mpa, hume6;
        bme_decode(&linparm, buf + 1, &t_mdegc, &p_mpa, &hume6);

        serial_printf(&USART1, " t:%ld  p:%ld h:%ld\n", t_mdegc, p_mpa, hume6);

        delay(1000 * 1000);

        //		IWDG.KR = 0xAAAA; // kick the watchdog

    }  // forever

    return 0;
}
