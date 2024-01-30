/* bmi160 driver for hardware spi/dma with data ready interrupt (stm32f103c8t6) */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "bmi160.h"

#define BMI160_SPI (SPI2)
#define BMI160_DMA (DMA1)

#if (BMI160_DMA == DMA1)
 #if (BMI160_SPI == SPI1)
  #error TODO: add SPI1 support
 #elif (BMI160_SPI == SPI2) 
  #define BMI160_GPIO_PORT_MISO (GPIO_BANK_SPI2_MISO)
  #define BMI160_GPIO_PORT_MOSI (GPIO_BANK_SPI2_MOSI)
  #define BMI160_GPIO_PORT_SCK  (GPIO_BANK_SPI2_SCK)
  #define BMI160_GPIO_PORT_NSS  (GPIO_BANK_SPI2_NSS)
  #define BMI160_GPIO_PIN_MISO  (GPIO_SPI2_MISO)
  #define BMI160_GPIO_PIN_MOSI  (GPIO_SPI2_MOSI)
  #define BMI160_GPIO_PIN_SCK   (GPIO_SPI2_SCK)
  #define BMI160_GPIO_PIN_NSS   (GPIO_SPI2_NSS)
  #define BMI160_DMA_CHANNEL_RX (DMA_CHANNEL4)
  #define BMI160_DMA_CHANNEL_TX (DMA_CHANNEL5)
  #define BMI160_DMA_ISR_RX     (dma1_channel4_isr)
  #define BMI160_DMA_ISR_TX     (dma1_channel5_isr)
  #define BMI160_DMA_IRQ_RX     (NVIC_DMA1_CHANNEL4_IRQ)
  #define BMI160_DMA_IRQ_TX     (NVIC_DMA1_CHANNEL5_IRQ)
 #else
  #error Invalid SPI device selected
 #endif
#else
 #error Invalid DMA device selected
#endif

#define BMI160_CHIP_ID_ADDR       (0x00)
#define BMI160_ERROR_REG_ADDR     (0x02)
#define BMI160_GYRO_DATA_ADDR     (0x0C)
#define BMI160_ACCEL_CONFIG_ADDR  (0x40)
#define BMI160_ACCEL_RANGE_ADDR   (0x41)
#define BMI160_GYRO_CONFIG_ADDR   (0x42)
#define BMI160_GYRO_RANGE_ADDR    (0x43)
#define BMI160_INT_ENABLE_1_ADDR  (0x51)
#define BMI160_INT_OUT_CTRL_ADDR  (0x53)
#define BMI160_INT_LATCH_ADDR     (0x54)
#define BMI160_INT_MAP_1_ADDR     (0x56)
#define BMI160_COMMAND_REG_ADDR   (0x7E)
#define BMI160_SPI_COMM_TEST_ADDR (0x7F)

typedef struct __attribute__((packed)) {
  int16_t x;
  int16_t y;
  int16_t z;
} bmi160_xyz_i16_t;

typedef struct __attribute__((packed)) {
  uint8_t reg;
  bmi160_xyz_i16_t gyr;
  bmi160_xyz_i16_t acc;
} bmi160_acc_gyr_data_t;

static bmi160_acc_gyr_data_t bmi160_acc_gyr_data = {};
static bmi160_xyz_i16_t bmi160_acc_cache = {};
static bmi160_xyz_i16_t bmi160_gyr_cache = {};

static void bmi160_delay_ms(uint16_t ms);
static void bmi160_select(void);
static void bmi160_release(void);
static void bmi160_write_u8(uint8_t reg, uint8_t val);
static uint8_t bmi160_read_u8(uint8_t reg);
static bool bmi160_device_init(void);

static void bmi160_delay_ms(uint16_t ms)
{
  uint32_t nops = 8000 * ms;
  while (nops--) {
    __asm__("nop");
  }
}

static void bmi160_select(void)
{
  gpio_clear(BMI160_GPIO_PORT_NSS, BMI160_GPIO_PIN_NSS);
}

static void bmi160_release(void)
{
  gpio_set(BMI160_GPIO_PORT_NSS, BMI160_GPIO_PIN_NSS);
}

static void bmi160_write_u8(uint8_t reg, uint8_t val)
{
  bmi160_select();
  spi_xfer(BMI160_SPI, reg);
  spi_xfer(BMI160_SPI, val);
  bmi160_release();
}

static uint8_t bmi160_read_u8(uint8_t reg)
{
  uint8_t val = 0xFF;
  bmi160_select();
  spi_xfer(BMI160_SPI, reg | 0x80);
  val = spi_xfer(BMI160_SPI, 0xFF);
  bmi160_release();
  return val;
}

static bool bmi160_device_init(void)
{
  bool result = false;
  uint8_t chip_id = 0;
  uint32_t i = 0;

  /* switch device to SPI mode */
  bmi160_select();
  bmi160_delay_ms(1);
  bmi160_release();
  bmi160_delay_ms(1);

  /* read chip id */
  chip_id = bmi160_read_u8(BMI160_CHIP_ID_ADDR);
  if (0xD1 == chip_id)
  {
    /* soft reset */
    bmi160_write_u8(BMI160_COMMAND_REG_ADDR, 0xB6);
    bmi160_delay_ms(1);

    /* switch device to SPI mode */
    bmi160_select();
    bmi160_delay_ms(1);
    bmi160_release();
    bmi160_delay_ms(1);

    /* read chip id */
    chip_id = bmi160_read_u8(BMI160_CHIP_ID_ADDR);
    if (0xD1 == chip_id)
    {
      result = true;
    }
  }

  return result;
}

/* dma spi rx complete interrupt */
void BMI160_DMA_ISR_RX(void)
{
  dma_clear_interrupt_flags(BMI160_DMA, BMI160_DMA_CHANNEL_RX, DMA_TCIF);
  spi_disable_rx_dma(BMI160_SPI);
  bmi160_release();

  bmi160_acc_cache.x = bmi160_acc_gyr_data.acc.x;
  bmi160_acc_cache.y = bmi160_acc_gyr_data.acc.y;
  bmi160_acc_cache.z = bmi160_acc_gyr_data.acc.z;

  bmi160_gyr_cache.x = bmi160_acc_gyr_data.gyr.x;
  bmi160_gyr_cache.y = bmi160_acc_gyr_data.gyr.y;
  bmi160_gyr_cache.z = bmi160_acc_gyr_data.gyr.z;
}

/* dma spi tx complete interrupt */
void BMI160_DMA_ISR_TX(void)
{
  dma_clear_interrupt_flags(BMI160_DMA, BMI160_DMA_CHANNEL_TX, DMA_TCIF);
  spi_disable_tx_dma(BMI160_SPI);
}

bool bmi160_init(void)
{
  bool result = false;

  /* configure spi gpio */
  gpio_set_mode(BMI160_GPIO_PORT_MISO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BMI160_GPIO_PIN_MISO);
  gpio_set_mode(BMI160_GPIO_PORT_MOSI, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, BMI160_GPIO_PIN_MOSI);
  gpio_set_mode(BMI160_GPIO_PORT_SCK, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, BMI160_GPIO_PIN_SCK);
  gpio_set_mode(BMI160_GPIO_PORT_NSS, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, BMI160_GPIO_PIN_NSS);

  /* configure dma for rx */
  dma_channel_reset(BMI160_DMA, BMI160_DMA_CHANNEL_RX);
  dma_set_peripheral_size(BMI160_DMA, BMI160_DMA_CHANNEL_RX, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(BMI160_DMA, BMI160_DMA_CHANNEL_RX, DMA_CCR_MSIZE_8BIT);
  dma_enable_memory_increment_mode(BMI160_DMA, BMI160_DMA_CHANNEL_RX);
  dma_disable_peripheral_increment_mode(BMI160_DMA, BMI160_DMA_CHANNEL_RX);
  dma_set_read_from_peripheral(BMI160_DMA, BMI160_DMA_CHANNEL_RX);
  dma_set_priority(BMI160_DMA, BMI160_DMA_CHANNEL_RX, DMA_CCR_PL_HIGH);
  dma_set_memory_address(BMI160_DMA, BMI160_DMA_CHANNEL_RX, (uint32_t)&bmi160_acc_gyr_data);
  dma_set_peripheral_address(BMI160_DMA, BMI160_DMA_CHANNEL_RX, (uint32_t)&(SPI_DR(BMI160_SPI)));

  /* configure dma for tx */
  dma_channel_reset(BMI160_DMA, BMI160_DMA_CHANNEL_TX);
  dma_set_peripheral_size(BMI160_DMA, BMI160_DMA_CHANNEL_TX, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(BMI160_DMA, BMI160_DMA_CHANNEL_TX, DMA_CCR_MSIZE_8BIT);
  dma_enable_memory_increment_mode(BMI160_DMA, BMI160_DMA_CHANNEL_TX);
  dma_disable_peripheral_increment_mode(BMI160_DMA, BMI160_DMA_CHANNEL_TX);
  dma_set_read_from_memory(BMI160_DMA, BMI160_DMA_CHANNEL_TX);
  dma_set_priority(BMI160_DMA, BMI160_DMA_CHANNEL_TX, DMA_CCR_PL_HIGH);
  dma_set_memory_address(BMI160_DMA, BMI160_DMA_CHANNEL_TX, (uint32_t)&bmi160_acc_gyr_data);
  dma_set_peripheral_address(BMI160_DMA, BMI160_DMA_CHANNEL_TX, (uint32_t)&(SPI_DR(BMI160_SPI)));

  /* configure dma rx complete interrupt */
  nvic_set_priority(BMI160_DMA_IRQ_RX, 0);
  nvic_enable_irq(BMI160_DMA_IRQ_RX);
  dma_enable_transfer_complete_interrupt(BMI160_DMA, BMI160_DMA_CHANNEL_RX);

  /* configure dma tx complete interrupt */
  nvic_set_priority(BMI160_DMA_IRQ_TX, 0);
  nvic_enable_irq(BMI160_DMA_IRQ_TX);
  dma_enable_transfer_complete_interrupt(BMI160_DMA, BMI160_DMA_CHANNEL_TX);

  /* configure spi */
  spi_init_master(BMI160_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_4, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable(BMI160_SPI);

  /* init bmi160 device */
  if (true == bmi160_device_init())
  {
    result = true;
  }

  return result;
}

/* TODO: pass data rate and range as parameters */
bool bmi160_set_sensivity(void)
{
  bool result = false;
  uint8_t err = 0;

  /* set accelerometer config - acc_us:no acc_bwp:norm acc_odr:1600Hz */
  bmi160_write_u8(BMI160_ACCEL_CONFIG_ADDR, 0x2C);
  /* set accelerometer range - acc_range:16g*/
  bmi160_write_u8(BMI160_ACCEL_RANGE_ADDR, 0x0C);
  /* set acceletometer power mode - cmd:0b000100xx + norm:0b01 */
  bmi160_write_u8(BMI160_COMMAND_REG_ADDR, 0x11);
  bmi160_delay_ms(4);

  /* set gyroscope config - gyr_bwp:norm gyr_odr:1600Hz */
  bmi160_write_u8(BMI160_GYRO_CONFIG_ADDR, 0x2C);
  /* set gyroscope range - gyr_range:2000dps (0.061dps/bit) */
  bmi160_write_u8(BMI160_GYRO_RANGE_ADDR, 0x00);
  /* set gyroscope power mode - cmd:0b000101xx + norm:0b01 */
  bmi160_write_u8(BMI160_COMMAND_REG_ADDR, 0x15);
  bmi160_delay_ms(80);

  /* check for errors */
  err = bmi160_read_u8(BMI160_ERROR_REG_ADDR);
  if (0x00 == err)
  {
    result = true;
  }

  return result;
}

/* TODO: pass int1/int2 as parameter */
void bmi160_enable_data_ready_interrupt(void)
{
  /* enable data ready interrupt - int_drdy_en:0b00010000 */
  bmi160_write_u8(BMI160_INT_ENABLE_1_ADDR, 0x10);
  /* configure interrupt pin - int1_lvl:0b00000010 + int1_output_en:0b00001000 */
  bmi160_write_u8(BMI160_INT_OUT_CTRL_ADDR, 0x0A);
  /* configure latch control - no inputs, no latching */
  bmi160_write_u8(BMI160_INT_LATCH_ADDR, 0x00);
  /* map hardware interrupt - int_map1_drdy:0b10000000*/
  bmi160_write_u8(BMI160_INT_MAP_1_ADDR, 0x80);
}

void bmi160_update(void)
{
  /* skip for ongoing data exchange */
  if (0 == dma_get_number_of_data(BMI160_DMA, BMI160_DMA_CHANNEL_RX))
  {
    dma_disable_channel(BMI160_DMA, BMI160_DMA_CHANNEL_RX);
    dma_disable_channel(BMI160_DMA, BMI160_DMA_CHANNEL_TX);

    bmi160_acc_gyr_data.reg = BMI160_GYRO_DATA_ADDR | 0x80;

    dma_set_number_of_data(BMI160_DMA, BMI160_DMA_CHANNEL_RX, sizeof(bmi160_acc_gyr_data));
    dma_set_number_of_data(BMI160_DMA, BMI160_DMA_CHANNEL_TX, sizeof(bmi160_acc_gyr_data));

    dma_enable_channel(BMI160_DMA, BMI160_DMA_CHANNEL_RX);
    dma_enable_channel(BMI160_DMA, BMI160_DMA_CHANNEL_TX);

    bmi160_select();

    spi_enable_tx_dma(BMI160_SPI);
    spi_enable_rx_dma(BMI160_SPI);
  }
}

void bmi160_get_acc(int16_t *ax, int16_t *ay, int16_t *az)
{
  if (NULL != ax)
  {
    *ax = bmi160_acc_cache.x;
  }

  if (NULL != ay)
  {
    *ay = bmi160_acc_cache.y;
  }

  if (NULL != az)
  {
    *az = bmi160_acc_cache.z;
  }
}

void bmi160_get_gyr(int16_t *gx, int16_t *gy, int16_t *gz)
{
  if (NULL != gx)
  {
    *gx = bmi160_gyr_cache.x;
  }

  if (NULL != gy)
  {
    *gy = bmi160_gyr_cache.y;
  }

  if (NULL != gz)
  {
    *gz = bmi160_gyr_cache.z;
  }
}
