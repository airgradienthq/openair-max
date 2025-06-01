/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef MAX_CONFIG_H
#define MAX_CONFIG_H

#include "soc/gpio_num.h"

#define MAX_PAYLOAD_CACHE 100

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

constexpr gpio_num_t EN_CO2 = GPIO_NUM_5;
constexpr gpio_num_t EN_PMS = GPIO_NUM_3;
constexpr gpio_num_t EN_NO2 = GPIO_NUM_4;
constexpr gpio_num_t IO_WDT = GPIO_NUM_2;
constexpr gpio_num_t EN_CE_CARD = GPIO_NUM_15;
constexpr gpio_num_t IO_CE_POWER = GPIO_NUM_23;
constexpr gpio_num_t IO_IIC_RESET = GPIO_NUM_21;
constexpr gpio_num_t IO_LED_INDICATOR = GPIO_NUM_10;

#define UART_RX_SUNLIGHT 0
#define UART_TX_SUNLIGHT 1
#define UART_BAUD_SUNLIGHT 9600
#define UART_PORT_SUNLIGHT 0

#define UART_RX_CE_CARD 17
#define UART_TX_CE_CARD 16
#define UART_BAUD_CE_CARD 115200
#define UART_BAUD_PORT_CE_CARD 1

#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT 0

#define DEFAULT_INVALID_TEMPERATURE -1000
#define DEFAULT_INVALID_HUMIDITY -1
#define DEFAULT_INVALID_PM -1
#define DEFAULT_INVALID_CO2 -1
#define DEFAULT_INVALID_TVOC -1
#define DEFAULT_INVALID_NOX -1
#define DEFAULT_INVALID_VOLT -1

#define IS_PM_VALID(val) (val >= 0)
#define IS_TEMPERATURE_VALID(val) ((val >= -40) && (val <= 125))
#define IS_HUMIDITY_VALID(val) ((val >= 0) && (val <= 100))
#define IS_CO2_VALID(val) ((val >= 0) && (val <= 10000))
#define IS_TVOC_VALID(val) (val >= 0)
#define IS_NOX_VALID(val) (val >= 0)
#define IS_VOLT_VALID(val) (val >= 0)

#define TRANSMIT_MEASUREMENTS_CYCLES 3
#define MEASURE_CYCLE_INTERVAL_SECONDS 180
#define FIRMWARE_UPDATE_CHECK_CYCLES 30 // 1 hour

#define DEFAULT_MEASURE_ITERATION_COUNT 20
#define DEFAULT_MEASURE_INTERVAL_MS_PER_ITERATION 2000

#endif
