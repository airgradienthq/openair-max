/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef MAX_CONFIG_H
#define MAX_CONFIG_H

#include "soc/gpio_num.h"

enum class NetworkOption {
  WiFi = 0,
  Cellular
};

#define MAX_PAYLOAD_CACHE 100

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

constexpr gpio_num_t EN_CO2 = GPIO_NUM_5;
constexpr gpio_num_t EN_PMS1 = GPIO_NUM_3;
constexpr gpio_num_t EN_PMS2 = GPIO_NUM_22;
constexpr gpio_num_t EN_ALPHASENSE = GPIO_NUM_4;
constexpr gpio_num_t IO_WDT = GPIO_NUM_2;
constexpr gpio_num_t EN_CE_CARD = GPIO_NUM_15;
constexpr gpio_num_t IO_CE_POWER = GPIO_NUM_23;
constexpr gpio_num_t IO_IIC_RESET = GPIO_NUM_21;
constexpr gpio_num_t IO_LED_INDICATOR = GPIO_NUM_10;
constexpr gpio_num_t IO_BOOT_BUTTON = GPIO_NUM_9;

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

#define I2C_ADDR_ADS1115_1 0x48
#define I2C_ADDR_ADS1115_2 0x49

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
#define DEFAULT_ABC_PERIOD_DAYS 7

#define RESET_CE_CARD_CYCLE_ON_RECONNECTION_MS (15 * 60000)       // 15 Minutes
#define TIMEOUT_ENSURING_CONNECTION_ON_BOOT_MS (60 * 60000)       // 1 hour
#define TIMEOUT_REGISTER_NETWORK_ON_FIRST_BOOT_MS (5 * 60 * 1000) // 5 minutes
#define TIMEOUT_REGISTER_NETWORK_WAKEUP_CYCLE_MS (100 * 1000)     // 100 seconds

#define MAX_SLEEP_TIME 300 // This is maximum it able to sleep before ext HW watchdog reset

#endif
