#ifndef MAX_CONFIG_H
#define MAX_CONFIG_H

#include "driver/gpio.h"

constexpr gpio_num_t EN_CO2 = GPIO_NUM_15;
constexpr gpio_num_t EN_PM1 = GPIO_NUM_3;
constexpr gpio_num_t EN_PM2 = GPIO_NUM_11;
constexpr gpio_num_t IO_WDT = GPIO_NUM_2;
constexpr gpio_num_t EN_CE_CARD = GPIO_NUM_22;
constexpr gpio_num_t IO_CE_POWER = GPIO_NUM_23;
constexpr gpio_num_t IO_IIC_RESET = GPIO_NUM_21;
constexpr gpio_num_t IO_LED_INDICATOR = GPIO_NUM_10;

#define UART_RX_SUNLIGHT 0
#define UART_TX_SUNLIGHT 1
#define UART_BAUD_SUNLIGHT 9600
#define UART_PORT_SUNLIGHT 0

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
#define DEFAULT_INVALID_VBATT -1
#define DEFAULT_INVALID_VPANEL -1

// TODO: check if invalid value macro

#endif
