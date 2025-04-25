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

#endif
