#include "AirgradientUART.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

bool AirgradientUART::open(int port, int baud, int rx, int tx) {
  uart_config_t uart_config = {
      .baud_rate = baud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  _port_num = static_cast<uart_port_t>(port);

  esp_err_t err;
  err = uart_driver_install(_port_num, BUF_SIZE * 2, 0, 0, nullptr, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
    return false;
  }

  err = uart_param_config(_port_num, &uart_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
    uart_driver_delete(_port_num);
    return false;
  }

  err = uart_set_pin(_port_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
    uart_driver_delete(_port_num);
    return false;
  }

  ESP_LOGI(TAG, "Success initialize UART");
  isOpen = true;
  return true;
}

void AirgradientUART::close() {
  if (isOpen) {
    uart_driver_delete(_port_num);
    isOpen = false;
  }
}

int AirgradientUART::available() {
  size_t length;
  uart_get_buffered_data_len(_port_num, &length);
  return length;
}

void AirgradientUART::print(const char *str) {
  if (isOpen && str) {
    uart_write_bytes(_port_num, str, strlen(str));
  }
}

int AirgradientUART::write(const uint8_t *data, int len) {
  if (isOpen && data && len > 0) {
    int sent = uart_write_bytes(_port_num, data, len);
    if (sent <= 0) {
      return 0;
    }
    return sent;
  }

  return 0;
}

int AirgradientUART::read() {
  if (!isOpen) {
    return -1;
  }

  int data;
  int len = uart_read_bytes(_port_num, &data, 1, 10 / portTICK_PERIOD_MS);
  if (len <= 0) {
    return -1;
  }
  return data;
}
