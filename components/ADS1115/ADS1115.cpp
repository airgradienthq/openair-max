/*****************************************
 *
 * A MODIFIED VERSION FOR USE IN ESP-IDF ENVIRONMENT!
 *
 * This is a library for the ADS1115 and ADS1015 A/D Converter
 *
 * You'll find an example which should enable you to use the library.
 *
 * You are free to use it, change it or build on it. In case you like
 * it, it would be cool if you give it a star.
 *
 * If you find bugs, please inform me!
 *
 * Written by Wolfgang (Wolle) Ewald
 * https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)
 * https://wolles-elektronikkiste.de/ads1115 (German)
 *
 *******************************************/

#include "ADS1115.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/projdefs.h"
#include <cstdint>

esp_err_t ADS1115::reset() {
  uint8_t value[1] = {0x06};
  ESP_RETURN_ON_ERROR(i2c_master_transmit(_dev_handle, value, 1, 500), TAG,
                      "i2c_master_transmit, reset failed");
  return ESP_OK;
}

esp_err_t ADS1115::init(i2c_master_bus_handle_t busHandle) {
  // Check if address exist on i2c line
  ESP_RETURN_ON_ERROR(i2c_master_probe(busHandle, i2cAddress, 1000), TAG,
                      "ADS1115 address (0x%.2x) not found", i2cAddress);

  // Initialize i2c device
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = i2cAddress,
      .scl_speed_hz = 500000,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(busHandle, &dev_cfg, &_dev_handle), TAG,
                      "Failed add i2c device of ADS1115");

  writeRegister(ADS1115_CONFIG_REG, ADS1115_REG_RESET_VAL);
  setVoltageRange_mV(ADS1115_RANGE_2048);
  writeRegister(ADS1115_LO_THRESH_REG, 0x8000);
  writeRegister(ADS1115_HI_THRESH_REG, 0x7FFF);
  deviceMeasureMode = ADS1115_SINGLE;
  autoRangeMode = false;
  return ESP_OK;
}

bool ADS1115::setAlertPinMode(ADS1115_COMP_QUE mode) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0x8003);
  currentConfReg |= mode;
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  return true;
}

bool ADS1115::setAlertLatch(ADS1115_LATCH latch) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0x8004);
  currentConfReg |= latch;
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  return true;
}

bool ADS1115::setAlertPol(ADS1115_ALERT_POL polarity) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0x8008);
  currentConfReg |= polarity;
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  return true;
}

bool ADS1115::setAlertModeAndLimit_V(ADS1115_COMP_MODE mode, float hiThres, float loThres) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0x8010);
  currentConfReg |= mode;
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  int16_t alertLimit = calcLimit(hiThres);
  if (writeRegister(ADS1115_HI_THRESH_REG, alertLimit) != ESP_OK) {
    return false;
  }
  alertLimit = calcLimit(loThres);
  if (writeRegister(ADS1115_LO_THRESH_REG, alertLimit) != ESP_OK) {
    return false;
  }
  return true;
}

bool ADS1115::setConvRate(ADS1115_CONV_RATE rate) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0x80E0);
  currentConfReg |= rate;
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  return true;
}

convRate ADS1115::getConvRate() {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    // NOTE: Just return random value
    return ADS1115_8_SPS;
  }
  return (convRate)(currentConfReg & 0xE0);
}

bool ADS1115::setMeasureMode(ADS1115_MEASURE_MODE mode) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0x8100);
  currentConfReg |= mode;
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  deviceMeasureMode = mode;
  return true;
}

bool ADS1115::setVoltageRange_mV(ADS1115_RANGE range) {
  uint16_t currentVoltageRange = voltageRange;
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  uint16_t currentRange = (currentConfReg >> 9) & 7;
  uint16_t currentAlertPinMode = currentConfReg & 3;

  if (!setMeasureMode(ADS1115_SINGLE)) {
    return false;
  }

  switch (range) {
  case ADS1115_RANGE_6144:
    voltageRange = 6144;
    break;
  case ADS1115_RANGE_4096:
    voltageRange = 4096;
    break;
  case ADS1115_RANGE_2048:
    voltageRange = 2048;
    break;
  case ADS1115_RANGE_1024:
    voltageRange = 1024;
    break;
  case ADS1115_RANGE_0512:
    voltageRange = 512;
    break;
  case ADS1115_RANGE_0256:
    voltageRange = 256;
    break;
  }

  if ((currentRange != range) && (currentAlertPinMode != ADS1115_DISABLE_ALERT)) {
    int16_t alertLimit = readRegisterDefault(ADS1115_HI_THRESH_REG);
    alertLimit = alertLimit * (currentVoltageRange * 1.0 / voltageRange);
    writeRegister(ADS1115_HI_THRESH_REG, alertLimit);

    alertLimit = readRegisterDefault(ADS1115_LO_THRESH_REG);
    alertLimit = alertLimit * (currentVoltageRange * 1.0 / voltageRange);
    writeRegister(ADS1115_LO_THRESH_REG, alertLimit);
  }

  currentConfReg &= ~(0x8E00);
  currentConfReg |= range;
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  convRate rate = getConvRate();
  delayAccToRate(rate);
  return true;
}

bool ADS1115::setAutoRange() {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }

  if (!setVoltageRange_mV(ADS1115_RANGE_6144)) {
    return false;
  }

  if (deviceMeasureMode == ADS1115_SINGLE) {
    if (setMeasureMode(ADS1115_CONTINUOUS) != ESP_OK) {
      return false;
    }
    convRate rate = getConvRate();
    delayAccToRate(rate);
  }

  int16_t rawResult = (readRegisterDefault(ADS1115_CONV_REG));
  rawResult = abs(rawResult);
  int16_t rawResultCopy = rawResult;
  if (rawResultCopy == -32768) {
    rawResultCopy++;
  }
  rawResultCopy = abs(rawResultCopy);

  range optRange = ADS1115_RANGE_6144;

  if (rawResultCopy < 1093) {
    optRange = ADS1115_RANGE_0256;
  } else if (rawResultCopy < 2185) {
    optRange = ADS1115_RANGE_0512;
  } else if (rawResultCopy < 4370) {
    optRange = ADS1115_RANGE_1024;
  } else if (rawResultCopy < 8738) {
    optRange = ADS1115_RANGE_2048;
  } else if (rawResultCopy < 17476) {
    optRange = ADS1115_RANGE_4096;
  }

  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }

  return setVoltageRange_mV(optRange);
}

void ADS1115::setPermanentAutoRangeMode(bool autoMode) {
  if (autoMode) {
    autoRangeMode = true;
  } else {
    autoRangeMode = false;
  }
}

void ADS1115::delayAccToRate(convRate cr) {
  switch (cr) {
  case ADS1115_8_SPS:
    vTaskDelay(pdMS_TO_TICKS(130));
    break;
  case ADS1115_16_SPS:
    vTaskDelay(pdMS_TO_TICKS(65));
    break;
  case ADS1115_32_SPS:
    vTaskDelay(pdMS_TO_TICKS(32));
    break;
  case ADS1115_64_SPS:
    vTaskDelay(pdMS_TO_TICKS(16));
    break;
  case ADS1115_128_SPS:
    vTaskDelay(pdMS_TO_TICKS(8));
    break;
  case ADS1115_250_SPS:
    vTaskDelay(pdMS_TO_TICKS(4));
    break;
  case ADS1115_475_SPS:
    vTaskDelay(pdMS_TO_TICKS(3));
    break;
  case ADS1115_860_SPS:
    vTaskDelay(pdMS_TO_TICKS(2));
    break;
  }
}

bool ADS1115::setCompareChannels(ADS1115_MUX mux) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0xF000);
  currentConfReg |= (mux);
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }

  if (!(currentConfReg & 0x0100)) { // => if not single shot mode
    convRate rate = getConvRate();
    for (int i = 0; i < 2; i++) { // waiting time for two measurements
      delayAccToRate(rate);
    }
  }
  return true;
}

bool ADS1115::setCompareChannels_nonblock(ADS1115_MUX mux) {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg &= ~(0xF000);
  currentConfReg |= (mux);
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  return true;
}

bool ADS1115::setSingleChannel(size_t channel) {
  if (channel >= 4) {
    ESP_LOGE(TAG, "Wrong channel input maximum 4");
    return false;
  }
  return setCompareChannels((ADS1115_MUX)(ADS1115_COMP_0_GND + ADS1115_COMP_INC * channel));
}

bool ADS1115::isBusy() {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  if (deviceMeasureMode == ADS1115_SINGLE) {
    return (!((currentConfReg >> 15) & 1));
  } else
    return 0;
}

bool ADS1115::startSingleMeasurement() {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  currentConfReg |= (1 << 15);
  if (writeRegister(ADS1115_CONFIG_REG, currentConfReg) != ESP_OK) {
    return false;
  }
  return true;
}

float ADS1115::getResult_V() {
  float result = getResult_mV();
  result /= 1000;
  return result;
}

float ADS1115::getResult_mV() {
  int16_t rawResult = getRawResult();
  float result = (rawResult * 1.0 / ADS1115_REG_FACTOR) * voltageRange;
  return result;
}

int16_t ADS1115::getRawResult() {
  int16_t rawResult = readRegisterDefault(ADS1115_CONV_REG);
  if (autoRangeMode) {
    int16_t rawResultCopy = rawResult;
    if (rawResultCopy == -32768) {
      rawResultCopy++;
    }
    rawResultCopy = abs(rawResultCopy);
    if ((rawResultCopy > 26214) && (voltageRange != 6144)) { // 80%
      setAutoRange();
      rawResult = readRegisterDefault(ADS1115_CONFIG_REG);
    } else if ((rawResultCopy < 9800) && (voltageRange != 256)) { //30%
      setAutoRange();
      rawResult = readRegisterDefault(ADS1115_CONFIG_REG);
    }
  }
  return rawResult;
}

int16_t ADS1115::getResultWithRange(int16_t min, int16_t max) {
  int16_t rawResult = getRawResult();
  int16_t result = map(rawResult, -32768, 32767, min, max);
  return result;
}

int16_t ADS1115::getResultWithRange(int16_t min, int16_t max, int16_t maxMillivolt) {
  int16_t result = getResultWithRange(min, max);
  result = static_cast<int16_t>((1.0 * result * voltageRange / maxMillivolt) + 0.5);
  return result;
}

uint16_t ADS1115::getVoltageRange_mV() { return voltageRange; }

void ADS1115::setAlertPinToConversionReady() {
  writeRegister(ADS1115_LO_THRESH_REG, (0 << 15));
  writeRegister(ADS1115_HI_THRESH_REG, (1 << 15));
}

bool ADS1115::clearAlert() {
  uint16_t currentConfReg;
  if (readRegister(ADS1115_CONFIG_REG, &currentConfReg) != ESP_OK) {
    return false;
  }
  return true;
}

/************************************************
    private functions
*************************************************/

int16_t ADS1115::calcLimit(float rawLimit) {
  int16_t limit = static_cast<int16_t>((rawLimit * ADS1115_REG_FACTOR / voltageRange) * 1000);
  return limit;
}

esp_err_t ADS1115::writeRegister(uint8_t reg, uint16_t val) {
  uint8_t lVal = val & 255;
  uint8_t hVal = val >> 8;
  uint8_t txBuf[3] = {reg, hVal, lVal};
  ESP_RETURN_ON_ERROR(i2c_master_transmit(_dev_handle, txBuf, 3, 500), TAG,
                      "i2c_master_transmit, write register failed (r: 0x%.2x, v:0x%.2x)", reg, val);
  return ESP_OK;
}

esp_err_t ADS1115::readRegister(uint8_t reg, uint16_t *output) {
  uint8_t buffer[2] = {0};
  ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(_dev_handle, &reg, 1, buffer, 2, 500), TAG,
                      "i2c_master_transmit_receive, write and read register failed (r: 0x%.2x)",
                      reg);
  *output = ((buffer[0] << 8) | buffer[1]);
  return ESP_OK;
}

uint16_t ADS1115::readRegisterDefault(uint8_t reg) {
  uint16_t readResult;
  if (readRegister(reg, &readResult) != ESP_OK) {
    return -1;
  }
  return readResult;
}

int ADS1115::map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
