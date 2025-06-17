/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "AlphaSenseSensor.h"
#include "MaxConfig.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"

bool AlphaSenseSensor::initGas(i2c_master_bus_handle_t busHandle) {
  ads1_ = new ADS1115(I2C_ADDR_ADS1115_1);
  if (ads1_->init(busHandle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed initialize ADS1115 1 (gas sensor)");
    return false;
  }
  ads1_->setVoltageRange_mV(ADS1115_RANGE_4096);
  return true;
}

bool AlphaSenseSensor::initTemperature(i2c_master_bus_handle_t busHandle) {
  ads2_ = new ADS1115(I2C_ADDR_ADS1115_2);
  if (ads2_->init(busHandle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed initialize ADS1115 2 (temperature sensor)");
    return false;
  }
  ads2_->setVoltageRange_mV(ADS1115_RANGE_4096);
  return true;
}

float AlphaSenseSensor::getO3WorkingElectrode() { return readChannel(ads1_, ADS1115_COMP_0_GND); }

float AlphaSenseSensor::getO3AuxiliaryElectrode() { return readChannel(ads1_, ADS1115_COMP_1_GND); }

float AlphaSenseSensor::getNO2WorkingElectrode() { return readChannel(ads1_, ADS1115_COMP_2_GND); }

float AlphaSenseSensor::getNO2AuxiliaryElectrode() {
  return readChannel(ads2_, ADS1115_COMP_3_GND);
}

float AlphaSenseSensor::getTemperature() { return readChannel(ads2_, ADS1115_COMP_0_GND); }

float AlphaSenseSensor::readChannel(ADS1115 *adc, ADS1115_MUX channel) {
  if (adc == nullptr) {
    ESP_LOGE(TAG, "readChannel() provided ads pointer is not valid");
    return DEFAULT_INVALID_VOLT;
  }

  if (adc->setCompareChannels(channel) == false) {
    return DEFAULT_INVALID_VOLT;
  }
  if (adc->startSingleMeasurement() == false) {
    return DEFAULT_INVALID_VOLT;
  }

  // Wait until result value is ready before reading it
  while (adc->isBusy()) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  float voltage = 0.0;
  voltage = adc->getResult_mV();
  return voltage;
}
