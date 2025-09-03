#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include "BQ25672.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include <cstdint>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

#define PERIOD_UPDATE_REG2E_MS 10000

// I2C Address for BQ25672
#define BQ25672_I2C_ADDRESS 0x6B

// Register Addresses
#define REG00_MINIMAL_SYSTEM_VOLTAGE 0x00
#define REG01_CHARGE_VOLTAGE_LIMIT 0x01
#define REG03_CHARGE_CURRENT_LIMIT 0x03
#define REG05_INPUT_VOLTAGE_LIMIT 0x05
#define REG06_INPUT_CURRENT_LIMIT 0x06
#define REG1B_CHG_STAT 0x1C // Register Address for Charger Status 0
#define REG2E_ADC_CTRL 0x2E
#define REG3B_VBAT_ADC 0x3B
#define REG3D_VSYS_ADC 0x3D
#define REG3C_BAT_PCT 0x3F
#define REG3F_TEMP_ADC 0x41
#define REG33_IBAT_ADC 0x33 // Register Address for IBAT ADC
#define REG37_VBUS_ADC1 0x37 // Register Address for VBUS ADC1
#define REG39_VBUS_ADC2 0x39 // Register Address for VBUS ADC2
#define REG15_MPPT 0x15
#define REG_SYSTEM_STATUS 0x0A
#define REG_FAULT_STATUS 0x0B
#define REG_CHARGE_STATUS 0x0C
#define REG_INPUT_STATUS 0x0D

BQ25672::BQ25672() {}
BQ25672::~BQ25672() {}

esp_err_t BQ25672::begin(i2c_master_bus_handle_t busHandle) {
  // Check if address exist on i2c line
  ESP_RETURN_ON_ERROR(i2c_master_probe(busHandle, BQ25672_I2C_ADDRESS, 1000), TAG,
                      "BQ25672 address (0x%.2x) not found", BQ25672_I2C_ADDRESS);

  esp_log_level_set(TAG, ESP_LOG_VERBOSE);

  // Initialize i2c device
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = BQ25672_I2C_ADDRESS,
      .scl_speed_hz = 500000,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(busHandle, &dev_cfg, &_dev_handle), TAG,
                      "Failed add i2c device of BQ25672");

  // To hold read register output
  uint16_t value;

  // Enable MPPT
  ESP_RETURN_ON_ERROR(writeRegister(0x15, 0xAB), TAG, "Failed write to enable MPPT");
  ESP_RETURN_ON_ERROR(writeReadRegister(0x15, 1, &value), TAG, "Failed read MPPT status");
  ESP_LOGI(TAG, "MPPT Status (0x15): 0x%.2x", value);
  if (value != 0xAB) {
    ESP_LOGE(TAG, "MPPT status is not as expected (0x%.2x)", (uint8_t)value);
    // return ESP_FAIL;
  }

  // Enable ADC with 15-bit resolution and Continuous Mode
  // ADC_EN = 1, 15-bit, Continuous
  ESP_RETURN_ON_ERROR(writeRegister(REG2E_ADC_CTRL, 0x80), TAG,
                      "Failed write to enable ADC 15-bit resolution with Continuous Mode");
  // Check if ADC is successfully enabled
  ESP_RETURN_ON_ERROR(writeReadRegister(REG2E_ADC_CTRL, 1, &value), TAG,
                      "Failed read to enable ADC 15-bit resolution with Continuous Mode");
  if (value & 0x80) {
    return ESP_OK;
  }

  ESP_LOGE(TAG, "ADC control value is not as expected (0x%.2x)", (uint8_t)value);

  // return ESP_FAIL;
  return ESP_OK;
}

esp_err_t BQ25672::update() {
  // Attempt enable MPTT and read the status
  uint16_t value = 0;
  writeRegister(0x15, 0xAB);
  writeReadRegister(0x15, 1, &value);
  ESP_LOGD(TAG, "MPPT Status (0x15): 0x%.2x", value);

  // Update REG2E every PERIOD_UPDATE_REG2E_MS
  esp_err_t err = ESP_OK;
  if ((MILLIS() - lastUpdateTime) > PERIOD_UPDATE_REG2E_MS) {
    // Write REG2E_ADC_CTRL to reset WD_RST
    err = writeRegister(REG2E_ADC_CTRL, 0x80);
    if (err != ESP_OK) { // ADC_EN = 1, 15-bit, Continuous
      ESP_LOGE(TAG, "Failed to reset WD_RST by write to REG2E_ADC_CTRL");
    }
    // writeRegister(0x05, 0xDC);
    lastUpdateTime = MILLIS();
  }

  return err;
}

esp_err_t BQ25672::getVBAT(uint16_t *output) {
  uint16_t result;
  ESP_RETURN_ON_ERROR(getVBATRaw(&result), TAG, "Failed get VBAT");
  *output = result * 1.0;
  return ESP_OK;
}

esp_err_t BQ25672::getVSYS(uint16_t *output) {
  uint16_t result;
  ESP_RETURN_ON_ERROR(getVSYSRaw(&result), TAG, "Failed get VSYS");
  *output = result * 1.0;
  return ESP_OK;
}

esp_err_t BQ25672::getBatteryPercentage(float *output) {
  uint16_t vbatAdc;
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3B_VBAT_ADC, 2, &vbatAdc), TAG,
                      "Failed get battery percentage");
  float vbat = vbatAdc * 1.0; // Convert to mV

  // Callulate percentage
  float batteryPercentage = ((vbat - 9000) / (12600 - 9000)) * 100;
  if (batteryPercentage > 100) {
    batteryPercentage = 100;
  } else if (batteryPercentage < 0) {
    batteryPercentage = 0;
  }
  *output = batteryPercentage;

  return ESP_OK;
}

esp_err_t BQ25672::getTemperature(float *output) {
  uint16_t temp;
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3F_TEMP_ADC, 2, &temp), TAG, "Failed get temperature");
  *output = (float)temp;
  return ESP_OK;
}

esp_err_t BQ25672::getVBUS(uint16_t *output) {
  uint16_t result;
  ESP_RETURN_ON_ERROR(getVBUSRaw(&result), TAG, "Failed get VBUS");
  *output = result * 1.0;
  return ESP_OK;
}

esp_err_t BQ25672::getBatteryCurrent(int16_t *output) {
  uint16_t raw;
  ESP_RETURN_ON_ERROR(getVBUSRaw(&raw), TAG, "Failed get battery current");
  int16_t batteryCurrent = (int16_t)(raw * 1.0);

  // Convert 2's Complement to signed integer
  if (batteryCurrent & 0x8000) {
    batteryCurrent = batteryCurrent - 0x10000;
  }
  *output = batteryCurrent;

  return ESP_OK;
}

BQ25672::ChargingStatus BQ25672::getChargingStatus() {
  // Read Charger Status 0
  uint16_t result;
  if (writeReadRegister(REG1B_CHG_STAT, 1, &result) != ESP_OK) {
    ESP_LOGE(TAG, "Failed get charging status raw");
    return ChargingStatus::Unknown;
  }

  // Extract CHG_STAT[2:0] (Bits 2 to 0)
  uint8_t chargingStatus = (uint8_t)result;
  uint8_t status = (chargingStatus >> 5) & 0x07;

  // Check charging status
  ChargingStatus cs = ChargingStatus::Unknown;
  switch (status) {
  case 0b000:
    cs = ChargingStatus::NotCharging;
    ESP_LOGI(TAG, "Charging status: not charging");
    break;
  case 0b001:
    cs = ChargingStatus::TrickleCharge;
    ESP_LOGI(TAG, "Charging status: trickle charge");
    break;
  case 0b010:
    cs = ChargingStatus::PreCharge;
    ESP_LOGI(TAG, "Charging status: pre-charge");
    break;
  case 0b011:
    cs = ChargingStatus::FastCharge;
    ESP_LOGI(TAG, "Charging status: fast charge (CC Mode)");
    break;
  case 0b100:
    cs = ChargingStatus::TaperCharge;
    ESP_LOGI(TAG, "Charging status: taper charge (CV Mode)");
    break;
  case 0b110:
    cs = ChargingStatus::TopOffTimerActiveCharging;
    ESP_LOGI(TAG, "Charging status: Top Off Timer Active Charging");
    break;
  case 0b111:
    cs = ChargingStatus::ChargeTerminationDone;
    ESP_LOGI(TAG, "Charging status: charge termination done");
    break;
  default:
    cs = ChargingStatus::Unknown;
    ESP_LOGI(TAG, "Charging status: unknown");
    break;
  }

  return cs;
}

esp_err_t BQ25672::getVBATRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3B_VBAT_ADC, 2, output), TAG, "Failed get VBAT raw");
  return ESP_OK;
}

esp_err_t BQ25672::getVSYSRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3D_VSYS_ADC, 2, output), TAG, "Failed get VSYS raw");
  return ESP_OK;
}

esp_err_t BQ25672::getTemperatureRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG3F_TEMP_ADC, 2, output), TAG,
                      "Failed get raw temperature");
  return ESP_OK;
}

esp_err_t BQ25672::getVBUSRaw(uint16_t *output) {
  // First try reading from ADC1
  uint16_t adc1_value;
  ESP_RETURN_ON_ERROR(writeReadRegister(REG37_VBUS_ADC1, 2, &adc1_value), TAG, "Failed get VBUS ADC1");
  
  // If ADC1 value is >= 1, use it
  if (adc1_value >= 1000) {
    *output = adc1_value;
    return ESP_OK;
  }
  
  // Otherwise, fall back to ADC2
  ESP_RETURN_ON_ERROR(writeReadRegister(REG39_VBUS_ADC2, 2, output), TAG, "Failed get VBUS ADC2");
  return ESP_OK;
}

esp_err_t BQ25672::getIBATRaw(uint16_t *output) {
  ESP_RETURN_ON_ERROR(writeReadRegister(REG39_VBUS_ADC2, 2, output), TAG, "Failed get raw IBAT");
  return ESP_OK;
}

void BQ25672::printSystemStatus() {
  ESP_LOGI(TAG, "==== SYSTEM STATUS ====");
  uint16_t value;
  if (writeReadRegister(REG_SYSTEM_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "System status (0x%.2x): 0x%.2x", REG_SYSTEM_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG_FAULT_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Fault status (0x%.2x): 0x%.2x", REG_FAULT_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG_CHARGE_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge status (0x%.2x): 0x%.2x", REG_CHARGE_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG_INPUT_STATUS, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Input status (0x%.2x): 0x%.2x", REG_INPUT_STATUS, (uint8_t)value);
  }
  if (writeReadRegister(REG15_MPPT, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "MPPT status (0x%.2x): 0x%.2x", REG15_MPPT, (uint8_t)value);
  }
  ESP_LOGI(TAG, "*********");
}

void BQ25672::printControlAndConfiguration() {
  ESP_LOGI(TAG, "==== CONTROL and CONFIGURATION REGISTERS ====");
  uint16_t value;
  if (writeReadRegister(REG00_MINIMAL_SYSTEM_VOLTAGE, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Minimal System Voltage Raw (0x%.2X): 0x%04X | Converted: %d mV",
             REG00_MINIMAL_SYSTEM_VOLTAGE, value, ((value * 250) + 2500));
  }
  if (writeReadRegister(REG01_CHARGE_VOLTAGE_LIMIT, 2, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge Voltage Limit Raw (0x%.2X): 0x%04X | Converted: %d mV",
             REG01_CHARGE_VOLTAGE_LIMIT, value, (value * 10));
  }
  if (writeReadRegister(REG03_CHARGE_CURRENT_LIMIT, 2, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Charge Current Limit Raw (0x%.2X): 0x%04X | Converted: %d mA",
             REG03_CHARGE_CURRENT_LIMIT, value, (value * 10));
  }
  if (writeReadRegister(REG05_INPUT_VOLTAGE_LIMIT, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Input Voltage Limit Raw (0x%.2X): 0x%04X | Converted: %d mV",
             REG05_INPUT_VOLTAGE_LIMIT, value, (value * 100));
  }
  if (writeReadRegister(REG06_INPUT_CURRENT_LIMIT, 2, &value) == ESP_OK) {
    ESP_LOGI(TAG, "Input Current Limit Raw (0x%.2X): 0x%04X | Converted: %d mA",
             REG06_INPUT_CURRENT_LIMIT, value, (value * 10));
  }
  if (writeReadRegister(REG2E_ADC_CTRL, 1, &value) == ESP_OK) {
    ESP_LOGI(TAG, "ADC Control (0x%.2X): 0x%02X", REG2E_ADC_CTRL, value);
  }
  ESP_LOGI(TAG, "*********");
}

esp_err_t BQ25672::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t txBuf[2] = {reg, value};
  ESP_RETURN_ON_ERROR(i2c_master_transmit(_dev_handle, txBuf, 2, 500), TAG,
                      "i2c_master_transmit, write register failed (r: 0x%.2x, v:0x%.2x)", reg,
                      value);
  return ESP_OK;
}

esp_err_t BQ25672::writeReadRegister(uint8_t reg, int size, uint16_t *output) {
  uint8_t buffer[2] = {0};
  ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(_dev_handle, &reg, 1, buffer, size, 500), TAG,
                      "i2c_master_transmit_receive, write and read register failed (r: 0x%.2x)",
                      reg);

  // Compile result
  uint16_t value = 0;
  for (int i = 0; i < size; i++) {
    value = (value << 8) | buffer[i];
  }
  *output = value;

  return ESP_OK;
}
