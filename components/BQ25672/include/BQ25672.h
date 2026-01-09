#ifndef BQ25672_H
#define BQ25672_H

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include <cstdint>

class BQ25672 {
public:
  BQ25672();
  ~BQ25672();

  enum class ChargingStatus {
    Unknown,
    NotCharging,
    TrickleCharge,
    PreCharge,
    FastCharge, // Fast Charge (CC Mode)
    TaperCharge, // Taper Charge (CV Mode)
    TopOffTimerActiveCharging,
    ChargeTerminationDone
  };

  // Initialize and enable ADC
  esp_err_t begin(i2c_master_bus_handle_t busHandle);

  // Update function to prevent WD_RST (Call this in loop)
  esp_err_t update();

  // Read VBAT, VSYS, VBUS in mV
  esp_err_t getVBAT(uint16_t *output);
  esp_err_t getVSYS(uint16_t *output);
  esp_err_t getVBUS(uint16_t *output);

  esp_err_t getBatteryPercentage(float *output);
  esp_err_t getTemperature(float *output);
  esp_err_t getBatteryCurrent(int16_t *output);
  esp_err_t getBusCurrent(int16_t *output);

  ChargingStatus getChargingStatus();

  // Control charging enable/disable
  esp_err_t setChargingEnabled(bool enable);

  // Read Raw ADC values
  esp_err_t getVBATRaw(uint16_t *output);
  esp_err_t getVSYSRaw(uint16_t *output);
  esp_err_t getTemperatureRaw(uint16_t *output);
  esp_err_t getVBUSRaw(uint16_t *output);
  esp_err_t getIBATRaw(uint16_t *output);
  esp_err_t getIBUSRaw(uint16_t *output);

  void printSystemStatus();
  void printControlAndConfiguration();
  void printFaultStatus0();
  void printFaultStatus1();

private:
  const char *const TAG = "BQ25672";
  i2c_master_dev_handle_t _dev_handle;

  // Variables for time tracking
  uint32_t lastUpdateTime = 0; // Last time of REG2E update

  esp_err_t writeRegister(uint8_t reg, uint8_t value);
  esp_err_t writeReadRegister(uint8_t reg, int size, uint16_t *output);
};

#endif // !BQ25672_H
