/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef ALPHASENSE_SENSOR_H
#define ALPHASENSE_SENSOR_H

#include "MaxConfig.h"
#include "ADS1115.h"
#include "driver/i2c_master.h"

class AlphaSenseSensor {
public:
  AlphaSenseSensor() {};
  ~AlphaSenseSensor() {};

  bool initGas(i2c_master_bus_handle_t busHandle);
  bool initTemperature(i2c_master_bus_handle_t busHandle);

  // Return invalid volt value if error happen
  float getO3WorkingElectrode();
  float getO3AuxiliaryElectrode();
  float getNO2WorkingElectrode();
  float getNO2AuxiliaryElectrode();
  float getTemperature();

private:
  const char *const TAG = "AlphaSenseSensor";
  ADS1115 *ads1_ = nullptr; // Attach to gas sensor
  ADS1115 *ads2_ = nullptr; // Attach to temperature sensor

  float readChannel(ADS1115 *adc, ADS1115_MUX channel);
};

#endif // !ALPHASENSE_SENSOR_H
