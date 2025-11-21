/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef AG_SENSOR_H
#define AG_SENSOR_H

#include "AirgradientIICSerial.h"
#include "AirgradientUART.h"
#include "BQ25672.h"
#include "PMS.h"
#include "Sunlight.h"
#include "airgradientClient.h"
#include "sht4x.h"
#include "sgp4x.h"
#include "AlphaSenseSensor.h"
#include "Configuration.h"

class Sensor {
public:
  Sensor(i2c_master_bus_handle_t busHandle);
  ~Sensor(){}
  bool init(Configuration::Model model, int co2ABCDays);
  bool startMeasures(int iterations, int intervalMs);
  void printMeasures();
  AirgradientClient::MaxSensorPayload getLastAverageMeasure();
  bool co2AttemptManualCalibration();
  float batteryVoltage();

private:
  const char *const TAG = "Sensor";

  void _measure(AirgradientClient::MaxSensorPayload &data);
  void _applyIteration(AirgradientClient::MaxSensorPayload &data);
  void _calculateMeasuresAverage();
  void _warmUpSensor();
  bool _applySunlightMeasurementSample();
  void _printPMData(int ch, PMS::Data &data);

  int _rco2IterationOkCount = 0;
  int _atmpIterationOkCount = 0;
  int _rhumIterationOkCount = 0;
  int _pm01IterationOkCount = 0;
  int _pm25IterationOkCount = 0;
  int _pm10IterationOkCount = 0;
  int _pm25SpIterationOkCount = 0;
  int _pm003CountIterationOkCount = 0;
  int _pm005CountIterationOkCount = 0;
  int _pm01CountIterationOkCount = 0;
  int _pm02CountIterationOkCount = 0;
  int _pm50CountIterationOkCount = 0;
  int _pm10CountIterationOkCount = 0;
  int _tvocIterationOkCount = 0;
  int _noxIterationOkCount = 0;
  int _vbatIterationOkCount = 0;
  int _vpanelIterationOkCount = 0;
  int _o3WEIterationOkCount = 0;
  int _o3AEIterationOkCount = 0;
  int _no2WEIterationOkCount = 0;
  int _no2AEIterationOkCount = 0;
  int _afeTempIterationOkCount = 0;
  AirgradientClient::MaxSensorPayload _averageMeasure;
  i2c_master_bus_handle_t _busHandle;

  bool _co2Available = true;
  AirgradientSerial *agsCO2_ = nullptr;
  Sunlight *co2_ = nullptr;

  bool _pms1Available = true;
  AirgradientSerial *agsPM1_ = nullptr;
  PMS *pms1_ = nullptr;

  bool _pms2Available = true;
  AirgradientSerial *agsPM2_ = nullptr;
  PMS *pms2_ = nullptr;

  bool _chargerAvailable = true;
  BQ25672 *charger_ = nullptr;

  bool _tempHumAvailable = true;
  sht4x_handle_t sht_dev_hdl = NULL;

  bool _tvocNoxAvailable = true;
  sgp4x_handle_t sgp_dev_hdl = NULL;

  bool _alphaSenseGasAvailable = true;
  bool _alphaSenseTempAvailable = true;
  AlphaSenseSensor *alphaSense_ = nullptr;
};

#endif // !AG_SENSOR_H
