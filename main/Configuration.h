/*
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "MaxConfig.h"
#include <string>

class Configuration {
public:
  struct Schedule {
    bool continuous;
    int pm02;
  };

  struct Firmware {
    std::string target;
    std::string url;
  };

  enum Model { O_M_1PPST_CE = 0, O_M_1PPSTON_CE };

  struct Config {
    int abcDays;
    bool co2CalibrationRequested;
    bool ledTestRequested;
    std::string model;
    Schedule schedule;
    Firmware firmware;
    NetworkOption networkOption;
    bool isWifiConfigured;
    bool runSystemSettings;
    std::string apn;
  };

  Configuration() {}
  ~Configuration() {}
  bool load();
  void reset();
  bool parseRemoteConfig(const std::string &config);

  // Getter
  bool isConfigChanged();
  bool isCO2CalibrationRequested();
  bool isLedTestRequested();
  int getABCDays();
  Firmware getConfigFirmware();
  Schedule getConfigSchedule();
  Model getModel();
  NetworkOption getNetworkOption();
  bool isWifiConfigured();
  bool runSystemSettings();
  std::string getAPN();

  // Setter
  void setNetworkOption(NetworkOption option);
  void setIsWifiConfigured(bool state);
  void setRunSystemSettings(bool state);
  void setAPN(const std::string &apn);

  void resetLedTestRequest();
  void resetCO2CalibrationRequest();

private:
  const char *const TAG = "Configuration";
  Config _config;
  bool _configChanged = false;

  bool _loadConfig();
  bool _saveConfig();
  void _setConfigToDefault();
};

#endif // !CONFIGURATION_H
