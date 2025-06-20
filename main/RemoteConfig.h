/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef REMOTE_CONFIG_H
#define REMOTE_CONFIG_H

#include <string>

class RemoteConfig {
public:
  struct Schedule {
    bool continuous;
    int pm02;
  };

  struct Firmware {
    std::string target;
    std::string url;
  };

  struct Config {
    bool co2CalibrationRequested;
    bool ledTestRequested;
    std::string model;
    Schedule schedule;
    Firmware firmware;
  };

  RemoteConfig() {}
  ~RemoteConfig() {}
  bool load();
  bool reset();
  bool parse(const std::string &config);

  // Getter
  bool isConfigChanged();
  bool isCO2CalibrationRequested();
  bool isLedTestRequested();
  Firmware getConfigFirmware();
  Schedule getConfigSchedule();

  void resetLedTestRequest();
  void resetCO2CalibrationRequest();

private:
  const char *const TAG = "RemoteConfig";
  Config _config;
  bool _configChanged = false;

  bool _loadConfig();
  bool _saveConfig();
  void _setConfigToDefault();
};

#endif // !REMOTE_CONFIG_H
