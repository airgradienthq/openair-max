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
  bool parse(const std::string &config);

private:
  const char *const TAG = "RemoteConfig";
  Config _config;
  bool _configChanged = false;

  bool _saveConfig();
  void _setConfigToDefault();
};

#endif // !REMOTE_CONFIG_H
