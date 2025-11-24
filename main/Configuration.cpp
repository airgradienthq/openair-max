/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "Configuration.h"
#include "MaxConfig.h"
#include "airgradientCellularClient.h"
#include "esp_log.h"
#include "ArduinoJson.h"
#include "nvs.h"
#include <cstdint>
#include <cstring>

#define REMOTE_CONFIG_NVS_STORAGE_NAME "remote-config" //!NOTE: Don't change this value!
#define NVS_KEY_CO2_CALIBRATION_REQUESTED "co2CalibReq"
#define NVS_KEY_LED_TEST_REQUESTED "ledTestReq"
#define NVS_KEY_ABC_DAYS "abcDays"
#define NVS_KEY_MODEL "model"
#define NVS_KEY_SCHEDULE_PM02 "pm02"
#define NVS_KEY_SCHEDULE_CONTINUOUS "cont"
#define NVS_KEY_FIRMWARE_URL "furl"
#define NVS_KEY_FIRMWARE_TARGET "ftarget"
#define NVS_KEY_NETWORK_OPTION "netOpt"
#define NVS_KEY_WIFI_CONFIGURED "wifiset"
#define NVS_KEY_SYSTEM_SETTINGS "sysset"
#define NVS_KEY_APN "apn"
#define NVS_KEY_MQTT_HOST "mqtt"
#define NVS_KEY_HTTP_DOMAIN "dom"
#define NVS_KEY_EXT_PM_MEASURES "extPmMeasures"

bool Configuration::load() {
  // At first, set every configuration to default
  //   to accomodate some config that are failed to read from NVS
  _setConfigToDefault();

  // Load configuration from NVS
  if (!_loadConfig()) {
    ESP_LOGW(TAG, "Cannot open NVS, set configuration to default");
    _setConfigToDefault();
  }

  // Printout configurations
  _printConfig();

  return true;
}

void Configuration::_printConfig() {
  ESP_LOGI(TAG, "**** CONFIGURATION ****");
  ESP_LOGI(TAG, "co2CalibrationRequested: %d", _config.co2CalibrationRequested);
  ESP_LOGI(TAG, "ledTestRequested: %d", _config.ledTestRequested);
  ESP_LOGI(TAG, "abcDays: %d", _config.abcDays);
  ESP_LOGI(TAG, "model: %s", _config.model.c_str());
  ESP_LOGI(TAG, "firmware.target: %s", _config.firmware.target.c_str());
  ESP_LOGI(TAG, "firmware.url: %s", _config.firmware.url.c_str());
  ESP_LOGI(TAG, "schedule.pm02: %d", _config.schedule.pm02);
  ESP_LOGI(TAG, "schedule.continuous: %d", _config.schedule.continuous);
  ESP_LOGI(TAG, "networkOption: %s",
           _config.networkOption == NetworkOption::Cellular ? "Cellular" : "WiFi");
  ESP_LOGI(TAG, "isWifiConfigured: %d", _config.isWifiConfigured);
  ESP_LOGI(TAG, "runSystemSettings: %d", _config.runSystemSettings);
  ESP_LOGI(TAG, "apn: %s", _config.apn.c_str());
  ESP_LOGI(TAG, "mqttBrokerUrl: %s", _config.mqttBrokerUrl.c_str());
  ESP_LOGI(TAG, "httpDomain: %s", _config.httpDomain.c_str());
  ESP_LOGI(TAG, "extendedPmMeasures: %d", _config.extendedPmMeasures);
  ESP_LOGI(TAG, "**** ****");
}

void Configuration::reset() {
  ESP_LOGI(TAG, "Resetting configuration..");
  _setConfigToDefault();
  _saveConfig();
}

bool Configuration::parseRemoteConfig(const std::string &config) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, config.c_str(), config.length());
  if (error) {
    ESP_LOGE(TAG, "Failed to parse remote configuration: %s", error.c_str());
    return false;
  }

  // Get the root object.
  JsonObject root = doc.as<JsonObject>();

  bool bool_val;
  int int_val;
  std::string str_val; // Using std::string for direct assignment

  // co2CalibrationRequested
  if (root["co2CalibrationRequested"].is<bool>()) {
    bool_val = root["co2CalibrationRequested"].as<bool>();
    if (_config.co2CalibrationRequested != bool_val) {
      ESP_LOGI(TAG, "co2CalibrationRequested value changed to %d", bool_val);
      _config.co2CalibrationRequested = bool_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "co2CalibrationRequested field not found or not a boolean");
  }

  // abcDays
  if (root["abcDays"].is<int>()) {
    int_val = root["abcDays"].as<int>();
    if (_config.abcDays != int_val) {
      ESP_LOGI(TAG, "abcDays value changed from %d to %d", _config.abcDays, int_val);
      _config.abcDays = int_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "abcDays field not found or not an integer");
  }

  // ledTestRequested
  if (root["ledBarTestRequested"].is<bool>()) {
    bool_val = root["ledBarTestRequested"].as<bool>();
    if (bool_val) {
      ESP_LOGI(TAG, "Led test is requested");
      _config.ledTestRequested = bool_val;
    }
  } else {
    ESP_LOGW(TAG, "ledTestRequested field not found or not a boolean");
  }

  // model
  if (root["model"].is<const char *>()) {
    str_val = root["model"].as<std::string>(); // Directly cast to std::string
    if (_config.model != str_val) {
      ESP_LOGI(TAG, "model value changed from %s to %s", _config.model.c_str(), str_val.c_str());
      _config.model = str_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "model field not found or not a string");
  }

  // schedule object
  if (root["schedule"].is<JsonObject>()) {
    JsonObject schedule = root["schedule"].as<JsonObject>();

    // schedule.continuous
    if (schedule["continuous"].is<bool>()) {
      bool_val = schedule["continuous"].as<bool>();
      if (_config.schedule.continuous != bool_val) {
        ESP_LOGI(TAG, "schedule.continuous value changed to %d", bool_val);
        _config.schedule.continuous = bool_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "schedule.continuous field not found or not a boolean");
    }

    // schedule.pm02
    if (schedule["pm02"].is<int>()) {
      int_val = schedule["pm02"].as<int>();
      if (_config.schedule.pm02 != int_val) {
        ESP_LOGI(TAG, "schedule.pm02 value changed from %d to %d", _config.schedule.pm02, int_val);
        _config.schedule.pm02 = int_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "schedule.pm02 field not found or not an integer");
    }
  } else {
    ESP_LOGW(TAG, "schedule field not found or not an object");
  }

  // firmware object
  if (root["firmware"].is<JsonObject>()) {
    JsonObject firmware = root["firmware"].as<JsonObject>();

    // firmware.target
    if (firmware["target"].is<const char *>()) {
      str_val = firmware["target"].as<std::string>();
      if (_config.firmware.target != str_val) {
        ESP_LOGI(TAG, "firmware.target value changed from %s to %s",
                 _config.firmware.target.c_str(), str_val.c_str());
        _config.firmware.target = str_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "firmware.target field not found or not a string");
    }

    // firmware.url
    if (firmware["url"].is<const char *>()) {
      str_val = firmware["url"].as<std::string>();
      if (_config.firmware.url != str_val) {
        ESP_LOGI(TAG, "firmware.url value changed from %s to %s", _config.firmware.url.c_str(),
                 str_val.c_str());
        _config.firmware.url = str_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "firmware.url field not found or not a string");
    }
  } else {
    ESP_LOGW(TAG, "firmware field not found or not an object");
  }

  // mqttBrokerUrl
  if (root["mqttBrokerUrl"].is<const char *>()) {
    str_val = root["mqttBrokerUrl"].as<std::string>();
    if (_config.mqttBrokerUrl != str_val) {
      ESP_LOGI(TAG, "mqttBrokerUrl value changed from %s to %s", _config.mqttBrokerUrl.c_str(),
               str_val.c_str());
      _config.mqttBrokerUrl = str_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "mqttBrokerUrl field not found or not a string");
    if (_config.mqttBrokerUrl.empty() == false) {
      // empty value on persistent storage means its disabled
      // field not found from server means its disabled
      ESP_LOGI(TAG, "Previously mqtt is enabled. Disabling it because now its not found");
      _config.mqttBrokerUrl = "";
      _configChanged = true;
    }
  }

  // extendedPmMeasures
  if (root["extendedPmMeasures"].is<bool>()) {
    bool_val = root["extendedPmMeasures"].as<bool>();
    if (_config.extendedPmMeasures != bool_val) {
      ESP_LOGI(TAG, "extendedPmMeasures value changed to %d", bool_val);
      _config.extendedPmMeasures = bool_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "extendedPmMeasures field not found or not a boolean");
  }

  ESP_LOGI(TAG, "Finish parsing remote configuration");

  if (_configChanged) {
    _saveConfig();
  }

  return true;
}

bool Configuration::_loadConfig() {
  ESP_LOGI(TAG, "Reading configurations from NVS");
  nvs_handle_t handle;
  esp_err_t err = nvs_open(REMOTE_CONFIG_NVS_STORAGE_NAME, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    return false;
  }

  // CO2 CALIBRATION
  uint8_t co2CalibrationRequested;
  err = nvs_get_u8(handle, NVS_KEY_CO2_CALIBRATION_REQUESTED, &co2CalibrationRequested);
  if (err == ESP_OK) {
    _config.co2CalibrationRequested = co2CalibrationRequested;
  } else {
    ESP_LOGW(TAG, "Failed to get co2CalibrationRequested");
  }

  // ABC DAYS
  int16_t abcDays;
  err = nvs_get_i16(handle, NVS_KEY_ABC_DAYS, &abcDays);
  if (err == ESP_OK) {
    _config.abcDays = abcDays;
  } else {
    ESP_LOGW(TAG, "Failed to get abcDays");
  }

  // MODEL
  size_t requiredSize = 0;
  err = nvs_get_str(handle, NVS_KEY_MODEL, NULL, &requiredSize);
  if (err == ESP_OK) {
    char *data = new char[requiredSize + 1];
    memset(data, 0, requiredSize + 1);
    err = nvs_get_str(handle, NVS_KEY_MODEL, data, &requiredSize);
    if (err == ESP_OK) {
      _config.model = data;
    } else {
      ESP_LOGW(TAG, "Failed to get model");
    }
    delete[] data;
  } else {
    ESP_LOGW(TAG, "Failed to get model");
  }

  // FIRMWARE.TARGET
  err = nvs_get_str(handle, NVS_KEY_FIRMWARE_TARGET, NULL, &requiredSize);
  if (err == ESP_OK) {
    char *data = new char[requiredSize + 1];
    memset(data, 0, requiredSize + 1);
    err = nvs_get_str(handle, NVS_KEY_FIRMWARE_TARGET, data, &requiredSize);
    if (err == ESP_OK) {
      _config.firmware.target = data;
    } else {
      ESP_LOGW(TAG, "Failed to get firmware.target");
    }
    delete[] data;
  } else {
    ESP_LOGW(TAG, "Failed to get firmware.target");
  }

  // FIRMWARE.URL
  err = nvs_get_str(handle, NVS_KEY_FIRMWARE_URL, NULL, &requiredSize);
  if (err == ESP_OK) {
    char *data = new char[requiredSize + 1];
    memset(data, 0, requiredSize + 1);
    err = nvs_get_str(handle, NVS_KEY_FIRMWARE_URL, data, &requiredSize);
    if (err == ESP_OK) {
      _config.firmware.url = data;
    } else {
      ESP_LOGW(TAG, "Failed to get firmware.url");
    }
    delete[] data;
  } else {
    ESP_LOGW(TAG, "Failed to get firmware.url");
  }

  // SCHEDULE.CONTINUOUS
  uint8_t continuous;
  err = nvs_get_u8(handle, NVS_KEY_SCHEDULE_CONTINUOUS, &continuous);
  if (err == ESP_OK) {
    _config.schedule.continuous = continuous;
  } else {
    ESP_LOGW(TAG, "Failed to get schedule.continuous");
  }

  // SCHEDULE.PM02
  uint32_t pm02;
  err = nvs_get_u32(handle, NVS_KEY_SCHEDULE_PM02, &pm02);
  if (err == ESP_OK) {
    _config.schedule.pm02 = pm02;
  } else {
    ESP_LOGW(TAG, "Failed to get schedule.pm02");
  }

  // Is wifi configured
  uint8_t wifiConfigured;
  err = nvs_get_u8(handle, NVS_KEY_WIFI_CONFIGURED, &wifiConfigured);
  if (err == ESP_OK) {
    _config.isWifiConfigured = wifiConfigured;
  } else {
    ESP_LOGW(TAG, "Failed to get isWifiConfigured");
  }

  // NETWORK OPTION
  uint8_t netopt;
  err = nvs_get_u8(handle, NVS_KEY_NETWORK_OPTION, &netopt);
  if (err == ESP_OK) {
    _config.networkOption = static_cast<NetworkOption>(netopt);
  } else {
    ESP_LOGW(TAG, "Failed to get networkOption");
  }

  // Run system setting
  uint8_t runSystemSettings;
  err = nvs_get_u8(handle, NVS_KEY_SYSTEM_SETTINGS, &runSystemSettings);
  if (err == ESP_OK) {
    _config.runSystemSettings = runSystemSettings;
  } else {
    ESP_LOGW(TAG, "Failed to get runSystemSettings");
  }

  // APN
  requiredSize = 0;
  err = nvs_get_str(handle, NVS_KEY_APN, NULL, &requiredSize);
  if (err == ESP_OK) {
    char *data = new char[requiredSize + 1];
    memset(data, 0, requiredSize + 1);
    err = nvs_get_str(handle, NVS_KEY_APN, data, &requiredSize);
    if (err == ESP_OK) {
      _config.apn = data;
    } else {
      ESP_LOGW(TAG, "Failed to get apn");
    }
    delete[] data;
  } else {
    ESP_LOGW(TAG, "Failed to get apn");
  }

  // MQTT
  requiredSize = 0;
  err = nvs_get_str(handle, NVS_KEY_MQTT_HOST, NULL, &requiredSize);
  if (err == ESP_OK) {
    char *data = new char[requiredSize + 1];
    memset(data, 0, requiredSize + 1);
    err = nvs_get_str(handle, NVS_KEY_MQTT_HOST, data, &requiredSize);
    if (err == ESP_OK) {
      _config.mqttBrokerUrl = data;
    } else {
      ESP_LOGW(TAG, "Failed to get mqttBrokerUrl");
    }
    delete[] data;
  } else {
    ESP_LOGW(TAG, "Failed to get mqttBrokerUrl");
  }

  // HTTP Domain
  requiredSize = 0;
  err = nvs_get_str(handle, NVS_KEY_HTTP_DOMAIN, NULL, &requiredSize);
  if (err == ESP_OK) {
    char *data = new char[requiredSize + 1];
    memset(data, 0, requiredSize + 1);
    err = nvs_get_str(handle, NVS_KEY_HTTP_DOMAIN, data, &requiredSize);
    if (err == ESP_OK) {
      _config.httpDomain = data;
    } else {
      ESP_LOGW(TAG, "Failed to get httpDomain");
    }
    delete[] data;
  } else {
    ESP_LOGW(TAG, "Failed to get httpDomain");
  }

  uint8_t extendedPmMeasures;
  err = nvs_get_u8(handle, NVS_KEY_EXT_PM_MEASURES, &extendedPmMeasures);
  if (err == ESP_OK) {
    _config.extendedPmMeasures = extendedPmMeasures;
  } else {
    ESP_LOGW(TAG, "Failed to get extendedPmMeasures");
  }

  // Close NVS
  nvs_close(handle);

  return true;
}

bool Configuration::_saveConfig() {
  ESP_LOGI(TAG, "Saving configurations to NVS");
  nvs_handle_t handle;
  esp_err_t err = nvs_open(REMOTE_CONFIG_NVS_STORAGE_NAME, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    return false;
  }

  // CO2 CALIBRATION
  err = nvs_set_u8(handle, NVS_KEY_CO2_CALIBRATION_REQUESTED, _config.co2CalibrationRequested);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save co2CalibrationRequested");
  }

  // ABC DAYS
  err = nvs_set_i16(handle, NVS_KEY_ABC_DAYS, _config.abcDays);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save abcDays");
  }

  // MODEL
  err = nvs_set_str(handle, NVS_KEY_MODEL, _config.model.c_str());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save model");
  }

  // FIRMWARE.TARGET
  err = nvs_set_str(handle, NVS_KEY_FIRMWARE_TARGET, _config.firmware.target.c_str());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save firmware.target");
  }

  // FIRMWARE.URL
  err = nvs_set_str(handle, NVS_KEY_FIRMWARE_URL, _config.firmware.url.c_str());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save firmware.url");
  }

  // SCHEDULE.CONTINUOUS
  err = nvs_set_u8(handle, NVS_KEY_SCHEDULE_CONTINUOUS, _config.schedule.continuous);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save schedule.continuous");
  }

  // SCHEDULE.PM02
  err = nvs_set_u32(handle, NVS_KEY_SCHEDULE_PM02, _config.schedule.pm02);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save schedule.pm02");
  }

  // NETWORK OPTION
  err = nvs_set_u8(handle, NVS_KEY_NETWORK_OPTION, static_cast<uint8_t>(_config.networkOption));
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save networkOption");
  }

  // WiFi configured
  err = nvs_set_u8(handle, NVS_KEY_WIFI_CONFIGURED, _config.isWifiConfigured);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save isWifiConfigured");
  }

  // Run System Settings
  err = nvs_set_u8(handle, NVS_KEY_SYSTEM_SETTINGS, _config.runSystemSettings);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save runSystemSettings");
  }

  // APN
  err = nvs_set_str(handle, NVS_KEY_APN, _config.apn.c_str());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save apn");
  }

  // MQTT
  err = nvs_set_str(handle, NVS_KEY_MQTT_HOST, _config.mqttBrokerUrl.c_str());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save mqttBrokerUrl");
  }

  // HTTP Domain
  err = nvs_set_str(handle, NVS_KEY_HTTP_DOMAIN, _config.httpDomain.c_str());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save httpDomain");
  }

  // Extended PM measures
  err = nvs_set_u8(handle, NVS_KEY_EXT_PM_MEASURES, _config.extendedPmMeasures);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save extendedPmMeasures");
  }

  // Commit changes
  ESP_LOGI(TAG, "Commit changes to NVS");
  err = nvs_commit(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to commit configuration to NVS");
    nvs_close(handle);
    return false;
  }

  nvs_close(handle);
  return true;
}

Configuration::Config Configuration::get() { return _config; }

bool Configuration::isConfigChanged() { return _configChanged; }

bool Configuration::isCO2CalibrationRequested() { return _config.co2CalibrationRequested; }

bool Configuration::isLedTestRequested() { return _config.ledTestRequested; }

int Configuration::getABCDays() { return _config.abcDays; }

Configuration::Firmware Configuration::getConfigFirmware() { return _config.firmware; }

Configuration::Schedule Configuration::getConfigSchedule() { return _config.schedule; }

Configuration::Model Configuration::getModel() {
  if (_config.model == "O-M-1PPSTON-CE") {
    return O_M_1PPSTON_CE;
  } else if (_config.model == "O-M-1PPST-CE") {
    return O_M_1PPST_CE;
  }

  // Default
  return O_M_1PPST_CE;
}

NetworkOption Configuration::getNetworkOption() { return _config.networkOption; }

bool Configuration::isWifiConfigured() { return _config.isWifiConfigured; }

bool Configuration::runSystemSettings() { return _config.runSystemSettings; }

std::string Configuration::getAPN() { return _config.apn; }

std::string Configuration::getMqttBrokerUrl() { return _config.mqttBrokerUrl; }

std::string Configuration::getHttpDomain() { return _config.httpDomain; }

bool Configuration::isExtendedPmMeasuresEnabled() { return _config.extendedPmMeasures; }

bool Configuration::set(Config config) {
  _config = config;
  _printConfig();
  return _saveConfig();
}

void Configuration::setNetworkOption(NetworkOption option) {
  _config.networkOption = option;
  _saveConfig();
}

void Configuration::setIsWifiConfigured(bool state) {
  _config.isWifiConfigured = state;
  _saveConfig();
}

void Configuration::setRunSystemSettings(bool state) {
  _config.runSystemSettings = state;
  _saveConfig();
}

void Configuration::setAPN(const std::string &apn) {
  _config.apn = apn;
  _saveConfig();
}

void Configuration::resetCO2CalibrationRequest() {
  _config.co2CalibrationRequested = false;
  _saveConfig();
}

void Configuration::_setConfigToDefault() {
  _config.co2CalibrationRequested = false;
  _config.ledTestRequested = false;
  _config.abcDays = DEFAULT_ABC_PERIOD_DAYS;
  _config.model = "O-M-1PPSTON-CE";
  _config.firmware.target = "";
  _config.firmware.url = "";
  _config.schedule.pm02 = MEASURE_CYCLE_INTERVAL_SECONDS;
  _config.schedule.continuous = false;
  _config.networkOption = NetworkOption::Cellular;
  _config.isWifiConfigured = false;
  _config.runSystemSettings = false;
  _config.apn = DEFAULT_AIRGRADIENT_APN;
  _config.mqttBrokerUrl = "";
  _config.httpDomain = AIRGRADIENT_HTTP_DOMAIN;
  _config.extendedPmMeasures = false;
}
