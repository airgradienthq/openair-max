/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "Configuration.h"
#include "MaxConfig.h"
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
  ESP_LOGI(TAG, "**** ****");

  return true;
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
  if (root["ledTestRequested"].is<bool>()) {
    bool_val = root["ledTestRequested"].as<bool>();
    if (_config.ledTestRequested != bool_val) {
      ESP_LOGI(TAG, "ledTestRequested value changed to %d", bool_val);
      _config.ledTestRequested = bool_val;
      _configChanged = true;
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

  // LED TEST
  uint8_t ledTestRequested;
  err = nvs_get_u8(handle, NVS_KEY_LED_TEST_REQUESTED, &ledTestRequested);
  if (err == ESP_OK) {
    _config.ledTestRequested = ledTestRequested;
  } else {
    ESP_LOGW(TAG, "Failed to get ledTestRequested");
  }

  // ABC DAYS
  uint16_t abcDays;
  err = nvs_get_u16(handle, NVS_KEY_ABC_DAYS, &abcDays);
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

  // LED TEST
  err = nvs_set_u8(handle, NVS_KEY_LED_TEST_REQUESTED, _config.ledTestRequested);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to save ledTestRequested");
  }

  // ABC DAYS
  err = nvs_set_u16(handle, NVS_KEY_ABC_DAYS, _config.abcDays);
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

  // Commit changes
  err = nvs_commit(handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to commit configuration to NVS");
    nvs_close(handle);
    return false;
  }

  nvs_close(handle);
  return true;
}

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

void Configuration::switchNetworkOption() {
  if (_config.networkOption == NetworkOption::Cellular) {
    ESP_LOGI(TAG, "Switch network option to WiFi");
    _config.networkOption = NetworkOption::WiFi;
  } else {
    ESP_LOGI(TAG, "Switch network option to Cellular");
    _config.networkOption = NetworkOption::Cellular;
  }
  _saveConfig();
}

void Configuration::setIsWifiConfigured(bool state) {
  _config.isWifiConfigured = state;
  _saveConfig();
}

void Configuration::resetLedTestRequest() {
  _config.ledTestRequested = false;
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
}
