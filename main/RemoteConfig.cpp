/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "RemoteConfig.h"
#include "MaxConfig.h"
#include "esp_log.h"
#include "json_parser.h"
#include "nvs.h"
#include <cstring>

#define REMOTE_CONFIG_NVS_STORAGE_NAME "remote-config"
#define NVS_KEY_CO2_CALIBRATION_REQUESTED "co2CalibReq"
#define NVS_KEY_LED_TEST_REQUESTED "ledTestReq"
#define NVS_KEY_MODEL "model"
#define NVS_KEY_SCHEDULE_PM02 "pm02"
#define NVS_KEY_SCHEDULE_CONTINUOUS "cont"
#define NVS_KEY_FIRMWARE_URL "furl"
#define NVS_KEY_FIRMWARE_TARGET "ftarget"

bool RemoteConfig::load() {
  // At first, set every configuration to default
  //   to accomodate some config that are failed to read from NVS
  _setConfigToDefault();

  // Load configuration from NVS
  if (!_loadConfig()) {
    ESP_LOGW(TAG, "Cannot open NVS, set configuration to default");
    _setConfigToDefault();
  }

  // Printout configurations
  ESP_LOGI(TAG, "**** REMOTE CONFIGURATION ****");
  ESP_LOGI(TAG, "co2CalibrationRequested: %d", _config.co2CalibrationRequested);
  ESP_LOGI(TAG, "ledTestRequested: %d", _config.ledTestRequested);
  ESP_LOGI(TAG, "model: %s", _config.model.c_str());
  ESP_LOGI(TAG, "firmware.target: %s", _config.firmware.target.c_str());
  ESP_LOGI(TAG, "firmware.url: %s", _config.firmware.url.c_str());
  ESP_LOGI(TAG, "schedule.pm02: %d", _config.schedule.pm02);
  ESP_LOGI(TAG, "schedule.continuous: %d", _config.schedule.continuous);
  ESP_LOGI(TAG, "**** ****");

  return true;
}

bool RemoteConfig::parse(const std::string &config) {
  jparse_ctx_t jctx;
  int ret = json_parse_start(&jctx, config.c_str(), config.length());
  if (ret != OS_SUCCESS) {
    ESP_LOGE(TAG, "Failed parse remote configuration");
    return false;
  }

  char str_val[64];
  int int_val;
  bool bool_val;

  if (json_obj_get_bool(&jctx, "co2CalibrationRequested", &bool_val) == OS_SUCCESS) {
    if (_config.co2CalibrationRequested != bool_val) {
      ESP_LOGI(TAG, "co2CalibrationRequested value changed to %d", bool_val);
      _config.co2CalibrationRequested = bool_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "co2CalibrationRequested field not found");
  }

  if (json_obj_get_bool(&jctx, "ledTestRequested", &bool_val) == OS_SUCCESS) {
    if (_config.ledTestRequested != bool_val) {
      ESP_LOGI(TAG, "ledTestRequested value changed to %d", bool_val);
      _config.ledTestRequested = bool_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "ledTestRequested field not found");
  }

  if (json_obj_get_string(&jctx, "model", str_val, sizeof(str_val)) == OS_SUCCESS) {
    if (_config.model != str_val) {
      ESP_LOGI(TAG, "model value changed from %s to %s", _config.model.c_str(), str_val);
      _config.model = str_val;
      _configChanged = true;
    }
  } else {
    ESP_LOGW(TAG, "model field not found");
  }

  if (json_obj_get_object(&jctx, "schedule") == OS_SUCCESS) {
    if (json_obj_get_bool(&jctx, "continuous", &bool_val) == OS_SUCCESS) {
      if (_config.schedule.continuous != bool_val) {
        ESP_LOGI(TAG, "schedule.continuous value changed to %d", bool_val);
        _config.schedule.continuous = bool_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "schedule.continuous field not found");
    }

    if (json_obj_get_int(&jctx, "pm02", &int_val) == OS_SUCCESS) {
      if (_config.schedule.pm02 != int_val) {
        ESP_LOGI(TAG, "schedule.pm02 value changed from %d to %d", _config.schedule.pm02, int_val);
        _config.schedule.pm02 = int_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "schedule.pm02 field not found");
    }
    // Go back to root object
    json_obj_leave_object(&jctx);
  } else {
    ESP_LOGW(TAG, "schedule field not found");
  }

  if (json_obj_get_object(&jctx, "firmware") == OS_SUCCESS) {
    if (json_obj_get_string(&jctx, "target", str_val, sizeof(str_val)) == OS_SUCCESS) {
      if (_config.firmware.target != str_val) {
        ESP_LOGI(TAG, "firmware.target value changed from %s to %s",
                 _config.firmware.target.c_str(), str_val);
        _config.firmware.target = str_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "firmware.target field not found");
    }

    if (json_obj_get_string(&jctx, "url", str_val, sizeof(str_val)) == OS_SUCCESS) {
      if (_config.firmware.url != str_val) {
        ESP_LOGI(TAG, "firmware.url value changed from %s to %s", _config.firmware.url.c_str(),
                 str_val);
        _config.firmware.url = str_val;
        _configChanged = true;
      }
    } else {
      ESP_LOGW(TAG, "firmware.url field not found");
    }
    // Go back to root object
    json_obj_leave_object(&jctx);
  } else {
    ESP_LOGW(TAG, "firmware field not found");
  }

  ESP_LOGI(TAG, "Finish parsing remote configuration");
  json_parse_end(&jctx);

  if (_configChanged) {
    _saveConfig();
  }

  return true;
}

bool RemoteConfig::_loadConfig() {
  ESP_LOGI(TAG, "Reading remote configuration from NVS");
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

  // Close NVS
  nvs_close(handle);

  return true;
}

bool RemoteConfig::_saveConfig() {
  ESP_LOGI(TAG, "Saving remote configuration to NVS");
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

bool RemoteConfig::isConfigChanged() { return _configChanged; }

bool RemoteConfig::isCO2CalibrationRequested() { return _config.co2CalibrationRequested; }

bool RemoteConfig::isLedTestRequested() { return _config.ledTestRequested; }

RemoteConfig::Firmware RemoteConfig::getConfigFirmware() { return _config.firmware; }

RemoteConfig::Schedule RemoteConfig::getConfigSchedule() { return _config.schedule; }

void RemoteConfig::resetLedTestRequest() {
  _config.ledTestRequested = false;
  _saveConfig();
}

void RemoteConfig::resetCO2CalibrationRequest() {
  _config.co2CalibrationRequested = false;
  _saveConfig();
}

void RemoteConfig::_setConfigToDefault() {
  _config.co2CalibrationRequested = false;
  _config.ledTestRequested = false;
  _config.model = "";
  _config.firmware.target = "";
  _config.firmware.url = "";
  _config.schedule.pm02 = MEASURE_CYCLE_INTERVAL_SECONDS;
  _config.schedule.continuous = false;
}
