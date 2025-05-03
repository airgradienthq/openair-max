#include "RemoteConfig.h"
#include "esp_log.h"
#include "json_parser.h"

bool RemoteConfig::load() {
  // TODO: Load from NVS, if failed use default

  ESP_LOGW(TAG, "Configuration not found on NVS, set to default");
  _setConfigToDefault();

  // Printout configurations
  ESP_LOGI(TAG, "**** REMOTE CONFIGURATION ****");
  ESP_LOGI(TAG, "co2CalbirationRequested: %d", _config.co2CalibrationRequested);
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
    ESP_LOGI(TAG, "Saving new configuration to NVS");
    _saveConfig();
  }

  return true;
}

bool RemoteConfig::_saveConfig() {
  //
  return true;
}

void RemoteConfig::_setConfigToDefault() {
  _config.co2CalibrationRequested = false;
  _config.ledTestRequested = false;
  _config.model = "";
  _config.firmware.target = "";
  _config.firmware.url = "";
  _config.schedule.pm02 = 180; // in seconds
  _config.schedule.continuous = false;
}
