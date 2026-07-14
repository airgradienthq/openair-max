/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "GasIndex.h"

#include "esp_attr.h"
#include "esp_log.h"

#include "MaxConfig.h"
#include "sensirion_gas_index_algorithm.h"

// Algorithm states persisted across deep sleep in RTC memory so the estimator
// keeps learning between wake cycles.
RTC_DATA_ATTR GasIndexAlgorithmParams rtcVocParams;
RTC_DATA_ATTR GasIndexAlgorithmParams rtcNoxParams;

void GasIndex::init(bool firstBoot) {
  if (!firstBoot) {
    // States already restored from RTC memory, nothing to do
    return;
  }

  GasIndexAlgorithm_init_with_sampling_interval(
      &rtcVocParams, GasIndexAlgorithm_ALGORITHM_TYPE_VOC,
      GAS_INDEX_SAMPLING_INTERVAL_SEC);
  GasIndexAlgorithm_init_with_sampling_interval(
      &rtcNoxParams, GasIndexAlgorithm_ALGORITHM_TYPE_NOX,
      GAS_INDEX_SAMPLING_INTERVAL_SEC);

  ESP_LOGI(TAG,
           "Initialized VOC & NOx algorithm at %.0fs interval (RTC state: %u "
           "bytes each)",
           GAS_INDEX_SAMPLING_INTERVAL_SEC,
           (unsigned)sizeof(GasIndexAlgorithmParams));
}

void GasIndex::process(int tvocRaw, int noxRaw, int *tvocIndex, int *noxIndex) {
  if (IS_TVOC_VALID(tvocRaw)) {
    int32_t index = 0;
    GasIndexAlgorithm_process(&rtcVocParams, tvocRaw, &index);
    *tvocIndex = index;
    ESP_LOGI(TAG, "TVOC index: %d", (int)index);
  } else {
    // Skip processing to avoid poisoning the estimator with invalid input
    *tvocIndex = DEFAULT_INVALID_TVOC;
    ESP_LOGW(TAG, "Invalid TVOC raw, skip index calculation");
  }

  if (IS_NOX_VALID(noxRaw)) {
    int32_t index = 0;
    GasIndexAlgorithm_process(&rtcNoxParams, noxRaw, &index);
    *noxIndex = index;
    ESP_LOGI(TAG, "NOx index: %d", (int)index);
  } else {
    // Skip processing to avoid poisoning the estimator with invalid input
    *noxIndex = DEFAULT_INVALID_NOX;
    ESP_LOGW(TAG, "Invalid NOx raw, skip index calculation");
  }
}
