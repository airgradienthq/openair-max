# Sensirion SGP4X Series of Sensors

[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Edited with VS Code](https://badgen.net/badge/icon/VS%20Code?icon=visualstudio&label=edited%20with)](https://visualstudio.microsoft.com)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red.svg)](https://shields.io/)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://shields.io/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_sgp4x.svg)](https://registry.platformio.org/libraries/k0i05/esp_sgp4x)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_sgp4x/badge.svg)](https://components.espressif.com/components/k0i05/esp_sgp4x)

This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Sensirion SGP4X series of sensors (SGP40 and SGP41).  Information on features and functionality are documented and can be found in the `sgp4x.h` header file and in the `documentation` folder.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_sgp4x>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `sgp4x.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

The SGP40 is not fully implemented, support for the SGP40 will be revisited once a sensor is available to test with.

```text
components
└── esp_sgp4x
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── sgp4x_version.h
    │   └── sgp4x.h
    └── sgp4x.c
```

## Basic Example

Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```c
#include <sgp4x.h>
#include <sensirion_gas_index_algorithm.h>

void i2c0_sgp4x_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    sgp4x_config_t dev_cfg          = I2C_SGP41_CONFIG_DEFAULT;
    sgp4x_handle_t dev_hdl;
    bool               dev_self_tested  = false;
    bool               dev_conditioned  = false;
    //
    // initialize gas index parameters
    GasIndexAlgorithmParams voc_params;
    GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithmParams nox_params;
    GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
    //
    // init device
    sgp4x_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "sgp4x handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SGP4X - START #########################");
        //
        // handle sensor
        if(dev_self_tested == false) {
            sgp4x_self_test_result_t self_test_result;
            esp_err_t result = sgp4x_execute_self_test(dev_hdl, &self_test_result);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "sgp4x device self-test failed (%s)", esp_err_to_name(result));
            } else {
                ESP_LOGI(APP_TAG, "VOC Pixel:   %d", self_test_result.pixels.voc_pixel_failed);
                ESP_LOGI(APP_TAG, "NOX Pixel:   %d", self_test_result.pixels.nox_pixel_failed);
            }
            dev_self_tested = true;
        }
        // 
        if(dev_conditioned == false) {
            for(int i = 0; i < 10; i++) {
                uint16_t sraw_voc; 
                esp_err_t result = sgp4x_execute_conditioning(dev_hdl, &sraw_voc);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
                } else {
                    ESP_LOGI(APP_TAG, "SRAW VOC: %u", sraw_voc);
                }
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second * 10 iterations = 10-seconds
            }
            dev_conditioned = true;
        } else {
            uint16_t sraw_voc; uint16_t sraw_nox;
            int32_t voc_index; int32_t nox_index;
            esp_err_t result = sgp4x_measure_signals(dev_hdl, &sraw_voc, &sraw_nox);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
            } else {
                GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index);
                GasIndexAlgorithm_process(&nox_params, sraw_nox, &nox_index);

                ESP_LOGI(APP_TAG, "SRAW VOC: %u | VOC Index: %li", sraw_voc, voc_index);
                ESP_LOGI(APP_TAG, "SRAW NOX: %u | NOX Index: %li", sraw_nox, nox_index);
            }
        }
        
        //
        ESP_LOGI(APP_TAG, "######################## SGP4X - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    sgp4x_delete( dev_hdl );
    vTaskDelete( NULL );
}
```

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
