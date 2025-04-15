idf_component_register(SRCS "AirgradientSerial.cpp" "AirgradientUART.cpp" "AirgradientIICSerial"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_driver_i2c
                    )
