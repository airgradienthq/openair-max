idf_component_register(SRCS "AirgradientSerial.cpp" "AirgradientUART.cpp" "AirgradientIICSerial.cpp" "DFRobot_IICSerial.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_driver_i2c esp_driver_gpio esp_timer
                    )
