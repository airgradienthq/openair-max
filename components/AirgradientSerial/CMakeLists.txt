idf_component_register(SRCS "AirgradientSerial.cpp" "AirgradientUART.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart
                    )
