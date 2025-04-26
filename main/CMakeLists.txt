set(srcs 
  "main.cpp"
  "StatusLed.cpp"
  "Sensor.cpp"
)

idf_component_register(SRCS "${srcs}"
                    PRIV_REQUIRES freertos esp_driver_gpio AirgradientSerial Sunlight PMS esp_sht4x esp_sgp4x sensirion_gas_index_algorithm BQ25672 AirgradientClient
                    INCLUDE_DIRS "")
