set(srcs 
  "main.cpp"
)

idf_component_register(SRCS "${srcs}"
                    PRIV_REQUIRES esp_driver_gpio AirgradientSerial Sunlight PMS esp_sht4x
                    INCLUDE_DIRS "")
