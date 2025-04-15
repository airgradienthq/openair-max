set(srcs 
  "main.cpp"
)

idf_component_register(SRCS "${srcs}"
                    PRIV_REQUIRES esp_driver_gpio AirgradientSerial Sunlight PMS
                    INCLUDE_DIRS "")
