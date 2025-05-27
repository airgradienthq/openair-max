set(srcs 
  "max.cpp"
  "StatusLed.cpp"
  "Sensor.cpp"
  "PayloadCache.cpp"
  "RemoteConfig.cpp"
)

set (
  components
  freertos
  nvs_flash
  esp_driver_gpio
  AirgradientSerial
  Sunlight
  PMS
  esp_sht4x
  esp_sgp4x
  sensirion_gas_index_algorithm
  BQ25672
  AirgradientClient
  AirgradientOTA
  esp_driver_ledc
  json_parser
  console 
  esp_driver_usb_serial_jtag
)

idf_component_register(SRCS "${srcs}"
                    PRIV_REQUIRES "${components}" 
                    INCLUDE_DIRS "")
