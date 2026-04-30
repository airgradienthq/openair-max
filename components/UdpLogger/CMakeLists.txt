idf_component_register(
    SRCS
        "src/UdpLogger.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        esp_timer
        log
        lwip
        freertos
)

target_compile_features(${COMPONENT_TARGET} PUBLIC cxx_std_17)
