idf_component_register(
    SRCS 
        "src/WiFiManager.cpp"
        "src/WiFiManagerParameter.cpp"
    INCLUDE_DIRS 
        "include"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        nvs_flash
        esp_timer
        lwip
        json
    PRIV_REQUIRES
        app_update
        esp_system
)

# Embed HTML assets
target_add_binary_data(${COMPONENT_TARGET} "assets/index.html" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "assets/wifi.html" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "assets/style.css" TEXT)
target_add_binary_data(${COMPONENT_TARGET} "assets/wm.js" TEXT)

# Compile definitions
target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    WM_VERSION="2.0.17-ESP-IDF"
)

# C++ standard
target_compile_features(${COMPONENT_TARGET} PUBLIC cxx_std_17) 