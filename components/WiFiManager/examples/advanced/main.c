/**
 * @file main.c
 * @brief Advanced WiFiManager Example
 * 
 * This example demonstrates advanced WiFiManager features:  
 * - Custom parameters in the captive portal
 * - Callbacks for AP start, save config, and mode changes
 * - Timeout configuration 
 * - Manual portal control
 * - Status information and diagnostics
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "WiFiManager.h"

static const char* TAG = "main";

// Custom parameter storage keys (for demonstration)
#define STORAGE_NAMESPACE "app_config"
#define SERVER_KEY "server"
#define PORT_KEY "port"
#define TOKEN_KEY "token"

// Forward declarations
void saveCustomParametersToNVS();
void loadCustomParametersFromNVS();
void displayStatus(WiFiManager& wifiManager);

// Global custom parameters
WiFiManagerParameter serverParam("server", "API Server", "api.example.com", 40);
WiFiManagerParameter portParam("port", "Port", "443", 6);
WiFiManagerParameter tokenParam("token", "API Token", "", 32);

/**
 * @brief Callback: Called when AP mode starts
 */
void onAPMode(WiFiManager* myWiFiManager) {
    ESP_LOGI(TAG, "🎯 AP Mode Started!");
    ESP_LOGI(TAG, "   SSID: %s", "Advanced-WiFiManager");
    ESP_LOGI(TAG, "   IP: 192.168.4.1");
    ESP_LOGI(TAG, "   🌐 Open browser to configure WiFi");
}

/**
 * @brief Callback: Called when configuration is saved
 */
void onSaveConfig() {
    ESP_LOGI(TAG, "💾 Configuration saved!");
    ESP_LOGI(TAG, "📊 Custom Parameters received:");
    ESP_LOGI(TAG, "   Server: %s", serverParam.getValue());
    ESP_LOGI(TAG, "   Port: %s", portParam.getValue());
    ESP_LOGI(TAG, "   Token: %s", strlen(tokenParam.getValue()) > 0 ? "***HIDDEN***" : "(empty)");
    
    // Here you would typically save custom parameters to your preferred storage
    // This example demonstrates saving to NVS
    saveCustomParametersToNVS();
}

/**
 * @brief Callback: Called when entering config mode  
 */
void onConfigMode(WiFiManager* myWiFiManager) {
    ESP_LOGI(TAG, "🔧 Entering configuration mode");
    ESP_LOGI(TAG, "   Portal will timeout in 5 minutes");
}

/**
 * @brief Save custom parameters to NVS (example implementation)
 */
void saveCustomParametersToNVS() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    
    if (err == ESP_OK) {
        // Save each parameter
        nvs_set_str(nvs_handle, SERVER_KEY, serverParam.getValue());
        nvs_set_str(nvs_handle, PORT_KEY, portParam.getValue());
        nvs_set_str(nvs_handle, TOKEN_KEY, tokenParam.getValue());
        
        // Commit changes
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        
        ESP_LOGI(TAG, "✅ Custom parameters saved to NVS");
    } else {
        ESP_LOGE(TAG, "❌ Failed to open NVS: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Load custom parameters from NVS (example implementation)
 */
void loadCustomParametersFromNVS() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
    
    if (err == ESP_OK) {
        char buffer[64];
        size_t required_size;
        
        // Load server
        required_size = sizeof(buffer);
        if (nvs_get_str(nvs_handle, SERVER_KEY, buffer, &required_size) == ESP_OK) {
            serverParam.setValue(buffer);
            ESP_LOGI(TAG, "📥 Loaded server: %s", buffer);
        }
        
        // Load port
        required_size = sizeof(buffer);
        if (nvs_get_str(nvs_handle, PORT_KEY, buffer, &required_size) == ESP_OK) {
            portParam.setValue(buffer);
            ESP_LOGI(TAG, "📥 Loaded port: %s", buffer);
        }
        
        // Load token
        required_size = sizeof(buffer);
        if (nvs_get_str(nvs_handle, TOKEN_KEY, buffer, &required_size) == ESP_OK) {
            tokenParam.setValue(buffer);
            ESP_LOGI(TAG, "📥 Loaded token: %s", strlen(buffer) > 0 ? "***HIDDEN***" : "(empty)");
        }
        
        nvs_close(nvs_handle);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "❌ Failed to open NVS: %s", esp_err_to_name(err));
    }
}

/**
 * @brief Display current WiFi status and custom configuration
 */
void displayStatus(WiFiManager& wifiManager) {
    ESP_LOGI(TAG, "📊 Current Status:");
    ESP_LOGI(TAG, "   WiFi Connected: %s", wifiManager.getWiFiIsSaved() ? "Yes" : "No");
    
    if (wifiManager.getWiFiIsSaved()) {
        ESP_LOGI(TAG, "   SSID: %s", wifiManager.getSSID().c_str());
        ESP_LOGI(TAG, "   Last Result: %d", wifiManager.getLastConxResult());
    }
    
    ESP_LOGI(TAG, "📊 Custom Configuration:");
    ESP_LOGI(TAG, "   Server: %s", serverParam.getValue());
    ESP_LOGI(TAG, "   Port: %s", portParam.getValue());
    ESP_LOGI(TAG, "   Token: %s", strlen(tokenParam.getValue()) > 0 ? "***SET***" : "(not set)");
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting Advanced WiFiManager Example");
    
    // Initialize NVS for custom parameter storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Load previously saved custom parameters
    ESP_LOGI(TAG, "📥 Loading saved custom parameters...");
    loadCustomParametersFromNVS();
    
    // Create WiFiManager instance
    WiFiManager wifiManager;
    
    // Setup custom parameters
    ESP_LOGI(TAG, "⚙️ Setting up custom parameters...");
    wifiManager.addParameter(&serverParam);
    wifiManager.addParameter(&portParam);  
    wifiManager.addParameter(&tokenParam);
    
    // Configure callbacks
    ESP_LOGI(TAG, "📞 Configuring callbacks...");
    wifiManager.setAPCallback(onAPMode);
    wifiManager.setSaveConfigCallback(onSaveConfig);
    wifiManager.setConfigModeCallback(onConfigMode);
    
    // Configure timeouts and behavior
    ESP_LOGI(TAG, "⏱️ Setting timeouts and options...");
    wifiManager.setConfigPortalTimeout(300);  // 5 minutes
    wifiManager.setConnectTimeout(30);        // 30 seconds
    wifiManager.setConfigPortalBlocking(true); // Blocking mode
    
    // Set WiFi scanning options
    wifiManager.setMinimumSignalQuality(8);   // Minimum signal quality
    wifiManager.setRemoveDuplicateAPs(true);  // Remove duplicate SSIDs
    
    ESP_LOGI(TAG, "🔄 Attempting WiFi connection...");
    
    // Try to connect with saved credentials or start captive portal
    if (wifiManager.autoConnect("Advanced-WiFiManager")) {
        // Successfully connected to WiFi!
        ESP_LOGI(TAG, "✅ WiFi connected successfully!");
        
        // Display current status
        displayStatus(wifiManager);
        
        // Optional: Stop captive portal servers to free memory
        ESP_LOGI(TAG, "🛑 Stopping captive portal servers...");
        wifiManager.stopServers();
        
        // Your main application code starts here
        ESP_LOGI(TAG, "🏃 Starting main application...");
        
        int counter = 0;
        while (1) {
            ESP_LOGI(TAG, "💚 Application running (cycle: %d)", ++counter);
            
            // Example: Every 10 cycles, show status
            if (counter % 10 == 0) {
                displayStatus(wifiManager);
            }
            
            // Example: Simulate reset after 50 cycles (for demo purposes)
            if (counter == 50) {
                ESP_LOGW(TAG, "🔄 Demo: Resetting WiFi settings in 5 seconds...");
                vTaskDelay(pdMS_TO_TICKS(5000));
                
                if (wifiManager.resetSettings()) {
                    ESP_LOGI(TAG, "✅ WiFi settings reset - restarting...");
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "❌ Failed to reset WiFi settings");
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds
        }
        
    } else {
        // Failed to connect or user cancelled configuration
        ESP_LOGE(TAG, "❌ WiFi connection failed or timed out");
        ESP_LOGI(TAG, "🔄 Restarting in 5 seconds...");
        
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
} 