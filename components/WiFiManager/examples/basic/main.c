/**
 * @file main.c
 * @brief Basic WiFiManager Example
 * 
 * This example demonstrates the simplest usage of WiFiManager:
 * - Automatic connection with saved credentials
 * - Captive portal fallback if no credentials exist
 * - Clean application code after successful connection
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "WiFiManager.h"

static const char* TAG = "main";

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting Basic WiFiManager Example");
    
    // Create WiFiManager instance
    WiFiManager wifiManager;
    
    ESP_LOGI(TAG, "🔄 Attempting WiFi connection...");
    
    // Try to connect with saved credentials or start captive portal
    // The AP name will be "MyDevice-WiFiManager" if captive portal starts
    if (wifiManager.autoConnect("MyDevice-WiFiManager")) {
        // Successfully connected to WiFi!
        ESP_LOGI(TAG, "✅ WiFi connected successfully!");
        ESP_LOGI(TAG, "📶 SSID: %s", wifiManager.getSSID().c_str());
        
        // Optional: Stop captive portal servers to free memory
        // Uncomment if you want to save memory after connection
        // ESP_LOGI(TAG, "🛑 Stopping captive portal servers...");
        // wifiManager.stopServers();
        
        // Your main application code starts here
        ESP_LOGI(TAG, "🏃 Starting main application...");
        
        while (1) {
            ESP_LOGI(TAG, "💚 Application running...");
            
            // Your application logic goes here
            // For example: read sensors, send data, etc.
            
            vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds
        }
        
    } else {
        // Failed to connect or user cancelled configuration
        ESP_LOGE(TAG, "❌ WiFi connection failed");
        ESP_LOGI(TAG, "🔄 Restarting in 3 seconds...");
        
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
} 