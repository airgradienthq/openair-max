/**
 * @file main.c
 * @brief Non-Blocking WiFiManager Example
 * 
 * This example demonstrates non-blocking operation of WiFiManager:
 * - Portal runs in background while app continues
 * - Regular process() calls handle portal requests
 * - Status LEDs show current WiFi state
 * - Background tasks continue during configuration
 * - Button handling for manual control
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "WiFiManager.h"

static const char* TAG = "main";

// Configuration
#define STATUS_LED_PIN      2    // Built-in LED on many boards
#define BUTTON_PIN         0    // Boot button on many boards
#define SENSOR_READ_INTERVAL 5000  // 5 seconds
#define LED_BLINK_INTERVAL   500   // 500ms

// LED States
typedef enum {
    LED_OFF = 0,
    LED_RED,      // Not connected, no portal
    LED_YELLOW,   // Portal active
    LED_GREEN,    // Connected
    LED_BLUE      // Connecting
} led_state_t;

// Global state
static led_state_t current_led_state = LED_OFF;
static bool button_pressed = false;
static uint32_t button_press_time = 0;
static uint32_t last_sensor_read = 0;
static uint32_t last_led_update = 0;
static float simulated_temperature = 23.5;
static int simulated_battery = 87;

/**
 * @brief Initialize GPIO pins
 */
void init_gpio() {
    // Configure LED pin
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << STATUS_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);
    
    // Configure button pin
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);
    
    ESP_LOGI(TAG, "✅ GPIO initialized (LED: %d, Button: %d)", STATUS_LED_PIN, BUTTON_PIN);
}

/**
 * @brief Update status LED based on current state
 */
void update_status_led(led_state_t state) {
    static bool led_on = false;
    uint32_t now = esp_timer_get_time() / 1000;
    
    current_led_state = state;
    
    switch (state) {
        case LED_OFF:
            gpio_set_level(STATUS_LED_PIN, 0);
            break;
            
        case LED_RED:
            // Slow blink - not connected
            if (now - last_led_update > 1000) {
                led_on = !led_on;
                gpio_set_level(STATUS_LED_PIN, led_on ? 1 : 0);
                last_led_update = now;
            }
            break;
            
        case LED_YELLOW:
            // Fast blink - portal active
            if (now - last_led_update > LED_BLINK_INTERVAL) {
                led_on = !led_on;
                gpio_set_level(STATUS_LED_PIN, led_on ? 1 : 0);
                last_led_update = now;
            }
            break;
            
        case LED_GREEN:
            // Solid on - connected
            gpio_set_level(STATUS_LED_PIN, 1);
            break;
            
        case LED_BLUE:
            // Double blink - connecting
            if (now - last_led_update > 200) {
                static int blink_count = 0;
                led_on = !led_on;
                gpio_set_level(STATUS_LED_PIN, led_on ? 1 : 0);
                blink_count++;
                
                if (blink_count >= 4) { // 2 blinks
                    blink_count = 0;
                    last_led_update = now + 800; // Pause between patterns
                } else {
                    last_led_update = now;
                }
            }
            break;
    }
}

/**
 * @brief Check button state and handle presses
 */
void handle_button(WiFiManager& wifiManager) {
    bool button_state = !gpio_get_level(BUTTON_PIN); // Active low
    uint32_t now = esp_timer_get_time() / 1000;
    
    // Button pressed
    if (button_state && !button_pressed) {
        button_pressed = true;
        button_press_time = now;
        ESP_LOGI(TAG, "🔘 Button pressed");
        
    // Button released
    } else if (!button_state && button_pressed) {
        uint32_t press_duration = now - button_press_time;
        button_pressed = false;
        
        if (press_duration > 5000) {
            // Long press (>5s) - Reset WiFi settings
            ESP_LOGW(TAG, "🔄 Long press detected - resetting WiFi settings");
            if (wifiManager.resetSettings()) {
                ESP_LOGI(TAG, "✅ WiFi settings reset - restarting in 3 seconds");
                vTaskDelay(pdMS_TO_TICKS(3000));
                esp_restart();
            }
            
        } else if (press_duration > 100) {
            // Short press - Start manual portal
            if (!wifiManager.isConfigPortalActive()) {
                ESP_LOGI(TAG, "🌐 Starting manual configuration portal");
                wifiManager.startConfigPortal("Manual-Portal");
            } else {
                ESP_LOGI(TAG, "ℹ️  Portal already active");
            }
        }
    }
}

/**
 * @brief Read simulated sensors (replace with real sensor code)
 */
void read_sensors() {
    uint32_t now = esp_timer_get_time() / 1000;
    
    if (now - last_sensor_read > SENSOR_READ_INTERVAL) {
        // Simulate sensor readings with slight variations
        simulated_temperature += (float)(esp_random() % 20 - 10) / 100.0; // ±0.1°C
        simulated_battery -= (esp_random() % 3); // Slowly decreasing
        
        if (simulated_temperature < 20.0) simulated_temperature = 20.0;
        if (simulated_temperature > 30.0) simulated_temperature = 30.0;
        if (simulated_battery < 0) simulated_battery = 100; // Reset for demo
        
        ESP_LOGI(TAG, "🌡️  Temperature: %.1f°C", simulated_temperature);
        ESP_LOGI(TAG, "🔋 Battery: %d%%", simulated_battery);
        
        last_sensor_read = now;
    }
}

/**
 * @brief Update WiFi status LED based on current state
 */
void update_wifi_status_led(WiFiManager& wifiManager) {
    static bool was_connecting = false;
    
    if (!wifiManager.getWiFiIsSaved()) {
        // No saved WiFi
        if (wifiManager.isConfigPortalActive()) {
            update_status_led(LED_YELLOW); // Portal active
        } else {
            update_status_led(LED_RED);    // Not connected
        }
        was_connecting = false;
        
    } else {
        // Have saved WiFi
        wl_status_t status = wifiManager.getLastConxResult();
        
        if (status == WL_CONNECTED) {
            update_status_led(LED_GREEN);  // Connected
            if (was_connecting) {
                ESP_LOGI(TAG, "✅ WiFi connected!");
                ESP_LOGI(TAG, "📶 SSID: %s", wifiManager.getSSID().c_str());
                was_connecting = false;
            }
        } else {
            update_status_led(LED_BLUE);   // Connecting
            was_connecting = true;
        }
    }
}

/**
 * @brief Background tasks that run continuously
 */
void run_background_tasks(WiFiManager& wifiManager) {
    // Update WiFi status LED
    update_wifi_status_led(wifiManager);
    
    // Handle button presses
    handle_button(wifiManager);
    
    // Read sensors periodically
    read_sensors();
    
    // Add your other background tasks here:
    // - Update displays
    // - Handle other inputs
    // - Communicate with other devices
    // - Watchdog feeding
    // - Data logging
}

/**
 * @brief Display current system status
 */
void display_status(WiFiManager& wifiManager) {
    ESP_LOGI(TAG, "📊 System Status:");
    ESP_LOGI(TAG, "   WiFi Saved: %s", wifiManager.getWiFiIsSaved() ? "Yes" : "No");
    ESP_LOGI(TAG, "   Portal Active: %s", wifiManager.isConfigPortalActive() ? "Yes" : "No");
    ESP_LOGI(TAG, "   Connection Status: %d", wifiManager.getLastConxResult());
    ESP_LOGI(TAG, "   LED State: %d", current_led_state);
    ESP_LOGI(TAG, "   Temperature: %.1f°C", simulated_temperature);
    ESP_LOGI(TAG, "   Battery: %d%%", simulated_battery);
    
    if (wifiManager.getWiFiIsSaved() && wifiManager.getLastConxResult() == WL_CONNECTED) {
        ESP_LOGI(TAG, "   SSID: %s", wifiManager.getSSID().c_str());
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting Non-Blocking WiFiManager Example");
    
    // Initialize hardware
    init_gpio();
    
    // Create WiFiManager instance
    WiFiManager wifiManager;
    
    // Configure for non-blocking operation
    ESP_LOGI(TAG, "🚫 Setting non-blocking mode");
    wifiManager.setConfigPortalBlocking(false);
    
    // Set reasonable timeouts
    wifiManager.setConfigPortalTimeout(0);  // No automatic timeout in non-blocking mode
    wifiManager.setConnectTimeout(30);      // 30 second connection timeout
    
    // Configure WiFi scanning
    wifiManager.setMinimumSignalQuality(8);
    wifiManager.setRemoveDuplicateAPs(true);
    
    ESP_LOGI(TAG, "🔄 Starting WiFi connection process...");
    
    // Start the connection process (non-blocking)
    wifiManager.autoConnect("NonBlocking-WiFiManager");
    
    ESP_LOGI(TAG, "🏃 Entering main application loop");
    ESP_LOGI(TAG, "💡 LED States: Red=No WiFi, Yellow=Portal, Green=Connected, Blue=Connecting");
    ESP_LOGI(TAG, "🔘 Button: Short press=Manual portal, Long press=Reset WiFi");
    
    uint32_t status_display_timer = 0;
    uint32_t loop_count = 0;
    
    // Main application loop
    while (true) {
        // CRITICAL: Process WiFiManager requests
        wifiManager.process();
        
        // Run background tasks
        run_background_tasks(wifiManager);
        
        // Display status every 30 seconds
        uint32_t now = esp_timer_get_time() / 1000;
        if (now - status_display_timer > 30000) {
            display_status(wifiManager);
            status_display_timer = now;
        }
        
        // Periodic log message (every 50 loops = ~5 seconds)
        if (++loop_count % 50 == 0) {
            if (wifiManager.isConfigPortalActive()) {
                ESP_LOGI(TAG, "🟡 Portal active - background tasks running (loop: %lu)", loop_count);
            } else if (wifiManager.getWiFiIsSaved() && wifiManager.getLastConxResult() == WL_CONNECTED) {
                ESP_LOGI(TAG, "💚 WiFi connected - background tasks running (loop: %lu)", loop_count);
            } else {
                ESP_LOGI(TAG, "🔄 WiFi connecting - background tasks running (loop: %lu)", loop_count);
            }
        }
        
        // Sleep to prevent busy waiting (adjust based on your needs)
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms - good balance for responsiveness
    }
} 