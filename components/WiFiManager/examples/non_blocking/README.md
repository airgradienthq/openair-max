# Non-Blocking WiFiManager Example

This example demonstrates non-blocking operation of WiFiManager, allowing your application to continue running other tasks while the captive portal is active.

## What it demonstrates

- **🚫 Non-blocking Mode**: Portal runs in background while your app continues
- **🔄 Process Loop**: Call `process()` regularly to handle portal requests
- **⚙️ Multitasking**: Run sensors, LEDs, buttons, etc. during configuration
- **📊 Status Monitoring**: Check portal status and connection state
- **🎯 Responsive UI**: Update displays/LEDs based on WiFi status

## Key Differences from Blocking Mode

### Blocking Mode (default):
```cpp
if (wifiManager.autoConnect("MyDevice")) {
    // Blocks here until WiFi connected or timeout
    printf("Connected!\n");
}
```

### Non-Blocking Mode:
```cpp
wifiManager.setConfigPortalBlocking(false);
wifiManager.autoConnect("MyDevice");

while (true) {
    wifiManager.process();  // Handle portal requests
    
    // Your tasks continue running!
    updateLEDs();
    readSensors();
    handleButtons();
    
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

## Use Cases

Perfect for applications that need to:
- **💡 Update status LEDs** during configuration
- **🔘 Handle button presses** (reset, manual portal, etc.)
- **📡 Continue sensor readings** while connecting
- **🖥️ Update displays** with connection status
- **⏰ Implement watchdogs** or keep-alive mechanisms
- **🔄 Run periodic tasks** during long connection attempts

## Code Structure

```
non_blocking/
├── README.md     # This file  
├── main.c       # Main application with async loop
└── status_led.h  # LED status indicator helper
```

## Example Features

### WiFi Status LED
- **🔴 Red**: Not connected, no portal
- **🟡 Yellow**: Portal active, waiting for config
- **🟢 Green**: Connected to WiFi
- **🔵 Blue**: Connecting to WiFi

### Button Handling
- **Short Press**: Manual portal start
- **Long Press**: Reset WiFi settings
- **Double Press**: Toggle LED brightness

### Sensor Monitoring  
- **Temperature/Humidity**: Continue readings during config
- **Motion Detection**: Background monitoring
- **Battery Level**: Periodic checks

## Expected Output

### Startup (No WiFi):
```
I (1234) main: 🚀 Starting Non-Blocking Example
I (1234) main: 🚫 Setting non-blocking mode
I (1234) main: 🔄 Starting WiFi connection process...
I (1234) main: 🟡 Portal active - waiting for configuration
I (1234) main: 📊 Background tasks running...
I (1234) main: 🌡️  Temperature: 23.5°C
I (1234) main: 🔋 Battery: 87%
```

### During Configuration:
```
I (5678) main: 🔄 Processing portal requests...
I (5678) main: 🌐 Client connected to portal
I (5678) main: 📊 Background tasks running...
I (5678) main: 🌡️  Temperature: 23.6°C
I (5678) main: 🔘 Button pressed - checking status
```

### After Connection:
```
I (12345) main: ✅ WiFi connected!
I (12345) main: 🟢 Status LED: Connected
I (12345) main: 📶 SSID: YourNetwork
I (12345) main: 🔄 Continuing normal operation...
```

## Key Implementation Points

### 1. Set Non-Blocking Mode
```cpp
wifiManager.setConfigPortalBlocking(false);
```

### 2. Regular Process Calls
```cpp
// Call every 100ms or less
wifiManager.process();
```

### 3. Status Checking
```cpp
if (wifiManager.isConfigPortalActive()) {
    // Portal is running
    setStatusLED(YELLOW);
} else if (wifiManager.getWiFiIsSaved()) {
    // Connected
    setStatusLED(GREEN);
}
```

### 4. Manual Portal Control
```cpp
// Start portal manually
if (buttonPressed()) {
    wifiManager.startConfigPortal("Manual-Portal");
}
```

## Build and Flash

From your project root:
```bash
idf.py build flash monitor
```

## Performance Notes

- **CPU Usage**: Minimal overhead, ~1-2% when portal inactive
- **Memory**: Portal uses ~40KB RAM when active
- **Response Time**: Process calls should be <100ms apart for responsiveness
- **Task Priority**: WiFiManager uses standard FreeRTOS priorities

## Common Patterns

### Pattern 1: Status LED Update
```cpp
void updateStatusLED(WiFiManager& wm) {
    if (!wm.getWiFiIsSaved()) {
        setLED(wm.isConfigPortalActive() ? YELLOW : RED);
    } else {
        setLED(GREEN);
    }
}
```

### Pattern 2: Button-Triggered Portal
```cpp
if (buttonPressed() && !wm.isConfigPortalActive()) {
    ESP_LOGI(TAG, "🔘 Starting manual portal");
    wm.startConfigPortal("Manual-Portal");
}
```

### Pattern 3: Timeout Handling
```cpp
static uint32_t portal_start_time = 0;

if (wm.isConfigPortalActive()) {
    if (portal_start_time == 0) {
        portal_start_time = esp_timer_get_time() / 1000;
    }
    
    // Custom timeout logic
    if ((esp_timer_get_time() / 1000) - portal_start_time > 600000) { // 10 minutes
        ESP_LOGW(TAG, "⏰ Portal timeout - restarting");
        esp_restart();
    }
}
```

## Next Steps

- See [**custom_html**](../custom_html/) for UI customization
- Combine with your existing application logic
- Add more sophisticated status indicators 