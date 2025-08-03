# Basic WiFiManager Example

This example demonstrates the simplest usage of WiFiManager - automatic connection with saved credentials or captive portal fallback.

## What it does

1. **🔄 Auto Connect**: Tries to connect using previously saved WiFi credentials
2. **📱 Captive Portal**: If no credentials exist or connection fails, starts a captive portal
3. **✅ Success**: Continues with your application code once connected

## Code Overview

```cpp
#include "WiFiManager.h"

void app_main() {
    WiFiManager wifiManager;
    
    if (wifiManager.autoConnect("MyDevice-WiFiManager")) {
        // Connected! Your app code goes here
        printf("✅ WiFi connected successfully!\n");
        
        while (1) {
            // Your main application loop
            printf("💚 Application running...\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    } else {
        // Failed to connect
        printf("❌ WiFi connection failed\n");
        esp_restart();
    }
}
```

## How to use

1. **First Run**: Device creates "MyDevice-WiFiManager" AP → Connect & configure WiFi
2. **Subsequent Runs**: Device automatically connects to configured WiFi
3. **Reset**: Call `wifiManager.resetSettings()` to clear saved credentials

## Expected Behavior

### First Time Setup:
```
I (1234) WiFiManager: 📱 Connect to WiFi network: MyDevice-WiFiManager  
I (1234) WiFiManager: 🌐 Open browser to: http://192.168.4.1
```

### Normal Operation:
```
I (1234) WiFiManager: ✅ Connected to saved WiFi: YourNetwork
I (1234) main: ✅ WiFi connected successfully!
I (1234) main: 💚 Application running...
```

## Build and Flash

From your project root:
```bash
idf.py build flash monitor
```

## Next Steps

- See [**advanced**](../advanced/) example for custom parameters and callbacks
- See [**non_blocking**](../non_blocking/) example for async operation 