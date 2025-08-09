# Advanced WiFiManager Example

This example demonstrates advanced WiFiManager features including custom parameters, callbacks, timeouts, and manual portal control.

## What it demonstrates

- **⚙️ Custom Parameters**: Add your own configuration fields to the portal
- **📞 Callbacks**: Handle AP start, save config, and mode change events  
- **⏱️ Timeouts**: Configure portal and connection timeouts
- **🎛️ Manual Control**: Start/stop portal manually
- **🔄 Reset Settings**: Clear saved WiFi credentials
- **📊 Status Information**: Get connection status and diagnostics

## Features Shown

### Custom Parameters
```cpp
WiFiManagerParameter serverParam("server", "API Server", "api.example.com", 40);
WiFiManagerParameter portParam("port", "Port", "443", 6); 
WiFiManagerParameter tokenParam("token", "API Token", "", 32);
```

### Callbacks
```cpp
wifiManager.setAPCallback(onAPMode);           // Called when AP starts
wifiManager.setSaveConfigCallback(onSaveConfig); // Called after successful connection
wifiManager.setConfigModeCallback(onConfigMode); // Called when entering config mode
```

### Timeouts & Settings
```cpp
wifiManager.setConfigPortalTimeout(300);  // 5 minute portal timeout
wifiManager.setConnectTimeout(30);        // 30 second connection timeout
wifiManager.setConfigPortalBlocking(true); // Blocking mode
```

## Code Structure

```
advanced/
├── README.md          # This file
├── main.c            # Main application code
└── advanced_config.h  # Configuration definitions
```

## How to use

1. **First Run**: 
   - Creates "Advanced-WiFiManager" AP
   - Shows custom fields: Server, Port, API Token
   - Configure WiFi + custom parameters

2. **Normal Operation**:
   - Connects automatically to saved WiFi
   - Custom parameters are available but NOT auto-saved
   - You handle custom parameter persistence

3. **Reset Demo**:
   - Press a button (or trigger) to call `resetSettings()`
   - Clears WiFi credentials, forces reconfiguration

## Expected Output

### First Time Setup:
```
I (1234) main: 🚀 Starting Advanced WiFiManager Example
I (1234) main: ⚙️ Setting up custom parameters...
I (1234) main: 📞 Configuring callbacks...
I (1234) main: ⏱️ Setting timeouts...
I (1234) WiFiManager: 📱 Connect to WiFi network: Advanced-WiFiManager
I (1234) main: 🎯 AP Mode Started! SSID: Advanced-WiFiManager
```

### Configuration Save:
```
I (45678) main: 💾 Configuration saved!
I (45678) main: 📊 Custom Parameters:
I (45678) main:   Server: api.example.com
I (45678) main:   Port: 443  
I (45678) main:   Token: your-secret-token
I (45678) main: ✅ WiFi connected: YourNetwork
```

### Normal Operation:
```
I (1234) WiFiManager: ✅ Connected to saved WiFi: YourNetwork
I (1234) main: 📊 Connection Status:
I (1234) main:   SSID: YourNetwork
I (1234) main:   IP: 192.168.1.100
I (1234) main:   Signal: -45 dBm
```

## Build and Flash

From your project root:
```bash
idf.py build flash monitor
```

## Key Learnings

1. **Custom Parameters**: Show in portal but require manual persistence
2. **Callbacks**: Perfect for handling configuration events
3. **Timeouts**: Prevent infinite portal runtime
4. **Manual Control**: Full control over portal lifecycle
5. **Status Info**: Rich diagnostics for debugging

## Next Steps

- See [**non_blocking**](../non_blocking/) for async operation
- See [**custom_html**](../custom_html/) for UI customization 