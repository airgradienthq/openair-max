# Custom HTML WiFiManager Example

This example demonstrates how to customize the WiFiManager web interface with your own branding, styling, and additional features.

## What it demonstrates

- **🎨 Custom Styling**: Modify colors, fonts, and layout
- **🏢 Brand Integration**: Add logos, company info, and custom headers
- **⚙️ Enhanced UI**: Additional form fields and improved UX
- **📱 Responsive Design**: Mobile-friendly customizations
- **🌐 Multi-language**: Localization support examples

## Customization Methods

### 1. CSS Styling
```cpp
// Inject custom CSS
wifiManager.setCustomHeadElement(
    "<style>"
    "body { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }"
    ".logo { width: 120px; margin: 20px auto; display: block; }"
    "</style>"
);
```

### 2. Custom Headers
```cpp
// Add company branding
wifiManager.setCustomHeadElement(
    "<div class='header'>"
    "<img src='data:image/png;base64,iVBOR...' class='logo'>"
    "<h1>MyCompany WiFi Setup</h1>"
    "</div>"
);
```

### 3. Footer Information
```cpp
// Add custom footer
wifiManager.setCustomMenuHTML(
    "<div class='footer'>"
    "<p>© 2024 MyCompany - Support: help@mycompany.com</p>"
    "<p>Device ID: " + String(ESP.getChipId()) + "</p>"
    "</div>"
);
```

## Example Features

### Visual Customizations
- **🎨 Color Scheme**: Corporate branding colors
- **🖼️ Logo Integration**: Base64-encoded logo images  
- **📝 Custom Fonts**: Web fonts and typography
- **🎯 Button Styling**: Branded buttons and interactions

### Functional Enhancements
- **ℹ️ Device Information**: Show chip ID, version, etc.
- **📞 Support Links**: Contact information and help links
- **🌍 Language Selection**: Multi-language interface
- **🔧 Advanced Settings**: Additional configuration options

### Mobile Optimization
- **📱 Touch-Friendly**: Larger buttons and inputs
- **🔄 Responsive Layout**: Adapts to screen sizes
- **⚡ Fast Loading**: Optimized assets and minimal bandwidth

## Code Structure

```
custom_html/
├── README.md          # This file
├── main.c            # Main application with custom styling
├── custom_style.h    # CSS and HTML customizations
└── assets/           # Custom images and resources
    ├── logo.png      # Company logo
    └── favicon.ico   # Custom favicon
```

## Implementation Examples

### Corporate Branding
```cpp
const char* CUSTOM_STYLE = 
    "<style>"
    ":root {"
    "  --primary-color: #2c3e50;"
    "  --secondary-color: #3498db;"
    "  --accent-color: #e74c3c;"
    "}"
    "body { font-family: 'Arial', sans-serif; }"
    ".container { box-shadow: 0 4px 6px rgba(0,0,0,0.1); }"
    "</style>";

wifiManager.setCustomHeadElement(CUSTOM_STYLE);
```

### Device Information Display
```cpp
String deviceInfo = 
    "<div class='device-info'>"
    "<h3>Device Information</h3>"
    "<p>Model: ESP32-WiFiManager</p>"
    "<p>Version: 2.0.17</p>"
    "<p>Chip ID: " + String(ESP.getChipId(), HEX) + "</p>"
    "</div>";

wifiManager.setCustomMenuHTML(deviceInfo.c_str());
```

### Multi-language Support
```cpp
// English/Spanish toggle example
const char* getLocalizedText(const char* key) {
    static bool useSpanish = false; // Toggle based on user preference
    
    if (strcmp(key, "title") == 0) {
        return useSpanish ? "Configuración WiFi" : "WiFi Configuration";
    } else if (strcmp(key, "connect") == 0) {
        return useSpanish ? "Conectar" : "Connect";
    }
    return key; // Fallback
}
```

## Expected Output

### Startup:
```
I (1234) main: 🚀 Starting Custom HTML Example
I (1234) main: 🎨 Loading custom styling...
I (1234) main: 🏢 Applying corporate branding...
I (1234) WiFiManager: 📱 Connect to WiFi network: MyCompany-Setup
I (1234) main: 🌐 Custom portal active with branding
```

### Portal Features:
- **Custom Logo**: Company branding at top
- **Corporate Colors**: Matching brand guidelines
- **Device Info**: Chip ID, version, support contact
- **Responsive Design**: Works on all screen sizes
- **Enhanced UX**: Smooth animations and feedback

## Advanced Customizations

### 1. Dynamic Content
```cpp
// Update content based on device state
String buildCustomHTML() {
    String html = "<div class='status'>";
    html += "<p>Uptime: " + String(millis()/1000) + "s</p>";
    html += "<p>Free Heap: " + String(ESP.getFreeHeap()) + " bytes</p>";
    html += "</div>";
    return html;
}
```

### 2. JavaScript Integration
```cpp
const char* CUSTOM_JS = 
    "<script>"
    "function updateStatus() {"
    "  // Add dynamic behavior"
    "  document.getElementById('status').innerHTML = 'Updated!';"
    "}"
    "setInterval(updateStatus, 5000);"
    "</script>";
```

### 3. Form Validation
```cpp
const char* VALIDATION_JS = 
    "<script>"
    "function validateForm() {"
    "  var ssid = document.querySelector('input[name=\"s\"]').value;"
    "  if (ssid.length < 1) {"
    "    alert('Please select a WiFi network');"
    "    return false;"
    "  }"
    "  return true;"
    "}"
    "</script>";
```

## Build and Flash

From your project root:
```bash
idf.py build flash monitor
```

## Customization Tips

1. **Base64 Images**: Embed small logos directly in CSS
2. **Mobile First**: Design for mobile, enhance for desktop
3. **Minimal JS**: Keep JavaScript simple for reliability
4. **Testing**: Test on various devices and screen sizes
5. **Performance**: Optimize for slow WiFi connections

## Common Use Cases

- **🏢 Corporate Deployments**: Company branding and support info
- **🏠 Consumer Products**: User-friendly setup experiences  
- **🏭 Industrial Devices**: Technical information and diagnostics
- **🎓 Educational Projects**: Learning and demonstration purposes
- **🔧 Development Tools**: Debug information and advanced controls

## Next Steps

- Combine with other examples for full-featured applications
- Add real-time status updates via JavaScript
- Integrate with your existing web design system
- Consider accessibility features for broader user support

## Resources

- [HTML5 Documentation](https://developer.mozilla.org/en-US/docs/Web/HTML)
- [CSS Grid Guide](https://css-tricks.com/snippets/css/complete-guide-grid/)
- [Mobile-First Design](https://developers.google.com/web/fundamentals/design-and-ux/responsive/)
- [Base64 Image Encoder](https://www.base64-image.de/) 