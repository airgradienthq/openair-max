#pragma once

#include "wm_config.h"
#include <string>
#include <string_view>

/**
 * WiFiManagerParameter class for custom configuration parameters
 * Compatible with Arduino WiFiManager API
 */
class WiFiManagerParameter {
public:
    // Constructor for custom HTML
    WiFiManagerParameter(const char* customHtml);
    
    // Constructor for standard parameters
    WiFiManagerParameter(const char* id, 
                        const char* placeholder, 
                        const char* defaultValue, 
                        int length, 
                        const char* custom = nullptr, 
                        int type = WMP_TYPE_TEXT);

    // Destructor
    ~WiFiManagerParameter();

    // Copy constructor
    WiFiManagerParameter(const WiFiManagerParameter& other);
    
    // Assignment operator
    WiFiManagerParameter& operator=(const WiFiManagerParameter& other);

    // Getters
    const char* getID() const;
    const char* getValue() const;
    const char* getPlaceholder() const;
    const char* getLabel() const;
    int getValueLength() const;
    const char* getCustomHTML() const;
    int getType() const;

    // Setters
    void setValue(const char* value, int length = -1);
    void setValue(const std::string& value);
    void setValue(std::string_view value);
    
    // Internal methods
    void init(const char* id, 
              const char* placeholder, 
              const char* defaultValue, 
              int length, 
              const char* custom, 
              int type);

private:
    std::string _id;
    std::string _placeholder;
    std::string _label;
    std::string _value;
    std::string _customHTML;
    int _length;
    int _type;
    
    void copyFrom(const WiFiManagerParameter& other);
}; 