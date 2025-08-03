#include "WiFiManagerParameter.h"
#include "wm_config.h"
#include <cstring>
#include <algorithm>

WiFiManagerParameter::WiFiManagerParameter(const char* customHtml) 
    : _length(0), _type(WMP_TYPE_TEXT) {
    if (customHtml) {
        _customHTML = customHtml;
    }
}

WiFiManagerParameter::WiFiManagerParameter(const char* id, 
                                         const char* placeholder, 
                                         const char* defaultValue, 
                                         int length, 
                                         const char* custom, 
                                         int type) {
    init(id, placeholder, defaultValue, length, custom, type);
}

WiFiManagerParameter::~WiFiManagerParameter() {
    // Destructors for std::string members are called automatically
}

WiFiManagerParameter::WiFiManagerParameter(const WiFiManagerParameter& other) {
    copyFrom(other);
}

WiFiManagerParameter& WiFiManagerParameter::operator=(const WiFiManagerParameter& other) {
    if (this != &other) {
        copyFrom(other);
    }
    return *this;
}

void WiFiManagerParameter::init(const char* id, 
                               const char* placeholder, 
                               const char* defaultValue, 
                               int length, 
                               const char* custom, 
                               int type) {
    _id = id ? id : "";
    _placeholder = placeholder ? placeholder : "";
    _label = placeholder ? placeholder : "";  // Use placeholder as label if not specified
    _value = defaultValue ? defaultValue : "";
    _customHTML = custom ? custom : "";
    _length = std::max(length, static_cast<int>(_value.length()));
    _type = type;
    
    WM_LOGD("Created parameter: id=%s, placeholder=%s, length=%d, type=%d", 
            _id.c_str(), _placeholder.c_str(), _length, _type);
}

void WiFiManagerParameter::copyFrom(const WiFiManagerParameter& other) {
    _id = other._id;
    _placeholder = other._placeholder;
    _label = other._label;
    _value = other._value;
    _customHTML = other._customHTML;
    _length = other._length;
    _type = other._type;
}

const char* WiFiManagerParameter::getID() const {
    return _id.c_str();
}

const char* WiFiManagerParameter::getValue() const {
    return _value.c_str();
}

const char* WiFiManagerParameter::getPlaceholder() const {
    return _placeholder.c_str();
}

const char* WiFiManagerParameter::getLabel() const {
    return _label.c_str();
}

int WiFiManagerParameter::getValueLength() const {
    return _value.length();
}

const char* WiFiManagerParameter::getCustomHTML() const {
    return _customHTML.c_str();
}

int WiFiManagerParameter::getType() const {
    return _type;
}

void WiFiManagerParameter::setValue(const char* value, int length) {
    if (!value) {
        _value.clear();
        return;
    }
    
    if (length >= 0) {
        _value.assign(value, std::min(length, static_cast<int>(strlen(value))));
    } else {
        _value = value;
    }
    
    // Update length if necessary
    if (static_cast<int>(_value.length()) > _length) {
        _length = _value.length();
    }
    
    WM_LOGD("Parameter %s value set to: %s", _id.c_str(), _value.c_str());
}

void WiFiManagerParameter::setValue(const std::string& value) {
    setValue(value.c_str(), value.length());
}

void WiFiManagerParameter::setValue(std::string_view value) {
    _value = value;
    if (static_cast<int>(_value.length()) > _length) {
        _length = _value.length();
    }
    
    WM_LOGD("Parameter %s value set to: %s", _id.c_str(), _value.c_str());
} 