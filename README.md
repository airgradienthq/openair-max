> Firmware for the AirGradient Open Air Max

## Table of Contents

- [Led indicator](#led-indicator)
  - [Powered On (1st boot)](#powered-on-1st-boot)
  - [Every wake-up cycle](#every-wake-up-cycle)
- [System Settings Portal](#system-settings-portal)
- [Transmission](#transmission)
  - [Fetch Configuration](#1.-fetch-configuration)
  - [FOTA Update](#2.-fota-update)
  - [Send Measurements](#3.-send-measurements)
    - [Cellular](#cellular)
      - [Data Scaling for Efficiency](#data-scaling-for-efficiency)
      - [Example Payload Breakdown](#example-payload-breakdown)
      - [Handling Data Loss](#handling-data-loss)
    - [WiFi](#wifi)
      - [Example Payload Breakdown](#example-payload-breakdown-1)
- [Building the Firmware](#building-the-firmware)

## Led indicator

### Powered On (1st boot)

1. Led indicator turned on right away
2. Quick blink 2x when starting initialize CE card and attempt to connect to network
3. Quick blink 3x when successfully connect to network
4. Quick blink 4x when successfully post 1st measurement to server
5. Slow blink 2x if there's one or more sensor failed to initialize
6. Slow blink 3x when failed post to server
7. Slow blink 4x when failed post to server because of server issue
8. Slow blink 5x when ensure connection failed after unsuccessful post to server
9. Slow blink 6x when give up after 3 attempts of failed post measures because of network reasons
10. Slow blink 10x when failed connect to network before retry
11. Quick blink 5x when trigger to start/stop system settings portal
12. Quick blink (forever) when system settings portal is currently active

### Every wake-up cycle

1. Led indicator is off unless there's error
2. Slow blink 2x if there's one or more sensor failed to initialize
3. Slow blink 3x when failed post to server

## System Settings Portal

If, on the first boot, the **boot button** is held for 5 seconds or longer, the MAX will launch the _System Settings Portal_. From there, you can configure settings such as:

- Switching the transmission mode between cellular and Wi-Fi
- Setting the cellular APN
- Entering Wi-Fi credentials
- Changing the URL for domain for HTTP requests

To exit, either **save the new settings**, press the **Exit** button on the home page, or hold the **boot button** again for 5 seconds or longer.

## Transmission

Has 2 network options, Cellular and WiFi that can be choose from _System Settings Portal_. Default set to _Cellular_.

### 1. Fetch Configuration

Fetch remote configuration from AirGradient server every 1 hour, and configuration will be applied on the next wake-up cycle

### 2. FOTA Update

Check for firmware update every 1 hour, then fetch and apply if firmware version is not latest

### 3. Send Measurements

Payload format is sent based on what is the network options

#### Cellular

> Applied when transmit through HTTP and MQTT

Device operates on a regular schedule, attempting to send payload every 9 minutes. Each payload contains a set of comma-separated values. The first value is an **`interval`** number in seconds, which serves as a reference for the timestamp of the subsequent measurements. After that, the payload contains three distinct sets of sensor measurements. Each set represents a snapshot of the device's sensor readings taken at a 3-minute interval. The measurements are always in the same order:

- CO2 (ppm)
- Temperature (°C) multiply by 10
- Humidity (%) multiply by 10
- PM1.0 (μg/m³) multiply by 10
- PM2.5 (μg/m³) multiply by 10
- PM10 (μg/m³) multiply by 10
- TVOC (Raw)
- NOx (Raw)
- PM 0.3 Particle Count
- Signal Strength (dbm)
- Battery Voltage (V) multiply by 100
- Solar Panel Voltage (V) multiply by 100
- O3 Working Electrode (mV) multiply by 1000 → _O-M-1PPSTON-CE_ only
- O3 Auxiliary Electrode (mV) multiply by 1000 → _O-M-1PPSTON-CE_ only
- NO2 Working Electrode (mV) multiply by 1000 → _O-M-1PPSTON-CE_ only
- NO2 Auxiliary Electrode (mV) multiply by 1000 → _O-M-1PPSTON-CE_ only
- AFE Temperature (mV) multiply by 10 → _O-M-1PPSTON-CE_ only

##### Data Scaling for Efficiency

To reduce the payload size and still maintain precision for decimal values, certain measurements are scaled up before transmission. To get the actual reading, these value must be divided by a specific factor:

- **Divide by 10**: `Temperature`, `Humidity`, `PM1.0`, `PM2.5`, `PM10`, and `AFE Temperature`.
- **Divide by 100**: `Battery Voltage` and `Solar Panel Voltage`.
- **Divide by 1000**: `O3 WE`, `O3 AE`, `NO2 WE`, and `NO2 AE`.

For example, a transmitted `Temperature` value of `285` should be divided by 10 to get the actual reading of `28.5`.

##### Example Payload Breakdown

A typical payload might look like this:

```
180,452,285,751,163,277,295,34012,20429,3225,-69,1124,14,580981,581369,581319,582400,5796,450,286,751,170,283,300,34140,20589,3272,-69,1124,15,581000,581400,581388,582456,5795,446,286,753,167,287,302,34139,20577,3237,-69,1124,15,581013,581438,581350,582494,5796
```

In this example:

- `180` → The `interval` value at the start in seconds
- `452,...,5796` → The first measurements set
- `450,...,5795` → The second measurements set
- `446,...,5796` → The third measurements set

##### Handling Data Loss

The device has a built-in caching mechanism to handle network outages. If a scheduled transmission fails, the measurements are stored locally. On the next successful transmission, the device will send the new measurements along with the previously cached ones. This means you may receive payloads with more than three sets of measurements. For example, a payload could contain six measurements if one transmission cycle was missed. This ensures data integrity and prevents permanent data loss during temporary network disruptions.

> This caching is not permanent. If the device experiences a power loss, any cached data that has not yet been transmitted will be lost.

#### WiFi

> Applied when transmit through HTTP

Data from the device is packaged into a JSON payload. The device operates on a regular schedule, sending a new payload every 3 minutes. Each payload contains a single set of sensor measurements. Device transmits measurements as they are taken. There is no data caching or retransmission mechanism built into the device. If a transmission fails, the data for that 3-minute interval is not resent.

##### Example Payload Breakdown

```json
{
  "rco2": 452, // CO2
  "atmp": 28.5, // Temperature
  "rhum": 75.1, // Humidity
  "pm01": 16.3, // PM1.0
  "pm25": 27.7, // PM2.5
  "pm10": 29.5, // PM10
  "tvocRaw": 34012, // TVOC Raw
  "noxRaw": 20429, // NOx Raw
  "pm003Count": 3225, // PM 0.3 Particle Count
  "wifi": -69, // Wifi Signal
  "volt": 11.24, // Battery Volt
  "light": 1.4, // Solar Panel Volt
  "measure0": 580.981, // O3 WE
  "measure1": 581.369, // O3 AE
  "measure2": 581.319, // NO2 WE
  "measure3": 582.400, // NO2 AE
  "measure4": 579.6 // AFE Temperature
}
```

## Building the Firmware

This project is based on **ESP-IDF v5.4** and includes [AirgradientClient](https://github.com/airgradienthq/airgradient-client) and [AirgradientOTA](https://github.com/airgradienthq/airgradient-ota) as submodules located in the `components/` directory, so make sure they are properly initialized before building.

**1. Install ESP-IDF**

Follow the official ESP-IDF Getting Started Guide to install the required tools and set up your environment:
- **Getting Started with ESP-IDF**: [ESP-IDF Official Guide](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32c6/get-started/index.html)

**2. Clone the repository and initialize submodules**

```bash
$ git clone --recursive https://github.com/airgradienthq/openair-max
$ cd openair-max
```

If you've already cloned without `--recursive`:

```bash
$ git submodule update --init --recursive
```

**3. Build, flash, and monitor**

```bash
$ idf.py build
$ idf.py -p <PORT> flash monitor
```

### Notes

- Make sure the components/ submodules are up-to-date to avoid build errors.
- The ESP-IDF v5.4 should pull in the compatible toolchain and CMake versions—refer to the ESP-IDF Getting Started Guide for details.
- If installing ESP-IDF through IDE (not manual), make sure to follow the respective guide to build, flash and monitor.
