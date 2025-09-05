# SECoT Victim Firmware

## Overview

This firmware is designed for the **SECoT - Security Evaluation and Compromise Toolkit** project, serving as a realistic *IoT victim device* for demonstrating security audits and attacks in educational settings. It combines two classic IoT use-cases—**Weather Station** and **Smart Light Controller**—into one ESP8266-based device, featuring MQTT connectivity (with various security levels), on-device status indication, and physical feedback via LCD, buzzer, and RGB LEDs.

**Purpose:**  
- Simulate a vulnerable or secure IoT device for attack demonstration (deauth, spoofing, ARP, etc).
- Provide real-time status and feedback to visualize the impact of security attacks.
- Enable hands-on learning with the SECoT toolkit, CLI, and related modules.

---

## Features

- **Weather Station:**  
  Monitors temperature and humidity using DHT11 sensor; reports data over MQTT.
- **Smart Light Controller:**  
  Controls 3 LEDs (RGB) via MQTT; supports brightness, color, and on/off commands.
- **Secure MQTT Integration:**  
  Supports three modes:
  - **No Security** (plain MQTT)
  - **Basic Security** (username/password)
  - **TLS Security** (encrypted MQTT)
- **16x2 I2C LCD Display:**  
  Shows live sensor readings and LED status.
- **Buzzer Alerts:**  
  Audible feedback for threshold alerts (high/low temperature/humidity).
- **Status LED:**  
  Indicates system and network status.
- **Timing & Statistics:**  
  Tracks LED on/off duration, publishes status updates.
- **Robust MQTT Handling:**  
  Auto-reconnect, subscribes to relevant topics, publishes device state and alerts in JSON format.
- **Designed for Attack Simulation:**  
  Integrates seamlessly with SECoT core firmware (the attacker/auditor), allowing testing of attacks like MQTT spoofing, WiFi deauth, ARP poisoning, etc.

---

## Hardware Requirements

- **NodeMCU ESP8266 board**
- **DHT11 Temperature & Humidity Sensor**
- **16x2 I2C LCD Display** (`LCD_ADDRESS = 0x27`)
- **Buzzer**
- **3 LEDs** (Red, Yellow, Blue) + resistors
- **Breadboard/jumper wires**

**Pin Assignments (default):**
| Component    | Pin   |
|--------------|-------|
| DHT11 Data   | D4    |
| Red LED      | D1    |
| Yellow LED   | D2    |
| Blue LED     | D3    |
| Buzzer       | D5    |
| Status LED   | D0    |

---

## Software Requirements

- **Arduino IDE** (1.8.x or newer)
- **ESP8266 Board Package**
- **Required Libraries:**
    - ESP8266WiFi
    - PubSubClient
    - WiFiClientSecure
    - DHT sensor library
    - Adafruit Unified Sensor
    - ArduinoJson
    - LiquidCrystal_I2C
    - Wire

---

## MQTT Configuration

Supports multiple MQTT broker security modes (edit in code as needed):

```cpp
const MqttConfig mqttConfigs[] = {
  // No security
  {"192.168.152.76", 1883, "", "", false},
  // Basic security
  {"192.168.152.76", 1884, "weather_station", "1234", false},
  // Secure TLS
  {"192.168.152.76", 8883, "weather_station", "WeatherStation2024!", true}
};
```
- **client_id:** `"MergedWeatherSmartLight"`
- **Topics:**  
  - Weather: `home/weatherstation/temperature`, `.../humidity`, `.../status`, `.../data`
  - Light: `home/smartlight/state`, `.../command`, `.../brightness`, `.../color`, `.../status`

---

## Installation & Usage

1. **Install Arduino IDE** and ESP8266 board package.
2. **Install required libraries** via Library Manager.
3. **Connect hardware components** as per pinout table.
4. **Edit WiFi and MQTT credentials** in the code.
5. **Upload the firmware** to your NodeMCU ESP8266 board.
6. **Open Serial Monitor** at 115200 baud for debug output.
7. **Connect to the same network** as your SECoT core device and MQTT Broker.

**During operation:**
- Device reads temperature/humidity, displays on LCD, and publishes to MQTT.
- Receives MQTT commands for the smart light (on/off, brightness, color).
- Buzzer alerts if readings exceed thresholds.
- Status LED blinks on events.
- Can be targeted by SECoT core firmware for attack demonstration.

---

## Example Scenarios

- **Attack Simulation:**  
  Use SECoT core (ESP32) to launch WiFi deauth, MQTT spoofing, or ARP poisoning attacks against this victim device. Observe results on LCD, LEDs, buzzer, and MQTT logs.
- **IoT Security Lab:**  
  Demonstrate best/worst practices by switching between insecure/plain MQTT and secure/TLS setups.
- **Standalone Demonstrator:**  
  Use as a basic weather station and smart light on your home MQTT network.

---

## Extending

- Add more sensors (motion, door lock, etc) to simulate additional victim devices.
- Customize MQTT topics or security levels to match your testing environment.
- Integrate with SECoT CLI or other interfaces for automated testing.

---

## License

MIT License. See LICENSE file for details.

---

## Acknowledgments

- Part of the **PFA_SECOT** project.
- Inspired by educational IoT security toolkits and prior work:
    - ESP32 Marauder
    - ESP Ghost
    - Flipper Zero

---

## Disclaimer

**This firmware is intended for educational, lab, and authorized testing only. Do not deploy or use for unauthorized attacks or access. The authors are not responsible for any misuse or damages.**
