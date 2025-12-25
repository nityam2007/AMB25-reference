# AMB25/AMB26 Complete Reference Guide

**Realtek RTL8720DF (AmebaD) Development Board**

This is a comprehensive reference for developing on AMB25/AMB26 boards with Arduino IDE.

---

## Table of Contents

1. [Board Overview](#board-overview)
2. [Arduino IDE Setup](#arduino-ide-setup)
3. [GPIO Reference](#gpio-reference)
4. [Platform Detection](#platform-detection)
5. [WiFi](#wifi)
6. [Serial Communication](#serial-communication)
7. [System Functions](#system-functions)
8. [Storage (Flash/EEPROM)](#storage)
9. [Common Issues & Fixes](#common-issues--fixes)
10. [Code Examples](#code-examples)

---

## Board Overview

| Specification | Value |
|---------------|-------|
| **Chip** | Realtek RTL8720DF |
| **CPU** | ARM Cortex-M33 @ 200MHz |
| **RAM** | 512KB |
| **Flash** | 4MB (2MB for user) |
| **WiFi** | 2.4GHz + 5GHz (802.11 a/b/g/n) |
| **Bluetooth** | BLE 5.0 |
| **Operating Voltage** | 3.3V |
| **Digital I/O** | ~20 pins |
| **ADC** | 3 channels |
| **PWM** | Multiple channels |
| **Interfaces** | UART, SPI, I2C |

### AMB25 vs AMB26

- **AMB25**: Standard module with pins exposed
- **AMB26**: Same chip, different form factor/pinout

---

## Arduino IDE Setup

### 1. Install Board Package

1. Open Arduino IDE
2. Go to **File → Preferences**
3. Add this URL to "Additional Board Manager URLs":
   ```
   https://github.com/ambiot/ambd_arduino/raw/master/Arduino_package/package_realtek_amebad_index.json
   ```
4. Go to **Tools → Board → Board Manager**
5. Search for "Realtek AmebaD"
6. Install **Realtek AmebaD Boards** (version 3.1.9 or later)

### 2. Select Board

- **Tools → Board → AmebaD ARM (32-bit) Boards → AMB25**
- Or for AMB26: **AMB26**

### 3. Upload Settings

| Setting | Value |
|---------|-------|
| **Port** | `/dev/ttyUSB0` (Linux) or `COMx` (Windows) |
| **Upload Speed** | 2000000 (default) |
| **Erase Flash** | Disable (unless needed) |

### 4. Enter Download Mode

Before uploading:
1. Press and hold **BOOT** button
2. Press **RESET** button
3. Release **RESET**
4. Release **BOOT**
5. Click Upload in Arduino IDE

---

## GPIO Reference

### Pin Mapping

| Arduino Pin | GPIO | Function | Notes |
|-------------|------|----------|-------|
| `PA_7` | GPIO 7 | Digital I/O | General purpose |
| `PA_8` | GPIO 8 | Digital I/O | General purpose |
| `PA_12` | GPIO 12 | Digital I/O, PWM | ✅ Safe for relay |
| `PA_13` | GPIO 13 | Digital I/O, PWM | ✅ Safe for relay |
| `PA_14` | GPIO 14 | Digital I/O, PWM | ✅ Safe for relay |
| `PA_15` | GPIO 15 | Digital I/O, PWM | ✅ Safe for relay |
| `PA_21` | GPIO 21 | Digital I/O | ✅ Safe for button |
| `PA_22` | GPIO 22 | Digital I/O | ✅ Safe for button |
| `PA_23` | GPIO 23 | Digital I/O | General purpose |
| `PA_24` | GPIO 24 | Digital I/O | General purpose |
| `PA_25` | GPIO 25 | Digital I/O, ADC | Analog capable |
| `PA_26` | GPIO 26 | Digital I/O, ADC | Analog capable |
| `PA_27` | GPIO 27 | SPI MOSI | |
| `PA_28` | GPIO 28 | SPI MISO | |
| `PA_29` | GPIO 29 | SPI CLK | |
| `PA_30` | GPIO 30 | SPI CS | |

### Recommended Pin Usage

```cpp
// Relay outputs (active LOW)
const int RELAY_PINS[] = {PA_12, PA_13, PA_14, PA_15};

// Button inputs (with internal pullup)
const int BUTTON_PINS[] = {PA_21, PA_22, PA_23, PA_24};

// LED indicator
const int LED_PIN = PA_7;

// I2C
const int SDA_PIN = PA_26;
const int SCL_PIN = PA_25;

// Serial2 (hardware UART)
const int TX2_PIN = PA_14;
const int RX2_PIN = PA_13;
```

### GPIO Configuration

```cpp
// Digital output
pinMode(PA_12, OUTPUT);
digitalWrite(PA_12, HIGH);
digitalWrite(PA_12, LOW);

// Digital input with pullup
pinMode(PA_21, INPUT_PULLUP);
int state = digitalRead(PA_21);

// Digital input with pulldown
pinMode(PA_21, INPUT_PULLDOWN);

// PWM output (0-255)
pinMode(PA_12, OUTPUT);
analogWrite(PA_12, 128);  // 50% duty cycle

// Analog input (0-1023)
int value = analogRead(PA_25);
```

---

## Platform Detection

### Detecting AMB25/AmebaD in Code

```cpp
// Method 1: Check for AmebaD defines
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  // AMB25/26 specific code
  #define IS_AMEBA 1
#else
  #define IS_AMEBA 0
#endif

// Method 2: Check architecture
#if defined(ARDUINO_ARCH_AMEBA)
  // AmebaD platform
#endif

// Method 3: Comprehensive platform detection
#if defined(ESP8266)
  #define PLATFORM_NAME "ESP8266"
#elif defined(ESP32)
  #define PLATFORM_NAME "ESP32"
#elif defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #define PLATFORM_NAME "AMB25"
#else
  #define PLATFORM_NAME "Unknown"
#endif
```

### Platform-Specific Includes

```cpp
// WiFi includes
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32) || defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #include <WiFi.h>
#endif

// Prefer defines (AMB25 doesn't have all ESP features)
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #define NEXUS_AMEBA 1
#elif defined(ESP8266)
  #define NEXUS_ESP8266 1
#elif defined(ESP32)
  #define NEXUS_ESP32 1
#endif
```

---

## WiFi

### Basic WiFi Connection

```cpp
#include <WiFi.h>

const char* ssid = "YourNetwork";
const char* password = "YourPassword";

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect");
  }
}

void loop() {
  // Your code here
}
```

### WiFi with Static IP

```cpp
#include <WiFi.h>

IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

void setup() {
  Serial.begin(115200);
  
  // Configure static IP before connecting
  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin("SSID", "PASSWORD");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(WiFi.localIP());
}
```

### Access Point Mode

```cpp
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  
  // Start Access Point
  WiFi.apbegin("AMB25_AP", "12345678");
  
  Serial.println("AP Started");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Handle clients
}
```

### TCP Server

```cpp
#include <WiFi.h>

WiFiServer server(8088);

void setup() {
  Serial.begin(115200);
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  
  server.begin();
  Serial.print("Server at ");
  Serial.print(WiFi.localIP());
  Serial.println(":8088");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String line = client.readStringUntil('\n');
    line.trim();
    
    if (line == "PING") {
      client.println("PONG");
    } else if (line == "STATUS") {
      client.println("OK");
    }
    
    client.stop();
  }
}
```

### TCP Client

```cpp
#include <WiFi.h>

void sendToServer(const char* host, uint16_t port, const char* message) {
  WiFiClient client;
  
  if (client.connect(host, port)) {
    client.println(message);
    
    // Wait for response
    unsigned long timeout = millis();
    while (!client.available() && millis() - timeout < 1000);
    
    if (client.available()) {
      String response = client.readStringUntil('\n');
      Serial.println("Response: " + response);
    }
    
    client.stop();
  } else {
    Serial.println("Connection failed");
  }
}
```

### UDP Communication

```cpp
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const uint16_t UDP_PORT = 32108;

void setup() {
  Serial.begin(115200);
  WiFi.begin("SSID", "PASSWORD");
  while (WiFi.status() != WL_CONNECTED) delay(500);
  
  udp.begin(UDP_PORT);
  Serial.println("UDP ready on port " + String(UDP_PORT));
}

void loop() {
  // Check for incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buffer[256];
    int len = udp.read(buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';
    
    Serial.print("Received: ");
    Serial.println(buffer);
    
    // Reply
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write("ACK");
    udp.endPacket();
  }
}

void sendBroadcast(const char* message) {
  IPAddress broadcastIP = WiFi.localIP();
  broadcastIP[3] = 255;  // x.x.x.255
  
  udp.beginPacket(broadcastIP, UDP_PORT);
  udp.write(message);
  udp.endPacket();
}
```

---

## Serial Communication

### Basic Serial

```cpp
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait up to 3 seconds
  
  Serial.println("AMB25 Ready!");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    Serial.print("Received: ");
    Serial.println(cmd);
  }
}
```

### Printf-Style Output

AMB25 supports `Serial.printf()` natively:

```cpp
int value = 42;
float temp = 25.5;
const char* name = "AMB25";

Serial.printf("Value: %d, Temp: %.1f, Name: %s\n", value, temp, name);
```

### Cross-Platform Printf Macro

For code that works on all platforms:

```cpp
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA) || defined(ESP32) || defined(ESP8266)
  // These platforms support printf
  #define Serial_printf(fmt, ...) do { \
    char _buf[256]; \
    snprintf(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
    Serial.print(_buf); \
  } while(0)
#else
  #define Serial_printf(fmt, ...) Serial.print(fmt)
#endif

// Usage:
Serial_printf("Relay %d is %s\n", 1, "ON");
```

### Hardware Serial2

```cpp
// Serial2 on PA_14 (TX) and PA_13 (RX)
void setup() {
  Serial.begin(115200);   // USB Serial
  Serial2.begin(9600);    // Hardware UART
  
  Serial.println("Dual serial ready");
}

void loop() {
  // Forward Serial2 to Serial
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  
  // Forward Serial to Serial2
  while (Serial.available()) {
    Serial2.write(Serial.read());
  }
}
```

---

## System Functions

### Reboot/Reset

```cpp
// AMB25 does NOT have ESP.restart()
// Use ARM NVIC_SystemReset() instead

void reboot() {
  #if defined(ESP8266)
    ESP.reset();
  #elif defined(ESP32)
    ESP.restart();
  #elif defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
    NVIC_SystemReset();
  #endif
}

// Usage
Serial.println("Rebooting...");
delay(100);
reboot();
```

### Get Free Memory

```cpp
#if defined(ESP8266)
  uint32_t freeHeap = ESP.getFreeHeap();
#elif defined(ESP32)
  uint32_t freeHeap = ESP.getFreeHeap();
#elif defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  // AMB25 - use os_get_free_heap_size() or approximate
  extern "C" uint32_t os_get_free_heap_size(void);
  uint32_t freeHeap = os_get_free_heap_size();
#endif

Serial.printf("Free heap: %u bytes\n", freeHeap);
```

### Get Chip ID / MAC Address

```cpp
String getChipId() {
  #if defined(ESP8266)
    return String(ESP.getChipId(), HEX);
  #elif defined(ESP32)
    return String((uint32_t)ESP.getEfuseMac(), HEX);
  #elif defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
    // Use MAC address as ID
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char id[13];
    sprintf(id, "%02X%02X%02X%02X%02X%02X", 
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(id);
  #endif
}
```

### Delays and Timing

```cpp
// Standard delays work on AMB25
delay(1000);        // 1 second delay
delayMicroseconds(100);  // 100 microsecond delay

// millis() and micros() work normally
unsigned long startTime = millis();
while (millis() - startTime < 5000) {
  // Do something for 5 seconds
}
```

### Watchdog Timer

```cpp
// AMB25 has watchdog but API differs from ESP
// Basic approach - feed the watchdog in loop

void loop() {
  // Your code here
  
  // Feed watchdog (if enabled)
  delay(1);  // Small delay helps
}
```

---

## Storage

### Flash Storage (FlashMemory)

AMB25 uses FlashMemory instead of EEPROM:

```cpp
#include <FlashMemory.h>

// Flash sector for user data
#define FLASH_SECTOR 0x000FF000

void saveData(const char* data, size_t len) {
  FlashMemory.begin(FLASH_SECTOR, len + 4);
  
  // Write length first
  uint32_t length = len;
  FlashMemory.update(0, (uint8_t*)&length, 4);
  FlashMemory.update(4, (uint8_t*)data, len);
  
  FlashMemory.end();
}

String loadData() {
  FlashMemory.begin(FLASH_SECTOR, 256);
  
  uint32_t length;
  FlashMemory.read(0, (uint8_t*)&length, 4);
  
  if (length > 250) {
    FlashMemory.end();
    return "";
  }
  
  char buffer[256];
  FlashMemory.read(4, (uint8_t*)buffer, length);
  buffer[length] = '\0';
  
  FlashMemory.end();
  return String(buffer);
}
```

### Cross-Platform Storage

```cpp
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #include <FlashMemory.h>
  #define USE_FLASH_MEMORY 1
#elif defined(ESP8266) || defined(ESP32)
  #include <EEPROM.h>
  #define USE_EEPROM 1
#endif

void storageBegin() {
  #if USE_FLASH_MEMORY
    // AMB25 - flash initialized on each read/write
  #elif USE_EEPROM
    EEPROM.begin(512);
  #endif
}

void storageWrite(int addr, uint8_t value) {
  #if USE_FLASH_MEMORY
    FlashMemory.begin(0x000FF000, 512);
    FlashMemory.update(addr, &value, 1);
    FlashMemory.end();
  #elif USE_EEPROM
    EEPROM.write(addr, value);
    EEPROM.commit();
  #endif
}

uint8_t storageRead(int addr) {
  #if USE_FLASH_MEMORY
    FlashMemory.begin(0x000FF000, 512);
    uint8_t value;
    FlashMemory.read(addr, &value, 1);
    FlashMemory.end();
    return value;
  #elif USE_EEPROM
    return EEPROM.read(addr);
  #endif
}
```

### JSON Configuration Storage

```cpp
#include <ArduinoJson.h>

#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #include <FlashMemory.h>
  #define STORAGE_ADDR 0x000FF000
#else
  #include <EEPROM.h>
#endif

struct Config {
  char ssid[32];
  char password[64];
  char deviceName[32];
};

void saveConfig(const Config& cfg) {
  JsonDocument doc;
  doc["ssid"] = cfg.ssid;
  doc["pass"] = cfg.password;
  doc["name"] = cfg.deviceName;
  
  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  
  #if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
    FlashMemory.begin(STORAGE_ADDR, len + 4);
    uint32_t length = len;
    FlashMemory.update(0, (uint8_t*)&length, 4);
    FlashMemory.update(4, (uint8_t*)buffer, len);
    FlashMemory.end();
  #else
    EEPROM.begin(512);
    EEPROM.put(0, (uint16_t)len);
    for (size_t i = 0; i < len; i++) {
      EEPROM.write(2 + i, buffer[i]);
    }
    EEPROM.commit();
  #endif
}

bool loadConfig(Config& cfg) {
  char buffer[256];
  uint32_t length;
  
  #if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
    FlashMemory.begin(STORAGE_ADDR, 260);
    FlashMemory.read(0, (uint8_t*)&length, 4);
    if (length > 250) {
      FlashMemory.end();
      return false;
    }
    FlashMemory.read(4, (uint8_t*)buffer, length);
    FlashMemory.end();
  #else
    EEPROM.begin(512);
    uint16_t len;
    EEPROM.get(0, len);
    if (len > 250) return false;
    for (size_t i = 0; i < len; i++) {
      buffer[i] = EEPROM.read(2 + i);
    }
    length = len;
  #endif
  
  buffer[length] = '\0';
  
  JsonDocument doc;
  if (deserializeJson(doc, buffer) != DeserializationError::Ok) {
    return false;
  }
  
  strlcpy(cfg.ssid, doc["ssid"] | "", sizeof(cfg.ssid));
  strlcpy(cfg.password, doc["pass"] | "", sizeof(cfg.password));
  strlcpy(cfg.deviceName, doc["name"] | "", sizeof(cfg.deviceName));
  
  return true;
}
```

---

## Common Issues & Fixes

### Issue 1: ESP.restart() Not Available

**Error:** `'class EspClass' has no member named 'restart'`

**Solution:**
```cpp
// DON'T use on AMB25:
// ESP.restart();

// DO use:
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  NVIC_SystemReset();
#elif defined(ESP32)
  ESP.restart();
#elif defined(ESP8266)
  ESP.reset();
#endif
```

### Issue 2: EEPROM Not Available

**Error:** `'EEPROM' was not declared`

**Solution:**
```cpp
// AMB25 uses FlashMemory, not EEPROM
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #include <FlashMemory.h>
#else
  #include <EEPROM.h>
#endif
```

### Issue 3: IPAddress.toString() Returns String

**Error:** `'class String' has no member named 'toString'`

AMB25's `WiFi.localIP()` may return String directly:
```cpp
// Works on all platforms:
Serial.println(WiFi.localIP());

// If you need String:
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  String ip = WiFi.localIP();  // Already a String
#else
  String ip = WiFi.localIP().toString();
#endif
```

### Issue 4: Upload Fails

**Symptoms:** Upload hangs or fails

**Solutions:**
1. Enter download mode (BOOT + RESET)
2. Use correct port (`/dev/ttyUSB0`)
3. Try different USB cable
4. Reduce upload speed in Tools menu
5. Close Serial Monitor before upload

### Issue 5: WiFi Connection Fails

**Solutions:**
1. Check credentials (case-sensitive)
2. Use 2.4GHz network (5GHz may have issues)
3. Move closer to router
4. Increase connection timeout:
```cpp
int attempts = 0;
while (WiFi.status() != WL_CONNECTED && attempts < 60) {
  delay(500);
  attempts++;
}
```

### Issue 6: GPIO Not Working

**Check:**
1. Pin is correct (`PA_12` not just `12`)
2. pinMode is set before use
3. Pin is not used by another peripheral

### Issue 7: Compilation Warnings

**ArduinoJson v7 warnings:**
```cpp
// DON'T use:
// StaticJsonDocument<256> doc;
// DynamicJsonDocument doc(256);

// DO use (v7+):
JsonDocument doc;
```

### Issue 8: Random Crashes

**Solutions:**
1. Add small delays in tight loops:
```cpp
void loop() {
  // Your code
  delay(1);  // Helps stability
}
```
2. Don't block for too long
3. Check for memory leaks in String usage

---

## Code Examples

### Complete Relay Controller

```cpp
#include <WiFi.h>

const char* SSID = "YourNetwork";
const char* PASS = "YourPassword";

const int RELAY_PINS[] = {PA_12, PA_13, PA_14, PA_15};
const int RELAY_COUNT = 4;
bool relayStates[4] = {false, false, false, false};

WiFiServer server(8088);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("AMB25 Relay Controller");
  
  // Init relays (active LOW)
  for (int i = 0; i < RELAY_COUNT; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);  // OFF
  }
  
  // Connect WiFi
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  server.begin();
  Serial.println("Server ready on port 8088");
}

void loop() {
  handleTCP();
  handleSerial();
  delay(1);
}

void handleTCP() {
  WiFiClient client = server.available();
  if (!client) return;
  
  String cmd = client.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "PING") {
    client.println("{\"ok\":true,\"msg\":\"PONG\"}");
  }
  else if (cmd.startsWith("SET ")) {
    int relay = cmd.substring(4, 5).toInt();
    String val = cmd.substring(6);
    if (relay >= 1 && relay <= RELAY_COUNT) {
      bool on = (val == "ON" || val == "1");
      setRelay(relay - 1, on);
      client.println("{\"ok\":true}");
    } else {
      client.println("{\"ok\":false,\"msg\":\"invalid relay\"}");
    }
  }
  else if (cmd.startsWith("TOGGLE ")) {
    int relay = cmd.substring(7).toInt();
    if (relay >= 1 && relay <= RELAY_COUNT) {
      setRelay(relay - 1, !relayStates[relay - 1]);
      client.println("{\"ok\":true}");
    }
  }
  else if (cmd == "STATUS") {
    String json = "{\"ok\":true,\"relays\":[";
    for (int i = 0; i < RELAY_COUNT; i++) {
      if (i > 0) json += ",";
      json += relayStates[i] ? "1" : "0";
    }
    json += "]}";
    client.println(json);
  }
  
  client.stop();
}

void handleSerial() {
  if (!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "STATUS") {
    for (int i = 0; i < RELAY_COUNT; i++) {
      Serial.printf("Relay %d: %s\n", i + 1, relayStates[i] ? "ON" : "OFF");
    }
  }
  else if (cmd.startsWith("ON ")) {
    int r = cmd.substring(3).toInt();
    if (r >= 1 && r <= RELAY_COUNT) setRelay(r - 1, true);
  }
  else if (cmd.startsWith("OFF ")) {
    int r = cmd.substring(4).toInt();
    if (r >= 1 && r <= RELAY_COUNT) setRelay(r - 1, false);
  }
  else if (cmd == "REBOOT") {
    Serial.println("Rebooting...");
    delay(100);
    NVIC_SystemReset();
  }
  else if (cmd == "HELP") {
    Serial.println("Commands: STATUS, ON <n>, OFF <n>, REBOOT");
  }
}

void setRelay(int idx, bool on) {
  relayStates[idx] = on;
  digitalWrite(RELAY_PINS[idx], on ? LOW : HIGH);  // Active LOW
  Serial.printf("Relay %d: %s\n", idx + 1, on ? "ON" : "OFF");
}
```

### Button Input with Debounce

```cpp
const int BUTTON_PIN = PA_21;
const uint32_t DEBOUNCE_MS = 50;

uint32_t lastPress = 0;
bool lastState = true;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastState = digitalRead(BUTTON_PIN);
}

void loop() {
  bool currentState = digitalRead(BUTTON_PIN);
  
  if (currentState != lastState) {
    if (millis() - lastPress > DEBOUNCE_MS) {
      lastPress = millis();
      lastState = currentState;
      
      if (currentState == LOW) {  // Button pressed (active LOW)
        Serial.println("Button pressed!");
        onButtonPress();
      }
    }
  }
}

void onButtonPress() {
  // Your action here
}
```

---

## Quick Reference Card

```cpp
// PLATFORM DETECTION
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #define IS_AMB25 1
#endif

// REBOOT
NVIC_SystemReset();

// GPIO
const int RELAY = PA_12;
const int BUTTON = PA_21;
pinMode(RELAY, OUTPUT);
pinMode(BUTTON, INPUT_PULLUP);

// WIFI
#include <WiFi.h>
WiFi.begin(ssid, pass);
WiFi.localIP();

// TCP SERVER
WiFiServer server(8088);
server.begin();
WiFiClient client = server.available();

// UDP
#include <WiFiUdp.h>
WiFiUDP udp;
udp.begin(32108);

// STORAGE
#include <FlashMemory.h>
FlashMemory.begin(0x000FF000, size);
FlashMemory.read(addr, buffer, len);
FlashMemory.update(addr, buffer, len);
FlashMemory.end();

// SERIAL PRINTF
Serial.printf("Value: %d\n", value);
```

---

## Resources

- [Realtek AmebaD Arduino GitHub](https://github.com/ambiot/ambd_arduino)
- [AMB25 Pinout Diagram](https://www.amebaiot.com/en/amebad-amb25-arduino-getting-started/)
- [RTL8720DF Datasheet](https://www.realtek.com/en/products/communications-network-ics/item/rtl8720df)

---

*Document Version: 2.0.0 | Last Updated: December 2024*
