<div align="center">

# 🔌 AMB25/AMB26 Complete Reference Guide

### Realtek RTL8720DF (AmebaD) Development Board

[![Platform](https://img.shields.io/badge/Platform-AMB25%20%7C%20AMB26-blue?style=for-the-badge)](https://www.amebaiot.com/en/amebad-amb25-arduino-getting-started/)
[![Chip](https://img.shields.io/badge/Chip-RTL8720DF-green?style=for-the-badge)](https://www.realtek.com)
[![WiFi](https://img.shields.io/badge/WiFi-2.4GHz%20%2B%205GHz-orange?style=for-the-badge)](#wifi)
[![BLE](https://img.shields.io/badge/BLE-5.0-purple?style=for-the-badge)](#)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-teal?style=for-the-badge)](https://www.arduino.cc/)

---

**📖 Comprehensive reference for developing on AMB25/AMB26 boards with Arduino IDE**

[📚 Official Docs](https://www.amebaiot.com/en/amebad-amb25-arduino-getting-started/) • 
[🛒 Buy Board](https://www.amebaiot.com) • 
[💬 Community](https://forum.amebaiot.com/)

</div>

---

NOTE FROM AUTHOR : 

I made this reference cause AMB25 i.e RealTek Forgot

---

## 📋 Table of Contents

<details>
<summary>Click to expand</summary>

- [🔧 Board Overview](#-board-overview)
- [⚙️ Arduino IDE Setup](#️-arduino-ide-setup)
- [📍 GPIO Reference](#-gpio-reference)
- [🔍 Platform Detection](#-platform-detection)
- [📶 WiFi](#-wifi)
- [💬 Serial Communication](#-serial-communication)
- [🔄 System Functions](#-system-functions)
- [💾 Storage (Flash/EEPROM)](#-storage)
- [🐛 Common Issues & Fixes](#-common-issues--fixes)
- [📝 Code Examples](#-code-examples)
- [📋 Quick Reference Card](#-quick-reference-card)

</details>

---

## 🔧 Board Overview

> **Realtek RTL8720DF** is a Wi-Fi and Bluetooth IC that supports 2.4GHz and 5GHz dual bands for Wi-Fi communication, and Bluetooth Low Energy (BLE) 5.0. AMB25 is a development board integrating the RTL8720DF module with USB Type-C connector and Auto Upload circuit.

### 📊 Specifications

| Specification | Value |
|:-------------:|:-----:|
| **🧠 Chip** | Realtek RTL8720DF |
| **⚡ CPU** | ARM Cortex-M33 @ 200MHz |
| **📦 RAM** | 512KB |
| **💿 Flash** | 4MB (2MB for user) |
| **📶 WiFi** | 2.4GHz + 5GHz dual band (802.11 a/b/g/n) |
| **🔵 Bluetooth** | BLE 5.0 |
| **🔌 Operating Voltage** | 3.3V |
| **📐 Board Size** | 50.7 × 17.8 mm |
| **🔗 USB** | Type-C (power + upload) |
| **🔢 Digital I/O** | 20 pins |
| **📈 ADC** | 3 channels (A4, A5, A6) |
| **〰️ PWM** | 12 channels |
| **🔀 Interfaces** | 2× UART, 2× SPI, 1× I2C, IR |

### 📷 Board Images

<div align="center">

| PINs | BOARD View |
|:----------:|:---------:|
| ![AMB25 PINS](https://www.amebaiot.com/wp-content/uploads/2022/12/amb25/P02.png) | ![AMB25 BOARD](https://www.amebaiot.com/wp-content/uploads/2022/12/amb25/P01.png) |

</div>

### 🔄 AMB25 vs AMB26

| Feature | AMB25 | AMB26 |
|---------|:-----:|:-----:|
| Chip | RTL8720DF | RTL8720DF |
| USB | Type-C ✅ | Varies |
| Auto Upload | ✅ Yes | Board dependent |
| Form Factor | Standard | Different pinout |

### 🔘 Board Buttons

| Button | Location | Function |
|:------:|:--------:|:---------|
| **RST** | ⬅️ Left of USB | Reset the board |
| **Burn** | ➡️ Right of USB | Enter upload mode (hold during reset) |

---

## ⚙️ Arduino IDE Setup

<details>
<summary><b>📥 Step 1: Install Board Package</b></summary>

1. Open Arduino IDE (version 1.6.5 or later)
2. Go to **File → Preferences**
3. Add this URL to "Additional Boards Manager URLs":

```
https://github.com/ambiot/ambd_arduino/raw/master/Arduino_package/package_realtek_amebad_index.json
```

4. Go to **Tools → Board → Boards Manager**
5. Search for `Realtek`
6. Find **"Realtek Ameba Boards (32-bits ARM Cortex-M33 @200MHz)"**
7. Click **Install** (may take several minutes)

</details>

<details>
<summary><b>📴 Offline Installation</b></summary>

> ⚠️ Use this if you have network issues

1. Download from https://www.amebaiot.com/en/ameba-arduino-summary/
2. Download these sections:
   - ✅ **AmebaD_Arduino_patch1_SDK** (required - choose latest)
   - ✅ **AmebaD_Arduino_patch2_Tools** (required - choose your OS)
   - 📦 **AmebaD_Arduino_Source_Code** (optional)
3. Unzip and run installation tool from `Offline_SDK_installation_tool` folder

</details>

<details>
<summary><b>🎯 Step 2: Select Board</b></summary>

Go to **Tools → Board → Ameba ARM (32-bits) Boards → AMB25/AMB26 (RTL8720DF)**

</details>

<details>
<summary><b>🔌 Step 3: Install USB Driver</b></summary>

- Connect board via USB Type-C
- Driver usually installs automatically
- If not, download PL2303GC driver from: https://www.prolific.com.tw/US/ShowProduct.aspx?p_id=225&pcid=41

</details>

<details>
<summary><b>🔗 Step 4: Select Port</b></summary>

| OS | Port Location |
|:---|:--------------|
| 🪟 **Windows** | Check Device Manager for COM port |
| 🐧 **Linux** | Usually `/dev/ttyUSB0` |
| 🍎 **macOS** | Usually `/dev/cu.usbserial-*` |

Go to **Tools → Port** and select the correct port.

</details>

<details>
<summary><b>⚙️ Step 5: Upload Settings</b></summary>

| Setting | Recommended | Notes |
|:--------|:-----------:|:------|
| **Upload Speed** | `1,500,000` | Higher = faster |
| **Erase Flash** | `Disable` | Enable only to wipe flash |
| **Auto Upload Mode** | `Enable` | For auto-upload circuit |
| **Standard Lib** | `Arduino_STD_PRINTF` | For printf compatibility |

</details>

<details>
<summary><b>📤 Step 6: Enter Upload Mode</b></summary>

#### 🖐️ Manual Method
```
1. Press and hold [Burn] button
2. Press and release [RST] button  
3. Release [Burn] button
4. Click Upload in Arduino IDE
```

#### 🤖 Auto Method (Recommended)
```
1. Enable: Tools → Auto Upload Mode → Enable
2. Just click Upload (board enters upload mode automatically)
```

</details>

<details>
<summary><b>🧹 Step 7: First-Time Flash Erase</b></summary>

> 💡 Some boards require initial flash erase

1. Set **Tools → Erase Flash → Enable**
2. Enter upload mode
3. Click **Sketch → Upload**
4. Wait for "Erase flash done."
5. Set **Tools → Erase Flash → Disable**
6. Check Serial Monitor - if only `#` shows, erase was successful ✅

</details>

---

## 📍 GPIO Reference

### 🗺️ Official AMB25 Pinmap

> 💡 **Tip:** All GPIO pins support interrupts (INT). Use ✅ marked pins for general purpose I/O.

| Pin# | GPIO | INT | ADC | PWM | UART | SPI | I2C | IR | SWD | Notes |
|------|------|-----|-----|-----|------|-----|-----|-----|-----|-------|
| 0 | PA15 | ✓ | | | | SPI1_SS | | | | ✅ Output |
| 1 | PA14 | ✓ | | | | SPI1_SCLK | | | | ✅ Output |
| 2 | PA13 | ✓ | | ✓ | SERIAL2_RX | SPI1_MISO | | | | ✅ Output/PWM |
| 3 | PA12 | ✓ | | ✓ | SERIAL2_TX | SPI1_MOSI | | | | ✅ Output/PWM |
| 4 | PA30 | ✓ | | ✓ | | | | | | ✅ Output/PWM |
| 5 | PA28 | ✓ | | ✓ | | | | | | ✅ Output/PWM |
| 6 | PA26 | ✓ | | ✓ | | | I2C_SDA | IR_RX | | I2C Data |
| 7 | PA25 | ✓ | | ✓ | | | I2C_SCL | IR_TX | | I2C Clock |
| 8 | PA27 | ✓ | | | | | | | SWD_DATA | Debug |
| 9 | PB3 | ✓ | A6 | | | | | | SWD_CLK | ADC/Debug |
| 10 | PB2 | ✓ | A5 | | | | | | | ✅ ADC Input |
| 11 | PB1 | ✓ | A4 | | | | | | | ✅ ADC Input |
| 12 | PA7 | ✓ | | | LOG_TX | | | | | Serial Log TX |
| 13 | PA8 | ✓ | | | LOG_RX | | | | | Serial Log RX |
| 14 | PB23 | ✓ | | ✓ | | | | IR_TX | | ✅ Output/PWM |
| 15 | PB22 | ✓ | | ✓ | | | | IR_RX | | ✅ Output/PWM |
| 16 | PB19 | ✓ | | ✓ | SERIAL1_TX | SPI_MISO | | | | UART1/SPI |
| 17 | PB18 | ✓ | | ✓ | SERIAL1_RX | SPI_MOSI | | | | UART1/SPI |
| 18 | PB21 | ✓ | | ✓ | | SPI_SS | | | | SPI Select |
| 19 | PB20 | ✓ | | ✓ | | SPI_SCLK | | | | SPI Clock |

### Recommended Pin Usage

Based on official pinmap:

```cpp
// Relay outputs (PWM capable, active LOW)
// Use pins with PWM capability for relay control
const int RELAY_PINS[] = {
  PA12,   // Pin 3: PWM, SERIAL2_TX, SPI1_MOSI
  PA13,   // Pin 2: PWM, SERIAL2_RX, SPI1_MISO  
  PA14,   // Pin 1: SPI1_SCLK
  PA15,   // Pin 0: SPI1_SS
  PA30,   // Pin 4: PWM
  PA28,   // Pin 5: PWM
  PB22,   // Pin 15: PWM, IR_RX
  PB23    // Pin 14: PWM, IR_TX
};

// Button inputs (with interrupt capability)
// All GPIOs support interrupts
const int BUTTON_PINS[] = {
  PA25,   // Pin 7: I2C_SCL (use if not using I2C)
  PA26,   // Pin 6: I2C_SDA (use if not using I2C)
  PB1,    // Pin 11: ADC A4
  PB2     // Pin 10: ADC A5
};

// I2C Bus (Wire library)
const int SDA_PIN = PA26;  // Pin 6
const int SCL_PIN = PA25;  // Pin 7

// Hardware Serial
// SERIAL1 (Serial1): PB19 TX, PB18 RX
// SERIAL2 (Serial2): PA12 TX, PA13 RX
// LOG: PA7 TX, PA8 RX

// SPI Bus (default)
// MISO: PB19, MOSI: PB18, SS: PB21, SCLK: PB20

// SPI1 Bus
// MISO: PA13, MOSI: PA12, SS: PA15, SCLK: PA14

// ADC Pins (analogRead)
const int ADC_PINS[] = {
  PB3,    // Pin 9: A6 (shared with SWD_CLK)
  PB2,    // Pin 10: A5
  PB1     // Pin 11: A4
};
```

### ⚠️ Important Pin Notes

> **Warning:** Some pins have shared functions. Check before using!

| Function | Pins | ⚠️ Warning |
|:---------|:----:|:-----------|
| **📟 LOG Serial** | PA7/PA8 | Used for Serial Monitor output |
| **🔧 SWD Debug** | PA27/PB3 | Avoid if using debugger |
| **🔗 I2C** | PA25/PA26 | Shared with IR TX/RX |
| **📡 UART1** | PB18/PB19 | Shared with SPI MISO/MOSI |
| **📡 UART2** | PA12/PA13 | Shared with SPI1 MISO/MOSI |

### 🎮 GPIO Configuration

```cpp
// Digital output
pinMode(PA12, OUTPUT);
digitalWrite(PA12, HIGH);
digitalWrite(PA12, LOW);

// Digital input with pullup
pinMode(PA25, INPUT_PULLUP);
int state = digitalRead(PA25);

// Digital input with pulldown
pinMode(PA25, INPUT_PULLDOWN);

// PWM output (0-255)
// PWM capable: PA12, PA13, PA30, PA28, PA25, PA26, PB22, PB23, PB19, PB18, PB21, PB20
pinMode(PA12, OUTPUT);
analogWrite(PA12, 128);  // 50% duty cycle

// Analog input (0-4095 on AMB25, 12-bit ADC)
// ADC capable: PB3(A6), PB2(A5), PB1(A4)
int value = analogRead(PB2);  // Read from A5
```

---

## 🔍 Platform Detection

> 💡 Use these macros to write code that works across ESP8266, ESP32, and AMB25

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

## 📶 WiFi

> 📡 AMB25 supports both 2.4GHz and 5GHz WiFi bands

<details>
<summary><b>📶 Basic WiFi Connection</b></summary>

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

</details>

<details>
<summary><b>🔒 WiFi with Static IP</b></summary>

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

</details>

<details>
<summary><b>📡 Access Point Mode</b></summary>

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

</details>

<details>
<summary><b>🖥️ TCP Server</b></summary>

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

</details>

<details>
<summary><b>📤 TCP Client</b></summary>

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

</details>

<details>
<summary><b>📨 UDP Communication</b></summary>

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

</details>

---

## 💬 Serial Communication

> 🔌 AMB25 has 3 hardware serial ports - more than most Arduino boards!

<details>
<summary><b>📟 Basic Serial</b></summary>

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

</details>

<details>
<summary><b>🖨️ Printf-Style Output</b></summary>

AMB25 supports `Serial.printf()` natively:

```cpp
int value = 42;
float temp = 25.5;
const char* name = "AMB25";

Serial.printf("Value: %d, Temp: %.1f, Name: %s\n", value, temp, name);
```

</details>

<details>
<summary><b>🔀 Cross-Platform Printf Macro</b></summary>

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

</details>

### 🔗 Hardware Serial Interfaces

AMB25 has **3 hardware serial ports**:

| Serial Port | TX Pin | RX Pin | Notes |
|:-----------:|:------:|:------:|:------|
| `Serial` (LOG) | PA7 (Pin 12) | PA8 (Pin 13) | USB Serial Monitor |
| `Serial1` | PB19 (Pin 16) | PB18 (Pin 17) | Hardware UART1, shared with SPI |
| `Serial2` | PA12 (Pin 3) | PA13 (Pin 2) | Hardware UART2, shared with SPI1 |

```cpp
// Using all serial ports
void setup() {
  Serial.begin(115200);   // USB Serial Monitor (LOG)
  Serial1.begin(9600);    // Hardware UART1 on PB19/PB18
  Serial2.begin(9600);    // Hardware UART2 on PA12/PA13
  
  Serial.println("All serial ports ready");
}

void loop() {
  // Forward Serial1 to Serial
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  
  // Forward Serial2 to Serial  
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  
  // Forward USB Serial to Serial1
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }
}
```

---

## 🔄 System Functions

> ⚠️ AMB25 uses ARM-specific functions instead of ESP functions

<details>
<summary><b>🔁 Reboot/Reset</b></summary>

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

</details>

<details>
<summary><b>📊 Get Free Memory</b></summary>

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

</details>

<details>
<summary><b>🆔 Get Chip ID / MAC Address</b></summary>

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

</details>

<details>
<summary><b>⏱️ Delays and Timing</b></summary>

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

</details>

<details>
<summary><b>🐕 Watchdog Timer</b></summary>

```cpp
// AMB25 has watchdog but API differs from ESP
// Basic approach - feed the watchdog in loop

void loop() {
  // Your code here
  
  // Feed watchdog (if enabled)
  delay(1);  // Small delay helps
}
```

</details>

---

## 💾 Storage

> 💡 AMB25 uses **FlashMemory** instead of EEPROM

<details>
<summary><b>💿 Flash Storage (FlashMemory)</b></summary>

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

</details>

<details>
<summary><b>🔀 Cross-Platform Storage</b></summary>

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

</details>

<details>
<summary><b>📄 JSON Configuration Storage</b></summary>

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

</details>

---

## 🐛 Common Issues & Fixes

> 🔧 Quick solutions to common AMB25 development problems

<details>
<summary><b>❌ Issue 1: ESP.restart() Not Available</b></summary>

**Error:** `'class EspClass' has no member named 'restart'`

**✅ Solution:**
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

</details>

<details>
<summary><b>❌ Issue 2: EEPROM Not Available</b></summary>

**Error:** `'EEPROM' was not declared`

**✅ Solution:**
```cpp
// AMB25 uses FlashMemory, not EEPROM
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #include <FlashMemory.h>
#else
  #include <EEPROM.h>
#endif
```

</details>

<details>
<summary><b>❌ Issue 3: IPAddress.toString() Returns String</b></summary>

**Error:** `'class String' has no member named 'toString'`

AMB25's `WiFi.localIP()` may return String directly:

**✅ Solution:**
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

</details>

<details>
<summary><b>❌ Issue 4: Upload Fails</b></summary>

**Symptoms:** Upload hangs or fails

**✅ Solutions:**
1. Enter download mode (BOOT + RESET)
2. Use correct port (`/dev/ttyUSB0`)
3. Try different USB cable
4. Reduce upload speed in Tools menu
5. Close Serial Monitor before upload

</details>

<details>
<summary><b>❌ Issue 5: WiFi Connection Fails</b></summary>

**✅ Solutions:**
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

</details>

<details>
<summary><b>❌ Issue 6: GPIO Not Working</b></summary>

**🔍 Check:**
1. Pin is correct (`PA_12` not just `12`)
2. pinMode is set before use
3. Pin is not used by another peripheral

</details>

<details>
<summary><b>❌ Issue 7: Compilation Warnings (ArduinoJson)</b></summary>

**ArduinoJson v7 warnings:**
```cpp
// DON'T use:
// StaticJsonDocument<256> doc;
// DynamicJsonDocument doc(256);

// DO use (v7+):
JsonDocument doc;
```

</details>

<details>
<summary><b>❌ Issue 8: Random Crashes</b></summary>

**✅ Solutions:**
1. Add small delays in tight loops:
```cpp
void loop() {
  // Your code
  delay(1);  // Helps stability
}
```
2. Don't block for too long
3. Check for memory leaks in String usage

</details>

---

## 📝 Code Examples

> 💡 Complete working examples you can copy-paste

<details>
<summary><b>🔌 Complete Relay Controller</b></summary>

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

</details>

---

## 📋 Quick Reference Card

<table>
<tr>
<td width="50%">

### 🔍 Platform Detection
```cpp
#if defined(BOARD_RTL8720DN) || defined(ARDUINO_AMEBA)
  #define IS_AMB25 1
#endif
```

### 🔁 Reboot
```cpp
NVIC_SystemReset();
```

### 🔌 GPIO
```cpp
const int RELAY = PA_12;
const int BUTTON = PA_21;
pinMode(RELAY, OUTPUT);
pinMode(BUTTON, INPUT_PULLUP);
```

</td>
<td width="50%">

### 📶 WiFi
```cpp
#include <WiFi.h>
WiFi.begin(ssid, pass);
WiFi.localIP();
```

### 🖥️ TCP Server
```cpp
WiFiServer server(8088);
server.begin();
WiFiClient client = server.available();
```

### 📨 UDP
```cpp
#include <WiFiUdp.h>
WiFiUDP udp;
udp.begin(32108);
```

</td>
</tr>
<tr>
<td>

### 💾 Storage
```cpp
#include <FlashMemory.h>
FlashMemory.begin(0x000FF000, size);
FlashMemory.read(addr, buffer, len);
FlashMemory.update(addr, buffer, len);
FlashMemory.end();
```

</td>
<td>

### 🖨️ Serial Printf
```cpp
Serial.printf("Value: %d\n", value);
```

### 🆔 Get MAC
```cpp
uint8_t mac[6];
WiFi.macAddress(mac);
```

</td>
</tr>
</table>

---

## 🔗 Resources

<div align="center">

| Resource | Link |
|:---------|:-----|
| 📚 **Official Arduino SDK** | [github.com/ambiot/ambd_arduino](https://github.com/ambiot/ambd_arduino) |
| 📍 **AMB25 Getting Started** | [amebaiot.com/.../amb25-getting-started](https://www.amebaiot.com/en/amebad-amb25-arduino-getting-started/) |
| 📄 **RTL8720DF Datasheet** | [realtek.com/.../rtl8720df](https://www.realtek.com/en/products/communications-network-ics/item/rtl8720df) |
| 💬 **Community Forum** | [forum.amebaiot.com](https://forum.amebaiot.com/) |

</div>

---

<div align="center">

### 📄 Document Info

| | |
|:-:|:-:|
| **Version** | 2.0.0 |
| **Last Updated** | December 2024 |
| **Author** | [Nityam](https://github.com/Nityam2007) |
| **License** | MIT |

---

**Made with ❤️ for the AMB25 Community**

[![GitHub](https://img.shields.io/badge/GitHub-nexusio--library-black?style=flat-square&logo=github)](https://github.com/Nityam2007/nexusio-library)

</div>
