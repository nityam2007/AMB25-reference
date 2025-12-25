---
title: I Made a Reference Guide for AMB25 Because Realtek Forgot To 🤷‍♂️
published: true
description: A complete reference guide for the AMB25/AMB26 development board that should have existed from day one
tags: iot, arduino, embedded, hardware
cover_image: https://www.amebaiot.com/wp-content/uploads/2022/12/amb25/P01.png
---

# I Made a Reference Guide for AMB25 Because Realtek Forgot To 🤷‍♂️

Hey devs! 👋

So, picture this: You just bought a shiny new development board. It's got dual-band WiFi, BLE 5.0, a beefy ARM Cortex-M33 processor, and it's Arduino compatible. You're pumped. You're ready to build the next big IoT thing. You crack open the documentation and... it's a mess. Scattered. Incomplete. You spend more time hunting for information than actually coding.

Sound familiar? Yeah, that was me with the **AMB25/AMB26 (Realtek RTL8720DF)** board.

So I did what any slightly frustrated developer would do — I made my own complete reference guide. And today, I'm sharing it with you!

## 🔗 The Repo

**GitHub:** [https://github.com/nityam2007/AMB25-reference](https://github.com/nityam2007/AMB25-reference)

Star it, fork it, contribute to it. Let's make this the resource that *should* have existed from day one.

---

## Wait, What's an AMB25 Anyway?

Great question! The AMB25 is a development board built around Realtek's RTL8720DF chip. It's actually pretty impressive for its size:

- **CPU:** ARM Cortex-M33 running at 200MHz (that's fast for a microcontroller!)
- **RAM:** 512KB
- **Flash:** 4MB (2MB available for your code)
- **WiFi:** Dual-band 2.4GHz AND 5GHz (802.11 a/b/g/n)
- **Bluetooth:** BLE 5.0
- **Size:** Tiny 50.7 × 17.8 mm form factor
- **USB:** Type-C (finally, a reversible connector!)

Think of it as an ESP32's cool cousin that nobody talks about at family gatherings.

---

## Why I Made This Reference Guide

Here's the thing about the AMB25 — it's genuinely a solid board. Dual-band WiFi is rare at this price point. BLE 5.0 is nice. The 200MHz processor is speedy. But the documentation? Let's just say it could use some love.

I found myself constantly jumping between:
- Forum posts from 2021 that may or may not still be relevant
- Scattered code examples that didn't compile
- Pinout diagrams that required a magnifying glass and a prayer
- API references that assumed you already knew everything

After the fifth time I had to search "AMB25 how to connect WiFi" and got five different answers, I snapped. Not in a bad way — in a "let me fix this" way.

---

## What's In The Reference Guide?

I tried to cover everything you'd actually need:

### 📍 Complete GPIO Reference

Every. Single. Pin. With actual useful information like:
- Which pins support PWM
- Which pins have ADC capability
- What each pin is shared with (so you don't accidentally fight over resources)
- Recommended pins for common use cases

Here's a taste of what the pinout table looks like:

| Pin# | GPIO | ADC | PWM | UART | SPI | I2C |
|------|------|-----|-----|------|-----|-----|
| 0 | PA15 | | | | SPI1_SS | |
| 6 | PA26 | | ✓ | | | I2C_SDA |
| 7 | PA25 | ✓ | | | | I2C_SCL |
| 11 | PB1 | A4 | | | | |

No more guessing which pin does what!

### 📶 WiFi Examples That Actually Work

Both 2.4GHz and 5GHz examples. Station mode. AP mode. Scanning networks. Actually connecting to your home WiFi without it being a three-hour ordeal. All tested. All working.

### 🔌 Serial Communication

Did you know the AMB25 has THREE UART interfaces? I didn't! Until I dug through the datasheet at 2 AM. Now you don't have to:
- `Serial` — USB Serial (for Serial Monitor)
- `Serial1` — Hardware UART1 (pins PB19/PB18)
- `Serial2` — Hardware UART2 (pins PA12/PA13)

### 💾 Flash Storage

How to save data that survives power cycles using FlashMemory. Because sometimes you need to remember things.

### 🐛 Common Issues & Fixes

This section alone probably saved me hours of future frustration:

**"Upload failed!"** — Try holding the Burn button while pressing Reset, then release both. Now upload.

**"Serial Monitor shows garbage!"** — Set baud rate to 115200.

**"WiFi won't connect to 5GHz!"** — Make sure you're using the right channel. Some regions restrict certain 5GHz channels.

---

## My Favorite Discovery: Auto Upload Mode

Okay, so normally to upload code to the AMB25, you have to do this weird dance:
1. Hold the Burn button
2. Press Reset
3. Release both
4. Quickly click Upload in Arduino IDE
5. Hope you timed it right

But guess what? There's an **Auto Upload Mode**! Just enable it in Tools → Auto Upload Mode → Enable, and the board handles it automatically. Mind. Blown.

Why isn't this the default? Why wasn't this in big bold letters on the product page? I have no idea. But now you know!

---

## Code Snippet: Connect to WiFi

Here's a quick snippet to get you connected:

```cpp
#include <WiFi.h>

char ssid[] = "YourNetworkName";
char pass[] = "YourPassword";

void setup() {
    Serial.begin(115200);
    
    WiFi.begin(ssid, pass);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("\nConnected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

void loop() {
    // Your awesome code here
}
```

Simple, right? That's the goal of the whole reference — make things simple and actually usable.

---

## Contributing

The reference is open source (obviously). If you find an error, have a better example, or discovered some hidden AMB25 magic — please contribute! Open an issue, submit a PR, or just leave a comment.

I especially want to hear about:
- Edge cases I haven't covered
- Projects you've built with the AMB25
- Things that confused you that I should clarify

---

## Final Thoughts

The AMB25/AMB26 is genuinely an underrated board. Dual-band WiFi, BLE 5.0, Arduino compatible, USB-C — it checks a lot of boxes. The only thing holding it back is documentation, and hopefully this reference guide helps fix that.

If you're working with these boards, give the guide a look. If it helps you, drop a star on the repo. If you find something wrong, let me know!

**📚 Full Reference:** [https://github.com/nityam2007/AMB25-reference](https://github.com/nityam2007/AMB25-reference)

Happy hacking! 🚀

---

*P.S. — If anyone from Realtek is reading this... please improve your docs. Love the hardware though. ❤️*
