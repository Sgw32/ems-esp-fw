[![Build EMS FW](https://github.com/Sgw32/ems-esp-fw/actions/workflows/build_main.yml/badge.svg)](https://github.com/Sgw32/ems-esp-fw/actions/workflows/build_main.yml)
# **EMS Box Firmware for ESP32-S3**  

## **Project Overview**  
This project is the firmware for the **EMS Box**, a muscle stimulation device based on the **ESP32-S3** microcontroller. The firmware manages hardware components, processes user interactions, and transmits data via Bluetooth.  

## **Features**  
✅ Capacitive touch button for power control  
✅ OLED display (SSD1306, 128x64, I2C) 
✅ Pulse sensor **MAX30100** for heart rate measurement  
✅ Bluetooth Low Energy (BLE) data transmission  
✅ State Machine for structured device control  
✅ Power management with **PWR_LATCH** mechanism  
✅ Fixed 20 ms execution loop for stable operation  

---

## **Hardware Components**  
| Component           | Interface  | GPIOs Used        | Description |
|--------------------|-----------|------------------|-------------|
| **ESP32-S3**       | -         | -                | Microcontroller |
| **Capacitive Button (TTP223B)** | Digital Input | GPIO34 | Toggles power |
| **PWR_LATCH Signal** | Digital Output | GPIO46 | Controls power state |
| **OLED Display (SSD1306)** | I2C | GPIO10 (SDA), GPIO11 (SCL) | UI display |
| **Pulse Sensor (MAX30100)** | I2C | GPIO10 (SDA), GPIO11 (SCL) | Heart rate measurement |
| **Bluetooth LE** | BLE | - | Wireless data transmission |

---

## **Software Architecture**  
The project follows a modular design, with the following key components:  

### **1️⃣ State Machine (`device_sm.c/h`)**  
Handles the device's operation states:  
- `OFF` → Waiting for button press  
- `BOOT` → Initializing display & peripherals  
- `IDLE` → Waiting for user input  
- `SHUTDOWN` → Powering off the device  

### **2️⃣ Display Handling (`display.c/h`)**  
- Initializes **SSD1306** OLED display  
- Displays messages and real-time data  

### **3️⃣ Button Handling (`button.c/h`)**  
- Configures **capacitive button**  
- Detects touch input and triggers state transitions  

### **4️⃣ Pulse Sensor (`pulse_sensor.c/h`)**  
- Interfaces with **MAX30100** over **I2C**  
- Reads and processes heart rate data  

### **5️⃣ Bluetooth LE (`ble_service.c/h`)**  
- Implements **BLE GATT service**  
- Sends heart rate data to external devices  

---

## **Build & Flash Instructions**  
### **1️⃣ Prerequisites**
- **ESP-IDF** installed (`v5.x` recommended)
- **ESP32-S3** development board  

### **2️⃣ Build the Project**
```bash
idf.py set-target esp32s3
idf.py build
```
### **3️⃣ Flash to ESP32-S3**
```bash
idf.py flash monitor
```
### **4️⃣ Monitor Serial Output**
```bash
idf.py monitor
```