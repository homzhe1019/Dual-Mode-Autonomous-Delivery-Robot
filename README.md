# Dual-Mode-Autonomous-Delivery-Robot
### This project is a Dual-Mode Delivery Robot powered by ESP32. It is designed to navigate autonomously using PID line tracking algorithms and can be manually controlled via Bluetooth
## ✨ Features: 

### 🎮Dual-Mode Operation:
- Auto Mode: Follows lines automatically
- Manual Mode: Bluetooth control via Smartphone Bluetooth App

### ⚡Smooth Navigation: 
- Implements PID (Proportional-Integral-Derivative) algorithm for stable line tracking

### 📡 Obstacle Avoidance: 
- Automatically stops when an object is detected (Ultrasonic Sensor)

### 🔗 Bluetooth Control: 
- Switch modes and drive manually using serial commands

### 📦Delivery System: 
- Servo-controlled mechanism to drop/pick up items

### 🖥️ Real-time Status: 
- OLED display shows current system status (AUTO/MANUAL)

## 📱App Setup & Controls

1. Download Bluetooth App
- Since this project uses ESP32 Bluetooth Classic, it is recommended to use an Android phone for control
- Android: Search for **Arduino Bluetooth Control** on the Google Play Store and install it
- iOS: iOS devices require BLE to connect, because ESP32 Classic Bluetooth isn’t supported

2. Connection Steps
- Turn on Bluetooth on your phone then pair with **IoT_ESP32**
- Open the Arduino Bluetooth Control
- Tap the connection icon at the top-right corner and select **IoT_ESP32**
- Wait for the notification to show **Connected**

## 🎮 Controls
- After connected, type the following characters in the input box
- Tap the box to send command and control the robot
  
| Key | Action | Description |
| :---: | :--- | :--- |
| **`A`** | **Auto Mode** | Switch to Auto Mode |
| **`M`** | **Manual Mode** | Switch to Manual Mode |
| **`F`** | **Forward** | Move forward |
| **`B`** | **Backward** | Move backward |
| **`L`** | **Spin Left** | Rotate left |
| **`R`** | **Spin Right** | Rotate right |
| **`X Y`** | **Toggle Servo** | Activate delivery mechanism|
