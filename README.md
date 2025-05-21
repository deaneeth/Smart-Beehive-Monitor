# 🐝 Smart Beehive Monitoring System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![PlatformIO](https://img.shields.io/badge/Built%20with-PlatformIO-orange)](https://platformio.org/)
[![ESP32](https://img.shields.io/badge/ESP32-IoT-blue.svg)](https://www.espressif.com/)

> ESP32-based IoT system that monitors beehive conditions and uploads data to Firebase in real-time, enabling beekeepers to remotely track hive health and activity.

<p align="center">
  <img src="assets/BeehivePrototype.png" alt="Smart Beehive Prototype" width="850">
</p>

## 📋 Overview

This project creates a comprehensive monitoring solution for beekeepers, providing real-time data on hive conditions, bee activity, weight changes, and potential threats. The system is solar-powered and designed for outdoor deployment with robust environmental protection.

## ✨ Features

### Environmental Monitoring
- 🌡️ Temperature, humidity, air pressure monitoring
- 🌱 VOC detection for air quality assessment
- 📊 Historical data tracking and trend analysis

### Bee Activity Tracking
- 👁️ Infrared bee movement detection at hive entrance
- 📈 Activity pattern recognition and anomaly detection
- 🔍 Combined entry/exit tracking for overall colony activity

### Weight Monitoring
- ⚖️ Continuous hive weight tracking with 100kg load cell
- 📆 Seasonal trend analysis
- 🍯 Harvest timing optimization

### Advanced Features
- 🚨 Predator detection with motion sensing
- 📍 GPS location tracking for anti-theft
- ☀️ Solar power system with battery management
- 📱 Real-time Firebase database integration
- 🌐 Web dashboard visualization

## 🛠️ Hardware

<p align="center">
  <img src="assets/HardwareSystem.png" alt="Smart Beehive System Hardware" width="800">
</p>

### 🔭 Main Components
- ESP32-WROOM-32U Development Board
- BME680 Environmental Sensor
- TCRT5000 IR Sensor Array (2 kits)
- ADS1115 ADC Modules (2x)
- 100kg Load Cell with HX711 Amplifier
- HC-SR501 PIR Motion Sensor
- NEO-6M GPS Module

### 🔋 Power System
- 18650 3.7V 3500mAh Batteries (2x)
- 5W 8.8V Solar Panel
- TP4056 Charging Module
- MT3608 DC-DC Step-Up Module

## 📝 Documentation

- [Component List](Docs/BOM.md) - Complete bill of materials
- [Wiring Guide](Docs/Smart_Beehive_Monitoring_System-Wiring_Guide.pdf) - Detailed connection guide
- [System Setup](SETUP.md) - Step-by-step instructions for setting up and configuring.
- [Firebase Setup](FIREBASE.md) - Database configuration & structure steps
- [Dashboard Setup](DASHBOARD.md) - Web dashboard for visualizing and analyzing sensor data.

## ⚙️ Setup & Installation

### 📜 Prerequisites
- PlatformIO IDE
- Firebase Account
- Required libraries (configured in platformio.ini)

### ⚡ Quick Start
1. Clone this repository
2. Open in PlatformIO
3. Configure WiFi and Firebase credentials
4. Upload to ESP32
5. Follow calibration steps in [SETUP.md](SETUP.md)

## 📊 Data Structure

```
smart-beehive/
├─ environment/
│  ├─ current/
│  └─ history/
├─ beeActivity/
├─ weight/
├─ location/
└─ system/
```

## 📱 Dashboard Preview

<p align="center">
  <img src="assets/dashboard-preview.png" alt="Dashboard Preview" width="1000">
</p>

## 🤝 Contributing

Contributions welcome! Please read the [contributing guidelines](CONTRIBUTING.md) first.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
