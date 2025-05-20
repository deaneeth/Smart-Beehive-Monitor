```markdown
# Smart Beehive Monitoring System

ESP32-based IoT system that monitors beehive conditions and uploads data to Firebase in real-time, enabling beekeepers to remotely track hive health and activity.

![Project Image](assets/project-photo.jpg)

## Features

- **Environmental Monitoring**: Temperature, humidity, air pressure, and VOC levels via BME680 sensor
- **Bee Activity Tracking**: Bee movement monitoring using TCRT5000 IR sensors
- **Weight Monitoring**: Continuous hive weight tracking with 100kg load cell and HX711
- **Predator Detection**: Motion detection using HC-SR501 PIR sensor
- **Location Tracking**: GPS coordinates via NEO-6M module
- **Power Management**: Solar charging system with battery monitoring
- **Real-time Data**: All sensor data uploaded to Firebase database
- **Web Dashboard**: Data visualization through custom web interface

## Hardware Requirements

### Main Components
- ESP32-WROOM-32U Development Board
- BME680 Environmental Sensor
- TCRT5000 IR Sensor Array (2 kits)
- ADS1115 ADC Modules (2x)
- 100kg Load Cell with HX711 Amplifier
- HC-SR501 PIR Motion Sensor
- NEO-6M GPS Module
- 18650 3.7V 3500mAh Batteries (2x)
- 5W 8.8V Solar Panel
- TP4056 Charging Module
- MT3608 DC-DC Step-Up Module

### Additional Components
- Various resistors and capacitors (see wiring guide)
- Weatherproof enclosure

## Wiring Diagram

![Wiring Diagram](assets/wiring-diagram.png)

For detailed wiring instructions, see [WIRING.md](WIRING.md)

## Software Setup

### Prerequisites
- [PlatformIO IDE](https://platformio.org/install/ide?install=vscode)
- [Firebase Account](https://firebase.google.com/)
- [Arduino IDE](https://www.arduino.cc/en/software) (optional, for testing individual components)

### Firebase Configuration
1. Create a Firebase project at [firebase.google.com](https://firebase.google.com/)
2. Set up Realtime Database
3. Generate API keys
4. Update the following constants in the code:
   ```cpp
   #define API_KEY "YourAPIKey"
   #define DATABASE_URL "YourDatabaseURL"