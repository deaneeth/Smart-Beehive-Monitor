# Hardware Setup Guide

This guide provides step-by-step instructions for setting up and configuring your Smart Beehive Monitoring System hardware.

## Prerequisites

- All components from the [Bill of Materials](docs/BOM.md)
- Soldering equipment
- Basic electronics tools (wire strippers, multimeter, etc.)
- ESP32 development environment (PlatformIO IDE or Arduino IDE recommended)
- 3D printed enclosure (optional but recommended)

## 1. Hardware Assembly

### 1.1 Prepare the Enclosure

1. If using a 3D printed enclosure, ensure all components fit before proceeding
2. Drill holes for sensors, cables, and ventilation as needed
3. Apply weatherproofing sealant to exterior cable entry points

### 1.2 Assemble Power System

1. Mount the solar panel in a position that receives maximum sunlight
2. Connect the solar panel to the TP4056 charging module via the 1N5817 diode
3. Connect the 18650 batteries in parallel to the battery terminals on the TP4056
4. Connect the TP4056 output to the MT3608 step-up converter input
5. Adjust the MT3608 output to exactly 5V using a multimeter
6. Connect the MT3608 output to the 5V pin on the ESP32

### 1.3 Wire the Sensors

Follow the [Wiring Guide](Docs/Smart_Beehive_Monitoring_System-Wiring_Guide.pdf) to connect all sensors to the ESP32.

### 1.4 Mount the Sensors

1. **BME680**: Place inside the hive near the top but away from direct bee contact
2. **IR Sensors**: Mount in a straight line 2-3cm above the hive entrance
3. **Load Cell**: Place beneath the entire hive with proper support structure
4. **PIR Sensor**: Mount externally facing the hive entrance area
5. **GPS Module**: Position for best satellite reception, typically on top

## 2. Software Configuration

### 2.1 Install Required Software

1. Install Visual Studio Code or Arduino IDE (if you're comfortable with that)
2. Install PlatformIO extension
3. Clone this repository and open in VS Code

### 2.2 Configure Project Settings

1. Update WiFi credentials in `src/main.cpp`:
   ```
   #define WIFI_SSID "YourSSID"
   #define WIFI_PASSWORD "YourPassword"
   ```
2. Configure Firebase settings as described in FIREBASE.md

### 2.3 Upload Code to ESP32

1. Connect ESP32 to your computer via USB
2. Select the correct COM port in PlatformIO
3. Click the Upload button or use the command:
```
pio run -t upload
```

## 3. Sensor Calibration

### 3.1 Load Cell Calibration

1. Power on the system
2. Connect to the ESP32 via Serial Monitor (115200 baud)
3. Enter command c to start load cell calibration
4. Follow the on-screen instructions:
5. Verify the calibration is accurate

### 3.2 IR Sensor Calibration

1. In Serial Monitor, enter command i to start IR sensor calibration
2. Follow the on-screen instructions:
3. Test detection by moving objects under the sensors

### 3.3 System Test

1. Enter command ```z``` to run a full system test
2. Verify all sensors are working properly
3. Check that data is being uploaded to Firebase

## 4. Maintenance

1. Clean the solar panel monthly for optimal charging
2. Check all exterior cables and connections quarterly
3. Inspect the electronics enclosure for moisture after heavy rain
4. Replace batteries every 1-2 years depending on usage

These documents should cover all the necessary aspects of connecting your hardware project to your web dashboard, setting up the system, and providing guidelines for contributors. You can adapt and customize them to fit your specific project requirements.

The key is to maintain enough information in the IoT repository to understand the web dashboard's purpose and connection, while keeping the detailed implementation in the separate web dashboard repository.

For additional help, create an issue on the GitHub repository or contact the project maintainers.