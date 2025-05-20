1. Main Controller:

- ESP32-WROOM-32U- Purpose: Main microcontroller, handles all sensor data and communication


2. Environmental Monitoring:

- BME680 Sensor
  Purpose: Temperature, humidity, air pressure, VOC monitoring
  Additional: 0.1µF ceramic capacitor for noise suppression
  Additional: 4.7kΩ pull-up resistors (2x) for I2C


3. Bee Activity Monitoring:

- TCRT5000 IR Sensor Array Kit (2 sets)
  Purpose: Bee movement detection
  Each kit has: 5 IR sensors, pre-assembled on PCB (5 output pins each but we are using only 4 from each to match the pins in the ads1115 module)

- ADS1115 ADC Module (2x)
  Purpose: Analog-to-Digital conversion for IR sensors
  Additional: 0.1µF ceramic capacitors for each module


4. Weight Monitoring:

- 100kg Load Cell
  Purpose: Beehive weight measurement

- HX711 Load Cell Amplifier
  Purpose: Load cell signal amplification
  Additional: 0.1µF ceramic capacitor
  Additional: Shielded twisted pair cable (0.5m) for load cell connection


5. Motion Detection:

- HC-SR501 PIR Sensor
  Purpose: Large predator movement detection
  Additional Components for 5V to 3.3V Level Shifting:
  - 10kΩ resistor
  - 20kΩ resistor

6. Location Tracking:

- NEO-6M GPS Module
  Purpose: Hive location monitoring
  Additional: 0.1µF ceramic capacitor


7. Power System:

Primary Power:
- 18650 3.7V 3500mAh Batteries (2x)
  Purpose: Main power source (connected in parallel 7000mah) 

Charging System:
- TP4056 Charging Module
  Purpose: Battery charging & protection

- MT3608 DC-DC Step-Up Module
  Purpose: Voltage regulation

- 5W 8.8V Solar Panel
  Purpose: Solar charging

Monitoring:
- HMD-EB120 Battery Indicator Module
  Purpose: Battery status monitoring

Protection & Control:
- 2N7000 MOSFET
  Purpose: Sensor power control
- 1N5817 Schottky diode
  Purpose: Reverse polarity protection


8. Voltage Management:

Voltage Divider Components:
- 47kΩ resistor
- 10kΩ resistor
  Purpose: Battery voltage monitoring

Additional Protection:
- 100Ω resistor
  Purpose: Power rail protection
- 4.7kΩ resistor
  Purpose: MOSFET gate control

9. Noise Suppression & Stability:

Capacitors:
- 0.1µF ceramic capacitors (8x)
  Purpose: Noise suppression near sensor power pins
- 10μF electrolytic capacitor
  Purpose: Power stabilization for ESP32
- 100µF electrolytic capacitor
  Purpose: Bulk power rail filtering

10. Optional/Recommended Components:

- Logic Level Converter (4-Channel 3.3V-5V)
  Purpose: Safe voltage interfacing if needed
- 2S 20A 7.4V 18650 Protection Board
  Purpose: Additional battery protection
- 10kΩ NTC thermistor
  Purpose: Battery temperature monitoring


11. Miscellaneous- not compulsory:

- Heat shrink tubing (various sizes)
  Purpose: Wire insulation and protection
- Mounting hardware
  Purpose: Secure components
- Weatherproof enclosure
  Purpose: Environmental protection
- Shielded cables
  Purpose: Signal integrity for sensitive sensors


Important Notes:
1. All sensors should have decoupling capacitors (0.1µF) near power pins
2. I2C lines (SDA/SCL) need pull-up resistors (~4.7k ohm)
3. Sensitive analog signals should use shielded cables
4. Power rails should have bulk capacitors
5. All ground connections should be properly tied together
6. Components should be housed in a weatherproof enclosure