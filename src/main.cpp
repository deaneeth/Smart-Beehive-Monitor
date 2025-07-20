#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_ADS1X15.h>
#include <HX711.h>
#include <TinyGPS++.h>
#include <time.h>
#include <EEPROM.h>
#include "esp_system.h"
#include "esp_task_wdt.h"

// Debug level
#define DEBUG_MODE 1  // 0=minimal, 1=normal, 2=verbose

// Removed Firebase Helpers because they are no longer needed. New Firebase library handles this internally.

// Pin Definitions
#define BME_I2C_ADDR 0x77
#define PIR_PIN 25
#define LED_PIN 2
#define LOADCELL_DOUT_PIN 32
#define LOADCELL_SCK_PIN 33
#define GPS_RX 16
#define GPS_TX 17
#define GPS_POWER_PIN 13  // Optional power control
#define BATTERY_VOLTAGE_PIN 34 // ADC pin for battery voltage checking

// Watchdog timeout
#define WDT_TIMEOUT 30  // 30 seconds

// EEPROM configuration
#define EEPROM_SIZE 512
#define LOADCELL_CAL_ADDR 0    // 4 bytes for float
#define IR_THRESHOLDS_ADDR 10  // 8 * 4 bytes for thresholds
#define EEPROM_INITIALIZED_ADDR 500 // Flag to check if EEPROM is initialized
#define EEPROM_MAGIC_NUMBER 0xBEE5 // Magic number to validate EEPROM data

// Wi-Fi and Firebase Credentials
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "Password"
#define API_KEY "APIKey"
#define DATABASE_URL "DatabaseURL"

// NTP Server Settings
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 19800  // 5.5 hours (IST)
#define DAYLIGHT_OFFSET_SEC 0

// Sensor-specific Parameters
// BME680
#define SEALEVELPRESSURE_HPA (1013.25)
#define TEMP_THRESHOLD_HIGH 37.78 // 100°F
#define TEMP_THRESHOLD_LOW 10     // 50°F

// IR Sensors
#define SENSOR_THRESHOLD 10000     // Adjust based on calibration
#define DETECTION_DELAY 50         // Minimum ms between detections
#define CURRENT_UPDATE_INTERVAL 60000  // Update current count every minute
#define DAILY_RESET_HOUR 0        // Reset counter at midnight

// Load Cell
#define MEASURE_INTERVAL 1000     // Read sensor every second
#define UPLOAD_INTERVAL 3600000   // Upload weight hourly
#define SAMPLE_SIZE 10            // Number of readings to average
#define EMPTY_HIVE_WEIGHT 5.0     // Empty hive weight in kg
#define WEIGHT_CHANGE_ALERT 2.0   // Alert threshold in kg
#define DEFAULT_CALIBRATION 424.9 // Default calibration value if none stored

// PIR Motion Detection
#define MOTION_COOLDOWN 5000      // Minimum time between detections (ms)
#define MOTION_DURATION_MIN 2000  // Min duration for significant motion (ms)
#define DETECTION_WINDOW 1000     // Time window for consistent motion (ms)
#define SAMPLES_REQUIRED 3        // HIGH readings required in window

// GPS
#define GPS_TIMEOUT 120000        // 2 minutes timeout for GPS fix
#define VALID_LOCATION_UPDATES 3  // Number of valid readings to average
#define MIN_SATELLITES 4          // Minimum satellites for valid fix
#define GPS_CHECK_INTERVAL 60000  // Check GPS status every minute

// Battery Monitoring
#define BATTERY_READ_INTERVAL 60000  // Read battery every minute
#define BATTERY_R1 47.0  // 47kΩ resistor (high side)
#define BATTERY_R2 10.0  // 10kΩ resistor (low side)
#define BATTERY_READS 10 // Number of readings to average
#define BATTERY_MAX_V 4.2 // Max voltage for 2x 18650 in parallel
#define BATTERY_MIN_V 3.0 // Min voltage for 2x 18650 in parallel
#define BATTERY_LOW_THRESHOLD 30 // Low battery threshold (percentage)

// Firebase retry settings
#define FIREBASE_MAX_RETRIES 3
#define FIREBASE_RETRY_INTERVAL 1000

// Global Objects
Adafruit_BME680 bme;
Adafruit_ADS1115 ads1;  // First ADS1115 (Address: 0x48)
Adafruit_ADS1115 ads2;  // Second ADS1115 (Address: 0x49)
HX711 loadcell;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2); // Use UART2

// Firebase Objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool firebaseConnected = false;

// Global Variables for Sensors
// BME680
struct {
  float temperature = 0;
  float humidity = 0;
  float pressure = 0;
  float gas_resistance = 0;
  unsigned long lastRead = 0;
  unsigned long lastUpload = 0;
  bool initialized = false;
  bool error = false;
} bmeData;

// IR Sensors
struct {
  unsigned long lastDetectionTime = 0;
  unsigned long lastUploadTime = 0;
  uint32_t activityCount = 0;
  uint32_t lastUploadedCount = 0;
  int thresholds1[4] = {10000, 10000, 10000, 10000}; // Default thresholds
  int thresholds2[4] = {10000, 10000, 10000, 10000}; // Default thresholds
  bool initialized = false;
  bool error = false;
  unsigned long lastRead = 0;
} irData;

// Load Cell
struct {
  long lastMeasureTime = 0;
  long lastUploadTime = 0;
  float currentWeight = 0;
  float lastWeight = 0;
  float calibrationValue = DEFAULT_CALIBRATION;
  bool initialized = false;
  bool error = false;
  unsigned long lastRead = 0;
} weightData;

// PIR
struct {
  const int READING_HISTORY_SIZE = 10;
  bool readingHistory[10];
  int readingIndex = 0;
  unsigned long lastReadingTime = 0;
  unsigned long lastValidMotion = 0;
  unsigned long lastMotionTime = 0;
  unsigned long motionStartTime = 0;
  unsigned long lastStatusUpdate = 0;
  bool motionActive = false;
  int motionCountToday = 0;
  bool initialized = false;
  bool error = false;
  unsigned long lastRead = 0;
} pirData;

// GPS
struct {
  unsigned long lastLocationUpdate = 0;
  unsigned long lastCheck = 0;
  unsigned long fixStartTime = 0;
  bool acquiring = false;
  float latitude = 0;
  float longitude = 0;
  int satellites = 0;
  bool locationValid = false;
  bool initialized = false;
  bool error = false;
  byte state = 0; // 0=idle, 1=acquiring, 2=success, 3=timeout
} gpsData;

// Battery monitoring
struct {
  float voltage = 0;
  float percentage = 0;
  bool charging = false;
  bool lowBattery = false;
  unsigned long lastRead = 0;
  unsigned long lastChargeCheck = 0;
} batteryData;

// System state
struct {
  time_t lastResetTime = 0;
  struct tm timeinfo;
  unsigned long lastDeepSleepCheck = 0;
  bool lowPowerMode = false;
  unsigned long uptimeSeconds = 0;
  unsigned long lastUptimeUpdate = 0;
  bool timeInitialized = false;
  bool eepromInitialized = false;
} systemState;

// Function Declarations
void setupWiFi();
void setupFirebase();
bool setupBME680();
bool setupADS1115();
void setupHX711();
void setupPIR();
void setupGPS();
bool initTime();
String getISOTimestamp();
void readBME680();
void readIRSensors();
void readLoadCell();
void handleMotion();
void processGPS();
void uploadBME680Data();
void uploadActivityCount(bool isDaily = false);
void uploadWeight(bool isDaily = false);
void uploadLocation();
void updateSystemStatus();
void checkDailyReset();
void readBatteryVoltage();
void checkSolarCharging();
void printSystemStatus();
void checkDeepSleepConditions();
void calibrateIRSensors();
void calibrateLoadCell();
void saveCalibrationData();
void loadCalibrationData();
void setupWatchdog();
void feedWatchdog();
float calculateBatteryPercentage(float voltage);
bool firebaseSetJSON(const String& path, FirebaseJson& json, int maxRetries = FIREBASE_MAX_RETRIES);
bool firebaseUpdateNode(const String& path, FirebaseJson& json, int maxRetries = FIREBASE_MAX_RETRIES);
bool firebasePushJSON(const String& path, FirebaseJson& json, int maxRetries = FIREBASE_MAX_RETRIES);
void testAllSensors();
void handleSerialCommands();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Smart Beehive Monitoring System ===");
  Serial.println("Version 1.0 - May 2025");
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Setup Watchdog
  setupWatchdog();
  
  // Load calibration data
  loadCalibrationData();
  
  // Setup components
  setupBME680();
  setupADS1115();
  setupHX711();
  setupPIR();
  setupGPS();
  
  // Connect to WiFi and Firebase
  setupWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    initTime();
    setupFirebase();
    updateSystemStatus();
  }
  
  // Initial battery reading
  readBatteryVoltage();
  
  Serial.println("\nSystem initialized and ready!");
  
  // Print available commands
  Serial.println("\nAvailable commands:");
  Serial.println("h - Help");
  Serial.println("s - System status");
  Serial.println("r - Reset counters");
  Serial.println("t - Tare load cell");
  Serial.println("c - Calibrate load cell");
  Serial.println("i - Calibrate IR sensors");
  Serial.println("g - Get GPS fix");
  Serial.println("u - Upload all current data");
  Serial.println("w - WiFi reconnect");
  Serial.println("f - Firebase reconnect");
  Serial.println("b - Battery check");
  Serial.println("z - Test sensors");
}

void loop() {
  // Feed the watchdog
  feedWatchdog();
  
  // Update system uptime
  if (millis() - systemState.lastUptimeUpdate >= 60000) {  // Every minute
    systemState.uptimeSeconds += 60;
    systemState.lastUptimeUpdate = millis();
  }
  
  // Non-blocking sensor readings
  readBME680();
  readIRSensors();
  readLoadCell();
  handleMotion();
  processGPS();
  
  // Battery monitoring (every minute)
  if (millis() - batteryData.lastRead >= BATTERY_READ_INTERVAL) {
    readBatteryVoltage();
  }
  
  // Check solar charging (every 5 minutes)
  if (millis() - batteryData.lastChargeCheck >= 300000) {
    checkSolarCharging();
    batteryData.lastChargeCheck = millis();
  }
  
  // Upload data to Firebase if connected
  if (firebaseConnected) {
    // BME680 data - every 10 minutes
    if (millis() - bmeData.lastUpload >= 600000) {
      uploadBME680Data();
      bmeData.lastUpload = millis();
    }
    
    // Bee activity count - every minute
    if (millis() - irData.lastUploadTime >= CURRENT_UPDATE_INTERVAL) {
      uploadActivityCount();
    }
    
    // Weight data - every hour
    if (millis() - weightData.lastUploadTime >= UPLOAD_INTERVAL) {
      uploadWeight();
      weightData.lastUploadTime = millis();
    }
    
    // System status - every 15 minutes
    if (millis() - pirData.lastStatusUpdate >= 900000) {
      updateSystemStatus();
      pirData.lastStatusUpdate = millis();
    }
    
    // Location data - if GPS fix acquired and not yet uploaded
    if (gpsData.state == 2 && gpsData.locationValid && millis() - gpsData.lastLocationUpdate > 10000) {
      uploadLocation();
      gpsData.lastLocationUpdate = millis();
      gpsData.state = 0; // Return to idle state
    }
  }
  
  // Check for daily reset at midnight
  checkDailyReset();
  
  // Print system status every 60 seconds
  static unsigned long lastPrintStatus = 0;
  if (millis() - lastPrintStatus >= 60000) {
    printSystemStatus();
    lastPrintStatus = millis();
  }
  
  // Check deep sleep conditions
  checkDeepSleepConditions();
  
  // Process any serial commands
  handleSerialCommands();
  
  // Check WiFi connection and attempt reconnection if needed
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 300000) { // Every 5 minutes
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting to reconnect...");
      WiFi.disconnect();
      setupWiFi();
      if (WiFi.status() == WL_CONNECTED && !firebaseConnected) {
        setupFirebase();
      }
    }
    lastWiFiCheck = millis();
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

void setupWatchdog() {
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Enable panic on timeout
  esp_task_wdt_add(NULL);  // Add current thread to WDT watch
  Serial.println("Watchdog initialized");
}

void feedWatchdog() {
  esp_task_wdt_reset();  // Feed the watchdog
}

void setupWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // Try for 20 seconds
  int attempts = 0;
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 20000) {
    delay(500);
    Serial.print(".");
    attempts++;
    feedWatchdog(); // Feed the watchdog during connection attempts
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Running in offline mode.");
  }
}

bool setupBME680() {
  Serial.print("Initializing BME680... ");
  
  if (!bme.begin(BME_I2C_ADDR)) {
    Serial.println("Failed! Check wiring.");
    bmeData.error = true;
    return false;
  }
  
  // Set up BME680 parameters
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150ms
  
  bmeData.initialized = true;
  Serial.println("Success!");
  return true;
}

bool setupADS1115() {
  Serial.print("Initializing ADS1115 modules... ");
  
  // First ADS1115 (address 0x48)
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize first ADS1115!");
    irData.error = true;
    return false;
  }
  
  // Second ADS1115 (address 0x49)
  if (!ads2.begin(0x49)) {
    Serial.println("Failed to initialize second ADS1115!");
    irData.error = true;
    return false;
  }
  
  // Set gain
  ads1.setGain(GAIN_ONE);
  ads2.setGain(GAIN_ONE);
  
  irData.initialized = true;
  Serial.println("Success!");
  return true;
}

void setupHX711() {
  Serial.print("Initializing HX711... ");
  
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  if (!loadcell.wait_ready_timeout(2000)) {
    Serial.println("Failed! Check wiring.");
    weightData.error = true;
    return;
  }
  
  // Use the loaded calibration value
  loadcell.set_scale(weightData.calibrationValue);
  Serial.printf("Using calibration: %.1f\n", weightData.calibrationValue);
  
  // Tare the scale
  loadcell.tare();
  
  weightData.initialized = true;
  Serial.println("Success!");
}

void setupPIR() {
  Serial.print("Initializing PIR sensor... ");
  
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize reading history
  for(int i = 0; i < pirData.READING_HISTORY_SIZE; i++) {
    pirData.readingHistory[i] = false;
  }
  
  pirData.initialized = true;
  Serial.println("Success!");
  Serial.println("Waiting for PIR to stabilize...");
  delay(2000);  // Reduced stabilization time for testing
  Serial.println("PIR Ready!");
}

void setupGPS() {
  Serial.print("Initializing GPS module... ");
  
  // Optional power control
  if (GPS_POWER_PIN > 0) {
    pinMode(GPS_POWER_PIN, OUTPUT);
    digitalWrite(GPS_POWER_PIN, HIGH);
  }
  
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  gpsData.initialized = true;
  Serial.println("Success!");
}

void setupFirebase() {
  Serial.print("Connecting to Firebase... ");
  
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Success!");
    firebaseConnected = true;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  } else {
    Serial.println("Failed! Check credentials.");
    firebaseConnected = false;
  }
}

bool initTime() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  if(!getLocalTime(&systemState.timeinfo)) {
    Serial.println("Failed to obtain time!");
    return false;
  }
  
  time(&systemState.lastResetTime);
  systemState.timeInitialized = true;
  
  char timeStr[30];
  strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %H:%M:%S", &systemState.timeinfo);
  Serial.print("Current time: ");
  Serial.println(timeStr);
  
  return true;
}

String getISOTimestamp() {
  if(!getLocalTime(&systemState.timeinfo)) {
    return "time-error";
  }
  char timestamp[25];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H:%M:%S", &systemState.timeinfo);
  return String(timestamp);
}

void loadCalibrationData() {
  Serial.println("Loading calibration data...");
  
  // Check if EEPROM has been initialized
  uint16_t magicNumber;
  EEPROM.get(EEPROM_INITIALIZED_ADDR, magicNumber);
  
  if (magicNumber != EEPROM_MAGIC_NUMBER) {
    Serial.println("EEPROM not initialized, using default values");
    // Set defaults for calibration values
    weightData.calibrationValue = DEFAULT_CALIBRATION;
    
    // Initialize thresholds with default values
    for (int i = 0; i < 4; i++) {
      irData.thresholds1[i] = 10000;
      irData.thresholds2[i] = 10000;
    }
    
    // Mark EEPROM as initialized
    EEPROM.put(EEPROM_INITIALIZED_ADDR, (uint16_t)EEPROM_MAGIC_NUMBER);
    EEPROM.commit();
    return;
  }
  
  // Load load cell calibration value
  EEPROM.get(LOADCELL_CAL_ADDR, weightData.calibrationValue);
  
  // Validate calibration value
  if (isnan(weightData.calibrationValue) || weightData.calibrationValue < 10 || weightData.calibrationValue > 10000) {
    Serial.println("Invalid load cell calibration, using default");
    weightData.calibrationValue = DEFAULT_CALIBRATION;
  } else {
    Serial.printf("Loaded load cell calibration: %.1f\n", weightData.calibrationValue);
  }
  
  // Load IR sensor thresholds
  int addr = IR_THRESHOLDS_ADDR;
  for (int i = 0; i < 4; i++) {
    EEPROM.get(addr, irData.thresholds1[i]);
    addr += sizeof(int);
  }
  
  for (int i = 0; i < 4; i++) {
    EEPROM.get(addr, irData.thresholds2[i]);
    addr += sizeof(int);
  }
  
  // Validate thresholds (simple range check)
  bool validThresholds = true;
  for (int i = 0; i < 4; i++) {
    if (irData.thresholds1[i] < 100 || irData.thresholds1[i] > 30000 ||
        irData.thresholds2[i] < 100 || irData.thresholds2[i] > 30000) {
      validThresholds = false;
      break;
    }
  }
  
  if (!validThresholds) {
    Serial.println("Invalid IR thresholds, using defaults");
    for (int i = 0; i < 4; i++) {
      irData.thresholds1[i] = 10000;
      irData.thresholds2[i] = 10000;
    }
  } else {
    Serial.println("Loaded IR sensor thresholds");
  }
  
  systemState.eepromInitialized = true;
}

void saveCalibrationData() {
  // Save load cell calibration
  EEPROM.put(LOADCELL_CAL_ADDR, weightData.calibrationValue);
  
  // Save IR thresholds
  int addr = IR_THRESHOLDS_ADDR;
  for (int i = 0; i < 4; i++) {
    EEPROM.put(addr, irData.thresholds1[i]);
    addr += sizeof(int);
  }
  
  for (int i = 0; i < 4; i++) {
    EEPROM.put(addr, irData.thresholds2[i]);
    addr += sizeof(int);
  }
  
  // Mark EEPROM as initialized
  EEPROM.put(EEPROM_INITIALIZED_ADDR, (uint16_t)EEPROM_MAGIC_NUMBER);
  
  // Commit changes
  if (EEPROM.commit()) {
    Serial.println("Calibration data saved to EEPROM");
  } else {
    Serial.println("ERROR: Failed to save calibration data");
  }
}

void readBME680() {
  unsigned long currentTime = millis();
  
  // Read every 10 seconds
  if (currentTime - bmeData.lastRead < 10000) return;
  bmeData.lastRead = currentTime;
  
  // Check if sensor is initialized
  if (!bmeData.initialized) return;
  
  // Check sensor health
  Wire.beginTransmission(BME_I2C_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("BME680 communication error!");
    bmeData.error = true;
    return;
  }
  
  // Perform reading
  if (!bme.performReading()) {
    Serial.println("Failed to perform BME680 reading!");
    bmeData.error = true;
    return;
  }
  
  // Store values
  bmeData.temperature = bme.temperature;
  bmeData.humidity = bme.humidity;
  bmeData.pressure = bme.pressure / 100.0;
  bmeData.gas_resistance = bme.gas_resistance / 1000.0;
  bmeData.error = false;
  
  // Validate readings
  if (bmeData.temperature < TEMP_THRESHOLD_LOW || bmeData.temperature > TEMP_THRESHOLD_HIGH) {
    Serial.println("Temperature outside expected range!");
  }
  
  if (bmeData.humidity < 0 || bmeData.humidity > 100) {
    Serial.println("Invalid humidity reading!");
    return;
  }
  
  // Print to serial if in debug mode
  if (DEBUG_MODE >= 2) {
    Serial.printf("BME680 - Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2fhPa, Gas: %.2fkΩ\n", 
                 bmeData.temperature, bmeData.humidity, bmeData.pressure, bmeData.gas_resistance);
  }
}

void readIRSensors() {
  unsigned long currentTime = millis();
  
  // Limit read frequency to avoid excessive CPU usage
  if (currentTime - irData.lastRead < 50) return; // 50ms minimum between reads
  irData.lastRead = currentTime;
  
  // Check if sensors are initialized
  if (!irData.initialized) return;
  
  static int lastValues1[4] = {0, 0, 0, 0};
  static int lastValues2[4] = {0, 0, 0, 0};
  int currentValues1[4], currentValues2[4];
  bool activity = false;
  
  // Read all sensors from first ADS1115
  for (int i = 0; i < 4; i++) {
    currentValues1[i] = ads1.readADC_SingleEnded(i);
    if (abs(currentValues1[i] - lastValues1[i]) > irData.thresholds1[i]) {
      activity = true;
    }
    lastValues1[i] = currentValues1[i];
  }
  
  // Read all sensors from second ADS1115
  for (int i = 0; i < 4; i++) {
    currentValues2[i] = ads2.readADC_SingleEnded(i);
    if (abs(currentValues2[i] - lastValues2[i]) > irData.thresholds2[i]) {
      activity = true;
    }
    lastValues2[i] = currentValues2[i];
  }
  
  // If activity detected and enough time has passed
  if (activity && (currentTime - irData.lastDetectionTime > DETECTION_DELAY)) {
    irData.activityCount++;
    irData.lastDetectionTime = currentTime;
    
    if (DEBUG_MODE >= 1) {
      Serial.printf("Bee activity detected! Count: %d\n", irData.activityCount);
    }
  }
}

void readLoadCell() {
  unsigned long currentTime = millis();
  
  if (currentTime - weightData.lastRead < MEASURE_INTERVAL) return;
  weightData.lastRead = currentTime;
  
  // Check if sensor is initialized
  if (!weightData.initialized) return;
  
  if (!loadcell.wait_ready_timeout(100)) {
    Serial.println("HX711 not responding!");
    weightData.error = true;
    return;
  }
  
  weightData.error = false;
  
  float sum = 0;
  int validReadings = 0;
  
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    float reading = loadcell.get_units();
    if (reading > -10000 && reading < 10000) {  // Sanity check
      sum += reading;
      validReadings++;
    }
    yield(); // Allow other tasks to run
  }
  
  if (validReadings == 0) return;
  
  float newWeight = sum / validReadings;
  
  // Check for significant changes
  if (abs(newWeight - weightData.lastWeight) > WEIGHT_CHANGE_ALERT) {
    Serial.printf("Significant weight change: %.2f -> %.2f kg\n", 
                 weightData.lastWeight, newWeight);
                 
    // Log alert to Firebase if connected
    if (firebaseConnected) {
      FirebaseJson json;
      json.set("previous_weight", weightData.lastWeight);
      json.set("current_weight", newWeight);
      json.set("change", newWeight - weightData.lastWeight);
      json.set("timestamp", getISOTimestamp());
      
      firebaseSetJSON("alerts/current/weight", json);
      
      // Also log to history
      String historyPath = "alerts/history/" + getISOTimestamp() + "/weight";
      firebaseSetJSON(historyPath, json);
    }
  }
  
  // Update weight values
  weightData.currentWeight = newWeight;
  weightData.lastWeight = weightData.currentWeight;
  
  if (DEBUG_MODE >= 2) {
    Serial.printf("Weight: %.2f kg (Net: %.2f kg)\n", 
                 weightData.currentWeight, weightData.currentWeight - EMPTY_HIVE_WEIGHT);
  }
}

bool checkValidMotion() {
  unsigned long currentTime = millis();
  
  // Add new reading to history every 100ms
  if (currentTime - pirData.lastReadingTime >= 100) {
    pirData.lastReadingTime = currentTime;
    pirData.readingHistory[pirData.readingIndex] = (digitalRead(PIR_PIN) == HIGH);
    pirData.readingIndex = (pirData.readingIndex + 1) % pirData.READING_HISTORY_SIZE;
    
    // Count HIGH readings in history
    int highCount = 0;
    for(int i = 0; i < pirData.READING_HISTORY_SIZE; i++) {
      if(pirData.readingHistory[i]) highCount++;
    }
    
    // Valid motion if enough HIGH readings
    if (highCount >= SAMPLES_REQUIRED) {
      pirData.lastValidMotion = currentTime;
      return true;
    }
  }
  
  // Motion is still valid for a short time after last detection
  return (currentTime - pirData.lastValidMotion < DETECTION_WINDOW);
}

void handleMotion() {
  unsigned long currentTime = millis();
  
  // Limit check frequency
  if (currentTime - pirData.lastRead < 50) return;
  pirData.lastRead = currentTime;
  
  // Check if sensor is initialized
  if (!pirData.initialized) return;
  
  bool validMotion = checkValidMotion();
  
  // Motion started
  if (validMotion && !pirData.motionActive) {
    pirData.motionActive = true;
    pirData.motionStartTime = currentTime;
    digitalWrite(LED_PIN, HIGH);
    
    // Only count as new motion if cooldown passed
    if (currentTime - pirData.lastMotionTime > MOTION_COOLDOWN) {
      pirData.motionCountToday++;
      pirData.lastMotionTime = currentTime;
      
      if (DEBUG_MODE >= 1) {
        Serial.printf("Motion detected! Count today: %d\n", pirData.motionCountToday);
      }
    }
  }
  // Motion ended
  else if (!validMotion && pirData.motionActive) {
    pirData.motionActive = false;
    digitalWrite(LED_PIN, LOW);
    
    // Calculate motion duration
    unsigned long duration = currentTime - pirData.motionStartTime;
    
    // Only log significant motions
    if (duration >= MOTION_DURATION_MIN) {
      // Log to Firebase if connected
      if (firebaseConnected) {
        String timestamp = getISOTimestamp();
        FirebaseJson json;
        
        // Update current motion status
        json.set("active", true);
        json.set("last_detected", timestamp);
        json.set("count_today", pirData.motionCountToday);
        json.set("duration", (int)(duration / 1000));  // Convert to seconds
        firebaseSetJSON("alerts/current/motion", json);
        
        // Log motion event in history
        FirebaseJson eventJson;
        eventJson.set("time", timestamp);
        eventJson.set("duration", (int)(duration / 1000));
        eventJson.set("type", duration > MOTION_DURATION_MIN * 2 ? "significant" : "brief");
        
        char datePath[11];
        if (getLocalTime(&systemState.timeinfo)) {
          strftime(datePath, sizeof(datePath), "%Y-%m-%d", &systemState.timeinfo);
          String path = "alerts/history/" + String(datePath) + "/motion_events";
          firebasePushJSON(path, eventJson);
        }
      }
      
      if (DEBUG_MODE >= 1) {
        Serial.printf("Significant motion ended. Duration: %lu ms\n", duration);
      }
    }
  }
}

void processGPS() {
  unsigned long currentTime = millis();
  
  // Only process GPS at appropriate intervals or when actively acquiring
  if (!gpsData.acquiring && currentTime - gpsData.lastCheck < GPS_CHECK_INTERVAL) {
    return;
  }
  
  // Check if we should start a daily GPS update (once per day)
  if (!gpsData.acquiring && 
      (currentTime - gpsData.lastLocationUpdate > 24*60*60*1000 || gpsData.lastLocationUpdate == 0)) {
    
    // Start GPS acquisition
    gpsData.acquiring = true;
    gpsData.fixStartTime = currentTime;
    gpsData.state = 1; // Acquiring state
    
    if (GPS_POWER_PIN > 0) {
      digitalWrite(GPS_POWER_PIN, HIGH); // Ensure GPS is powered on
    }
    
    Serial.println("\nStarting GPS fix acquisition...");
  }
  
  // Check for timeout if we're acquiring
  if (gpsData.acquiring && currentTime - gpsData.fixStartTime >= GPS_TIMEOUT) {
    Serial.println("GPS fix acquisition timed out!");
    gpsData.acquiring = false;
    gpsData.state = 3; // Timeout state
    gpsData.lastCheck = currentTime;
    
    if (GPS_POWER_PIN > 0 && !gpsData.locationValid) {
      // Only power down if we have no valid location at all
      digitalWrite(GPS_POWER_PIN, LOW); // Power down to save energy
    }
    
    return;
  }
  
  // Process available GPS data
  if (gpsData.acquiring || gpsData.state == 1) {
    static int validReadings = 0;
    static float totalLat = 0;
    static float totalLon = 0;
    static int totalSat = 0;
    
    // Process all available GPS data
    while (GPSSerial.available()) {
      if (gps.encode(GPSSerial.read())) {
        if (gps.location.isValid() && gps.satellites.value() >= MIN_SATELLITES) {
          validReadings++;
          totalLat += gps.location.lat();
          totalLon += gps.location.lng();
          totalSat += gps.satellites.value();
          
          if (DEBUG_MODE >= 1) {
            Serial.printf("Valid GPS Reading %d: Lat: %.6f, Lon: %.6f, Satellites: %d\n",
                       validReadings, gps.location.lat(), gps.location.lng(), gps.satellites.value());
          }
          
          // If we have enough readings, compute the average and finish
          if (validReadings >= VALID_LOCATION_UPDATES) {
            gpsData.latitude = totalLat / validReadings;
            gpsData.longitude = totalLon / validReadings;
            gpsData.satellites = totalSat / validReadings;
            gpsData.locationValid = true;
            gpsData.acquiring = false;
            gpsData.state = 2; // Success state
            
            Serial.println("\nGPS Fix Obtained!");
            Serial.printf("Final Location: %.6f, %.6f\n", gpsData.latitude, gpsData.longitude);
            Serial.printf("Average Satellites: %d\n", gpsData.satellites);
            
            // Reset for next time
            validReadings = 0;
            totalLat = totalLon = totalSat = 0;
            
            return;
          }
        }
      }
    }
    
    // Print status every 5 seconds during acquisition
    static unsigned long lastStatusPrint = 0;
    if (currentTime - lastStatusPrint >= 5000) {
      Serial.printf("GPS: Waiting... Satellites: %d, Valid readings: %d\n", 
                   gps.satellites.value(), validReadings);
      lastStatusPrint = currentTime;
    }
  }
  
  gpsData.lastCheck = currentTime;
}

void readBatteryVoltage() {
  // Take multiple readings for stability
  long sum = 0;
  for (int i = 0; i < BATTERY_READS; i++) {
    sum += analogRead(BATTERY_VOLTAGE_PIN);
    delay(10);
  }
  
  // Calculate average
  float adcValue = sum / BATTERY_READS;
  
  // Convert ADC value to voltage (ESP32 ADC is 12-bit, 0-4095)
  float outputVoltage = adcValue * 3.3 / 4095.0;
  
  // Apply voltage divider formula: Vin = Vout * (R1 + R2) / R2
  float batteryVoltage = outputVoltage * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
  
  // Store values
  batteryData.voltage = batteryVoltage;
  batteryData.percentage = calculateBatteryPercentage(batteryVoltage);
  batteryData.lastRead = millis();
  
  // Check for low battery
  if (batteryData.percentage < BATTERY_LOW_THRESHOLD && !batteryData.lowBattery) {
    batteryData.lowBattery = true;
    Serial.println("WARNING: Low battery detected!");
  } else if (batteryData.percentage > BATTERY_LOW_THRESHOLD + 10 && batteryData.lowBattery) {
    batteryData.lowBattery = false;
    Serial.println("Battery level recovered.");
  }
  
  if (DEBUG_MODE >= 2) {
    Serial.printf("Battery: %.2fV (%.1f%%)\n", batteryData.voltage, batteryData.percentage);
  }
}

float calculateBatteryPercentage(float voltage) {
  float percentage = (voltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0;
  
  // Constrain to 0-100%
  if (percentage > 100.0) percentage = 100.0;
  if (percentage < 0.0) percentage = 0.0;
  
  return percentage;
}

void displayBatteryStatus() {
  float voltage = batteryData.voltage;
  float percentage = batteryData.percentage;
  bool isCharging = batteryData.charging;
  // Or use CHARGING_PIN if you have it: bool isCharging = (digitalRead(CHARGING_PIN) == LOW);

  Serial.println("=== BATTERY STATUS ===");
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.println("V");
  Serial.print("Percentage: ");
  Serial.print(percentage, 1);
  Serial.println("%");
  Serial.print("Status: ");
  if (isCharging) {
    Serial.println("CHARGING");
  } else if (percentage > 50) {
    Serial.println("GOOD");
  } else if (percentage > 20) {
    Serial.println("LOW");
  } else {
    Serial.println("CRITICAL");
  }
  Serial.println("=====================");
}


void checkSolarCharging() {
  static float lastVoltage = 0;
  
  // If voltage is increasing or staying the same with high light conditions, likely charging
  bool charging = batteryData.voltage > lastVoltage + 0.01;  // Voltage increased by at least 0.01V
  
  // Update status if changed
  if (charging != batteryData.charging) {
    batteryData.charging = charging;
    if (DEBUG_MODE >= 1) {
      Serial.printf("Solar charging status changed to: %s\n", charging ? "Charging" : "Not charging");
    }
  }
  
  lastVoltage = batteryData.voltage;
}

void uploadBME680Data() {
  if (!firebaseConnected || bmeData.error) return;
  
  FirebaseJson json;
  String timestamp = getISOTimestamp();
  
  // Add environment data
  json.set("temperature", bmeData.temperature);
  json.set("humidity", bmeData.humidity);
  json.set("pressure", bmeData.pressure);
  json.set("gas_resistance", bmeData.gas_resistance);
  json.set("timestamp", timestamp);
  
  // Update current values
  if (firebaseSetJSON("environment/current", json)) {
    if (DEBUG_MODE >= 1) {
      Serial.println("Environment data uploaded to Firebase!");
    }
  }
  
  // Also log to hourly history
  char datePath[20];
  char hourPath[6];
  if (getLocalTime(&systemState.timeinfo)) {
    strftime(datePath, sizeof(datePath), "%Y-%m-%d", &systemState.timeinfo);
    strftime(hourPath, sizeof(hourPath), "%H:00", &systemState.timeinfo);
    
    String historyPath = "environment/history/" + String(datePath) + "/hourly/" + String(hourPath);
    firebaseSetJSON(historyPath, json);
    
    // Calculate and update daily averages
    String dailyAvgPath = "environment/history/" + String(datePath) + "/daily_average";
    Firebase.RTDB.getJSON(&fbdo, dailyAvgPath);
    
    FirebaseJson avgJson;
    FirebaseJsonData result;
    
    float tempSum = bmeData.temperature;
    float humSum = bmeData.humidity;
    float presSum = bmeData.pressure;
    float gasSum = bmeData.gas_resistance;
    int count = 1;
    
    if (fbdo.httpCode() == 200) {
      if (fbdo.jsonObject().get(result, "count")) {
        count = result.to<int>() + 1;
        
        fbdo.jsonObject().get(result, "temp");
        tempSum += result.to<float>() * (count - 1);
        
        fbdo.jsonObject().get(result, "humidity");
        humSum += result.to<float>() * (count - 1);
        
        fbdo.jsonObject().get(result, "pressure");
        presSum += result.to<float>() * (count - 1);
        
        fbdo.jsonObject().get(result, "gas_resistance");
        gasSum += result.to<float>() * (count - 1);
      }
    }
    
    avgJson.set("temp", tempSum / count);
    avgJson.set("humidity", humSum / count);
    avgJson.set("pressure", presSum / count);
    avgJson.set("gas_resistance", gasSum / count);
    avgJson.set("count", count);
    avgJson.set("last_update", timestamp);
    
    firebaseSetJSON(dailyAvgPath, avgJson);
  }
}

void uploadActivityCount(bool isDaily) {
  if (!firebaseConnected) return;
  
  String timestamp = getISOTimestamp();
  FirebaseJson json;
  
  if (isDaily) {
    // Store daily count in historical data
    char datePath[11];
    strftime(datePath, sizeof(datePath), "%Y-%m-%d", &systemState.timeinfo);
    
    json.set("total_count", irData.activityCount);
    json.set("timestamp", timestamp);
    
    String historyPath = "beeActivity/history/" + String(datePath);
    firebaseSetJSON(historyPath, json);
    
    if (DEBUG_MODE >= 1) {
      Serial.println("Daily bee activity count saved to history!");
    }
  } else {
    // Update current count
    json.set("count", irData.activityCount);
    json.set("timestamp", timestamp);
    
    firebaseSetJSON("beeActivity/current", json);
    
    // Also update hourly counts in daily structure
    char datePath[11];
    char hourPath[6];
    if (getLocalTime(&systemState.timeinfo)) {
      strftime(datePath, sizeof(datePath), "%Y-%m-%d", &systemState.timeinfo);
      strftime(hourPath, sizeof(hourPath), "%H:00", &systemState.timeinfo);
      
      String hourlyPath = "beeActivity/daily/" + String(datePath) + "/hourly_counts/" + String(hourPath);
      FirebaseJson hourlyJson;
      hourlyJson.set("count", irData.activityCount - irData.lastUploadedCount);
      firebaseSetJSON(hourlyPath, hourlyJson);
      
      // Update total count
      FirebaseJson totalJson;
      totalJson.set("total_count", irData.activityCount);
      firebaseSetJSON("beeActivity/daily/" + String(datePath), totalJson);
    }
    
    irData.lastUploadedCount = irData.activityCount;
    irData.lastUploadTime = millis();
    
    if (DEBUG_MODE >= 1) {
      Serial.println("Current bee activity count updated!");
    }
  }
}

void uploadWeight(bool isDaily) {
  if (!firebaseConnected || weightData.currentWeight <= 0) return;
  
  FirebaseJson json;
  String timestamp = getISOTimestamp();
  
  json.set("total_weight", weightData.currentWeight);
  json.set("net_weight", weightData.currentWeight - EMPTY_HIVE_WEIGHT);
  json.set("timestamp", timestamp);
  
  if (isDaily) {
    char datePath[20];
    strftime(datePath, sizeof(datePath), "%Y-%m-%d", &systemState.timeinfo);
    
    // Store daily average
    String historyPath = "weight/history/" + String(datePath);
    firebaseSetJSON(historyPath + "/daily_average", json);
    
    // Add measurement to the day's measurements
    char timePath[6];
    strftime(timePath, sizeof(timePath), "%H:00", &systemState.timeinfo);
    
    FirebaseJson timeJson;
    timeJson.set(timePath, weightData.currentWeight);
    firebaseUpdateNode(historyPath + "/measurements", timeJson);
    
    if (DEBUG_MODE >= 1) {
      Serial.println("Daily weight data saved to history!");
    }
  } else {
    // Update current weight
    firebaseSetJSON("weight/current", json);
    if (DEBUG_MODE >= 1) {
      Serial.println("Current weight updated!");
    }
  }
}

void uploadLocation() {
  if (!firebaseConnected || !gpsData.locationValid) return;
  
  FirebaseJson json;
  String timestamp = getISOTimestamp();
  
  json.set("latitude", gpsData.latitude);
  json.set("longitude", gpsData.longitude);
  json.set("satellites", gpsData.satellites);
  json.set("timestamp", timestamp);
  
  // Update current location
  firebaseSetJSON("location/current", json);
  
  // Store in daily history
  char dateStr[11];
  if (getLocalTime(&systemState.timeinfo)) {
    strftime(dateStr, sizeof(dateStr), "%Y-%m-%d", &systemState.timeinfo);
    String historyPath = "location/history/" + String(dateStr);
    
    firebaseSetJSON(historyPath, json);
    if (DEBUG_MODE >= 1) {
      Serial.println("Location data updated and saved to history!");
    }
  }
}

void updateSystemStatus() {
  if (!firebaseConnected) return;
  
  FirebaseJson statusJson;
  statusJson.set("last_update", getISOTimestamp());
  statusJson.set("uptime", systemState.uptimeSeconds);
  statusJson.set("wifi_strength", WiFi.RSSI());
  
  // Sensor status
  FirebaseJson sensorStatus;
  
  // Check BME680
  Wire.beginTransmission(BME_I2C_ADDR);
  sensorStatus.set("bme680", Wire.endTransmission() == 0 ? "ok" : "error");
  
  // Check ADS1115 modules
  Wire.beginTransmission(0x48);
  sensorStatus.set("ir_sensors_1", Wire.endTransmission() == 0 ? "ok" : "error");
  
  Wire.beginTransmission(0x49);
  sensorStatus.set("ir_sensors_2", Wire.endTransmission() == 0 ? "ok" : "error");
  
  // Check HX711
  sensorStatus.set("loadcell", loadcell.wait_ready_timeout(100) ? "ok" : "error");
  
  // Check GPS
  sensorStatus.set("gps", gpsData.locationValid ? "ok" : "no_fix");
  
  statusJson.set("sensors", sensorStatus);
  
  // Power status
  FirebaseJson powerJson;
  powerJson.set("voltage", batteryData.voltage);
  powerJson.set("percentage", batteryData.percentage);
  powerJson.set("charging", batteryData.charging);
  powerJson.set("timestamp", getISOTimestamp());
  
  // Update status
  firebaseSetJSON("system/status", statusJson);
  firebaseSetJSON("system/power/current", powerJson);
  
  // Update power history
  char datePath[11];
  if (getLocalTime(&systemState.timeinfo)) {
    strftime(datePath, sizeof(datePath), "%Y-%m-%d", &systemState.timeinfo);
    String historyPath = "system/power/history/" + String(datePath);
    
    // Get existing power history or create new
    Firebase.RTDB.getJSON(&fbdo, historyPath);
    
    FirebaseJson historyJson;
    if (fbdo.httpCode() == 200) {
      FirebaseJsonData data;
      
      float minV = batteryData.voltage;
      fbdo.jsonObject().get(data, "min_voltage");
      if (data.success) minV = min(minV, data.to<float>());
      
      float maxV = batteryData.voltage;
      fbdo.jsonObject().get(data, "max_voltage");
      if (data.success) maxV = max(maxV, data.to<float>());
      
      float sum = batteryData.voltage;
      int count = 1;
      fbdo.jsonObject().get(data, "sum");
      if (data.success) sum += data.to<float>();
      
      fbdo.jsonObject().get(data, "count");
      if (data.success) count += data.to<int>();
      
      historyJson.set("min_voltage", minV);
      historyJson.set("max_voltage", maxV);
      historyJson.set("sum", sum);
      historyJson.set("count", count);
      historyJson.set("average", sum / count);
    } else {
      historyJson.set("min_voltage", batteryData.voltage);
      historyJson.set("max_voltage", batteryData.voltage);
      historyJson.set("sum", batteryData.voltage);
      historyJson.set("count", 1);
      historyJson.set("average", batteryData.voltage);
    }
    
    firebaseSetJSON(historyPath, historyJson);
  }
  
  if (DEBUG_MODE >= 1) {
    Serial.println("System status updated!");
  }
}

void checkDailyReset() {
  if(!getLocalTime(&systemState.timeinfo)) {
    return;
  }
  
  time_t now;
  time(&now);
  
  // If it's a new day (past midnight) and we haven't reset yet
  if (systemState.timeinfo.tm_hour == 0 && systemState.timeinfo.tm_min == 0 && 
      difftime(now, systemState.lastResetTime) > 3600) { // Ensure at least 1 hour has passed
    
    // Save daily data before resetting
    uploadActivityCount(true);
    uploadWeight(true);
    
    // Reset counters
    irData.activityCount = 0;
    irData.lastUploadedCount = 0;
    pirData.motionCountToday = 0;
    
    // Update reset time
    time(&systemState.lastResetTime);
    
    // Reset motion status in Firebase
    if (firebaseConnected) {
      FirebaseJson json;
      json.set("count_today", 0);
      json.set("active", false);
      firebaseSetJSON("alerts/current/motion", json);
    }
    
    Serial.println("Daily counters reset completed");
  }
}

void checkDeepSleepConditions() {
  // Check if it's safe to enter sleep mode
  // Only check every 5 minutes
  if (millis() - systemState.lastDeepSleepCheck < 300000) return;
  systemState.lastDeepSleepCheck = millis();
  
  // Don't sleep if we're in the middle of important operations
  if (pirData.motionActive || gpsData.acquiring) return;
  
  // Enter low power mode if battery is below threshold
  if (batteryData.percentage < BATTERY_LOW_THRESHOLD && !batteryData.charging) {
    systemState.lowPowerMode = true;
    Serial.println("Entering low power mode due to low battery!");
    
    // Update status in Firebase
    if (firebaseConnected) {
      FirebaseJson json;
      json.set("low_battery", true);
      json.set("last_warning", getISOTimestamp());
      firebaseSetJSON("alerts/current/battery", json);
    }
  } else if (batteryData.percentage > BATTERY_LOW_THRESHOLD + 10 && systemState.lowPowerMode) {
    // Exit low power mode if battery recovered
    systemState.lowPowerMode = false;
    Serial.println("Exiting low power mode, battery recovered!");
    
    // Update status in Firebase
    if (firebaseConnected) {
      FirebaseJson json;
      json.set("low_battery", false);
      firebaseSetJSON("alerts/current/battery", json);
    }
  }
}

void calibrateLoadCell() {
  Serial.println("\n*** Starting Load Cell Calibration ***");
  Serial.println("1. Remove all weight and press Enter");
  
  while (!Serial.available()) {
    feedWatchdog();
    delay(100);
  }
  Serial.read();
  
  loadcell.tare();
  Serial.println("Scale tared to zero");
  delay(1000);
  
  Serial.println("2. Place a known weight (e.g., 1kg) and enter the weight in kg:");
  String input = "";
  while (!Serial.available()) {
    feedWatchdog();
    delay(100);
  }
  
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') break;
    input += c;
    delay(10);
  }
  
  float calibrationWeight = input.toFloat();
  if (calibrationWeight <= 0) {
    Serial.println("Invalid weight! Calibration aborted.");
    return;
  }
  
  long rawValue = loadcell.read_average(10);
  float newCalibrationValue = rawValue / calibrationWeight;
  
  // Validity check
  if (newCalibrationValue < 100 || newCalibrationValue > 10000) {
    Serial.println("Calibration value out of expected range! Calibration aborted.");
    return;
  }
  
  loadcell.set_scale(newCalibrationValue);
  weightData.calibrationValue = newCalibrationValue;
  
  // Verify calibration
  float verificationWeight = loadcell.get_units(10);
  Serial.printf("Measured weight: %.2fkg (should be close to %.2fkg)\n", 
               verificationWeight, calibrationWeight);
  Serial.printf("Calibration value: %.1f\n", newCalibrationValue);
  
  // Save to EEPROM
  saveCalibrationData();
}

void calibrateIRSensors() {
  Serial.println("\n*** Starting IR Sensor Calibration ***");
  Serial.println("Place a white/reflective object ~2cm below sensors and press Enter");
  
  while (!Serial.available()) {
    feedWatchdog();
    delay(100);
  }
  Serial.read();
  
  // Record baseline high values when no bee is present
  int highValues1[4], highValues2[4];
  for (int i = 0; i < 4; i++) {
    highValues1[i] = 0;
    highValues2[i] = 0;
    
    // Take multiple readings for accuracy
    for (int j = 0; j < 10; j++) {
      highValues1[i] += ads1.readADC_SingleEnded(i);
      highValues2[i] += ads2.readADC_SingleEnded(i);
      delay(50);
    }
    
    highValues1[i] /= 10;
    highValues2[i] /= 10;
  }
  
  Serial.println("Recorded high values (white/reflective):");
  for (int i = 0; i < 4; i++) {
    Serial.printf("  Sensor 1-%d: %d, Sensor 2-%d: %d\n", 
                 i+1, highValues1[i], i+1, highValues2[i]);
  }
  
  Serial.println("\nPlace a dark/non-reflective object ~2cm below sensors and press Enter");
  while (!Serial.available()) {
    feedWatchdog();
    delay(100);
  }
  Serial.read();
  
  // Record baseline low values when object is present
  int lowValues1[4], lowValues2[4];
  for (int i = 0; i < 4; i++) {
    lowValues1[i] = 0;
    lowValues2[i] = 0;
    
    // Take multiple readings for accuracy
    for (int j = 0; j < 10; j++) {
      lowValues1[i] += ads1.readADC_SingleEnded(i);
      lowValues2[i] += ads2.readADC_SingleEnded(i);
      delay(50);
    }
    
    lowValues1[i] /= 10;
    lowValues2[i] /= 10;
  }
  
  Serial.println("Recorded low values (dark/non-reflective):");
  for (int i = 0; i < 4; i++) {
    Serial.printf("  Sensor 1-%d: %d, Sensor 2-%d: %d\n", 
                 i+1, lowValues1[i], i+1, lowValues2[i]);
  }
  
  // Calculate thresholds (midpoint between high and low, adjusted for sensitivity)
  for (int i = 0; i < 4; i++) {
    int diff1 = abs(highValues1[i] - lowValues1[i]);
    int diff2 = abs(highValues2[i] - lowValues2[i]);
    
    // Set threshold to 40% of the difference to be more sensitive
    irData.thresholds1[i] = diff1 * 0.4;
    irData.thresholds2[i] = diff2 * 0.4;
    
    // Ensure minimum threshold to avoid noise
    if (irData.thresholds1[i] < 500) irData.thresholds1[i] = 500;
    if (irData.thresholds2[i] < 500) irData.thresholds2[i] = 500;
  }
  
  Serial.println("\nCalculated thresholds:");
  for (int i = 0; i < 4; i++) {
    Serial.printf("  Sensor 1-%d: %d, Sensor 2-%d: %d\n", 
                 i+1, irData.thresholds1[i], i+1, irData.thresholds2[i]);
  }
  
  // Save to EEPROM
  saveCalibrationData();
  
  Serial.println("\nIR sensor calibration complete!");
  Serial.println("Move objects below sensors to test detection.");
}

bool firebaseSetJSON(const String& path, FirebaseJson& json, int maxRetries) {
  if (!firebaseConnected) return false;
  
  bool success = false;
  int attempts = 0;
  
  while (!success && attempts < maxRetries) {
    if (Firebase.RTDB.setJSON(&fbdo, path, &json)) {
      success = true;
    } else {
      Serial.printf("Firebase setJSON failed: %s, attempt %d/%d\n", 
                   fbdo.errorReason().c_str(), attempts + 1, maxRetries);
      attempts++;
      delay(FIREBASE_RETRY_INTERVAL);
    }
  }
  
  return success;
}

bool firebaseUpdateNode(const String& path, FirebaseJson& json, int maxRetries) {
  if (!firebaseConnected) return false;
  
  bool success = false;
  int attempts = 0;
  
  while (!success && attempts < maxRetries) {
    if (Firebase.RTDB.updateNode(&fbdo, path, &json)) {
      success = true;
    } else {
      Serial.printf("Firebase updateNode failed: %s, attempt %d/%d\n", 
                   fbdo.errorReason().c_str(), attempts + 1, maxRetries);
      attempts++;
      delay(FIREBASE_RETRY_INTERVAL);
    }
  }
  
  return success;
}

bool firebasePushJSON(const String& path, FirebaseJson& json, int maxRetries) {
  if (!firebaseConnected) return false;
  
  bool success = false;
  int attempts = 0;
  
  while (!success && attempts < maxRetries) {
    if (Firebase.RTDB.pushJSON(&fbdo, path, &json)) {
      success = true;
    } else {
      Serial.printf("Firebase pushJSON failed: %s, attempt %d/%d\n", 
                   fbdo.errorReason().c_str(), attempts + 1, maxRetries);
      attempts++;
      delay(FIREBASE_RETRY_INTERVAL);
    }
  }
  
  return success;
}

void printSystemStatus() {
  Serial.println("\n=== System Status ===");
  Serial.print("Time: ");
  Serial.println(getISOTimestamp());
  
  Serial.println("\nSensor Readings:");
  Serial.printf("  BME680:  Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2fhPa, Gas: %.2fkΩ\n", 
               bmeData.temperature, bmeData.humidity, bmeData.pressure, bmeData.gas_resistance);
  Serial.printf("  Activity: %d bee movements today\n", irData.activityCount);
  Serial.printf("  Weight:   %.2fkg (Net: %.2fkg)\n", weightData.currentWeight, 
               weightData.currentWeight - EMPTY_HIVE_WEIGHT);
  Serial.printf("  Motion:   %d detections today, Currently %s\n", 
               pirData.motionCountToday, pirData.motionActive ? "ACTIVE" : "inactive");
  
  if (gpsData.locationValid) {
    Serial.printf("  Location: %.6f, %.6f (Satellites: %d)\n", 
                 gpsData.latitude, gpsData.longitude, gpsData.satellites);
  } else {
    Serial.printf("  Location: No valid GPS fix, Status: %s\n", 
                 gpsData.state == 1 ? "Acquiring" : (gpsData.state == 3 ? "Timeout" : "Idle"));
  }
  
  Serial.println("\nPower Status:");
  Serial.printf("  Battery: %.2fV (%.1f%%) - %s\n", 
               batteryData.voltage, batteryData.percentage, batteryData.lowBattery ? "LOW" : "OK");
  Serial.printf("  Charging: %s\n", batteryData.charging ? "Yes" : "No");
  Serial.printf("  Power Mode: %s\n", systemState.lowPowerMode ? "LOW POWER" : "Normal");
  
  Serial.println("\nConnectivity:");
  Serial.printf("  WiFi: %s (RSSI: %ddBm)\n", 
               WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected", WiFi.RSSI());
  Serial.printf("  Firebase: %s\n", firebaseConnected ? "Connected" : "Disconnected");
  
  Serial.printf("  System uptime: %lu minutes\n", systemState.uptimeSeconds / 60);
  
  Serial.println("\nSensor Health:");
  Serial.printf("  BME680: %s\n", bmeData.error ? "ERROR" : "OK");
  Serial.printf("  IR Sensors: %s\n", irData.error ? "ERROR" : "OK");
  Serial.printf("  Load Cell: %s\n", weightData.error ? "ERROR" : "OK");
  Serial.printf("  PIR Sensor: %s\n", pirData.error ? "ERROR" : "OK");
  Serial.printf("  GPS Module: %s\n", gpsData.error ? "ERROR" : "OK");
  
  Serial.println("=====================\n");
}

void testAllSensors() {
  Serial.println("\n=== Starting Sensor Tests ===");
  
  // Test BME680
  Serial.print("BME680 test: ");
  Wire.beginTransmission(BME_I2C_ADDR);
  byte error = Wire.endTransmission();
  if (error == 0 && bme.performReading()) {
    Serial.println("PASS");
    Serial.printf("  Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2fhPa, Gas: %.2fkΩ\n", 
                 bme.temperature, bme.humidity, bme.pressure / 100.0, bme.gas_resistance / 1000.0);
    bmeData.error = false;
  } else {
    Serial.println("FAIL - Check wiring");
    bmeData.error = true;
  }
  
  // Test ADS1115 #1
  Serial.print("ADS1115 #1 test: ");
  Wire.beginTransmission(0x48);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("PASS");
    Serial.print("  Readings: ");
    for (int i = 0; i < 4; i++) {
      Serial.printf("A%d=%d ", i, ads1.readADC_SingleEnded(i));
    }
    Serial.println();
    irData.error = false;
  } else {
    Serial.println("FAIL - Check wiring");
    irData.error = true;
  }
  
  // Test ADS1115 #2
  Serial.print("ADS1115 #2 test: ");
  Wire.beginTransmission(0x49);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("PASS");
    Serial.print("  Readings: ");
    for (int i = 0; i < 4; i++) {
      Serial.printf("A%d=%d ", i, ads2.readADC_SingleEnded(i));
    }
    Serial.println();
    irData.error = false;
  } else {
    Serial.println("FAIL - Check wiring");
    irData.error = true;
  }
  
  // Test HX711
  Serial.print("HX711 test: ");
  if (loadcell.wait_ready_timeout(1000)) {
    Serial.println("PASS");
    Serial.printf("  Current reading: %.2f kg\n", loadcell.get_units(5));
    weightData.error = false;
  } else {
    Serial.println("FAIL - Check wiring");
    weightData.error = true;
  }
  
  // Test PIR
  Serial.print("PIR sensor test: ");
  int pirValue = digitalRead(PIR_PIN);
  Serial.println("PASS");
  Serial.printf("  Current state: %s\n", pirValue ? "Motion detected" : "No motion");
  pirData.error = false;
  
  // Test GPS
  Serial.print("GPS test: ");
  unsigned long startTime = millis();
  int bytesReceived = 0;
  
  while (millis() - startTime < 2000) {  // Check for 2 seconds
    while (GPSSerial.available()) {
      GPSSerial.read();
      bytesReceived++;
    }
    yield();
  }
  
  if (bytesReceived > 0) {
    Serial.println("PASS");
    Serial.printf("  Received %d bytes in 2 seconds\n", bytesReceived);
    Serial.printf("  Last valid location: %s (Lat: %.6f, Lon: %.6f)\n", 
                 gpsData.locationValid ? "Valid" : "Invalid", gpsData.latitude, gpsData.longitude);
    gpsData.error = false;
  } else {
    Serial.println("FAIL - Check wiring or antenna");
    gpsData.error = true;
  }
  
  // Test battery voltage
  Serial.print("Battery voltage test: ");
  readBatteryVoltage();
  if (batteryData.voltage > 2.5 && batteryData.voltage < 4.5) {
    Serial.println("PASS");
    Serial.printf("  Voltage: %.2fV (%.1f%%)\n", batteryData.voltage, batteryData.percentage);
  } else {
    Serial.println("WARNING - Unusual reading, check voltage divider");
    Serial.printf("  Voltage: %.2fV\n", batteryData.voltage);
  }
  
  Serial.println("=== Sensor Tests Complete ===\n");
}

void handleSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'h': // Help
        Serial.println("\nAvailable commands:");
        Serial.println("h - Help");
        Serial.println("s - System status");
        Serial.println("r - Reset counters");
        Serial.println("t - Tare load cell");
        Serial.println("c - Calibrate load cell");
        Serial.println("i - Calibrate IR sensors");
        Serial.println("g - Get GPS fix");
        Serial.println("u - Upload all current data");
        Serial.println("w - WiFi reconnect");
        Serial.println("f - Firebase reconnect");
        Serial.println("b - Battery check");  // can either use 'b' to read voltage or 'B' to display status
        Serial.println("B - Show battery status");    // can either use 'B' to read voltage or 'b' to display status
        Serial.println("z - Test sensors");
        break;
        
      case 's': // Status
        printSystemStatus();
        break;
        
      case 'r': // Reset counters
        irData.activityCount = 0;
        irData.lastUploadedCount = 0;
        pirData.motionCountToday = 0;
        Serial.println("Counters reset to zero");
        break;
        
      case 't': // Tare
        loadcell.tare();
        Serial.println("Load cell tared to zero");
        break;
        
      case 'c': // Calibrate load cell
        calibrateLoadCell();
        break;
        
      case 'i': // Calibrate IR
        calibrateIRSensors();
        break;
        
      case 'g': // GPS fix
        if (!gpsData.acquiring) {
          gpsData.acquiring = true;
          gpsData.fixStartTime = millis();
          gpsData.state = 1; // Start acquisition
          Serial.println("Starting GPS fix acquisition...");
          
          if (GPS_POWER_PIN > 0) {
            digitalWrite(GPS_POWER_PIN, HIGH);
          }
        } else {
          Serial.println("GPS acquisition already in progress.");
        }
        break;
        
      case 'u': // Upload all
        if (firebaseConnected) {
          uploadBME680Data();
          uploadActivityCount();
          uploadWeight();
          if (gpsData.locationValid) {
            uploadLocation();
          }
          updateSystemStatus();
          Serial.println("All data uploaded to Firebase");
        } else {
          Serial.println("Firebase not connected. Connect first with 'f' command");
        }
        break;
        
      case 'w': // WiFi reconnect
        WiFi.disconnect();
        delay(1000);
        setupWiFi();
        break;
        
      case 'f': // Firebase reconnect
        if (WiFi.status() == WL_CONNECTED) {
          setupFirebase();
        } else {
          Serial.println("WiFi not connected. Connect WiFi first with 'w' command");
        }
        break;
        
      case 'b': // Battery check, you can also use 'B' to display status
        readBatteryVoltage();
        checkSolarCharging();
        Serial.printf("Battery: %.2fV (%.1f%%)\n", batteryData.voltage, batteryData.percentage);
        Serial.printf("Charging: %s\n", batteryData.charging ? "Yes" : "No");
        
        if (batteryData.percentage < 20.0) {
          Serial.println("WARNING: Low battery!");
        }
        break;
        
        case 'B': // Show battery status you can also use 'b' to read voltage
          displayBatteryStatus();
        break;


      case 'z': // Test sensors
        testAllSensors();
        break;
    }
    
    // Clear any remaining characters
    while (Serial.available()) {
      Serial.read();
    }
  }
}