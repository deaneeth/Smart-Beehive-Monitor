# Firebase Setup Guide

This document provides instructions for setting up Firebase Realtime Database for the Smart Beehive Monitoring System.

## Prerequisites

- A Google account
- ESP32 with the Smart Beehive Monitoring System code
- Web dashboard code (if using the companion dashboard)

## 1. Create a Firebase Project

1. Go to [Firebase Console](https://console.firebase.google.com/)
2. Click **Add project**
3. Enter a project name (e.g., "Smart-Beehive-Monitor")
4. Optional: Enable Google Analytics
5. Click **Create project**

## 2. Set up Realtime Database

1. In your Firebase project console, navigate to **Build > Realtime Database**
2. Click **Create Database**
3. Choose a location close to your deployment
4. Start in **test mode** for development (we'll secure it later)
5. Click **Enable**

## 3. Get Firebase Credentials

1. Go to **Project Settings** (gear icon in top left)
2. Navigate to the **Service accounts** tab
3. For ESP32, you need:
   - **Database URL**: Found in the Realtime Database section
   - **API Key**: Found in the Web API Key section

## 4. Configure ESP32 Code

Update the following constants in your ESP32 code (`src/main.cpp`):

```cpp
// Firebase credentials
#define API_KEY "YourAPIKey"         // Example: "AIzaSyA1BCyDExaMpLe123456789"
#define DATABASE_URL "YourDBURL"      // Example: "smart-beehive-monitor-default-rtdb.firebaseio.com"

// WiFi credentials
#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourPassword"

## 5. Database Structure 

The Smart Beehive Monitoring System uses the following database structure:

smart-beehive/
├─ environment/
│  ├─ current/
│  │  ├─ temperature: 24.5
│  │  ├─ humidity: 65.3
│  │  ├─ pressure: 1012.4
│  │  ├─ gasResistance: 45.2
│  │  └─ timestamp: "2025-04-04_15:30:00"
│  └─ history/
│     └─ 2025-04-04/
│        ├─ hourly/
│        │  ├─ "15:00": { temp: 24.5, humidity: 65.3, ... }
│        │  └─ "16:00": { temp: 25.1, humidity: 64.8, ... }
│        └─ daily_average: { temp: 24.8, humidity: 65.0, ... }
│
├─ beeActivity/
│  ├─ current/
│  │  ├─ count: 2356
│  │  └─ timestamp: "2025-04-04_15:30:00"
│  ├─ daily/
│  │  └─ 2025-04-04/
│  │     ├─ total_count: 8945
│  │     ├─ peak_hour: "14:00"
│  │     └─ hourly_counts: {
│  │        "13:00": 2345,
│  │        "14:00": 3456,
│  │        ...
│  │     }
│  └─ history/
│     ├─ 2025-04-03: { total: 7845, peak: "15:00" }
│     └─ 2025-04-04: { total: 8945, peak: "14:00" }
│
├─ weight/
│  ├─ current/
│  │  ├─ total_weight: 42.5
│  │  ├─ net_weight: 37.5  // minus hive weight
│  │  └─ timestamp: "2025-04-04_15:30:00"
│  ├─ calibration/
│  │  ├─ tare_weight: 5.0
│  │  └─ calibration_factor: -600.0
│  └─ history/
│     └─ 2025-04-04/
│        ├─ daily_average: 42.3
│        └─ measurements: {
│           "00:00": 42.1,
│           "06:00": 42.3,
│           ...
│        }
│
├─ location/
│  ├─ current/
│  │  ├─ latitude: 37.7749
│  │  ├─ longitude: -122.4194
│  │  ├─ altitude: 12.5
│  │  ├─ satellites: 8
│  │  └─ timestamp: "2025-04-04_15:30:00"
│  └─ history/
│     └─ 2025-04-04: {
│        lat: 37.7749,
│        lng: -122.4194,
│        ...
│     }
│
├─ system/
│  ├─ power/
│  │  ├─ current/
│  │  │  ├─ voltage: 7.8
│  │  │  ├─ percentage: 85.3
│  │  │  ├─ charging: true
│  │  │  └─ timestamp: "2025-04-04_15:30:00"
│  │  └─ history/
│  │     └─ 2025-04-04: {
│  │        min_voltage: 7.6,
│  │        max_voltage: 8.2,
│  │        average: 7.9
│  │     }
│  └─ status/
│     ├─ last_update: "2025-04-04_15:30:00"
│     ├─ uptime: 345600
│     ├─ wifi_strength: -67
│     └─ sensors/
│        ├─ bme680: "ok"
│        ├─ loadcell: "ok"
│        ├─ ir_sensors: "ok"
│        └─ gps: "ok"
│
└─ alerts/
   ├─ current/
   │  ├─ motion/
   │  │  ├─ active: true
   │  │  ├─ last_detected: "2025-04-04_15:30:00"
   │  │  └─ count_today: 5
   │  ├─ battery/
   │  │  ├─ low_battery: false
   │  │  └─ last_warning: null
   │  └─ system/
   │     ├─ sensor_errors: false
   │     └─ wifi_status: "connected"
   └─ history/
      └─ 2025-04-04/
         ├─ motion_events: [
         │  {
         │    time: "15:30:00",
         │    duration: 45,
         │    type: "significant"
         │  }
         ]
         └─ system_events: [
            {
              time: "12:00:00",
              type: "low_battery",
              resolved: true
            }
         ]

This structure is automatically created by the ESP32 code.

## 6. Security Rules

After development, secure your database with appropriate rules:

1. Go to Realtime Database > Rules tab
2. Set up rules to restrict access. Example:

{
  "rules": {
    ".read": "auth != null",
    ".write": "auth != null",
    "smart-beehive": {
      "environment": {
        "current": {
          ".read": true,
          ".write": "auth != null"
        },
        "history": {
          ".read": true,
          ".write": "auth != null"
        }
      },
      // Similar rules for other nodes
    }
  }
}

For public dashboards without authentication, you can set read permissions to true and restrict write permissions.

## 7. Web Dashboard Integration

If using the companion web dashboard, create a .env.local file with:

NEXT_PUBLIC_FIREBASE_API_KEY=YourAPIKey
NEXT_PUBLIC_FIREBASE_AUTH_DOMAIN=your-project-id.firebaseapp.com
NEXT_PUBLIC_FIREBASE_DATABASE_URL=https://your-project-id-default-rtdb.firebaseio.com
NEXT_PUBLIC_FIREBASE_PROJECT_ID=your-project-id
NEXT_PUBLIC_FIREBASE_STORAGE_BUCKET=your-project-id.appspot.com
NEXT_PUBLIC_FIREBASE_MESSAGING_SENDER_ID=your-messaging-sender-id
NEXT_PUBLIC_FIREBASE_APP_ID=your-app-id

## 8. Firebase Storage (Optional)

If you plan to store images or larger files:

1. Go to Build > Storage
2. Click Get Started
3. Choose security rules
4. Follow the prompts to complete setup

## 9. Firebase Authentication (Optional)

For secured dashboards:

1. Go to Build > Authentication
2. Click Get Started
3. Enable desired sign-in methods (Email/Password is recommended for simplicity)
4. Update security rules to use authentication

## 10. Troubleshooting

1. Connection Issues: Verify API key and Database URL are correct
2. Permission Denied: Check security rules configuration
3. Data Not Appearing: Verify WiFi connection on ESP32 and check serial output for errors