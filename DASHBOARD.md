# Web Dashboard Interface

The Smart Beehive Monitoring System includes a companion web dashboard for visualizing and analyzing sensor data.

<p align="center">
  <img src="assets/dashboard-preview.png" alt="Dashboard Overview" width="850">
</p>

## Features

- Real-time data visualization of all beehive sensors
- Historical data analysis with trend charts
- Bee activity patterns visualization
- Weight tracking and honey production estimates
- Temperature and humidity correlation analysis
- Predator alert notifications and history
- Mobile-responsive design for monitoring on any device

## Dashboard Repository

The dashboard is maintained in a separate repository for better organization and independent development:

[Smart Beehive Monitor System Dashboard Repository](https://github.com/deaneeth/Smart-Beehive-Monitor-System-Dashboard)

## Integration

The web dashboard connects to the same Firebase Realtime Database that the ESP32 hardware uploads data to. This creates a seamless data flow:

1. ESP32 collects sensor data → 
2. Data is uploaded to Firebase → 
3. Dashboard retrieves and visualizes the data

<p align="center">
  <img src="assets/data-flow-diagram.png" alt="Data Flow" width="700">
</p>

## Key Screens

### Environmental Monitoring
<p align="center">
  <img src="assets/dashboard-environment.png" alt="Environment Screen" width="850">
</p>

### Bee Activity Analysis
<p align="center">
  <img src="assets/dashboard-activity.png" alt="Activity Screen" width="850">
</p>

### Weight Tracking
<p align="center">
  <img src="assets/dashboard-weight.png" alt="Weight Screen" width="850">
</p>

### Real-Time Location
<p align="center">
  <img src="assets/real-time-location.png" alt="Real-Time Location" width="850">
</p>

### System Status
<p align="center">
  <img src="assets/system-status.png" alt="System Status" width="850">
</p>

## Quick Setup

For detailed setup instructions, please refer to the dashboard repository. In brief:

1. Clone the dashboard repository
2. Install dependencies (`npm install`)
3. Configure Firebase credentials in `.env.local`
4. Run the development server (`npm run dev`)

See [FIREBASE.md](FIREBASE.md) for details on creating and configuring the Firebase project that connects both systems.