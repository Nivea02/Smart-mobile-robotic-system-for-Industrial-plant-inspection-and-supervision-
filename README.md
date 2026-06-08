Smart Mobile Robotic System
ESP32-based autonomous inspection robot with LiDAR sensing, multi-parameter environmental monitoring, and real-time YOLO hazard detection.
� � � � � �
Problem Statement
Manual inspection of confined, hazardous, or hard-to-reach spaces poses significant safety risks to personnel. Static monitoring systems lack mobility and real-time response capability. This project addresses these limitations by developing an autonomous mobile robot that navigates enclosed environments, maps surroundings using LiDAR, monitors environmental parameters, and identifies hazards using onboard AI — without requiring human entry.
System Overview
The robot integrates three core subsystems:
Navigation & Mapping — VL53L0X LiDAR provides real-time obstacle detection and distance measurement up to 2m, enabling autonomous path planning in constrained spaces.
Environmental Sensing — DHT11 sensor continuously monitors temperature and humidity; power telemetry tracks system health.
Hazard Detection — A lightweight YOLO-based object detection model runs onboard, classifying hazards from a live camera feed with approximately 80% identification accuracy.
System Architecture
┌─────────────────────────────────────────────────┐
│                   ESP32 Controller               │
│                                                 │
│  ┌──────────────┐    ┌──────────────────────┐  │
│  │ VL53L0X      │    │  Camera Module        │  │
│  │ LiDAR (2m)   │    │  (640×480 feed)       │  │
│  └──────┬───────┘    └──────────┬───────────┘  │
│         │                       │               │
│  ┌──────▼───────┐    ┌──────────▼───────────┐  │
│  │ Obstacle Map │    │  YOLO Inference       │  │
│  │ & Navigation │    │  (~80% accuracy)      │  │
│  └──────┬───────┘    └──────────┬───────────┘  │
│         │                       │               │
│  ┌──────▼───────┐    ┌──────────▼───────────┐  │
│  │ DHT11        │    │  Hazard Alert         │  │
│  │ Temp/Humidity│    │  Output               │  │
│  └──────────────┘    └───────────────────────┘  │
│                                                 │
│         Power Telemetry (continuous)            │
└─────────────────────────────────────────────────┘
Hardware Components
Component
Role
Specification
ESP32
Main microcontroller
240 MHz dual-core, Wi-Fi + BT
VL53L0X
Distance / obstacle sensing
Range up to 2m, I2C interface
DHT11
Temperature & humidity sensing
±2°C / ±5% RH accuracy
Camera Module
Object detection input
640×480 @ 30fps
Power sensor
System health telemetry
Voltage & current monitoring
Motor driver
Mobility control
DC motor interface
Key Results
Metric
Value
Object detection accuracy
~80% across test scenarios
LiDAR obstacle detection range
Up to 2m
Inference type
On-device (no cloud dependency)
Test environment
Indoor / lab
Power monitoring
Real-time onboard telemetry
Project Structure
smart-mobile-robotic-system/
├── README.md
├── assets/
│   ├── robot_photo.jpg          ← Add your robot photo here
│   ├── system_architecture.png  ← Add your block diagram here
│   └── demo.gif                 ← Add a demo GIF if available
├── src/
│   ├── main.ino                 ← ESP32 main controller code
│   ├── lidar.ino                ← VL53L0X LiDAR interface
│   ├── sensors.ino              ← DHT11 & power telemetry
│   └── detection.py             ← YOLO inference script
├── docs/
│   └── wiring_diagram.pdf       ← Add your wiring diagram here
└── requirements.txt             ← Python dependencies
Setup & Installation
Hardware Setup
Connect VL53L0X to ESP32 via I2C (SDA → GPIO21, SCL → GPIO22)
Connect DHT11 data pin → GPIO4
Connect camera module via appropriate interface
Connect motor driver to GPIO pins (refer to wiring diagram in /docs)
Software Setup
ESP32 firmware:
# Open src/main.ino in Arduino IDE
# Install required libraries:
#   - VL53L0X by Pololu
#   - DHT sensor library by Adafruit
# Select board: ESP32 Dev Module
# Upload to device
YOLO inference (Python):
# Install dependencies
pip install -r requirements.txt

# Run detection
python src/detection.py
Limitations & Future Work
Current limitations:
Detection accuracy degrades in low-light conditions
YOLO model trained on limited hazard classes
LiDAR range constrained to 2m — insufficient for large open spaces
Single-camera setup creates blind spots
Planned improvements:
Integrate IR illumination for low-light performance
Expand training dataset with domain-specific hazard classes
Add multi-sensor LiDAR array for wider coverage
Implement SLAM for autonomous navigation
Wi-Fi based remote monitoring dashboard via ESP32
About
Nivea Pramod B.Tech Electronics & Communication Engineering — KTU, Kerala (2022–26)
� �
License
This project is licensed under the MIT License. 