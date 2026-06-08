Smart Mobile Robotic System

ESP32-based autonomous inspection robot with LiDAR sensing, environmental monitoring, and real-time YOLO hazard detection.

Problem Statement

Manual inspection of confined, hazardous, or hard-to-reach environments exposes personnel to safety risks. Traditional static monitoring systems lack mobility and real-time situational awareness.

This project addresses these challenges by developing an autonomous mobile robot capable of navigating enclosed spaces, mapping surroundings using LiDAR, monitoring environmental conditions, and detecting hazards using onboard AI—without requiring human entry.

---

Key Features

- Autonomous navigation and obstacle avoidance
- Real-time LiDAR-based distance sensing
- Temperature and humidity monitoring
- On-device YOLO hazard detection
- Power telemetry monitoring
- ESP32-based control architecture
- No cloud dependency for inference

---

System Overview

The robot integrates three major subsystems:

1. Navigation & Mapping

- VL53L0X LiDAR sensor
- Obstacle detection up to 2 meters
- Autonomous path planning in confined environments

2. Environmental Monitoring

- DHT11 temperature and humidity sensing
- Continuous system health monitoring
- Real-time power telemetry

3. Hazard Detection

- Lightweight YOLO object detection model
- Live camera-based hazard recognition
- Approximately 80% detection accuracy during testing

---

Hardware Components

Component| Role| Specification
ESP32| Main controller| 240 MHz Dual-Core, Wi-Fi & Bluetooth
VL53L0X| Distance sensing| Up to 2 m range
DHT11| Environment sensing| Temperature & Humidity
Camera Module| Object detection| 640×480 @ 30 FPS
Power Sensor| Telemetry| Voltage & Current Monitoring
Motor Driver| Mobility control| DC Motor Interface

---

Performance Results

Metric| Result
Object Detection Accuracy| ~80%
LiDAR Detection Range| Up to 2 m
Inference Type| On-device
Test Environment| Indoor / Laboratory
Power Monitoring| Real-time

---

Project Structure

smart-mobile-robotic-system/
├── README.md
├── assets/
│   ├── robot_photo.jpg
│   ├── system_architecture.png
│   └── demo.gif
├── src/
│   ├── main.ino
│   ├── lidar.ino
│   ├── sensors.ino
│   └── detection.py
├── docs/
│   └── wiring_diagram.pdf
└── requirements.txt

---

Installation

ESP32 Firmware

1. Open "src/main.ino" in Arduino IDE
2. Install required libraries:
   - VL53L0X (Pololu)
   - DHT Sensor Library (Adafruit)
3. Select ESP32 Dev Module
4. Upload firmware

YOLO Detection

pip install -r requirements.txt
python src/detection.py

---

Current Limitations

- Reduced performance in low-light conditions
- Limited hazard-class training dataset
- LiDAR range restricted to 2 meters
- Single-camera blind spots

---

Future Improvements

- Infrared illumination for night operation
- Expanded hazard detection dataset
- Multi-LiDAR sensor configuration
- SLAM-based autonomous navigation
- Remote monitoring dashboard via Wi-Fi
- Cloud-based data logging and analytics

---

Author

Nivea Pramod
B.Tech Electronics & Communication Engineering
APJ Abdul Kalam Technological University (KTU), Kerala
2022–2026

---

License

This project is licensed under the MIT License.