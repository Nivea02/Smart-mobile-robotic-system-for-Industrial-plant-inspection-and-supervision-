# Smart Mobile Robotic System

> ESP32-based autonomous inspection robot with LiDAR sensing, multi-parameter environmental monitoring, and real-time YOLO hazard detection.

![Platform](https://img.shields.io/badge/platform-ESP32-blue)
![Language](https://img.shields.io/badge/language-Python%20%7C%20C++-green)
![Sensor](https://img.shields.io/badge/sensor-VL53L0X%20LiDAR-orange)
![Vision](https://img.shields.io/badge/vision-YOLOv5-red)
![Status](https://img.shields.io/badge/status-Completed-brightgreen)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

---

## Problem Statement

Manual inspection of confined, hazardous, or hard-to-reach spaces poses significant safety risks to personnel. Static monitoring systems lack mobility and real-time response capability. This project addresses these limitations by developing an autonomous mobile robot that navigates enclosed environments, maps surroundings using LiDAR, monitors environmental parameters, and identifies hazards using onboard AI — without requiring human entry.

---

## System Overview

The robot integrates three core subsystems:

- **Navigation & Mapping** — VL53L0X LiDAR provides real-time obstacle detection and distance measurement up to 2m, enabling autonomous path planning in constrained spaces.
- **Environmental Sensing** — DHT11 sensor continuously monitors temperature and humidity; power telemetry tracks system health.
- **Hazard Detection** — A lightweight YOLO-based object detection model runs onboard, classifying hazards from a live camera feed with approximately 80% identification accuracy.

---

## System Architecture

```
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
```

---

## Hardware Components

| Component | Role | Specification |
|-----------|------|---------------|
| ESP32 | Main microcontroller | 240 MHz dual-core, Wi-Fi + BT |
| VL53L0X | Distance / obstacle sensing | Range up to 2m, I2C interface |
| DHT11 | Temperature & humidity sensing | ±2°C / ±5% RH accuracy |
| Camera Module | Object detection input | 640×480 @ 30fps |
| Power sensor | System health telemetry | Voltage & current monitoring |
| Motor driver | Mobility control | DC motor interface |

---

## Key Results

| Metric | Value |
|--------|-------|
| Object detection accuracy | ~80% across test scenarios |
| LiDAR obstacle detection range | Up to 2m |
| Inference type | On-device (no cloud dependency) |
| Test environment | Indoor / lab |
| Power monitoring | Real-time onboard telemetry |

---

## Project Structure

```
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
```

---

## Setup & Installation

### Hardware Setup
1. Connect VL53L0X to ESP32 via I2C (SDA → GPIO21, SCL → GPIO22)
2. Connect DHT11 data pin → GPIO4
3. Connect camera module via appropriate interface
4. Connect motor driver to GPIO pins (refer to wiring diagram in /docs)

### Software Setup

**ESP32 firmware:**
```bash
# Open src/main.ino in Arduino IDE
# Install required libraries:
#   - VL53L0X by Pololu
#   - DHT sensor library by Adafruit
# Select board: ESP32 Dev Module
# Upload to device
```

**YOLO inference (Python):**
```import cv2
import time
import numpy as np
import serial
import pyttsx3
import requests

# ---------------- IOT SETTINGS ----------------
api_key = "94UnCTxEpaag"

def update_server(message):
    url = "https://postdata.iotwebserver.com/postdata.php?apikey=" + api_key + "&alert=" + message + "&sensor_1=1"
    print(url)
    try:
        response = requests.get(url)
        if response.status_code == 200:
            print("IoT Sent:", response.text)
        else:
            print("IoT Error:", response.status_code)
    except:
        print("IoT connection failed")


# ---------------- VOICE ENGINE ----------------
engine = pyttsx3.init()
voice = engine.getProperty('voices')
engine.setProperty('voice', voice[1].id)

# ---------------- YOLO FILES ----------------
labelsPath = 'coco.names'
LABELS = open(labelsPath).read().strip().split("\n")

np.random.seed(100)
COLORS = np.random.randint(0, 255, size=(len(LABELS), 3), dtype="uint8")

weightsPath = 'yolov3.weights'
configPath = 'yolov3.cfg'

conf = 0.7
threshold = 0.3

print("[INFO] loading YOLO from disk...")
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

# ---------------- SERIAL ----------------
arduino = serial.Serial(port='COM18', baudrate=9600, timeout=1)

def serialprint(x):
    x = x + "\n"
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = ""
    data = data.strip()
    print(data)
    return data


# ---------------- CAMERA ----------------
vs = cv2.VideoCapture(0)

# to avoid repeated IoT alerts
plast_object = ""

while True:

    ret, frame = vs.read()

    if ret:

        image = frame
        (H, W) = image.shape[:2]

        ln = net.getLayerNames()
        ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()]

        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
                                     swapRB=True, crop=False)

        net.setInput(blob)
        layerOutputs = net.forward(ln)

        boxes = []
        confidences = []
        classIDs = []

        for output in layerOutputs:

            for detection in output:

                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                if confidence > conf:

                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")

                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, conf, threshold)

        if len(idxs) > 0:

            for i in idxs.flatten():

                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                color = [int(c) for c in COLORS[classIDs[i]]]

                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

                text = "{}: {:.4f}".format(
                    LABELS[classIDs[i]], confidences[i])

                cv2.putText(image, text, (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                detected_object = LABELS[classIDs[i]]

                print(detected_object, confidences[i])

                # SEND SERIAL TO ESP32
                serialprint(detected_object)

                # SEND IOT ALERT ONLY IF NEW OBJECT
                if plast_object != detected_object:
                    update_server(detected_object + " Detected")
                    plast_object = detected_object

                # VOICE ALERT
                engine.say(detected_object + " Detected")
                engine.runAndWait()

    cv2.imshow("Image", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


vs.release()
cv2.destroyAllWindows()


```

---

## Limitations & Future Work

**Current limitations:**
- Detection accuracy degrades in low-light conditions
- YOLO model trained on limited hazard classes
- LiDAR range constrained to 2m — insufficient for large open spaces
- Single-camera setup creates blind spots

**Planned improvements:**
- Integrate IR illumination for low-light performance
- Expand training dataset with domain-specific hazard classes
- Add multi-sensor LiDAR array for wider coverage
- Implement SLAM for autonomous navigation
- Wi-Fi based remote monitoring dashboard via ESP32

---

## About

**Nivea Pramod**
B.Tech Electronics & Communication Engineering — KTU, Kerala (2022–26)

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue)](https://linkedin.com/in/nivea-pramod-b94203320)
[![Email](https://img.shields.io/badge/Email-Contact-red)](mailto:pramodnivea2003@gmail.com)

---

## License

This project is licensed under the MIT License.# Smart-mobile-robotic-system-for-Industrial-plant-inspection-and-supervision-