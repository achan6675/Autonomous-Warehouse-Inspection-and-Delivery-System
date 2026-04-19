# Autonomous Warehouse Inspection and Delivery System (ROS2)

An autonomous robotics system built using ROS2 for warehouse inspection, delivery, and data collection tasks. The system integrates perception, decision-making, and control modules to enable structured autonomous behaviours in a simulated and physical TurtleBot environment.

---

## 🧠 System Overview

The system is designed as a modular ROS2 architecture consisting of perception, task management, and control layers. It enables a robot to navigate an environment, respond to user requests, collect data, and generate structured reports.

Key capabilities include:
- Autonomous exploration and wall-following navigation
- Inspection and delivery task execution
- QR code detection and spatial transformation
- Battery-aware behaviour and system state monitoring
- Structured report generation based on collected data

---

## 🏗️ Architecture

The system is structured into the following components:

### 1. Perception Layer
- QR code detection from camera input
- Transformation of detected QR positions into robot reference frame

### 2. Behaviour Layer
- Inspection robot logic for environment scanning tasks
- Delivery robot logic for task-based navigation
- Damage manager for processing and organising detected issues

### 3. System Services
- Battery monitoring node for simulating power state
- Text file generator for structured reporting outputs
- Robot position manager (client/server architecture for state tracking)

### 4. Control Layer
- Wall-following explorer for autonomous navigation
- Base robot interface defining shared robot behaviours

---

## ⚙️ Key Features

- Autonomous wall-following exploration for environment coverage
- Task-based robot behaviours (inspection + delivery modes)
- QR code detection and spatial mapping
- User request handling via client-server architecture
- Battery-aware behaviour including state monitoring
- Automated report generation for inspection results
- Modular ROS2 node-based system design

---

## 🧩 System Components

### Core Nodes
- `robot_node` – Main robot control logic
- `inspection_robot` – Handles inspection behaviour
- `delivery_robot` – Handles delivery task execution
- `wall_following_explorer` – Autonomous exploration behaviour

### Perception
- `qr_detector` – Detects QR codes in environment
- `qr_transform` – Converts detections into robot coordinate frame

### System Management
- `battery_life_node` – Simulates and tracks battery state
- `damage_manager_node` – Processes detected inspection data
- `text_file_generator` – Generates structured output reports

### Position Tracking
- `robot_position_manager_server_node`
- `robot_position_manager_client`

---

## 🛠️ Technologies Used

- ROS2
- C++
- Linux
- TurtleBot platform
- Gazebo simulation
- OpenCV (QR detection, if applicable)

---

## 🚀 System Behaviour

The robot operates in two primary modes:

### Inspection Mode
- Navigates environment using wall-following behaviour
- Detects and records QR-based inspection data
- Logs findings through the damage management system

### Delivery Mode
- Receives task requests via ROS2 communication system
- Navigates to target locations
- Completes delivery actions and returns system status

---

## 📊 Outputs

- Structured text reports generated from inspection data
- Battery status monitoring
- Logged task execution history

---

## 🧑‍💻 My Contribution

- Designed and implemented ROS2 node architecture
- Developed wall-following exploration behaviour
- Built inspection and delivery robot logic modules
- Created system services including reporting and battery monitoring
- Integrated client-server architecture for robot position management

---

## 📌 Notes

This project was completed as part of a group assignment. The repository reflects a modular ROS2 system design integrating multiple autonomous behaviours and perception pipelines.
