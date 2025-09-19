# Sniffi – Autonomous Gas Detection Robot

Sniffi is an autonomous mobile robot for environmental monitoring and security patrol. The system is built on **ROS 2 Humble** running on **Ubuntu 22.04**, with a **Raspberry Pi 4** as the main computer and an **Arduino Mega 2560** as the real-time controller.

---

## 📐 System Architecture

**Hardware**
- **Compute**: Raspberry Pi 4 (AI, navigation, vision)  
- **Controller**: Arduino Mega 2560 (gas sensors, motors, safety)  
- **Power**: 24 V LiFePO₄ system (2× 12 V batteries in series, with BMS)  
- **Mobility**: 4× NEMA 23 servo motors with planetary gearboxes  
- **Sensors**:
  - RPLIDAR 360° laser scanner
  - Orbbec depth camera
  - FLIR Lepton thermal camera
  - Multi-gas sensor array (14 sensors, MQ + electrochemical)
  - Environmental sensors (DHT22, etc.)

**Software**
- **ROS 2 Humble** workspace with three core packages:
  ```
  ros2_ws/src/
  ├── lidar_gui/           # LIDAR visualization
  ├── motor_driver/        # Motor control + motion interface
  └── sniffi_description/  # URDF/Xacro, meshes, robot model
  ```
- **Expected ROS 2 topics**:
  - `/scan` – LIDAR data
  - `/cmd_vel` – velocity commands
  - `/gas_sensors` – gas sensor readings
  - `/battery_status` – power system state
  - `/dock_status` – docking state

**Data Flow**
```
Gas Sensors → Arduino → Serial USB → Raspberry Pi → ROS 2 topic (/gas_sensors)
LIDAR / Camera → Raspberry Pi → ROS 2 topics (/scan, /camera/*)
ROS 2 Nav2 → /cmd_vel → Arduino → Motor Drivers → Motors
```

---

## ⚙️ Development Environment

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble (`ros-humble-desktop`)
- Colcon build system
- Python 3.10
- Git + VS Code (recommended)

### Workspace Setup
```bash
# Clone the repo
git clone https://github.com/<your-org>/sniffi_robot.git
cd sniffi_robot
mkdir -p ~/ros2_ws/src
mv sniffi_robot/* ~/ros2_ws/src/

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

---

## 🚀 Running Sniffi (current demo)

### 1. Test dummy node
```bash
ros2 run motor_driver motor_node
```
You should see logs confirming the node runs.

### 2. Run LIDAR driver
```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

### 3. Visualize robot model
```bash
ros2 launch sniffi_description display.launch.py
```

---

## 🛠️ Notes on Hardware Coordination

⚠️ **Important:** The current BMS (8S, 25.6 V) does not match the 24 V battery configuration.  
A **7S / 24 V BMS** or dual 12 V BMS is required before motor power testing. Until then, only non-motor subsystems (sensors, Pi, Arduino) should be powered for software development.

---

## ✅ Milestone 1 Delivery

- ROS 2 Humble environment configured and tested  
- Repository structure initialized (`lidar_gui`, `motor_driver`, `sniffi_description`)  
- Dummy node and LIDAR driver can be launched  
- System architecture documented (this README)  
- Coordination plan for BMS issue documented  

This closes **Week 1: Foundation & Power Management Setup**.
