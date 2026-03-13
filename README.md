# Visual Docking Station - ROS 2 Jazzy & Gazebo Harmonic

An autonomous mobile robot simulation that detects a docking station using ArUco markers and computer vision, then navigates into the dock automatically. Built for **ROS 2 Jazzy** and **Gazebo Harmonic**.

---

## 🚀 Quick Start

Ensure you have ROS 2 Jazzy and Gazebo Harmonic installed.

```bash
# 1. Prepare workspace
cd ~/Downloads/files
source /opt/ros/jazzy/setup.bash

# 2. Build
colcon build --symlink-install

# 3. Launch Simulation
source install/setup.bash
ros2 launch visual_docking docking_simulation.launch.py
```

---

## ✨ Features

- **ArUco Visual Detection** – Precision tracking using OpenCV `DICT_4X4_50`.
- **3-Phase Controller** – State-machine logic: (1) Search, (2) Approach, (3) Dock.
- **Custom Gazebo World** – 8x6m room with a dedicated docking alcove and platform.
- **Native SDF Modeling** – Robot defined in SDF 1.9 for high physics fidelity.
- **HUD Visualization** – Debug stream showing tracking data and distance overlays.

---

## 🛠 Project Structure

```text
visual_docking/
├── docs/                # Presentation slides and LaTeX source
├── launch/              # Simulation launch file
├── models/
│   ├── aruco_marker/    # ArUco panel model & texture
│   └── docking_robot/   # Pure SDF robot model
├── scripts/
│   ├── aruco_detector.py # OpenCV Vision node 
│   ├── dock_controller.py # Autonomous navigation node
│   └── generate_marker.py # ArUco PNG generator
├── urdf/                # Robot description (source for SDF)
├── worlds/              # Gazebo room environment
├── CMakeLists.txt       # Build configuration
└── package.xml          # Package dependencies
```

---

## 📺 Monitoring & Debugging

Open a new terminal after launching the simulation:

**View Camera Feed:**
```bash
ros2 run rqt_image_view rqt_image_view /docking/debug_image
```

**Check Mission Status:**
```bash
ros2 topic echo /docking/status
# Expected: SEARCHING -> APPROACHING -> DOCKED
```

**Manual Teleop Control (if needed):**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🏗 System Architecture

1.  **ArUco Detector**: Subscribes to `/camera/image_raw`, performs pose estimation via `solvePnP`, and publishes `/aruco/pose`.
2.  **Dock Controller**: Subscribes to `/aruco/pose`, implements a P-controller for angular alignment and linear approach, and publishes `/cmd_vel`.
3.  **Gazebo Bridge**: Synchronizes `/cmd_vel`, `/odom`, and `/camera` between ROS 2 and Gazebo transport.

---

## 🎓 Documentation

- **PPT Outline**: Located in `docs/presentation_outline.md`.
- **LaTeX Presentation**: Found in `docs/presentation.tex` (Compile with `pdflatex`).
- **Full Walkthrough**: Detailed technical decisions in `docs/walkthrough.md`.

---

## 📝 License

Apache 2.0
