# x4_vision

**x4_vision** is a ROS 2 package for visual perception on the Aurelia X4 drone. It uses a downward-facing **Sony RX0 II** camera to enable ArUco-based precision landing. The package performs real-time video streaming, intrinsic camera calibration, marker detection, and 6DOF pose estimation.

---

## Purpose

This package enables the drone to:

- Stream video from a USB UVC-compatible camera (e.g., RX0 II)
- Apply camera intrinsics from calibration
- Detect ArUco tags in real-time using OpenCV
- Estimate 3D pose (position + orientation) of markers
- Publish detections for downstream precision landing logic (e.g., via MAVROS)

---

## Dependencies

### ROS 2 (Humble)

- ros-humble-v4l2-camera  
- ros-humble-image-transport  
- ros-humble-camera-info-manager  
- ros-humble-cv-bridge  
- ros-humble-vision-msgs  

### System Libraries

- libopencv-dev  
- python3-opencv  

These provide OpenCV 4.x and ArUco detection support.

---

## Package Structure

x4_vision/  
├── config/  
│   ├── camera_calibration.yaml       (Intrinsic calibration)  
│   └── vision_debug.rviz             (RViz config for debugging)  
├── launch/  
│   └── camera.launch.py              (Launches camera + detection + RViz)  
├── include/x4_vision/  
│   └── aruco_detector.hpp            (ArUco detector class)  
├── src/  
│   └── aruco_detector_node.cpp       (Image processing and pose estimation)  
├── CMakeLists.txt  
├── package.xml  
└── README.md

---

## Build Instructions

From the workspace directory:

- Build the package with colcon  
- Source the workspace  

---

## Launch the Vision System

Use the camera launch file to:

- Start the v4l2 camera driver  
- Load the calibration file  
- Run the ArUco detection node  
- Visualize results in RViz  

---

## Published Topics

| Topic                 | Type                                | Description                                 |
|-----------------------|-------------------------------------|---------------------------------------------|
| /camera/image_raw     | sensor_msgs/msg/Image               | Live image stream from RX0 II               |
| /camera/camera_info   | sensor_msgs/msg/CameraInfo          | Camera intrinsics                           |
| /aruco/detections     | vision_msgs/msg/Detection2DArray    | 2D bounding boxes of detected markers       |
| /aruco/pose           | geometry_msgs/msg/PoseStamped       | 6DOF pose estimate of each detected marker  |

---

## RViz

RViz is automatically launched via `camera.launch.py` and displays:

- Image feed from `/camera/image_raw`  
- Pose arrows from `/aruco/pose`  
- Optional TF frames (if broadcasted)

---

## Integration with Drone System

| Module        | Role                                             |
|---------------|--------------------------------------------------|
| x4_vision     | Camera driver and ArUco detection                |
| x4_landing    | Precision landing controller (planned)           |
| mavros        | MAVLink bridge to ArduPilot                      |
| x4_bringup    | Launch and integration of all subsystems         |

Eventually, `x4_landing` will subscribe to `/aruco/pose` and publish landing setpoints to `/mavros/setpoint_position/local`.

---

## Author

**James Kaluna**

