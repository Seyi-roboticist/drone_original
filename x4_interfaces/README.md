# Aurelia X4 Drone Interface Package

## Overview

This package provides a set of ROS 2 components for interfacing with the Aurelia X4 drone, including action definitions, a hardware interface, and a user interface.

## Package Contents

### ✅ Actions (`/action` folder)
The package includes a set of ROS 2 actions useful for drone control tasks. These actions define commands that assist in interacting with the drone system.

### ⚙️ Hardware Interface
A custom hardware interface is provided for velocity-based control of the drone. While this interface is **not currently used** in the project (as control is managed by Ardupilot), it is included for potential future use — especially if there is a need to switch from Ardupilot to `ros2_control` for low-level control.

### 🖥️ User Interface
A graphical keyboard-based teleoperation interface is included to interact with the drone using the defined actions. It built with `tkinter` and integrated with ROS 2.

#### Features

- **Velocity Control Slider:**  
  A horizontal slider allows the user to set the desired drone velocity in the range `[0.0, 10.0]`.

- **Keyboard Command Input:**  
  Pressing keys like `W`, `A`, `S`, `D`, `Q`, and `E` sends directional or rotational commands:
  
  | Key | Command       |
  |-----|---------------|
  | `W` | Forward       |
  | `A` | Left          |
  | `S` | Backward      |
  | `D` | Right         |
  | `Q` | Rotate Left   |
  | `E` | Rotate Right  |

  The last key pressed is also shown in a **Command** display box.

- **ROS 2 Publishing:**
  - Publishes command strings to `/drone/keyboardcommands`
  - Publishes velocity values to `/drone/keyboardvelocity`

- **Ardupilot MAVROS Control Buttons:**
  - Arm: Arms the drone (calls /mavros/cmd/arming)
  - Disarm: Disarms the drone
  - Takeoff 15m: Runs ros2 run x4_service_scripts mavros_takeoff
  - Land: Runs ros2 run x4_service_scripts mavros_landing
  - Guided: Switches mode to GUIDED via /mavros/set_mode
  - Loiter: Switches mode to LOITER via /mavros/set_mode

#### Launching the Interface

First do `chmod +x gui.py`. Make sure your ROS 2 workspace is sourced, then run the UI:

```bash
ros2 run x4_interfaces gui.py
```

## Notes

- Ardupilot is currently responsible for drone control.
- The hardware interface is provided as a fallback or for future integration needs.

