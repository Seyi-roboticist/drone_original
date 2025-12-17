# Aurelia X4 Drone Project

## read-the-docs
A read-the-docs documentation for this project can be found in https://rsp-drone-project-doc.readthedocs.io/en/latest/

## Project Overview
For our capstone project, we aim to implement a ROS2-based control system for the Aurealia X4 drone. Our team of 5 students will create a complete system capable of visualizing the drone in RViz, simulating the drone in a Gazebo environment, and performing basic flight maneuvers on the physical hardware. 

## Team Members
- Seyi R. Afolayan (Remote PIC & Team Lead)
- Sameer Khan
- James Kaluna
- Xinhao Chen 
- Lavinia Kong

## Project Objectives
### Primary Objectives (Required)
- Create a comprehensive drone description package with URDF/xacro models (`x4_description`)
- Implement a simulation environment in Gazebo (`x4_gazebo`)
- Develop takeoff and landing capabilities in both simulation and real hardware (`x4_gazebo` and physical hardware)

### Stretch Goals (If Time Permits)
These additional work are doable but would require all **persons** of this repo to be working effectively.
- Implement basic path planning for autonomous navigation
- Add SLAM capabilities for mapping environments
- Develop pick-and-place functionality

## Repository Structure
```
/RSP_drone_project/src
├── /x4_description      # URDF/xacro files, meshes, materials (we would also have launch files here but this is just for debugging purposes)
├── /x4_gazebo           # Simulation environments and launch files
├── /x4_control          # Flight control algorithms are pretty much done,,, so just high-level commands
├── /x4_moveit           # Flight Moveit Package 
├── /x4_interface        # Hardware communication bridge
├── /x4_vision           # Computer vision with RX0II camera to enable precision landing later
└── /x4_bringup          # System launch files
```

## Package Details
## Minimum deliverables: description, msgs (if needed) move drone in both simulation and real (simple, hello world)

### x4_description
Contains the complete robot description model including:
- URDF/xacro files defining the robot's physical structure (not including the landing gear)
- Mesh files for visualization
- Physical properties (mass, inertia matrices)
- Sensor configurations 

### x4_gazebo
Responsible for simulation capabilities:
- Ignition Gazebo world files
- Simulation-specific launch files
- Plugin configurations
- Testing scenarios

### x4_control
Implements control algorithms for:
- Takeoff sequence
- Stable hover
- Landing sequence
- (Stretch) Waypoint navigation

### x4_interface
Handles communication with the physical drone:
- MAVLink protocol implementation
- Telemetry data processing
- Command conversion
- Safety monitoring
- Parameter management
- User interface

### x4_vision
Vision package for X4 using RX0 II camera:
- Stream video from a USB UVC-compatible camera (e.g., RX0 II)
- Apply camera intrinsics from calibration
- Detect ArUco tags in real-time using OpenCV
- Estimate 3D pose (position + orientation) of markers
- Publish detections for downstream precision landing logic (e.g., via MAVROS)

### x4_bringup
Contains launch files to start the system:
- Simulation startup
- Hardware connection
- Configuration management

### X4_moveit
High-level Moveit package for flight control


## Documentation

Documentation for starting up and running the Aurelia X4 ROS2 packages for real hardware and gazebo simulation.

### Prerequisites

Install Gazebo Fortress: https://gazebosim.org/docs/fortress/install/

Install ROS2 Humble: https://docs.ros.org/en/humble/Installation.html

### Setup

The following operations should be done in your workspace. For the purpose of these instructions, we will call your workspace "drone_workspace". In addition, every new terminal should be sourced! like ```cd <drone_workspace> & source install/setup.bash```

#### Ardupilot Setup
Refer to [this link](https://ardupilot.org/dev/docs/building-setup-linux.html) for full instructions:  
```
cd <drone_workspace>
mkdir src && cd src
git clone -b Copter-4.5 --recurse-submodules https://github.com/ArduPilot/ardupilot.git # (Using Copter-4.5 with our build. Adjust if needed) 
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Compile Ardupilot for SITL (Software in The Loop). For a list of other compile options, see [here](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md) 
```
./waf configure --board sitl # Assumes that you want to run in simulation 
./waf copter

export PATH=$PATH:$HOME/ardupilot/Tools/autotest # You may need to edit these paths
export PATH=/usr/lib/ccache:$PATH
. ~/.bashrc
```

At this point, you can test your installation in simulation:

```
cd ardupilot/ArduCopter
sim_vehicle.py -v copter --console --map -w
```

#### Setup Ardupilot for ROS2
Refer to [this link](https://ardupilot.org/dev/docs/ros2.html) for full instructions:  
NOTE: vcs import installs ardupilot master branch and micro-ros humble branch. Maybe dont run command and just install micro-ros

```
cd drone_workspace
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src
cd drone_workspace
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

#### Set up Micro-XRCE-DDS-Gen. 

Note: we currently use MavROS in place of Micro-XRCE-DDS-Gen.

Micro XRCE DDS is the backbone that allows communication between ROS2 and Ardupilot, replacing alternatives like MavROS. It is a tool implements a client-server protocol that enables resource-constrained devices (clients) to take part in DDS communications. The Micro XRCE-DDS Agent (server) acts as a bridge to make this communication possible. It acts on behalf of the Micro XRCE-DDS Clients by enabling them to take part to the DDS Global Data Space as DDS publishers and/or subscribers. XRCE-DDS Gen generates source code for DDS Agent and Clients to use.

For more documentation regarding the use of Micro XRCE DDS with ROS2 and Ardupilot, see [here](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md). It has a nice architecture diagram. Do not use the installation instructions found on that page. Just use it for reference. Also note that Micro ROS Agent is a separate product that we do not use. The "Micro ROS" in any Ardupilot documentation and here refers to a wrapper around XCRE DDS Agent. 

```
sudo apt install default-jre
cd drone_workspace # NOTE: make sure you are in the right directory here
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc # You may need to edit the path
```

Test microxrceddsgen installation
```
source ~/.bashrc
microxrceddsgen -help
```

Build:
```
cd drone_workspace
colcon build --packages-up-to ardupilot_sitl
```

#### Install the Gazebo plugin
Refer to [this repo](https://github.com/ArduPilot/ardupilot_gazebo/tree/fortress) for official instructions and documentation: 

```
sudo apt install rapidjson-dev libignition-gazebo6-dev
git clone https://github.com/ArduPilot/ardupilot_gazebo -b fortress
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Configure your paths
```
echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export IGN_GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
```

#### Install MavROS 
For official documentation, please see [here](https://github.com/mavlink/mavros/tree/ros2/mavros). MavROS2 is supposed to fill a similar as AP_DDS -- enable ROS2 communication, but we found that MavROS2 has way more commands and functionality than AP_DDS

```
sudo apt install ros-humble-mavros
ros2 run mavros install_geographiclib_datasets.sh
```

At this point, the workspace should be structured like this: 
```
drone_workspace
├── Micro-XRCE-DDS-Gen
├── src
│   ├── ardupilot
│   ├── micro_ros_agent
│   └── RSP_drone_project
```

## Simulation

In order to launch the simulation we have created multiple tools and launch files. x4_description contains the urdf descriptions and meshes need to load the simulation model of the X4 while the x4_gazebo packages contains configuratiosn need for the gazebo simulation specifically. in order to launch these models, x4_bringup package contains many launch files that can start combinations of RVIZ2, Gazebo Fortress, Ardupilot, and Mavproxy.

To compile this repository, use 

```colcon build --packages-select x4_interfaces x4_bringup x4_description x4_gazebo x4_service_scripts```

To launch the Gazebo simulation with Ardupilot support, launch:

```ros2 launch x4_bringup x4_startup.launch.py```


This starts Gazebo and Ardupilot with SITL and a MavROS node for to ROS2 commands. Now, commands can be send directly as ROS2 service calls in a separate terminal. See the MavROS information section below for more information. 

### Hardware

To connect to real drone, you jus need to modify the launch parameters slightly. Gazebo will not be available with the real hardware. Before using real drone, make sure you understand how to setup, arm, and control drone using remote control. This is vital to setting everything up.

```ros2 launch x4_bringup x4_startup.launch.py use_sim:=false, use_real:=true```

### Commands
There are three methods to send commands: through MavProxy, through DDS as ROS2 commands, or through MavROS as ROS2 commands. Below are some instructions for each method: 

### MavProxy (Supported)
ROS2 Commands are sent via ardupilot_sitl and ardupilot_msgs packages. They provide services that include a wide variety of Mavlink commands. To open mavproxy to send commands, run the following in a new terminal:

```mavproxy.py --master udp:127.0.0.1:14550  --console --map```

Demo MavProxy Commands:

``` 
mode guided
arm throttle
takeoff 5
velocity x y z (m/s for 3s)
mode land
```

Not all modes support all commands. Takeoff can happen in designated modes like guided mode. A full list of commands to use in guided mode, see [here](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html). See [here](https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html) for mavproxy communication for Copter. [here](https://ardupilot.org/mavproxy/index.html#home) is the full MavProxy documentation. 

### MavROS (Preferred)
ROS2 commands can be send in a new terminal. To see more details about individual commands, see x4_service_scripts package for a summary, or [here](https://mavlink.io/en/messages/common.html) for documentation and [here](https://github.com/mavlink/mavros/tree/ros2/mavros_msgs) for the code. 

There are scripts to demo and simplify the basic actions:
```ros2 run x4_service_scripts mavros_takeoff alt:=10.0``` -> Takeoff to 10 meters directly above home position in Guided mode

```ros2 run x4_serivce_scripts mavros_landing rlt:=false``` -> Land back at home position. Expects the vehicle to be in the air

``` ros2 run x4_serivce_scripts mavros_waypoint``` -> Takesoff vehicle and moves it to a few waypoints, then land at home position. In Guided mode. 

``` ros2 run x4_serivce_scripts mavros_loiter_mode alt:=15.0 loiter_time:=10.0``` -> Uses RC control to takesoff vehicle, loiter for 10 seconds at 15 meters above home position, then land using RC control. In Loiter mode. 

NOTE: We have also supplied scripts to demo certain high-level features in the x4_service_scripts package.

### DDS (Copter-4.6 or later)
Our launch files currently doesn't launch the DDS/Micro-ROS-Agent system because this system is not fleshed out in Copter-4.5 version. To our knowledge, Copter-4.6 support ModeSwitch, Arming, Takeoff and Waypoint, while Copter-4.5 only support ModeSwitch and Arming.

For tutorials to launch ArduPilot to support DDS, refer to [this ArduPilot page](https://ardupilot.org/dev/docs/ros2-waypoint-goal-interface.html), and [here](https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2/ardupilot_msgs) for ardupilot_msgs used to communicate to AP. Once it is launched, below are some demo commands.

```ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"``` -> change to Guided Mode. For other available modes, see [here](https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE)

```ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"``` -> Arm the motors

```ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 4}"``` -> takeoff to 4m altitude



