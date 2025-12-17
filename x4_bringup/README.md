# x4_bringup

## Contents

This package contains an assortment of launch files to bring up the simulation or control the actual hardware

## Using the Simulations

The main launch file is ```x4_startup.launch.py```. This has customizable parameters that can call an assortment of simualtion environments including RVIZ2 and Gazebo or interface with the real hardware. The simulations or hardware can be controlled via a ROS2 sercices or MAVProxy commands.

### RVIZ2

The RVIZ2 environemnt is mainly used to view the Aurelia X4 model and can give an easy to use interface to view sensor data generated from the hardware or gazebo simulation. ```x4_startup.launch.py``` defaults to having RVIZ off but can be turned on by appending ```use_rviz:=true```.

### Gazebo + SITL + Ardupilot

This is the main simulation environment that uses Ardupilot + SITL and gazebo physics plugins to model the drone's dynamics and send commands via ROS2 or MAVProxy. For more information on the plugins, visit the x4_description package. To understand what is launched with Gazebo, view the ```x4_gazebo.launch.py``` and ```x4_ardupilot.launch.py``` launch files.

### Real Hardware
In order to launch the project for the real hardware, launch parameters need to be adjusted. This will launch Mavproxy and MAVROS connected to the real drone:

```x4_startup.launch.py use_sim:=false use_real:=true```

## Controlling the Drone

The two main ways to send commands to the drone are via ROS2 sercives or MAVProxy commands. They can be used interchangeably and the documentation for the MAVProxy commands can be found here and the documentation for the Ardupilot ROS2 default services can be found here. We have also added some of our own services to add more functionality to flying which can be found in the x4_interfaces package.

### MAVROS Commands

In another terminal, use these service calls interface with MAVROS to control the robot similarly to MAVProxy. MAVROS tutorials has the ROS1 implementations that can be adjusted to ROS2: 

https://masoudir.github.io/mavros_tutorial/Chapter1_ArduRover_with_CLI/Step1_How_to_change_mode/

Arm Throttle:
```ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"``` 

Set Mode:
```ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"```

Takeoff:
```ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 3.0}"```

Land:
```ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"``

Local Waypoint (Relative to Home):
```ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 3.0}, orientation: {w: 1.0}}}"```

These are just some examples but full documentation can be found here (https://github.com/mavlink/mavros/blob/ros2/mavros/README.md).

### MAVProxy Commands

MAVproxy launches automatically with the startup launch file and the terminal created can be used to run the following commands for example. More commands can be found here.

```mode guided``` -> Set the drone to guided mode. Necessary for following commands
```arm throttle``` -> Arm the motors
```takeoff 5``` -> Takeoff to 5m altitude
```velocity x y z`` -> Where x y and z are values you enter in m/s
```mode land``` -> Land the drone

### ROS2 DDS Commands (Not Currently Supported)

In another terminal, use these commands to control the drone when using DDS. For more information (https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md)

```ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"``` -> Change to Guided Mode

```ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"``` -> Arm the motors

```ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 4}"``` -> Takeoff to 4m altitude
