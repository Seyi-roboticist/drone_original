## MAVROS2:
MAVROS2 and MavROS will be used interchangabily. MavROS is actually more robust and well supported than DDS, so you don't need to deal with the incomplete DDS implementations

```
ros2 launch x4_bringup x4_startup.launch.py # OR:
sim_vehicle.py -w -v ArduCopter --console --map -DG
```

Only if using sim_vehicles, launch (in a new terminal):
```
ros2 launch mavros apm.launch fcu_url:=tcp://localhost fcu_url:=udp://127.0.0.1:14550:@14555
```

### MAVROS Scripts:

Using the following scripts streamlines the more basic actions like TakeOff and Landing

To takeoff (Guided mode): 
```ros2 run x4_service_scripts mavros_takeoff alt:=10.0``` Takeoff to 10.0 meters above the current location

To land (Guided mode): 
```ros2 run x4_service_scripts mavros_landing rlt:=true``` Set rtl to false to land directly below the drone's current location (not recommended if drone has travelled a long distance or if terrain is uneven). Set it to true to land at home position. Expects the vehicle to be in the air

Waypoints (Guided mode): 
```ros2 run x4_service_scripts mavros_waypoint```
Takesoff vehicle and moves it to a few waypoints, then land at home position.

Loiter mode:
``` ros2 run x4_serivce_scripts mavros_loiter_mode alt:=15.0 loiter_time:=10.0``` -> Uses RC control to takesoff vehicle, loiter for 10 seconds at 15 meters above home position, then land using RC control. In Loiter mode, the pilot has more control of the drone using joystick controller. 


Below are some of the more basic, individual commands in MavROS
### MAVROS Commands: 
To switch modes:
```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```
#### Arming:
```
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
```

#### Takeoff: 
Corresponds to Command 22 MAV_CMD_NAV_TAKEOFF. Define the takeoff target position in global frame.
```
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 1.0, yaw: 1.0, latitude: 0.0, longitude: 13, altitude: 60.0}"
```

#### For waypointing relative to home position:
```
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 3.0}, orientation: {w: 1.0}}}"
```

#### Landing: 
Corresponds to Command 21 MAV_CMD_NAV_LAND. Define the landing target position in global frame.
```
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 1.0, yaw: 1.0, latitude: 0.0, longitude: 130, altitude: 600.0}"
```



#### Misc issues we ran into with MavROS: 
Working versions of these are incorporated into a script and/or documented above. 
Landing and takeoff are unsupported in the local frame. These commands are included just for your reference. 

Landing local: 
Corresponds to Command 23 MAV_CMD_LAV_LAND_LOCAL. Target position defined in local frame, but it is unsupported: 
```
ros2 service call /mavros/cmd/takeoff_local mavros_msgs/srv/CommandTOLLocal "{min_pitch: 1.0, yaw: 1.0, latitude: 0.0, longitude: 130, altitude: 600.0}"
```

Local Takeoff: 
Corresponds to Command 24 MAV_CMD_LAV_TAKEOFF_LOCAL. Target position defined in local frame, but it is unsupported: 
```
ros2 service call /mavros/cmd/takeoff_local mavros_msgs/srv/CommandTOLLocal "{min_pitch: 1.0, position: {x: 1, y: 2, z: 5.0}}"
```

In Loiter mode, changing thrust like this doesn't work: ```ros2 topic pub /mavros/setpoint_raw/target_attitude mavros_msgs/msg/AttitudeTarget "{header: {frame_id: 'map'}, body_rate: {x: 0.1, y: 0.1, z: 0.2}, thrust: 0.9}"```


## Instructions for working with DDS
```
ros2 run x4_service_scripts dds_takeoff alt:=15.0 # alt in meters 
```

For waypointing, the waypoint may be given as a global GPS coordinate (longitude and latitude are in degrees, and alt is in meters. Therefore a slight change correspond to a large change in meters) or as a relative offset in Forward-Right-Down frame (longitude, latitude, and alt are all defined in meters) relative to the vehicle's current heading. This can be set by the "frame" argument, where frame:=20 is the relative waypoint, and frame:=5 is the global waypoint. 

```
ros2 run x4_service_scripts dds_waypoint longitude:=149.17 latitude:=-35.3627 alt:=20.0 frame:=20  # All arguments are optional
```

A few notes about this package:

This package is compatible with ArduPilot (AP) version 4.6. Waypointing commands through ROS2/DDS is supported for Plane in version 4.5, but waypointing is supported for Copter in version 4.6 and later. In Copter-4.5, the AP_ExternalControl_Copter::set_global_position() function is not yet defined in ardupilot -> ArduCopter -> AP_ExternalControlCopter.hpp/cpp  It is a very small change that completely disables this functionality; you can refer to a later version of the same document to see how it is defined.  

As of Copter-4.6, AP only support ROS2 waypointing defined in the global frame, i.e. the latitude and longitude must be given in degrees and altitude must be in meters. To enable relative waypointing, the script takes the relative offsets that the user gives (in meters), the current pose of the vehicle, and computes the goal in FRAME_GLOBAL_INT. Some calculations may need to be verified and tested. 

Other approaches include modifying the AP library itself, in particular, how the ROS2 msg ardupilot_msgs.msg.GlobalPosition is defined and handled. An useful place to modify is AP_DDS_ExternalControl.cpp, but that seems rather difficult, and any hardware may not support the forked repo. 
