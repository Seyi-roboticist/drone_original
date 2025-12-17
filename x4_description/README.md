# x4_description

## Contents

This package contains the urdf and sdf implementations of the Aurelia X4 drone for use in rviz and gazebo respectively. Meshes for the drone and propellers are also stored here as well as a sample rviz layout to view your robot.

### Plugin Parameters

In order to change the flight parameters for the gazebo simulation, you will be mainly changing the given parameters for the Lift/Drag plugins and Ardupilot plugins found in the gazebo.xacro. This can give control over the PID parameters (integral and derivative were giving us trouble), Connection settings, propeller and motor parameters, and other ArduPilot sepcific parameters. It is currently in a working and tuned state but changes to the physical parameters may require retuning

We have also created example sensors in sensors.xacro to be used in your drone implementation if need be and example macro calls can be seen commented out in x4.xacro. We had trouble with the drones balance when placing massless sensor links/frames around the drone so they have been removed for simplicity in the current working model.
