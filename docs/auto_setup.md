# Autonomous Flight Setup

Autonmous flight setup using the Aurelia X4 drone and the x4 packages of this project.

## Pre-Requirements

All of these should be met before attempting to setup autonomous flight:

- Working X4 drone that is setup and able to fly via remote control. Look to hardware_setup.md for instructions on this
- Remote PIC (Authorized pilot)
- Safe place to fly outdoors (Few trees or obstructions)

## Materials

Additional materials necessary to allow for autonomous flight:

- Microcontroller
  - Raspberry Pi (We used Raspberry Pi 4)
  - Jetson (Jetson Orin Nano is recommended)
  - Anything that can run Linux 22.04
- External power for microcontroller
- Network for SSH

Optional:

- Long range communication for microcontroller
  - Satellite
  - Wifi extender
  - Anything that allows for SSH (Or however else you want to send ROS2 commands)

## Microcontroller Setup

This will be the microcontroller running ROS2 and the packages we have created. It will need to be setup with Linux 22.04 and run ardupilot, our repo, and everything that goes along with the setup from the main README.md aside from Gazebo related programs. You will know if everything is working if you can successfully build and run the real hardware launch file aside from the connection erros from not being plugged into the CubePilot. You will also need to establish an SSH connection from a main computer to the microcontroller. There are many tutorials and setups for acheiving this but the end goal is to be able to write to the CLI of the microcontroller from your main computer. Here is an example for a [Raspberry Pi](https://www.raspberrypi.com/documentation/computers/remote-access.html).

NOTE: If you use a Linux version with only CLI, you may find errors with the launch files as is. Changes will have to be made to stop ardupilot_sitl launch to occur and instead have MAVROS connect directly to the CubePilot.

Another important consideration is power. Depending on your microcontroller, you can find a way to draw power from the main battery but an external battery would also make sense. Make sure you can power the microcontroller and secure both the microcontroller and battery you use.

Finally, to connect your microcontroller to the drone, use a USB to micro USB cable to connect to the CubePilot and create ther serial connection between them. Our script expects the CubePilot to be on ttyACM0 but that can be changed in the launch files if need be.

## Usage 

After all of the setup, commands can be sent using MAVROS commands that are roughly detailed in ROS1 [here](https://masoudir.github.io/mavros_tutorial/Chapter1_ArduRover_with_CLI/Step2_How_to_Arm_and_Disarm/) or examples can be found in the x4-service_scripts package. The scripts from the x4_service_scripts can also be used to control the real robot. We recommend you try any script that starts with mavros except the loiter_mode script which currently has issues. Also, if SSHing into a GUI of the microcontroller, you can interact and use the xterm terminal created for mavproxy commands aswell.


