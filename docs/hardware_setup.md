# Aurelia X4 Setup and Checklist

## I. Preflight Preparation

1. Inventory & Physical Inspection
    - Drone frame and arms visually inspected for cracks, loose parts
    - Propeller arms tightened and secure
    - Propellers inspected for cracks, chips, or imbalance
    - Landing gear clearance and stability verified
2. Battery Safety and Preparation
   - Check for puffing, leakage, or visible damage
   - Balance charge LiPo batteries to 4.20V per cell (Max: 25.2V total for 6S)
   - Ensure both batteries have equal charge level
   - Store in LiPo-safe bags until ready to use
   - Storage Voltage: 3.85V/cell ≈ 23.1V
   - Minimum Safe Voltage: 3.2V/cell ≈ 19.2V
   - Never below: 3.0V/cell ≈ 18V
3. Environmental Safety Check
   - Verify no metal or magnetic interference nearby
   - Clear takeoff/landing zone (away from trees/buildings)
   - Confirm wind speed and weather are within safe flying limits
   - Comply with local UAV laws: https://aurelia-assist.aurelia-aerospace.com/world-drone-regulations

## II. Ground Control Station Setup (Mission Planner Only)

1. Software Installation
   - Install Mission Planner: https://firmware.ardupilot.org/Tools/MissionPlanner/
2. Telemetry & USB Connection Options
   - USB (recommended for initial setup): Connect USB cable to flight controller (rear port), Select correct COM port in MP, baud = 115200, Click CONNECT
   - Telemetry radio: USB connection to GCS, Select correct COM port, baud = 57600

## III. Radio Transmitter Setup

1. Calibration
   - Connect to MP: SETUP → Mandatory Hardware → Radio Calibration
   - Click Calibrate Radio and follow prompts (move all sticks/switches)
   - Confirm RC1–RC4 correspond to Roll, Pitch, Yaw, Throttle
   - Assign functions to switches (e.g., RC15 for arm/disarm) in Config → User Params
2. Failsafe & Mode Parameters
   - Set FS_THR_ENABLE = 0 to disable throttle failsafe if needed
   - Set startup flight mode: Guided
   - Check Mode 1 = Stabilize or Loiter, Mode 6 = RTL

## IV. ESC Calibration (if required)

Manual ESC Calibration (Buzzer Method)

   - Disconnect from Mission Planner and power
   - Power drone with throttle max on TX; ESCs will beep
   - Lower throttle; ESCs will arm
   - Wait for startup tones (calibration complete)
- 
Software ESC Configuration

  - Set MOT_PWM_TYPE = DShot600 in Mission Planner
  - Use Motor Test tab to verify each motor spins correctly 

Motor Mapping
  - Use Motor Test to verify ArduPilot motor order:
  - Motor 1 (front right) → CW
  - Motor 2 (rear right) → CCW
  - Motor 3 (rear left) → CW
  - Motor 4 (front left) → CCW
  - Ensure props are mounted in the correct CW/CCW orientation
  
## V. Sensor Calibration

1. Accelerometer Calibration
   - In Mission Planner: SETUP → Mandatory Hardware → Accel Calibration
   - Place drone in 6 orientations as prompted
   - Wait for 'Calibration Successful' message
   - Reboot drone to apply changes

2. Compass Calibration

   - In Mission Planner: SETUP → Mandatory Hardware → Compass
   - Set UAVCAN compass to priority 1
   - Enable all used compasses and Auto Learn Offsets
   - Click Start and rotate drone through 6 orientations
   - Wait for success tone and reboot drone
   - Verify correct heading on Mission Planner map

## VI. Preflight and Flight Operations

1. Final Preflight Checklist
   - Batteries connected and secured
   - Props secure and spin freely
   - GPS lock confirmed (3DGPS, ≥10 satellites, HDOP < 1.5)
   - Flight mode set (Stabilize/Loiter recommended)
   - Armed in Mission Planner or via transmitter switch
   - HUD and telemetry all nominal in Mission Planner

2. Arming
   - Enable safety switch (hold until LED is solid)
   - Arm via transmitter switch, Mission Planner button, or MAVLink command
   - Props should spin slowly, indicating armed state

3. Takeoff and Landing

   - Manual Takeoff: Gradually increase throttle or use takeoff switch
   - Auto Takeoff: Use Mission Planner takeoff command from Plan tab
   - Manual Landing: Slowly reduce throttle or switch to Land mode
   - Auto Landing: Use RTL or Land mission command
   - Wait until all props stop spinning before approaching the drone

**Refer to the X4 Standard Manual for more information.**