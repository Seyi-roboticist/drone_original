#!/usr/bin/env python3
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import tkinter as tk
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandTOL


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('MAVROS_GUI')
        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        # Client
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # State
        self.last_command    = ''
        self.velocity        = 0.0
        self.last_char       = ''

        self.root = tk.Tk()
        self.root.title('MAVROS GUI')

        frame = tk.Frame(self.root)
        frame.pack(padx=10, pady=10)

        # Velocity slider
        tk.Label(frame, text='Velocity:').grid(row=0, column=0, sticky='e')
        self.vel_scale = tk.Scale(
            frame,
            from_=0.0,
            to=10.0,
            resolution=0.1,
            orient='horizontal',
            length=200
        )
        self.vel_scale.set(self.velocity)
        self.vel_scale.grid(row=0, column=1, padx=(0,10))

        # Command display (read-only)
        tk.Label(frame, text='Command:').grid(row=0, column=2, sticky='e')
        self.cmd_var = tk.StringVar(value='')
        self.cmd_entry = tk.Entry(
            frame,
            textvariable=self.cmd_var,
            width=4,
            state='readonly'
        )
        self.cmd_entry.grid(row=0, column=3)

        # First row of buttons
        btn_start = tk.Button(frame, text='Arm', command=self.arm_throttle)
        btn_start.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(10,0))
        btn_guided = tk.Button(frame, text='Disarm',  command=self.disarm_throttle)
        btn_guided.grid( row=1, column=2, columnspan=2, sticky='ew', pady=(10,0))

        # Second row of buttons
        btn_takeoff = tk.Button(frame, text='Takeoff 15m (Not Working)',    command=self.takeoff)
        btn_takeoff.grid(row=2, column=0, columnspan=2, sticky='ew', pady=(5,0))
        btn_arm = tk.Button(frame, text='Land', command=self.land)
        btn_arm.grid(row=2, column=2, columnspan=2, sticky='ew', pady=(5,0))

        # Third row of button
        btn_demo = tk.Button(frame, text='Guided', command=self.guided_mode)
        btn_demo.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(10,0))
        btn_demo = tk.Button(frame, text='Loiter', command=self.loiter_mode)
        btn_demo.grid(row=3, column=2, columnspan=2, sticky='ew', pady=(10,0))

        self.root.bind('<KeyPress>', self.on_key_press)
        self.create_timer(0.1, self.timer_callback)

    def guided_mode(self):
        try:
            subprocess.Popen('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: \'GUIDED\'}"', shell=True)
            self.get_logger().info('Sent "mode GUIDED" command')
        except Exception as e:
            self.get_logger().error(f'Failed to send GUIDED mode command: {e}')

    def loiter_mode(self):
        try:
            subprocess.Popen('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: \'LOITER\'}"', shell=True)
            self.get_logger().info('Sent "mode LOITER" command')
        except Exception as e:
            self.get_logger().error(f'Failed to send LOITER mode command: {e}')

    def arm_throttle(self):
        try:
            subprocess.Popen('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"', shell=True)
            self.get_logger().info('Sent "arm throttle" command')
        except Exception as e:
            self.get_logger().error(f'Failed to send arm throttle command: {e}')

    def disarm_throttle(self):
        try:
            subprocess.Popen('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"', shell=True)
            self.get_logger().info('Sent "disarm throttle" command')
        except Exception as e:
            self.get_logger().error(f'Failed to send arm throttle command: {e}')

    def takeoff(self):
        try:
            subprocess.Popen('ros2 run x4_service_scripts mavros_takeoff', shell=True)
            self.get_logger().info('Sent "Takeoff Script" command')
        except Exception as e:
            self.get_logger().error(f'Failed to send takeoff command: {e}')
            

    def land(self):
        try:
            subprocess.Popen('ros2 run x4_service_scripts mavros_landing', shell=True)
            self.get_logger().info('Sent "Landing Script" command')
        except Exception as e:
            self.get_logger().error(f'Failed to send landing command: {e}')

    # def start_demo(self):
    #     # run the demo sequence in a separate thread so UI remains responsive
    #     threading.Thread(target=self.demo_sequence, daemon=True).start()
    # 
    # def demo_sequence(self):
    #     cmds = [
    #         'ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: \'GUIDED\'}"',
    #         'ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"',
    #         'ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 1.0, yaw: 1.0, latitude: 0.0, longitude: 130, altitude: 600.0}"',
    #         'ros2 service call /mavros/cmd/takeoff_local mavros_msgs/srv/CommandTOLLocal "{min_pitch: 1.0, position: {x: 1, y: 2, z: 5.0}}"',
    #         'ros2 service call /mavros/cmd/command_long mavros_msgs/srv/CommandLong "{command: 19, param1: 5, longitude: 149.1652}"',
    #         'ros2 service call /mavros/cmd/command_int mavros_msgs/srv/CommandInt "{command: 192, param3: 1, x: -19.0, y: 120.0, z: 20.0}"',
    #         'ros2 service call /mavros/mission/push mavros_msgs/srv/WaypointPush "{start_index: 0, waypoints: {x_lat: 10, y_long: 20}}"',
    #         'ros2 service call /mavros/cmd/command_int mavros_msgs/srv/CommandInt "{command: 300}"',
    #     ]
    #     for cmd in cmds:
    #         try:
    #             subprocess.Popen(cmd, shell=True)
    #             self.get_logger().info(f'Demo: ran `{cmd}`')
    #         except Exception as e:
    #             self.get_logger().error(f'Demo: failed to run `{cmd}`: {e}')
    #         time.sleep(3)

    def on_key_press(self, event):
        self.last_char = event.char
        self.cmd_entry.config(state='normal')
        self.cmd_var.set(self.last_char)
        self.cmd_entry.config(state='readonly')

        mapping = {'w':'forward','a':'left','s':'back','d':'right','q':'rotateleft','e':'rotateright', 'c':'stop'}
        cmd = mapping.get(event.char.lower())
        if cmd:
            self.last_command = cmd
            self.get_logger().info(f'Key "{event.char}" → "{cmd}"')

    def timer_callback(self):
        # process UI
        try:
            self.root.update()
        except tk.TclError:
            self.get_logger().info('UI closed, shutting down...')
            rclpy.shutdown()
            return
        
        self.velocity = float(self.vel_scale.get())
        twist = Twist()

        if self.last_command == 'forward':
            twist.linear.x = self.velocity
        elif self.last_command == 'back':
            twist.linear.x = -self.velocity
        elif self.last_command == 'left':
            twist.linear.y = self.velocity
        elif self.last_command == 'right':
            twist.linear.y = -self.velocity
        elif self.last_command == 'rotateleft':
            twist.angular.z = self.velocity
        elif self.last_command == 'rotateright':
            twist.angular.z = -self.velocity
        elif self.last_command == 'stop':
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0

        self.vel_pub.publish(twist)

    def destroy_node(self):
        super().destroy_node()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
