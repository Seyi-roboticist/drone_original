"""
Run takeoff on Copter on Guided Mdoe through MAVROS commands.
"""

import rclpy
import time

from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from sensor_msgs.msg import NavSatFix

class CopterTakeoff(Node):
    """Copter takeoff using guided control."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("copter_takeoff")

        self.declare_parameter("alt", 15.0)
        self.declare_parameter("mode", "GUIDED")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        self._mode_topic = "/mavros/set_mode"
        self._client_mode_switch = self.create_client(SetMode, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        self._arm_topic = "/mavros/cmd/arming"
        self._client_arm = self.create_client(CommandBool, self._arm_topic)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self._takeoff_topic = "/mavros/cmd/takeoff"
        self._client_takeoff = self.create_client(CommandTOL, self._takeoff_topic)
        while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        self._geopose_topic = "/mavros/global_position/global"
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )
        self._subscription_geopose = self.create_subscription(NavSatFix, self._geopose_topic, self.geopose_cb, qos)
        self._cur_geopose = NavSatFix()

        self._home_topic = "/mavros/home_position/home"
        self._subscription_geopose = self.create_subscription(HomePosition, self._home_topic, self.homepose_cb, qos)
        self._homepose = HomePosition()

    def geopose_cb(self, msg: NavSatFix):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec:

            # Store current state
            self._cur_geopose = msg

    def homepose_cb(self, msg: HomePosition):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec:
            # Store current state
            self._homepose = msg
            self._home_alt = msg.geo.altitude

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().success
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        req = SetMode.Request()
        # req.base_mode = 0
        req.custom_mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode and self.get_clock().now() - start < timeout:
            result = self.switch_mode(desired_mode)
            is_in_desired_mode = result.mode_sent 
            time.sleep(1)

        return is_in_desired_mode

    def takeoff(self, alt):
        req = CommandTOL.Request()
        req.latitude = self._homepose.geo.latitude
        req.longitude = self._homepose.geo.longitude
        req.altitude = self._homepose.geo.altitude + alt
        print(f"goal alt: {req.altitude}")
        future = self._client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff_with_timeout(self, alt: int, timeout: rclpy.duration.Duration):
        """Try to takeoff. Returns true on success, or false if takeoff fails or times out."""
        takeoff_success = False
        start = self.get_clock().now()
        while not takeoff_success and self.get_clock().now() - start < timeout:
            result = self.takeoff(alt)
            takeoff_success = result.success
            time.sleep(1)

        return takeoff_success

    def get_cur_geopose(self):
        """Return latest geopose."""
        return self._cur_geopose


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = CopterTakeoff()
    takeoff_alt = node.get_parameter("alt").get_parameter_value().double_value
    
    try:
        if not node.switch_mode_with_timeout("GUIDED", rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")
        
        # Block till armed, which will wait for EKF3 to initialize
        if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
            raise RuntimeError("Unable to arm")

        #time.sleep(1.0)
        # Block till in takeoff
        if not node.takeoff_with_timeout(takeoff_alt, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to takeoff")

        is_ascending_to_takeoff_alt = True
        while is_ascending_to_takeoff_alt:
            rclpy.spin_once(node)
            time.sleep(1.0)

            is_ascending_to_takeoff_alt = node.get_cur_geopose().altitude - node._home_alt < takeoff_alt

        if is_ascending_to_takeoff_alt:
            raise RuntimeError("Failed to reach takeoff altitude")

    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
