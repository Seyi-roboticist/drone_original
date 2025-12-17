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

# landing strategy: dropping down vertically
class CopterTakeoff(Node):
    """Copter takeoff using guided control."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("copter_takeoff")

        self.declare_parameter("mode", "land")
        self.declare_parameter("rtl", "true")

        self._mode_topic = "/mavros/set_mode"
        self._client_mode_switch = self.create_client(SetMode, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        self._land_topic = "/mavros/cmd/land"
        self._client_land = self.create_client(CommandTOL, self._land_topic)
        while not self._client_land.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('landing service not available, waiting again...')

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

    def switch_mode(self, mode):
        req = SetMode.Request()
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

    def land(self):
        req = CommandTOL.Request()
        if self.get_parameter("rtl").get_parameter_value().bool_value:
            req.latitude = self._homepose.geo.latitude
            req.longitude = self._homepose.geo.longitude
        else:
            req.latitude = self._cur_geopose.latitude
            req.longitude = self._cur_geopose.longitude
        req.altitude = self._homepose.geo.altitude
        future = self._client_land.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def land_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to takeoff. Returns true on success, or false if takeoff fails or times out."""
        takeoff_success = False
        start = self.get_clock().now()
        while not takeoff_success and self.get_clock().now() - start < timeout:
            result = self.land()
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
    
    try:
        if not node.switch_mode_with_timeout("LAND", rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")

        # Block till in takeoff
        if not node.land_with_timeout(rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to land")

        rclpy.spin_once(node)
        time.sleep(1.0)

    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
