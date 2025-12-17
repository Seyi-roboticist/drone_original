import rclpy
import time
import traceback

from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import math

class CopterTakeoff(Node):
    """Copter waypoints using guided control."""

    def __init__(self):
        super().__init__("copter_waypoint")

        self.declare_parameter("alt", 10.0)

        self._client_param = self.create_client(SetParameters, '/mavros/param/set_parameters')
        while not self._client_param.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/param/set_parameters service...')

        self._client_mode_switch = self.create_client(SetMode, "/mavros/set_mode")
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode switch service...')

        self._takeoff_topic = "/mavros/cmd/takeoff"
        self._client_takeoff = self.create_client(CommandTOL, self._takeoff_topic)
        while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        self._client_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm service...')

        self._land_topic = "/mavros/cmd/land"
        self._client_land = self.create_client(CommandTOL, self._land_topic)
        while not self._client_land.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('landing service not available, waiting again...')

        self._rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 1)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )
        self._subscription_geopose = self.create_subscription(NavSatFix, "/mavros/global_position/global", self.geopose_cb, qos)
        self._subscription_homepose = self.create_subscription(HomePosition, "/mavros/home_position/home", self.homepose_cb, qos)

        self._cur_geopose = NavSatFix()
        self._homepose = HomePosition()
        self._home_alt = 0.0
        self._home_received = False

    def geopose_cb(self, msg: NavSatFix):
        if msg.header.stamp.sec:
            self._cur_geopose = msg
    
    def homepose_cb(self, msg: HomePosition):
        if msg.header.stamp.sec:
            self._homepose = msg
            self._home_alt = msg.geo.altitude
            self._home_received = True

    def wait_for_home_position(self, timeout: rclpy.duration.Duration):
        self.get_logger().info("Waiting for home position...")
        start = self.get_clock().now()
        while not self._home_received and self.get_clock().now() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._home_received

    def set_param(self, name: str, value: int):
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)

        req = SetParameters.Request()
        req.parameters = [param]

        future = self._client_param.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_param_with_timeout(self, name: str, value: int, timeout: rclpy.duration.Duration):
        start = self.get_clock().now()
        while self.get_clock().now() - start < timeout:
            result = self.set_param(name, value)
            if result and all(r.successful for r in result.results):
                self.get_logger().info(f"Successfully set param {name}={value}")
                return True
            self.get_logger().warn(f"Failed to set param {name}={value}. Retrying in 1s...")
            time.sleep(1)
        self.get_logger().error(f"Timeout while setting param {name}")
        return False

    def arm(self, arm: bool):
        req = CommandBool.Request()
        req.value = arm
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, arm: bool, timeout: rclpy.duration.Duration):
        start = self.get_clock().now()
        while self.get_clock().now() - start < timeout:
            result = self.arm(arm)
            if result.success:
                self.get_logger().info("Drone armed" if arm else "Drone disarmed")
                return True
            self.get_logger().warn("Arming failed. Retrying...")
            time.sleep(1)
        self.get_logger().error("Arming timeout")
        return False

    def switch_mode(self, mode):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: str, timeout: rclpy.duration.Duration):
        start = self.get_clock().now()
        while self.get_clock().now() - start < timeout:
            result = self.switch_mode(desired_mode)
            if result and result.mode_sent:
                self.get_logger().info(f"Mode switched to {desired_mode}")
                return True
            self.get_logger().warn(f"Failed to switch to mode {desired_mode}. Retrying...")
            time.sleep(1)
        self.get_logger().error(f"Mode switch to {desired_mode} timed out")
        return False

    def takeoff(self, alt):
        req = CommandTOL.Request()
        req.latitude = self._homepose.geo.latitude
        req.longitude = self._homepose.geo.longitude
        req.altitude = self._home_alt + alt
        self.get_logger().info(f"Home altitude: {self._home_alt:.2f}")
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
    
    def land(self):
        req = CommandTOL.Request()
        # if self.get_parameter("rtl").get_parameter_value().bool_value:
        req.latitude = self._homepose.geo.latitude
        req.longitude = self._homepose.geo.longitude
        # else:
            # req.latitude = self._cur_geopose.latitude
            # req.longitude = self._cur_geopose.longitude
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
    rclpy.init(args=args)
    node = CopterTakeoff()
    time_to_reach_goal = 15
    tolerance = 0.1
    try:
        takeoff_alt = node.get_parameter("alt").get_parameter_value().double_value

        # if not node.set_param_with_timeout("SYSID_MYGCS", 1, rclpy.duration.Duration(seconds=20)):
        #     raise RuntimeError("Failed to set SYSID_MYGCS")

        # if not node.set_param_with_timeout("RC_OPTIONS", 64, rclpy.duration.Duration(seconds=20)):
        #     raise RuntimeError("Failed to set RC_OPTIONS")

        if not node.switch_mode_with_timeout("GUIDED", rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Failed to switch to DUIDED mode")

        if not node.arm_with_timeout(True, rclpy.duration.Duration(seconds=30)):
            raise RuntimeError("Failed to arm drone")

        if not node.wait_for_home_position(rclpy.duration.Duration(seconds=5)):
            raise RuntimeError("Home position not received in time")
        
        time.sleep(1.0)
        # Block till in takeoff
        if not node.takeoff_with_timeout(takeoff_alt, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to takeoff")

        is_ascending_to_takeoff_alt = True
        while is_ascending_to_takeoff_alt:
            rclpy.spin_once(node)
            time.sleep(1.0)

            is_ascending_to_takeoff_alt = node.get_cur_geopose().altitude - node._home_alt < takeoff_alt


        node.get_logger().info(f"Moving to first waypoint")
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = 0.0
        target.pose.position.y = 10.0
        target.pose.position.z = takeoff_alt
        target.pose.orientation.w = 1.0  # facing forward

        goal_start = time.time()
        reached_goal = False
        while time.time() - goal_start < time_to_reach_goal and not reached_goal:
            tgt = target.pose.position
            dist = math.sqrt((node._cur_geopose.longitude - tgt.x)**2 + (node._cur_geopose.latitude - tgt.y)**2 + (node._cur_geopose.altitude - tgt.z)**2)
            reached_goal = dist < tolerance
            node.waypoint_pub.publish(target)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(1)

        node.get_logger().info(f"Moving to second waypoint")
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = 10.0
        target.pose.position.y = 10.0
        target.pose.position.z = takeoff_alt
        target.pose.orientation.w = 1.0  # facing forward

        goal_start = time.time()
        reached_goal = False
        while time.time() - goal_start < time_to_reach_goal and not reached_goal:
            tgt = target.pose.position
            dist = math.sqrt((node._cur_geopose.longitude - tgt.x)**2 + (node._cur_geopose.latitude - tgt.y)**2 + (node._cur_geopose.altitude - tgt.z)**2)

            reached_goal = dist < tolerance
            node.waypoint_pub.publish(target)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(1)

        node.get_logger().info(f"Moving to third waypoint")
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = 10.0
        target.pose.position.y = 0.0
        target.pose.position.z = takeoff_alt
        target.pose.orientation.w = 1.0  # facing forward

        goal_start = time.time()
        reached_goal = False
        while time.time() - goal_start < time_to_reach_goal and not reached_goal:
            tgt = target.pose.position
            dist = math.sqrt((node._cur_geopose.longitude - tgt.x)**2 + (node._cur_geopose.latitude - tgt.y)**2 + (node._cur_geopose.altitude - tgt.z)**2)

            reached_goal = dist < tolerance
            node.waypoint_pub.publish(target)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(1)

        node.get_logger().info("Initiating landing")
        if not node.land_with_timeout(rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to land")

        node.get_logger().info("Flight complete")

    except Exception as e:
        node.get_logger().error(f"Exception occurred: {e}")
        node.get_logger().error(traceback.format_exc())

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
