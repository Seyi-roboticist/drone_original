"""
Run a single-waypoint mission on Copter.
"""

import math
import rclpy
import time

from rclpy.node import Node
from builtin_interfaces.msg import Time
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPoseStamped
from geopy import distance
from geopy import point
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from geographic_msgs.msg import GeoPointStamped

import numpy as np
from pyproj import Geod, Transformer
from scipy.spatial.transform import Rotation as R

COPTER_MODE_GUIDED = 4

# For explanation of the frames: see https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_RELATIVE_ALT
FRAME_GLOBAL_INT = 5
FRAME_LOCAL_FRD = 20 

"""
This script is compatible with ArduPilot (AP) version 4.6. Waypointing commands through ROS2/DDS is 
supported for Plane in version 4.5, but waypointing is supported for Copter in version 4.6 and later. 
Waypointing is set by the cmd_gps_pose topic (and published in this script) and defined in the 
ardupilot_msgs.msg.GlobalPosition message. AP take this message, and handles the path generation

In Copter-4.5, the AP_ExternalControl_Copter::set_global_position() function is not yet defined in ardupilot > ArduCopter > AP_ExternalControlCopter.hpp/cpp
It is a very small change that completely disables this functionality; you can refer to a later version of the same document to see how it is defined.  

As of Copter-4.6, waypointing longitude and latitude can only be inputted in degrees.
For relative waypointing, there are a couple approaches: 
    1. Modify GlobalPosition to include the other frames, this likely means forking the AP repo and modifying how GlobalPosition is defined and handled 
        An useful place to modify is AP_DDS_ExternalControl.cpp, but that seems rather difficult, and any hardware may not support 
        the forked repo. 
    2. Use GlobalPosition as is now. Modify this script to take the relative positions that the user gives, the current pose of
        the copter, and computes it in FRAME_GLOBAL_INT. 
        This is the approach this script is taking. However, some calculations may need to be verified and tested  
    2.5. Use geometry_msgs Twists to send velocity commands until the copter gets to the way point, but that involves figuring out extra controls 
    3. Figure out a way to send commands directly to MaxProxy. 
"""
# Note - Altitude in geopy is in km!

class CopterWaypointFollower(Node):
    """Copter follow waypoints using guided control."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("copter_waypoint_follower")

        # goal default parameters
        self.declare_parameter("longitude", 149.17)
        self.declare_parameter("latitude", -35.3627)
        self.declare_parameter("alt", 0.586)
        self.declare_parameter("frame", FRAME_LOCAL_FRD)

        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._client_arm = self.create_client(ArmMotors, self._arm_topic)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        self.declare_parameter("global_position_topic", "/ap/cmd_gps_pose")
        self._global_pos_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
        self._global_pos_pub = self.create_publisher(GlobalPosition, self._global_pos_topic, 1)

        # Create subscriptions after services are up
        self.declare_parameter("geopose_topic", "/ap/geopose/filtered")
        self._geopose_topic = self.get_parameter("geopose_topic").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )
        self._subscription_geopose = self.create_subscription(GeoPoseStamped, self._geopose_topic, self.geopose_cb, qos)
        self._cur_geopose = GeoPoseStamped()
        
        self.declare_parameter("goal_topic", "/ap/goal_lla")
        self._goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1
        )

        self._subscription_goal = self.create_subscription(GeoPointStamped, self._goal_topic, self.goal_cb, qos)
        self._cur_goal = GeoPointStamped()

        self.declare_parameter("global_origin", "/ap/gps_global_origin/filtered")
        self._global_origin = self.get_parameter("global_origin").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )

        self._subscription_origin = self.create_subscription(GeoPoseStamped, self._global_origin, self.global_origin_cb, qos)
        self._glob_origin = GeoPointStamped()
        
    def global_origin_cb(self, msg: GeoPointStamped):
        """Process a Goal message."""
        stamp = msg.header.stamp
        # Store current state
        self._glob_origin = msg

    def geopose_cb(self, msg: GeoPoseStamped):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec:
            self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))
            # Store current state
            self._cur_geopose = msg
            
    def goal_cb(self, msg: GeoPointStamped):
        """Process a Goal message."""
        stamp = msg.header.stamp
        self.get_logger().info("From AP : Goal [sec:{}, nsec: {}, lat:{} lon:{}]"
                               .format(stamp.sec, stamp.nanosec,msg.position.latitude, msg.position.longitude))

        # Store current state
        self._cur_goal = msg

    def frd_frame_to_global(self, offset_front, offset_right, offset_down):
        """
        Compute new global coordinates after applying a local offset in FRD.
        
        :param offset_front, offset_right, offset_down: offset in Front-Right-Down coordinate system in meters
        :return: (new_lat, new_lon, new_alt) in degrees and m
        """ 
        offset_local = np.array([offset_front, offset_right, -offset_down])  # convert to ENU
        quat = np.array([self._cur_geopose.pose.orientation.x, self._cur_geopose.pose.orientation.y, self._cur_geopose.pose.orientation.z, self._cur_geopose.pose.orientation.w])
        r = R.from_quat(quat)
        offset_enu = r.apply(offset_local)

        # Use pyproj to compute new lat/lon from ENU
        transformer = Transformer.from_crs("epsg:4979", "epsg:4979", always_xy=True)  # identity (we just need ellipsoid)
        geod = Geod(ellps="WGS84")

        # Compute horizontal (East, North) distance and azimuth
        east, north, up = offset_enu
        horizontal_dist = np.hypot(east, north)
        azimuth = np.degrees(np.arctan2(east, north))

        # Compute new lat/lon using forward geodetic
        new_lon, new_lat, _ = geod.fwd(self._cur_geopose.pose.position.longitude, self._cur_geopose.pose.position.latitude, azimuth, horizontal_dist)

        # Adjust altitude
        new_alt = self._cur_geopose.pose.position.altitude + up

        return new_lat, new_lon, new_alt

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        assert mode in [COPTER_MODE_GUIDED]
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode:
            result = self.switch_mode(desired_mode)
            # Handle successful switch or the case that the vehicle is already in expected mode
            is_in_desired_mode = result.status or result.curr_mode == desired_mode
            time.sleep(1)

        return is_in_desired_mode

    def get_cur_geopose(self):
        """Return latest geopose."""
        return self._cur_geopose
    
    def get_cur_goal(self):
        """Return latest goal."""
        return self._cur_goal 

    def send_goal_position(self, goal_global_pos):
        """Send goal position. Must be in guided for this to work."""
        self._global_pos_pub.publish(goal_global_pos)


def achieved_goal(goal_global_pos, cur_geopose):
    """Return true if the current position (LLH) is close enough to the goal (within the orbit radius)."""
    # Use 3D geopy distance calculation
    # https://geopy.readthedocs.io/en/stable/#module-geopy.distance

    p1 = (goal_global_pos.latitude, goal_global_pos.longitude, goal_global_pos.altitude)
    cur_pos = cur_geopose.pose.position
    p2 = (cur_pos.latitude, cur_pos.longitude, cur_pos.altitude)

    flat_distance = distance.distance(p1[:2], p2[:2]).m
    euclidian_distance = math.sqrt(flat_distance**2 + (p2[2] - p1[2]) ** 2)
    print(f"Goal is {euclidian_distance} meters away")
    return euclidian_distance < 150

def going_to_goal(goal_global_pos, cur_goal):
    p1 = (goal_global_pos.latitude, goal_global_pos.longitude, goal_global_pos.altitude)
    cur_pos_lla = cur_goal.position
    p2 = (cur_pos_lla.latitude, cur_pos_lla.longitude, cur_pos_lla.altitude)

    flat_distance = distance.distance(p1[:2], p2[:2]).m
    euclidian_distance = math.sqrt(flat_distance**2 + (p2[2] - p1[2]) ** 2)
    # print(f"Commanded and received goal are {euclidian_distance} meters away")
    return euclidian_distance < 1    

def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = CopterWaypointFollower()
    try:
        # Block till armed, which will wait for EKF3 to initialize
        if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
            raise RuntimeError("Unable to arm")

        # Block till in takeoff
        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to takeoff mode")

        is_ascending_to_takeoff_alt = True
        while is_ascending_to_takeoff_alt:
            rclpy.spin_once(node)
            time.sleep(1.0)

            # Hard code waiting in takeoff to reach operating altitude of 630m
            # This is just a hack because geopose is reported with absolute rather than relative altitude,
            # and this node doesn't have access to the terrain data
            is_ascending_to_takeoff_alt = node.get_cur_geopose().pose.position.altitude < node._glob_origin.position.altitude

        if is_ascending_to_takeoff_alt:
            raise RuntimeError("Failed to reach takeoff altitude")

        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")
        print("Setting goal now")
        # Send a guided WP with location, frame ID, alt frame
        goal_pos = GlobalPosition()
        goal_pos.header.frame_id = "map"
        goal_pos.coordinate_frame = FRAME_GLOBAL_INT
        if node.get_parameter("frame").get_parameter_value().integer_value == FRAME_GLOBAL_INT:
            goal_pos.latitude = node.get_parameter("latitude").get_parameter_value().double_value
            goal_pos.longitude = node.get_parameter("longitude").get_parameter_value().double_value
            goal_pos.altitude = node.get_parameter("alt").get_parameter_value().double_value
        elif node.get_parameter("frame").get_parameter_value().integer_value == FRAME_LOCAL_FRD:
            new_lat, new_lon, new_alt = node.frd_frame_to_global(node.get_parameter("latitude").get_parameter_value().double_value, 
                                                                    node.get_parameter("longitude").get_parameter_value().double_value,
                                                                    node.get_parameter("alt").get_parameter_value().double_value)
            goal_pos.latitude = new_lat
            goal_pos.longitude = new_lon
            goal_pos.altitude = new_alt

        
        node.send_goal_position(goal_pos)

        start = node.get_clock().now()
        has_achieved_goal = False
        is_going_to_goal = False
        while not has_achieved_goal and node.get_clock().now() - start < rclpy.duration.Duration(seconds=120):
            rclpy.spin_once(node)
            is_going_to_goal = going_to_goal(goal_pos, node.get_cur_goal())
            has_achieved_goal = achieved_goal(goal_pos, node.get_cur_geopose())
            time.sleep(1.0)

        if not is_going_to_goal:
            raise RuntimeError("Unable to go to goal location")
        if not has_achieved_goal:
            raise RuntimeError("Unable to achieve goal location")

        print("Goal achieved")

    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
