#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
import math
import statistics
import threading
from enum import Enum, auto
from detect_cart import DetectCart
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
"""
[geometry_msgs/Twist]:
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
"""
"""
user:~$ rosmsg show sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
"""

"""
[nav_msgs/Odometry]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
"""

class AttachState(Enum):
    INITIALIZING = auto()
    ALIGN_WITH_CART_ORIENTATION = auto()
    ALIGN_WITH_CART_Y = auto()
    MOVE_INTO_CART = auto()
    ATTACH = auto()

class AttachToCart():
    ORIENTATION_FUZZ = 0.01
    TRANSFORM_FUZZ = 0.01
    LINEAR_VELOCITY = 0.1
    ANGULAR_VELOCITY = 0.3
    ROBOT_BASE_LINK = "robot_base_link"
    CMD_VEL_TOPIC = "/robot/cmd_vel"
    
    def __init__(self):
        self._ctrl_c = False
        self._listener = tf.TransformListener()
        self._rate = rospy.Rate(1)
        rospy.on_shutdown(self._shutdownhook)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom)
        self._cmd_vel_pub = rospy.Publisher(AttachToCart.CMD_VEL_TOPIC, Twist)

        self._static_frame_trans = None
        self._static_frame_rot = None

        self._attach_state = AttachState.INITIALIZING
        self.__print_state()
        self.__begin_dock()
        

    def __print_state(self):
        rospy.loginfo(f"{self._attach_state=}")

    def __odom(self, odometry: Odometry):
        if self._attach_state == AttachState.INITIALIZING:
            return
        elif self._attach_state == AttachState.ALIGN_WITH_CART_ORIENTATION:
            robot_yaw = odometry.pose.pose.orientation.z
            goal_yaw = -self._static_frame_rot[2]
            yaw_diff = abs(goal_yaw - robot_yaw)
            rospy.loginfo(f"{yaw_diff=}")
            if yaw_diff < AttachToCart.ORIENTATION_FUZZ:
                twist = Twist()
                self._attach_state = AttachState.ALIGN_WITH_CART_Y
                self.__print_state()
                self._cmd_vel_pub.publish(twist)
            else:
                sign = 1 if robot_yaw < goal_yaw else -1
                twist = Twist()
                twist.angular.z = sign * AttachToCart.ANGULAR_VELOCITY
                self._cmd_vel_pub.publish(twist)

    
    def __get_transform(self, child_frame: str, parent_frame: str):
        # Get static_frame tf
        while not self._ctrl_c:
            rospy.loginfo("Waiting for {parent_frame} to {child_frame} tf...")
            self._listener.waitForTransform(
                child_frame, parent_frame, rospy.Time(0), rospy.Duration(3))
            rospy.loginfo("...received {parent_frame} to {child_frame} tf.")
            try:
                trans, rot = self._listener.lookupTransform(
                    child_frame, parent_frame, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return trans, rot

    def __begin_dock(self):

        # Get static_frame tf
        self._static_frame_trans, self._static_frame_rot = self.__get_transform(
            DetectCart.CART_FRAME, DetectCart.ODOM_FRAME)
        rospy.loginfo(f"{self._static_frame_trans=}")
        rospy.loginfo(f"{self._static_frame_rot=}")
        self._attach_state = AttachState.ALIGN_WITH_CART_ORIENTATION
        self.__print_state()
        
    def _shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self._ctrl_c = True
            
if __name__ == '__main__':
    rospy.init_node('attach_to_cart', anonymous=True)
    AttachToCart()
    rospy.spin()