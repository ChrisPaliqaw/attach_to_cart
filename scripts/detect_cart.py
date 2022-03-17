#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
import math
import statistics
import threading
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

class DetectCart():
    PLATE_INTENSITY = 8000
    # Intense readings in order to be able to detect a plate
    PLATE_DETECTION_FAILURE_THRESHOLD = 6

    SCAN_TOPIC = "scan"
    ODOM_FRAME = "robot_odom"
    HALF_PLATE_GAP = 0.3
    CART_FRAME = "static_cart"
    ROBOT_LASER_BASE_LINK = "robot_front_laser_link"
    RANGE_MAX = 20.0

    @staticmethod
    def yawAndDistanceToRosXY(yaw, distance):
        if math.isinf(distance):
            distance = DetectCart.RANGE_MAX
        rospy.loginfo(f"New distance is {distance}")
        theta = (math.pi / 2) + yaw
        y = -(distance * (math.cos(theta)))
        x = (distance * (math.sin(theta)))
        rospy.loginfo(f"{theta=} {x=} {y=}")
        return x, y
    
    # https://stackoverflow.com/questions/1243614/how-do-i-calculate-the-normal-vector-of-a-line-segment#:~:text=The%20normal%20vector%20(x'%2C,or%20(dx%2Cdy)%20.&text=Show%20activity%20on%20this%20post.,-m1%20%3D%20(y2%20%2D&text=m2%20%3D%20%2D1%20%2F%20m1%20%2F%2F,offset%20of%20new%20perpendicular%20line..
    @staticmethod
    def surfaceNormal(x1: float, y1: float, x2: float, y2: float) -> float:
        return (x2 - x1) / (y2 - y1)

    @staticmethod
    def getAverageHighIntensityIndex(laser_scan: LaserScan) -> int:
        high_intensity_indexes = []
        for i in range(len(laser_scan.intensities)):
            if laser_scan.intensities[i] == DetectCart.PLATE_INTENSITY:
                high_intensity_indexes.append(i)
        if len(high_intensity_indexes) < DetectCart.PLATE_DETECTION_FAILURE_THRESHOLD:
            raise ValueError(f"{len(high_intensity_indexes)=} < {DetectCart.PLATE_DETECTION_FAILURE_THRESHOLD=}")
        return statistics.mean(high_intensity_indexes)
    
    def __init__(self):
        self._transform_broadcaster = tf.TransformBroadcaster()
        self._ctrl_c = False
        self._listener = tf.TransformListener()
        self._rate = rospy.Rate(1)
        rospy.on_shutdown(self._shutdownhook)
        self._base_link_trans = None
        self._base_link_rot = None
        self._lock = threading.Lock()
        self.__robot_odom_tf()
        self._scan_sub = rospy.Subscriber(DetectCart.SCAN_TOPIC, LaserScan, self.__scan)

    def __robot_odom_tf(self):
        with self._lock:
            while not self._ctrl_c:
                
                # This is another option to guarantee that we wait until teh transofrms are ready
                # This avoids also the issue of not finding the frame by time out
                rospy.loginfo("Waiting for odom to base_link tf...")
                self._listener.waitForTransform(
                    DetectCart.ODOM_FRAME, DetectCart.ROBOT_LASER_BASE_LINK, rospy.Time(0), rospy.Duration(3))
                rospy.loginfo("...received odom to base_link tf.")
                try:
                    self._base_link_trans, self._base_link_rot = self._listener.lookupTransform(
                        DetectCart.ODOM_FRAME, DetectCart.ROBOT_LASER_BASE_LINK, rospy.Time(0))
                    return
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        
    def __scan(self, laser_scan: LaserScan):
        with self._lock:
            if self._base_link_trans is None or self._base_link_rot is None:
                return
            else:
                self._scan_sub.unregister()

            center_plate_index = None
            try:
                center_plate_index = DetectCart.getAverageHighIntensityIndex(laser_scan)
            except ValueError as v:
                rospy.logerr(f"{v}")
                return

            left_indexes = []
            left_ranges = []
            right_indexes = []
            right_ranges = []
            for index, intensity in enumerate(laser_scan.intensities):
                if intensity == DetectCart.PLATE_INTENSITY:
                    if index < center_plate_index:
                        left_indexes.append(index)
                        left_ranges.append(laser_scan.ranges[index])
                    else:
                        right_indexes.append(index)
                        right_ranges.append(laser_scan.ranges[index])
            left_plate_range = statistics.mean(left_ranges)
            right_plate_range = statistics.mean(right_ranges)
            left_plate_index = round(statistics.mean(left_indexes))
            right_plate_index = round(statistics.mean(right_indexes))

            center_index = len(laser_scan.intensities) // 2
            left_plate_yaw = (left_plate_index - center_index) * laser_scan.angle_increment
            right_plate_yaw = (right_plate_index - center_index) * laser_scan.angle_increment

            lx, ly = self.tf_relative_to_robot(left_plate_yaw, left_plate_range)
            rx, ry = self.tf_relative_to_robot(right_plate_yaw, right_plate_range)

            rospy.loginfo(f"{(lx, ly, rx, ry)=}")
            slope_surface_normal = DetectCart.surfaceNormal(lx, ly, rx, ry)
            rospy.loginfo(f"{slope_surface_normal=}")
            radians_surface_normal = math.atan(slope_surface_normal)
            orientation_quaternion = tf.transformations.quaternion_from_euler(
                0, 0, radians_surface_normal)
            cfx = statistics.mean((lx, rx))
            cfy = statistics.mean((ly, ry))

            broadcaster = tf.TransformBroadcaster()
            while not self._ctrl_c:
                broadcaster.sendTransform((cfx, cfy, 0),
                                                    orientation_quaternion,
                                                    rospy.Time.now(),
                                                    DetectCart.CART_FRAME,
                                                    DetectCart.ODOM_FRAME)
                self._rate.sleep()
        
    def tf_relative_to_robot(self, yaw, distance) -> (float, float):
        odom_to_base_link_euler = tf.transformations.euler_from_quaternion(self._base_link_rot)
        rospy.loginfo(f"{odom_to_base_link_euler=}")
        yaw -= odom_to_base_link_euler[2]
        x, y = DetectCart.yawAndDistanceToRosXY(yaw, distance)

        x = self._base_link_trans[0] + x
        y = self._base_link_trans[1] - y

        return x, y
        
    def _shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self._ctrl_c = True
            
if __name__ == '__main__':
    rospy.init_node('detect_cart', anonymous=True)
    DetectCart()
    rospy.spin()