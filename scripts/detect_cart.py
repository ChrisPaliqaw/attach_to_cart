#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
import math
from statistics import mean

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
    SCAN_TOPIC = "scan"
    ODOM_FRAME = "robot_odom"
    HALF_PLATE_GAP = 0.3
    CART_FRAME = "cart_frame"
    ROBOT_BASE_LINK = "robot_base_link"
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
    
    """
    @staticmethod
    def orientation(trans1, trans2, orientation):
        # If we define dx = x2 - x1 and dy = y2 - y1, then the normals are (-dy, dx) and (dy, -dx)
        dx = trans2[0] - trans1[0]
        dy = trans2[1] - trans1[1]

        normal1 = (-dy, dx)

        

        return y, x
    """
    
    def __init__(self):
        self._transform_broadcaster = tf.TransformBroadcaster()
        self._scan_sub = rospy.Subscriber(DetectCart.SCAN_TOPIC, LaserScan, self.__scan)
        self._ctrl_c = False
        self._listener = tf.TransformListener()
        self._rate = rospy.Rate(10)
        rospy.on_shutdown(self._shutdownhook)
        self._base_link_trans = None
        self._base_link_rot = None
        self.__robot_odom_tf_loop()

    def __robot_odom_tf_loop(self):

        while not self._ctrl_c:
            
            # This is another option to guarantee that we wait until teh transofrms are ready
            # This avoids also the issue of not finding the frame by time out
            rospy.loginfo("Waiting for odom to base_link tf...")
            self._listener.waitForTransform(
                DetectCart.ODOM_FRAME, DetectCart.ROBOT_BASE_LINK, rospy.Time(0), rospy.Duration(3))
            rospy.loginfo("Received odom to base_link tf...")
            try:
                self._base_link_trans, self._base_link_rot = self._listener.lookupTransform(
                    DetectCart.ODOM_FRAME, DetectCart.ROBOT_BASE_LINK, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self._rate.sleep()
        
    def __scan(self, laser_scan: LaserScan):
        if self._base_link_trans is None or self._base_link_rot is None:
            return
    
        self.publish_tf_relative_to_robot(0, 1, "front")
        self.publish_tf_relative_to_robot(-math.pi / 6, 1, "left")
        self.publish_tf_relative_to_robot(math.pi / 6, 1, "right")
        """

        first_index = 0
        last_index = len(laser_scan.intensities) - 1
        center_index = len(laser_scan.intensities) / 2
        first_index_yaw = (first_index - center_index) * laser_scan.angle_increment
        last_index_yaw = (last_index - center_index) * laser_scan.angle_increment
        first_index_distance = laser_scan.ranges[first_index]
        last_index_distance = laser_scan.ranges[last_index]
        self.publish_tf_relative_to_robot(first_index_yaw, first_index_distance, f"first_index")
        rospy.loginfo(f"{laser_scan.ranges[first_index]=}")
        self.publish_tf_relative_to_robot(last_index_yaw, last_index_distance, f"last_index")
        rospy.loginfo(f"{laser_scan.ranges[last_index]=}")

        first_plate_index = None
        last_plate_index = None
        for index, intensity in enumerate(laser_scan.intensities):
            if intensity == DetectCart.PLATE_INTENSITY:
                if first_plate_index is None:
                    first_plate_index = index
                    last_plate_index = index
                else:
                    last_plate_index = index
        if first_plate_index is None:
            rospy.loginfo("No plate detected")
            return
        elif last_plate_index == first_plate_index:
            rospy.loginfo(f"Distant plate or UFO suspected: {first_plate_index=}")
            return
        elif first_plate_index == 0 or last_plate_index == (len(laser_scan.intensities) - 1):
            rospy.loginfo(f"Plates behind the robot")
            return

        rospy.loginfo(f"{first_plate_index=} {last_plate_index=}")
        center_gap_index = (first_plate_index + last_plate_index) // 2
        
        right_gap_index = None
        for i in range(center_gap_index, len(laser_scan.intensities)):
            if laser_scan.intensities[i] == DetectCart.PLATE_INTENSITY:
                right_gap_index = i
                break
        if right_gap_index is None:
            rospy.logerr("Couldn't locate the left side of the right plate")
            return
        left_gap_index = None
        for i in range(center_gap_index, -1, -1):
            if laser_scan.intensities[i] == DetectCart.PLATE_INTENSITY:
                left_gap_index = i
                break
        if left_gap_index is None:
            rospy.logerr("Couldn't locate the right side of the left plate")
            return
        rospy.loginfo(f"{left_gap_index=} {center_gap_index=} {right_gap_index=}")

        gap_radians = laser_scan.angle_increment * abs(left_gap_index - right_gap_index)
        rospy.loginfo(f"{gap_radians=}")

        left_gap_distance = laser_scan.ranges[left_gap_index]
        right_gap_distance = laser_scan.ranges[right_gap_index]

        left_gap_yaw = (left_gap_index - center_index) * laser_scan.angle_increment
        right_gap_yaw = (right_gap_index - center_index) * laser_scan.angle_increment

        self.publish_tf_relative_to_robot(left_gap_yaw, left_gap_distance, 'left_gap')
        self.publish_tf_relative_to_robot(right_gap_yaw, right_gap_distance, 'right_gap')
        # self.publish_tf_relative_to_robot(center_gap_yaw, center_gap_distance, DetectCart.CART_FRAME)

        self.publish_cart_frame(left_gap_yaw, left_gap_distance, right_gap_yaw, right_gap_distance)
        """
        
    def publish_tf_relative_to_robot(self, yaw, distance, frame_id):
        orientation_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        odom_to_base_link_euler = tf.transformations.euler_from_quaternion(self._base_link_rot)
        rospy.loginfo(f"{odom_to_base_link_euler=}")
        yaw -= odom_to_base_link_euler[2]
        x, y = DetectCart.yawAndDistanceToRosXY(yaw, distance)

        x = self._base_link_trans[0] + x
        y = self._base_link_trans[1] - y

        self._transform_broadcaster.sendTransform(
            (x, y, 0),
            orientation_quaternion,
            rospy.Time.now(),
            frame_id,
            DetectCart.ODOM_FRAME)

    def publish_cart_frame(self, left_yaw, left_distance, right_yaw, right_distance):
        
        left_x, left_y = DetectCart.yawAndDistanceToRosXY(left_yaw, left_distance)
        right_x, right_y = DetectCart.yawAndDistanceToRosXY(right_yaw, right_distance)
        center_x = mean((left_x, right_x))
        center_y = mean((left_y, right_y))

        center_x_odom = self._base_link_trans[0] + center_x
        center_y_odom = self._base_link_trans[1] - center_y

        orientation_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self._transform_broadcaster.sendTransform(
            (center_x_odom, center_y_odom, 0),
            orientation_quaternion,
            rospy.Time.now(),
            DetectCart.CART_FRAME,
            DetectCart.ODOM_FRAME)
        
    def _shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self._ctrl_c = True
            
if __name__ == '__main__':
    rospy.init_node('detect_cart', anonymous=True)
    DetectCart()
    rospy.spin()