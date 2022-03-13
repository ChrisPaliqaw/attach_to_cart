#!/usr/bin/env python
import rospy 
import rospkg 
import tf
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = 'robot'
    state_msg.pose.position.x = 5.24
    state_msg.pose.position.y = -1.77
    state_msg.pose.position.z = -0.24802

    q = tf.transformations.quaternion_from_euler(0,0,-1.57)

    state_msg.pose.orientation.x = q[0]
    state_msg.pose.orientation.y = q[1]
    state_msg.pose.orientation.z = q[2]
    state_msg.pose.orientation.w = q[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass