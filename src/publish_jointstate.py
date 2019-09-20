#!/usr/bin/env python
import sys
import rospy
import kdl_parser_py.urdf  
import PyKDL as kdl
from sensor_msgs.msg import JointState

def node():
    rospy.init_node('publish_jointstate', anonymous=True)
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [0.0, 0.2, -0.2, -0.2, -0.2 ]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0 ]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0 ]
        msg.name = ["base", "body0_joint_yaw", "body1_joint", "body2_joint", "body3_joint"]

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        node()   
    except rospy.ROSInterruptException:
        pass