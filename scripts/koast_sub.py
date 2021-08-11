#!/usr/bin/env python
PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def callback(data):
    print (rospy.get_name(), "I heard %s"%str(data.data))

def listener():
    rospy.init_node('numpy_listener', anonymous=True)
    rospy.Subscriber("koast_dummy_topic", numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()