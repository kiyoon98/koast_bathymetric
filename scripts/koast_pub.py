#!/usr/bin/env python
PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy
def talker():
    pub = rospy.Publisher('koast_dummy_topic', numpy_msg(Floats),queue_size=10)
    rospy.init_node('numpy_talker', anonymous=True)
    r = rospy.Rate(0.5) # 10hz
    counter = 0
    while not rospy.is_shutdown():
        a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
        a[0] = counter
        counter += 1
        pub.publish(a)
        r.sleep()

if __name__ == '__main__':
    talker()