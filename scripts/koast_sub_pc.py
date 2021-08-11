#!/usr/bin/env python
PKG = 'koast_bathymetric'
import roslib; roslib.load_manifest(PKG)

import rospy
import std_msgs.msg
from koast_bathymetric.msg import KoastMessage
import pcl

def callback(data):
    # print (rospy.get_name(), "I heard %s"%str(data.header), end=" ")
    now = data.header.stamp
    msg = "Pub [{0:7d}.{1:7d}] [{2:s}]".format(now.secs, now.nsecs, data.header.frame_id)
#    print (msg)
    rospy.loginfo(msg)
    # Process data.pc (PointCloud2)
    print(data.pc.width, data.pc.height, "# fields=", len(data.pc.fields))

def listener():
    rospy.init_node('koast_listener', anonymous=False)
    rospy.Subscriber("koast_topic", KoastMessage, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()