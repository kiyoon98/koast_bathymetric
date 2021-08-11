#!/usr/bin/env python
PKG = 'koast_bathymetric'
import roslib; roslib.load_manifest(PKG)

import sys
import subprocess
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from koast_bathymetric.msg import KoastMessage
from koast_bathymetric.cfg import koastReconfConfig
import pcl

from dynamic_reconfigure.server import Server as DRServer
from time import strftime, localtime

import pyrealsense2 as rs
import numpy as np
import cv2
import pcl_ros

class KoastTalker(object):
    def __init__(self, input_dir = ""):
        self.enable = True
        self.server = DRServer(koastReconfConfig, self.reconfigure_cb)
        self.pub = rospy.Publisher("koast_topic", KoastMessage, queue_size=10)
        self.pc_pub = rospy.Publisher("koast_topic_pc", PointCloud2, queue_size=10)
        self.enable = rospy.get_param("~enable", True)
        self.int_a = rospy.get_param("~a", 999)
        self.int_b = rospy.get_param("~b", 2)
        self.message = rospy.get_param("~message", "hello")
        self.counter = 0

        self.pcd_files = []
        self.pipe = rs.pipeline()

        if input_dir != "":
            files = subprocess.Popen("ls " + input_dir + "*.pcd", shell=True, stdout=subprocess.PIPE).stdout.read()
            # print(files)
            self.pcd_files = files.decode().split('\n')
            for i in range(len(self.pcd_files)):
                if self.pcd_files[i] == '':
                    del self.pcd_files[i]
                #else:
                #    self.pcd_files[i] = input_dir + self.pcd_files[i]
            print(len(self.pcd_files), "pcd files are prepared")
        else:
            self.profile = self.pipe.start()

        if self.enable:
            self.start()
        else:
            self.stop()
        
        rate = 0.333
        rospy.Timer(rospy.Duration(1.0/rate), self.timer_cb)

    def start(self):
        self.pub = rospy.Publisher("koast_topic", KoastMessage, queue_size=10)
        self.pc_pub = rospy.Publisher("koast_topic_pc", PointCloud2, queue_size=10)
    
    def stop(self):
        self.pub.unregister()
        self.pc_pub.unregister()
    
    def timer_cb(self, _event):
        if not self.enable:
            return

        msg = KoastMessage()
        pc_msg = PointCloud2()

        timestr = strftime("%H:%M:%S", localtime())
        msg.header.frame_id = "F-" + str(self.counter) + "(" + timestr + ")"
        msg.header.stamp = rospy.Time.now()

        if len(self.pcd_files) > 0:
            if self.counter < len(self.pcd_files):
                # Read the PCD file to get the details of the file
                pcd = open(self.pcd_files[0], 'rb')
                lines = pcd.readlines(200)
                for line in lines:
                    l = line.decode("utf-8", "ignore")
                    l = l.strip()
                    if l.upper().startswith("FIELDS"):
                        fields_str = l.split()[1:]
                    elif l.upper().startswith("TYPE"):
                        type_str = l.split()[1:]
                    elif l.upper().startswith("SIZE"):
                        size_str = l.split()[1:]
                    elif l.upper().startswith("COUNT"):
                        count_str = l.split()[1:]

                print(fields_str, end="--")
                print(type_str)
                
                pcd.close()

                fields = []
                offset = 0
                for i in range(len(fields_str)):
                    if type_str[i] == 'U':
                        pf_type = PointField.UINT32
                    else:
                        pf_type = PointField.FLOAT32
                    
                    if (i < 3) or fields_str[i].startswith("rgb") or fields_str[i] == "i":
                        one_field = PointField(fields_str[i], offset, pf_type, int(count_str[i]))
                        fields.append(one_field)

                    offset += int(size_str[i])

                # load(), load_XYZRGB(), load_XYZRGBA(), or load_XYZI() - /usr/lib/python3/dist-packages/pcl/__init__.py
                if len(fields) > 3:
                    if fields_str[3] == "rgb":
                        p = pcl.load_XYZRGB(self.pcd_files[self.counter], format="pcd")
                    elif fields_str[3] == "rgba":
                        p = pcl.load_XYZRGBA(self.pcd_files[self.counter], format="pcd")
                    elif fields_str[3] == "i":
                        p = pcl.load_XYZI(self.pcd_files[self.counter], format="pcd")
                else:
                    p = pcl.load(self.pcd_files[self.counter], format="pcd")

                a = np.asarray(p)
                # print(p.width, "x", p.height, "size=", p.__str__)
                header = std_msgs.msg.Header()

                # fields is needed when create_cloud() is used
                # c.f. create_cloud_xyz32() is when oply x, y, z
                pc = pc2.create_cloud(header, fields, a)

                # Make the PointCloud organized by assigning width & height
                pc.width = p.width
                pc.height = p.height

                msg.pc = pc
                rospy.loginfo("PCD " + str(self.counter) + " (" + self.pcd_files[self.counter] + ") is processed and published")
                self.pc_pub.publish(pc)
            else:
                rospy.loginfo("All PCD files have been processed")
        else:
            msg.message = rospy.get_param("~message", self.message)
            msg.a = rospy.get_param("~a", self.int_a)
            msg.b = rospy.get_param("~b", self.int_b)

            frames = self.pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            '''
            for f in frames:
                print(f.profile)
            '''
            # rospy.loginfo(depth_frame.profile)
            depth_data = depth_frame.get_data()
            depth_image = np.asanyarray(depth_data)
            rospy.loginfo(type(frames))
            rospy.loginfo(type(depth_frame))
            rospy.loginfo(type(depth_data))
            rospy.loginfo(type(depth_image))

        self.pub.publish(msg)
        self.counter += 1

    def reconfigure_cb(self, config, dummy):
        self.message = config["message"]
        self.int_a = config["a"]
        self.int_b = config["b"]

        if self.enable != config["enable"]:
            if config["enable"]:
                self.start()
            else:
                self.stop()
        self.enable = config["enable"]

        return config
if __name__ == '__main__':
    input_dir = ""
    if len(sys.argv) == 2:
        input_dir = sys.argv[1]

    rospy.init_node("koast_talker")
    try:
        KoastTalker(input_dir)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()