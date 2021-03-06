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
        
        rate = 1
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
        # msg.message = rospy.get_param("~message", self.message)
        # msg.a = rospy.get_param("~a", self.int_a)
        # msg.b = rospy.get_param("~b", self.int_b)

        timestr = strftime("%H:%M:%S", localtime())
        msg.header.frame_id = "F-" + str(self.counter) + "(" + timestr + ")"
        msg.header.stamp = rospy.Time.now()
        pc_header = std_msgs.msg.Header()

        if len(self.pcd_files) > 0:
            # A folder with pcd files is provided as an argument
            if self.counter < len(self.pcd_files):
                msg.message = "Data from " + self.pcd_files[self.counter]
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

                    offset += int(size_str[i])*int(count_str[i])

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

                # fields is needed when create_cloud() is used
                # c.f. create_cloud_xyz32() is when oply x, y, z
                pc = pc2.create_cloud(pc_header, fields, a)

                # Make the PointCloud organized by assigning width & height
                pc.width = p.width
                pc.height = p.height

                msg.pc = pc
                rospy.loginfo("PCD " + str(self.counter) + " (" + self.pcd_files[self.counter] + ") is processed and published")
                self.pc_pub.publish(pc)
            else:
                rospy.loginfo("All PCD files have been processed. Resetting the counter.")
                self.counter = 0
        else:
            # No PCD files: PointCloud data come from the depth camera
            msg.message = "Data from Intel ReseSense D455"

            frames = self.pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            # decimate = rs.decimation_filter()
            # depth_frame = decimate.process(depth_frame)

            rospy.loginfo(depth_frame.profile)
            # depth_data = depth_frame.get_data()
            # depth_image = np.asanyarray(depth_data)

            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())

            ''' 
                (class) PointCloud2(*args, **kwds)
                Constructor. Any message fields that are implicitly/explicitly set to None will be assigned a default value. The recommend use is keyword arguments as this is more robust to future message changes. You cannot mix in-order arguments and keyword arguments.

                The available fields are:
                header,height,width,fields,is_bigendian,point_step,row_step,data,is_dense

                :param args: complete set of field values, in .msg order
                :param kwds: use keyword arguments corresponding to message field names to set specific fields.
            '''

            # Grab the intrinsics (may be changed by decimation)
            depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
            w, h = depth_intrinsics.width, depth_intrinsics.height
            # Although there is color image input, the resolution is different and needs mathematical processing to overlay
            # So, publish onlu PointCloud with out rgb
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]

            # depth_frame is a 2 dimensional data. Let's create 3D pointcloud using pointcloud class
            rs_pc = rs.pointcloud()
            points = rs_pc.calculate(depth_frame)
            # Once 3D points are created, get vertices!
            v, t = points.get_vertices(), points.get_texture_coordinates()
            
            rospy.loginfo("Dim = ({0:d}, {1:d}), dpeth_frame.get_data size={2:d}, bytes_per_pixel={3:d}, points.get_data_size()={4:d}".format(
                w, h, depth_frame.get_data_size(), depth_frame.bytes_per_pixel, points.get_data_size()))
            v_a = np.asanyarray(v).view(np.float32).reshape(-1, 3)
            t_a = np.asanyarray(t).view(np.float32).reshape(-1, 2)

            pc = pc2.create_cloud(pc_header, fields, v_a)
            pc.width = w
            pc.height = h

            msg.pc = pc

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