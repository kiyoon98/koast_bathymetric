#!/usr/bin/env python
# This program is based on opencv_pointcloud_viewer.py in PyRealSense2 (https://dev.intelrealsense.com/docs/python2) and
# simple_visualize.py, example.py, visualization.py in python-pcl (https://github.com/strawlab/python-pcl)

PKG = 'koast_bathymetric'
import roslib; roslib.load_manifest(PKG)

import math
import time
import cv2
import rospy
import std_msgs.msg
from koast_bathymetric.msg import KoastMessage
import pcl
import pcl.pcl_visualization

import numpy as np

'''
class AppState:

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'KOAST Bathymetric'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)

state = AppState()

def mouse_cb(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDOWN:
        state.mouse_btns[0] = True

    if event == cv2.EVENT_LBUTTONUP:
        state.mouse_btns[0] = False

    if event == cv2.EVENT_RBUTTONDOWN:
        state.mouse_btns[1] = True

    if event == cv2.EVENT_RBUTTONUP:
        state.mouse_btns[1] = False

    if event == cv2.EVENT_MBUTTONDOWN:
        state.mouse_btns[2] = True

    if event == cv2.EVENT_MBUTTONUP:
        state.mouse_btns[2] = False

    if event == cv2.EVENT_MOUSEMOVE:

        h, w = out.shape[:2]
        dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

        if state.mouse_btns[0]:
            state.yaw += float(dx) / w * 2
            state.pitch -= float(dy) / h * 2

        elif state.mouse_btns[1]:
            dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
            state.translation -= np.dot(state.rotation, dp)

        elif state.mouse_btns[2]:
            dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
            state.translation[2] += dz
            state.distance -= dz

    if event == cv2.EVENT_MOUSEWHEEL:
        dz = math.copysign(0.1, flags)
        state.translation[2] += dz
        state.distance -= dz

    state.prev_mouse = (x, y)

def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w
    # ignore divide by zero for invalid depth
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect, h) + (w/2.0, h/2.0)

    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj

def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
    """draw a 3d line from pt1 to pt2"""
    p0 = project(pt1.reshape(-1, 3))[0]
    p1 = project(pt2.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    p0 = tuple(p0.astype(int))
    p1 = tuple(p1.astype(int))
    rect = (0, 0, out.shape[1], out.shape[0])
    inside, p0, p1 = cv2.clipLine(rect, p0, p1)
    if inside:
        cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
    """draw a grid on xz plane"""
    pos = np.array(pos)
    s = size / float(n)
    s2 = 0.5 * size
    for i in range(0, n+1):
        x = -s2 + i*s
        line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
               view(pos + np.dot((x, 0, s2), rotation)), color)
    for i in range(0, n+1):
        z = -s2 + i*s
        line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
               view(pos + np.dot((s2, 0, z), rotation)), color)


def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
    """draw 3d axes"""
    line3d(out, pos, pos +
           np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
    line3d(out, pos, pos +
           np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
    line3d(out, pos, pos +
           np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
    """draw camera's frustum"""
    orig = view([0, 0, 0])
    w, h = intrinsics.width, intrinsics.height

    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            line3d(out, orig, view(p), color)
            return p

        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)

        line3d(out, view(top_left), view(top_right), color)
        line3d(out, view(top_right), view(bottom_right), color)
        line3d(out, view(bottom_right), view(bottom_left), color)
        line3d(out, view(bottom_left), view(top_left), color)


def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if state.scale:
        proj *= 0.5**state.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm

    cw, ch = color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = color[u[m], v[m]]

w = 640
h = 480

out = np.empty((h, w, 3), dtype=np.uint8)

''' 
###### Ignore up to this point

a = 220
b = 120
c = 100

#viewer = pcl.pcl_visualization.PCLVisualizering()
viewer = pcl.pcl_visualization.CloudViewing()

def callback(data):
    now = data.header.stamp
    msg = "Pub [{0:7d}.{1:7d}] [{2:s}]".format(now.secs, now.nsecs, data.header.frame_id)
    rospy.loginfo(msg)
    # print(type(data.pc.data), len(data.pc.data))
    print(data.message)
    print(data.pc.width, data.pc.height, "# fields=", len(data.pc.fields))

    # Process data.pc (PointCloud2)

    if len(data.pc.fields) == 3:
        cloud = pcl.PointCloud(data.pc.width * data.pc.height)
        cloud.from_array(np.ndarray(shape=(data.pc.width*data.pc.height, 3),
                            dtype=np.float32, buffer=data.pc.data))
        # cloud_filtered = cloud
    else:
        cloud = pcl.PointCloud_PointXYZRGB(data.pc.width * data.pc.height)
        cloud.from_array(np.ndarray(shape=(data.pc.width*data.pc.height, 4),
                            dtype=np.float32, buffer=data.pc.data))

    i = 0.1 * a
    j = 0.1 * b
    k = 0.1 * c

    # Printing to ensure that the passthrough filter values are changing if we move trackbars.
    # cout << "i = " << i << " j = " << j << " k = " << k << endl;
    # print("i = " + str(i) + " j = " + str(j) + " k = " + str(k))

    # Applying passthrough filters with XYZ limits
    # pcl::PassThrough<pcl::PointXYZRGBA> pass;
    # pass.setInputCloud (cloud);
    # pass.setFilterFieldName ("y");
    # //  pass.setFilterLimits (-0.1, 0.1);
    # pass.setFilterLimits (-k, k);
    # pass.filter (*cloud);
    pass_th = cloud.make_passthrough_filter()
    pass_th.set_filter_field_name("y")
    pass_th.set_filter_limits(-k, k)
    cloud = pass_th.filter()

    # pass.setInputCloud (cloud);
    # pass.setFilterFieldName ("x");
    # // pass.setFilterLimits (-0.1, 0.1);
    # pass.setFilterLimits (-j, j);
    # pass.filter (*cloud);
    # pass_th.setInputCloud(cloud)
    pass_th.set_filter_field_name("x")
    pass_th.set_filter_limits(-j, j)
    cloud = pass_th.filter()

    # pass.setInputCloud (cloud);
    # pass.setFilterFieldName ("z");
    # //  pass.setFilterLimits (-10, 10);
    # pass.setFilterLimits (-i, i);
    # pass.filter (*cloud);
    # pass_th.setInputCloud(cloud)
    pass_th.set_filter_field_name("z")
    pass_th.set_filter_limits(-10, 10)
    cloud = pass_th.filter()

    if len(data.pc.fields) == 3:
        # // Visualizing pointcloud
        # viewer.addPointCloud (cloud, "scene_cloud");
        # viewer.spinOnce();
        # viewer.removePointCloud("scene_cloud");

        viewer.ShowMonochromeCloud(cloud, b'cloud')

        ###viewer.AddPointCloud(cloud, b'scene_cloud', 0)
        ###viewer.SpinOnce()
        # viewer.Spin()
        ###viewer.RemovePointCloud(b'scene_cloud', 0)
    else:
        viewer.ShowColorCloud(cloud)

def listener():
    rospy.init_node('koast_listener', anonymous=False)
    rospy.Subscriber("koast_topic", KoastMessage, callback)
    rospy.spin()

if __name__ == '__main__':
    # cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
    # cv2.resizeWindow(state.WIN_NAME, w, h)
    # cv2.setMouseCallback(state.WIN_NAME, mouse_cb)

    listener()