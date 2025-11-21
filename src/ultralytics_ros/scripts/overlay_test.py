#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import message_filters

def erpProject_lonA(pC, W, H):
    dx, dy, dz = pC[0], pC[1], pC[2]
    r = math.sqrt(dx*dx + dz*dz)
    lonA = math.atan2(-dx, dz)
    lat  = math.atan2(dy, r if r > 1e-9 else 1e-9)

    u = int((lonA + math.pi) / (2 * math.pi) * W)
    v = int(((math.pi * 0.5) - lat) / math.pi * H)

    if u < 0: u += W
    if u >= W: u -= W

    valid = (0 <= v < H)
    return u, v, valid

class LidarERPOverlay:
    def __init__(self):
        rospy.init_node("lidar_erp_overlay_sync")

        self.W = rospy.get_param("~image_width", 1920)
        self.H = rospy.get_param("~image_height", 960)

        R_default = [
            0.0151811, 0.00399802, -0.999877,
           -0.999876, -0.00423808, -0.0151981,
           -0.00429832, 0.999983,  0.00393319
        ]
        T_default = [0.00298802, 0.0157414, -0.247406]

        R = rospy.get_param("~R_cam2lidar", R_default)
        T = rospy.get_param("~T_cam2lidar", T_default)

        self.R  = np.array(R, dtype=np.float32).reshape(3, 3)
        self.T  = np.array(T, dtype=np.float32)
        self.Rt_T = self.R.T

        self.Rz180 = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1],
        ], dtype=np.float32)

        self.bridge = CvBridge()

        img_sub = message_filters.Subscriber("/ricoh/image_raw", Image)
        lidar_sub = message_filters.Subscriber("/livox/lidar", PointCloud2)

        sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, lidar_sub],
            queue_size=30,
            slop=0.05
        )
        sync.registerCallback(self.sync_cb)

        self.pub = rospy.Publisher("/debug/lidar_erp_overlay", Image, queue_size=1)

        rospy.loginfo("lidar_erp_overlay SYNC node started.")
        rospy.spin()


    def sync_cb(self, img_msg, lidar_msg):

        frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img = frame.copy()

        for p in pc2.read_points(lidar_msg, ("x","y","z"), skip_nans=True):

            pL = np.array([p[0], p[1], p[2]], np.float32)

            pC = self.Rz180.dot(self.Rt_T.dot(pL - self.T))

            u, v, valid = erpProject_lonA(pC, self.W, self.H)
            if not valid:
                continue

            dist = np.linalg.norm(pC)
            d = min(dist / 20.0, 1.0)

            colors = [
                (255, 0, 0),
                (255,255,0),
                (0,255,0),
                (0,255,255),
                (0,165,255),
                (0,0,255)
            ]

            idx = int(d * (len(colors)-1))
            idx = min(idx, len(colors)-2)
            t = (d*(len(colors)-1)) - idx

            c0 = np.array(colors[idx], float)
            c1 = np.array(colors[idx+1], float)
            c = (1-t)*c0 + t*c1
            color = tuple(map(int, c))

            cv2.circle(img, (u,v), 2, color, -1)

        out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        out.header = lidar_msg.header
        self.pub.publish(out)


if __name__ == "__main__":
    LidarERPOverlay()

