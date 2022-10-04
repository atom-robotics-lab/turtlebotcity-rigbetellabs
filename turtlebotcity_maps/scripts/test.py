#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep


class LaneFollower:
    def __init__(self, lane_lower_thresh, lane_upper_thresh):

        rospy.init_node("lane_follower")
        self.image_sub = rospy.Subscriber("camera/image", Image, self.image_cb)
        #self.planner_vel_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_callback)
        #self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)

        self.cv_bridge = CvBridge()

        self.image_pub = rospy.Publisher("lane_follower/image", Image, queue_size=10)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.lane_lower_thresh = lane_lower_thresh
        self.lane_upper_thresh = lane_upper_thresh

        self.goal_reached = True
        self.blue_detected = False

        self.p_const = 0.008

        self.cx = 0

    def image_cb(self, img):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError:
            rospy.logerr("Cannot Convert Image")

        cv_img = cv_img[130:250, 50:250]
        #print(cv_img.shape)

        cv_img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        #cv_img_mask = cv2.inRange(cv_img_hsv, self.lane_lower_thresh, self.lane_upper_thresh)

        cv_img_mask = cv2.inRange(cv_img_hsv, np.array([110, 50, 50]), np.array([130, 255, 255]))

        contours, _ = cv2.findContours(image=cv_img_mask, 
                                        mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)

        # area thresholding to eliminate noise in lane detection
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 4 ]
        print(contours)
        #contours = max(contours, key = cv2.contourArea)

        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
        #print(self.cx)
        cv_img_cnts = cv2.drawContours(image=cv_img, contours=contours, 
                            contourIdx=-1, color=(255, 0, 0), 
                            thickness=2, lineType=cv2.LINE_AA)

        cv_img_cnts = cv2.line(cv_img_cnts, (self.cx, 0), (self.cx, 200), (0, 255, 0), 5)

        ros_img = self.cv_bridge.cv2_to_imgmsg(cv_img_cnts, encoding="rgb8")

        self.image_pub.publish(ros_img)


if __name__ == "__main__":
    lower_thresh = np.array([20, 100, 120])
    upper_thresh = np.array([30, 255, 255])
    lane_follower = LaneFollower(lower_thresh, upper_thresh)
    rospy.spin()