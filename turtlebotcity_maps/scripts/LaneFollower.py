#! /usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class LaneFollower:
    def __init__(self, lane_lower_thresh, lane_upper_thresh):

        rospy.init_node("lane_follower")
        self.image_sub = rospy.Subscriber("camera/image", Image, self.image_cb)
        self.planner_vel_sub = rospy.Subscriber("cmd_vel_nav", Twist, self.vel_cb)
        self.cv_bridge = CvBridge()

        self.image_pub = rospy.Publisher("lane_follower/image", Image, queue_size=10)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.lane_lower_thresh = lane_lower_thresh
        self.lane_upper_thresh = lane_upper_thresh

        self.cx = 0
        self.cy = 0

        self.p_const = 0.008

        self.vel = Twist()

    def image_cb(self, img):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError:
            rospy.logerr("Cannot Convert Image")

        cv_img = cv_img[80:200, 70:230]
        #print(cv_img.shape)

        cv_img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        cv_img_mask = cv2.inRange(cv_img_hsv, self.lane_lower_thresh, self.lane_upper_thresh)

        contours, _ = cv2.findContours(image=cv_img_mask, 
                                        mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)

        # area thresholding to eliminate noise in lane detection
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 4 ]

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

    def vel_cb(self, vel):

        if vel.linear.x != 0:
            self.follow_wall()
        else:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.vel_pub.publish(self.vel)



    def follow_wall(self):

        error = self.cx - 15
        print(self.cx)

        if self.cx != 15:
            self.vel.linear.x = 0.2
            self.vel.angular.z = -1 * error * self.p_const
        
        else:
            self.vel.linear.x = 0.1
            self.vel.angular.z = 0
        
        self.vel_pub.publish(self.vel)
            

        
if __name__ == "__main__":
    lower_thresh = np.array([20, 100, 120])
    upper_thresh = np.array([30, 255, 255])
    lane_follower = LaneFollower(lower_thresh, upper_thresh)
    rospy.spin()
        