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
        self.planner_vel_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)

        self.cv_bridge = CvBridge()

        self.image_pub = rospy.Publisher("lane_follower/image", Image, queue_size=10)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.lane_lower_thresh = lane_lower_thresh
        self.lane_upper_thresh = lane_upper_thresh

        self.goal_reached = True
        self.blue_detected = False


        self.p_const = 0.008


    def move(self, linear, angular):
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.vel_pub.publish(vel)

    def odom_cb(self, odom_data):
        self.bot_x = odom_data.pose.pose.position.x
        self.bot_y = odom_data.pose.pose.position.y
        self.bot_angle = euler_from_quaternion[odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, 
                                               odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w]

    def image_cb(self, img):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError:
            rospy.logerr("Cannot Convert Image")

        cv_img = cv_img[80:200, 50:250]
        #print(cv_img.shape)

        cv_img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        cv_img_mask = cv2.inRange(cv_img_hsv, self.lane_lower_thresh, self.lane_upper_thresh)

        cv_img_mask_blue = cv2.inRange(cv_img_hsv, self.lane_lower_thresh, self.lane_upper_thresh)
        

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


    def goal_callback(self, goal):
        goal_x = goal.pose.position.x 
        goal_y = goal.pose.position.y
        x = goal.pose.orientation.x
        y = goal.pose.orientation.y
        z = goal.pose.orientation.z
        w = goal.pose.orientation.w 
        orientation_list =  [x,y,z,w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print("<-----New Goal Recived----->")
        print("X: {}    |   Y: {}   |   Yaw: {}".format(goal_x, goal_y, yaw))


        go_to_goal(goal_x, goal_y, yaw)



    def get_error(self,dest_x,dest_y):
      
        bot_theta_error = np.arctan((dest_y - self.bot_y)/(dest_x - self.bot_x))
        bot_position_error = np.sqrt((pow(dest_y - self.bot_y), 2) + pow(dest_x - self.bot_x, 2))

        return [bot_theta_error, bot_position_error]

    

    def go_to_goal(self, goal_x, goal_y, yaw):

        self.theta_error, self.position_error = self.get_error(goal_x, goal_y)
        self.find_line()
        while self.position_error < 0.2:
            if self.blue_detected != True:
                self.follow_wall()

            else:
                self.blue_detected = False
                self.theta_error, _ = self.get_error(goal_x, goal_y)
                if self.theta_error < math.pi/4 and self.theta_error > -1 * math.pi/4:
                    print("Going Straight")
                    move(0.2,0)
                    time.sleep(4)

                if self.theta_error >  math.pi/4:
                    print("Turning Right")
                    move(0.2,0)
                    time.sleep(2)
                    move(0,0)
                    time.sleep(2)
                    move(0,0.2)
                    time.sleep(4)
                
                if self.theta_error >  math.pi/4:
                    print("Turning Left")
                    move(0.2,0)
                    time.sleep(2)
                    move(0,0)
                    time.sleep(2)
                    move(0,-0.2)
                    time.sleep(4)

        

                


    def find_line(self):
        while self.cx < 18 and self.cx > 12::
            move(0,0.2)

    def follow_wall(self):

        error = self.cx - 30
        print(self.cx)

        if self.cx != 15:
            self.move(0.2, -1 * error * self.p_const)
        
        else:
            self.move(0.1, 0)
                    

        
if __name__ == "__main__":
    lower_thresh = np.array([20, 100, 120])
    upper_thresh = np.array([30, 255, 255])
    lane_follower = LaneFollower(lower_thresh, upper_thresh)
    rospy.spin()
        