#! /usr/bin/env python

import rospy
import cv2
import numpy as np
import math
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
        self.yellow_detected = False

        self.cx = 0
        self.move(0, 0)

        print("***** Lane Following Node Started *****")


        self.p_const = 0.008


    def move(self, linear, angular):
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.vel_pub.publish(vel)

    def odom_cb(self, odom_data):
        self.bot_x = odom_data.pose.pose.position.x;
        self.bot_y = odom_data.pose.pose.position.y;
        x = odom_data.pose.pose.orientation.x;
        y = odom_data.pose.pose.orientation.y;
        z = odom_data.pose.pose.orientation.z;
        w = odom_data.pose.pose.orientation.w;
        orientation_list =  [x,y,z,w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.bot_angle = yaw

    def image_cb(self, img):
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError:
            rospy.logerr("Cannot Convert Image")

        cv_img = cv_img[80:200, 50:250]
        #print(cv_img.shape)

        cv_img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        cv_img_mask = cv2.inRange(cv_img_hsv, self.lane_lower_thresh, self.lane_upper_thresh)

        cv_img_mask_blue = cv2.inRange(cv_img_hsv, np.array([110, 100, 100]), np.array([130, 255, 255]))


        contours, _ = cv2.findContours(image=cv_img_mask, 
                                        mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)

        contours_blue, _ = cv2.findContours(image=cv_img_mask_blue, 
                                        mode=cv2.RETR_TREE, 
                                        method=cv2.CHAIN_APPROX_NONE)

        # area thresholding to eliminate noise in lane detection
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 10 ]

        if contours:
            self.yellow_detected = True
            Max_contour = max(contours, key = cv2.contourArea)

            M = cv2.moments(Max_contour)

            if M['m00'] != 0:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
            #print(self.cx)

        else:
            self.yellow_detected = False

        for cnt in contours_blue:
            #print(cv2.contourArea(cnt))
            if cv2.contourArea(cnt) > 800:
                #print(cv2.contourArea(cnt))
                #print("Intersection detected")
                self.blue_detected = True
            else:
                self.blue_detected = False
        

        cv_img_cnts = cv2.drawContours(image=cv_img, contours=contours, 
                            contourIdx=-1, color=(255, 0, 0), 
                            thickness=2, lineType=cv2.LINE_AA)

        cv_img_cnts = cv2.drawContours(image=cv_img_cnts, contours=contours_blue, 
                            contourIdx=-1, color=(0, 0, 255), 
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
        print("Bot_X: {}   |   Bot_Y: {}".format(self.bot_x, self.bot_y))


        self.go_to_goal(goal_x, goal_y, yaw)



    def get_error(self,dest_x,dest_y):
      
        bot_theta_error = np.arctan((dest_y - self.bot_y)/(dest_x - self.bot_x))
        bot_position_error = np.sqrt(pow(dest_y - self.bot_y, 2) + pow(dest_x - self.bot_x, 2))

        bot_theta = self.bot_angle      # bot making angle with x axis 
        bot_x_coordinate = round(self.bot_x, 2) # current x coordinate of bot
        bot_y_coordinate = round(self.bot_y,2)  # current y cordinate of bot
        bot_array = [np.cos(bot_theta),np.sin(bot_theta)]
        bot_vector = np.array(bot_array)
        reach_point_array = [dest_x - bot_x_coordinate , dest_y - bot_y_coordinate ]
        reach_point_vector = np.array(reach_point_array)
        dot_product = reach_point_vector.dot(bot_vector)
        bot_theta_error = np.arccos(dot_product/(np.sqrt((dest_x - bot_x_coordinate)**2+(dest_y - bot_y_coordinate)**2)))
        bot_position_error = np.sqrt((dest_x - bot_x_coordinate)**2+(dest_y - bot_y_coordinate)**2)
        #print("big big")
        #print(np.cos(bot_theta)*(dest_y - bot_y_coordinate) -  np.sin(bot_theta)*(dest_x - bot_x_coordinate))
        if np.cos(bot_theta)*(dest_y - bot_y_coordinate) -  np.sin(bot_theta)*(dest_x - bot_x_coordinate) < 0 :
           bot_theta_error = -bot_theta_error 

        return [bot_theta_error, bot_position_error]

    

    def go_to_goal(self, goal_x, goal_y, yaw):

        self.theta_error, self.position_error = self.get_error(goal_x, goal_y)
        print("position error: {}   |   Theta_error: {}".format(self.position_error, self.theta_error))
        self.find_line()
        print("---Start Navigation---")
        r = rospy.Rate(10)
        
        while self.position_error > 0.2:
            r.sleep()

            self.theta_error, self.position_error = self.get_error(goal_x, goal_y)

            if self.blue_detected != True:

                if self.yellow_detected!= True:
                    self.find_line()

                else:    
                    self.follow_wall()

            else:
                self.blue_detected = False
                print("Intersection detected")
                self.move(0,0)
                sleep(4)
                print("Moving straight")
                self.move(0.1,0.02)
                sleep(3)
                print("Theta error: {}".format(self.theta_error))
                print("Theta bot: {}".format(self.bot_x))

                if self.position_error < 0.2:
                    print("Already on the goal")
                    break
                self.move(0.1,0.02)
                sleep(3)

                if self.position_error < 0.2:
                    print("Already on the goal")
                    break

                if self.theta_error < math.pi/5 and self.theta_error > -1 * math.pi/5:
                    print("Going Straight")
                    self.move(0.1,0)
                    sleep(4)

                if self.theta_error >  math.pi/5:
                    print("Turning Left")
                    self.move(0,0.2)
                    sleep(3)
                    while self.cx not in range(15, 35):
                        self.move(0,0.15)
                        #print("Retured to yellow: {}".format(self.cx))
                        self.move(0, 0)
                
                if self.theta_error <  -1 * math.pi/5:
                    print("Turning Right")
                    self.move(0.03,-0.2)
                    sleep(3)
                    while self.cx not in range(15, 35):
                        self.move(0,-0.15)
                        #print("Retured to yellow: {}".format(self.cx))
                        self.move(0, 0)
                else:
                    print("Unknown Case, Blue detected: {}".format(self.blue_detected))
                    self.move(0, 0)

                self.blue_detected = False

        print("position error: {}   |   Theta_error: {}".format(self.position_error, self.theta_error))
        self.move(0,0)
        print("Position Reached Reached")



    def find_line(self):
        #print(self.cx)
        # self.move(0,0.5)
        # sleep(2)
        # self.move(0, 0)
        while self.cx not in range(20, 35):
            #print("Finding Line: {}".format(self.cx))
            self.move(0,0.15)
        print("Yellow Line Found: {}".format(self.cx))
        self.move(0, 0)

    def follow_wall(self):

        error = self.cx - 25
        #print(self.cx)

        if self.cx != 25:
            self.move(0.15, -1 * error * self.p_const)
        
        else:
            self.move(0.2, 0)
                    

        
if __name__ == "__main__":
    lower_thresh = np.array([22, 130, 150])
    upper_thresh = np.array([25, 255, 255])
    lane_follower = LaneFollower(lower_thresh, upper_thresh)
    rospy.spin()
        