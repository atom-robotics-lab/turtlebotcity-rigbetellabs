#! /usr/bin/env python

import rospy
from rospkg import RosPack
from nav_msgs.msg import Odometry
from nav_msgs.srv import LoadMap, LoadMapRequest, LoadMapResponse
from tf.transformations import euler_from_quaternion


class MapChanger:
    def __init__(self):
        
        rospy.init_node("map_changer")

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        self.yaw_precision = 0.1

        self.rospack = RosPack()

        rospy.loginfo("Waiting for /change_map service...")
        rospy.wait_for_service("change_map", timeout=10)
        rospy.logerr("COULD NOT FIND /change_map service. MAKE SURE NAVSTACK IS RUNNING!")
        self.change_map = rospy.ServiceProxy("change_map", LoadMap)

    def send_change_map_req(self, map_type):
        map_yaml_name = "turtlebot_city_" + map_type

        pkg_path = self.rospack.get_path('turtlebotcity_maps')
        change_map_req = LoadMapRequest()
        change_map_req.map_url = pkg_path + "/maps/" + map_yaml_name

        response = self.change_map(change_map_req)

        if response == 0:
            rospy.loginfo("Map Changed Successfully!")

    
    def odom_cb(self, odom_msg):
        print("in cb")
        quat_x = odom_msg.pose.pose.orientation.x
        quat_y = odom_msg.pose.pose.orientation.y
        quat_z = odom_msg.pose.pose.orientation.z
        quat_w = odom_msg.pose.pose.orientation.w

        (_, _, yaw) = euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])

        yaw_rounded = round(yaw, 1)

        # enter conditions here
        # if yaw_rounded =< 0.1 and yaw_rounded >= yaw_rounded == 1.6:
        #     self.send_change_map_req("top_right") 

        # if yaw_rounded == 3.1:
        #     pass
    

if __name__ == "__main__":
    map_changer = MapChanger()
    rospy.spin()