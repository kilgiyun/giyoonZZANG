#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from importlib.resources import path
import pstats
import sys
sys.path.append("/home/giyun/catkin_ws/src/my_test/include")

import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from sensor_msgs.msg import Imu
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from std_msgs.msg import Float64,Int16,Float32MultiArray
from detection_msgs.msg import BoundingBoxes
from math import cos,sin,sqrt,pow,atan2,pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj

from h_pure_pursuit import PurePursuit

class Start_planner(PurePursuit):
    def __init__(self):       
        self.pure__init__()
        self.astar_pub  = True
        self.yolo_pub   = False 
        self.gps_init   = True
        
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.main()               
                
    # def yoloCB(self, _data: BoundingBoxes):
    #     self.yolo_pub = True                        #### yolo를 pub 하면 true 로 변환

    #     if self.yolo_pub and _data.bounding_boxes[0].Class:
    #         Class = _data.bounding_boxes[0].Class 
    #         if Class == "4_red":                        #### 빨간불이면 엑셀:0 , 브레이크: 1
    #             print("4_red")
    #             self.ctrl_msg.accel = 0
    #             self.ctrl_msg.brake = 1
    #             self.ctrl_pub.publish(self.ctrl_msg)
    #         elif Class == "4_Yellow":                   #### 노란불이면 @@@@@@@@
    #             self.ctrl_msg.accel = 0
    #             self.ctrl_msg.brake = 1
    #             self.ctrl_pub.publish(self.ctrl_msg)
    #             print("4_yellow")
    #         elif Class == "4_green":
    #             print("4_green")
    #             pass
    #         elif Class == "4_left":                     #### @@@@@@@@@
    #                                              ###########3 직진은 못 하고 좌회전만
    #             print("4_left")
    #         elif Class == "4_str_green":
    #             pass
    #             print("4_str_green")
    #         elif Class == "4_stop_green":
    #             print("4_stop_green")
    #         elif Class == "4_yel_green":
    #             print("4_yel_green")
    #         elif Class == "3_red":
    #             print("3_red")
    #         elif Class == "3_Yellow":
    #             print("3_yellow")
    #         elif Class == "3_green":
    #             print("3_green")
    #     self.yolo_pub = False
        

        
    def main(self):

        # self.cmd_pub.publish(self.ctrl_msg)
        while not rospy.is_shutdown():
            self.ctrl_msg.steering = self.steering_angle()
            self.ctrl_msg.velocity    = self.vel()
            self.mode = 1 
            print(self.ctrl_msg)
            self.cmd_pub.publish(self.ctrl_msg)
            self.rate.sleep()
            # self.cmd_pub.publish(1)
            if self.local_path:
                self.mode = 1
                if self.astar_path:
                    self.mode = 2
                    dis = sqrt(pow(self.goal_pos_x - self.cur_x,2) + pow(self.goal_pos_y - self.cur_y, 2))
                    
                    if dis <= 2:
                        self.astar_path = False
            
            # elif self.yolo_pub:
                # self.mode = 3

            # if self.astar_path:
            #     self.mode = 2
            #     dis = sqrt(pow(self.goal_pos_x - self.cur_x,2) + pow(self.goal_pos_y - self.cur_y, 2))
            #     if dis < 2 :
            #         self.astar_path = False
            # else:
            #     # pass
            #     self.mode = 1
            print(self.mode)
            self.mode_pub.publish(self.mode)
            # if self.mode ==1:
            #     pass
                
            
def main(args):

    rospy.init_node('Autonomy_Manager', anonymous=False)

    _start_planner = Start_planner()

    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)