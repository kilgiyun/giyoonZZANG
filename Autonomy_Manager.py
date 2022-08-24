#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from importlib.resources import path
import pstats
import sys
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from std_msgs.msg import Float64,Int16,Float32MultiArray
from detection_msgs.msg import BoundingBoxes
from math import cos,sin,sqrt,pow,atan2,pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Start_planner():
    def __init__(self):       
        self.astar_pub  = False
        self.yolo_pub   = False 

        # pub
        # self.ctrl_pub       = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=10)
        self.mode_pub = rospy.Publisher('/mode',Int16, queue_size=10)
        
        # sub
        rospy.Subscriber('/object_data', Int16, self.dataCB)
        rospy.Subscriber('/astar_path', Path, self.astarCB)
        # class 
        self.ctrl_msg     = CtrlCmd()
 
        # variable
        self.vel_x = 0 
        
        self.mode = 1 
        
        self.global_path = 0
        self.local_path  = 0

        self.global_out_path = Path()
        self.local_out_path  = Path()
        
        self.astar_path_x = []
        self.astar_path_x = []
        
        self.main()               

    def CtrlCB(self, _data):
        self.ctrl_msg = _data

    def dataCB(self, _data):
        pass
                
    def yoloCB(self, _data: BoundingBoxes):
        self.yolo_pub = True                        #### yolo를 pub 하면 true 로 변환

        if self.yolo_pub and _data.bounding_boxes[0].Class:
            Class = _data.bounding_boxes[0].Class 
            if Class == "4_red":                        #### 빨간불이면 엑셀:0 , 브레이크: 1
                print("4_red")
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = 1
                self.ctrl_pub.publish(self.ctrl_msg)
            elif Class == "4_Yellow":                   #### 노란불이면 @@@@@@@@
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = 1
                self.ctrl_pub.publish(self.ctrl_msg)
                print("4_yellow")
            elif Class == "4_green":
                print("4_green")
                pass
            elif Class == "4_left":                     #### @@@@@@@@@
                                                 ###########3 직진은 못 하고 좌회전만
                print("4_left")
            elif Class == "4_str_green":
                pass
                print("4_str_green")
            elif Class == "4_stop_green":
                print("4_stop_green")
            elif Class == "4_yel_green":
                print("4_yel_green")
            elif Class == "3_red":
                print("3_red")
            elif Class == "3_Yellow":
                print("3_yellow")
            elif Class == "3_green":
                print("3_green")
        self.yolo_pub = False
        
    def astarCB(self, _data:Path):                      #### astar 받아오면
        self.astar_path = _data                         #### astar path 받아온거고
        if self.astar_path:
            self.mode = 2
            # print(self.mode)

    def main(self):
        
        while not rospy.is_shutdown():
            print(self.mode)
            self.mode_pub.publish(self.mode)
            
def main(args):

    rospy.init_node('Autonomy_Manager', anonymous=False)

    _start_planner = Start_planner()

    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)