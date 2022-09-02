#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from importlib.resources import path
from pickle import NONE
import pstats
import sys

from numpy import float64
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from std_msgs.msg import Float64,Int16,Float32MultiArray
from detection_msgs.msg import BoundingBoxes
from std_srvs.srv import SetBool, SetBoolRequest

from math import cos,sin,sqrt,pow,atan2,pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj
from std_msgs.msg import Float32MultiArray

class Yolo:                                        
    def yolo__init__(self):
        
        self.yolo_pub = True
        self.yolo_off = True  
        
        self.yolo_data = 0
        self.yolo_vel = 0
        self.ctrl_msg = CtrlCmd()
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
    
    def yoloCB(self, _data: BoundingBoxes):
        # print(len(_data.bounding_boxes)==0)
        # print(type(len(_data.bounding_boxes)==0))
        self.yolo_data = _data.bounding_boxes
        if len(self.yolo_data) == 0:
            self.yolo_off = True  
            print('yolo_offffff')
            # print(self.yolo_status)                       
        elif self.yolo_pub and len(self.yolo_data) > 0:
            self.yolo_off = False
            print('yolo_onnnnnnnc')
            Class = _data.bounding_boxes[0].Class 
            if Class == "4_red":                        #### 빨간불이면 엑셀:0 , 브레이크: 1
                # print("red")
                self.yolo_vel = 0
            elif Class == "4_green":
                pass
        
        else:
            pass
        #     elif Class == "Yellow":      
        #         print("yellow")
        #         self.ctrl_msg.accel = 0
        #         self.ctrl_msg.brake = 1
                
        #     elif Class == "left":       
        #         print("left")
        #         # if 내 앞 좌표 3~5개가 좌회전 하는 path라면 gogo
        #         self.ctrl_msg.accel = 0
        #         self.ctrl_msg.brake = 1        
                
        #     elif Class == "straight":
        #         print("straight")
        #         pass
        #     elif Class == "left_straight":
        #         print("left_straight")
        #         pass
            
        # self.yolo_pub = False
        
        return self.ctrl_msg
    
    def main_pure(self):
        pass
            
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _yolo = Yolo()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)