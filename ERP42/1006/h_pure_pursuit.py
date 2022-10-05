#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from genericpath import exists
from importlib.resources import path
from itertools import count
import pstats
import sys

from numpy import float64
import rospy
import rospkg
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Point,Vector3Stamped
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from std_msgs.msg import Int16,Float32
import pandas as pd

from math import cos,sin,sqrt,pow,atan2,pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj

class PurePursuit:                                          #### purePursuit 알고리즘 적용 ##
    def pure__init__(self):
        self.x = 0
        self.y = 0
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.local_status = False
        self.astar_pub    = True
        self.is_status    = False
        self.gps_init     = True
        
        self.basic_vel = 6
        
        self.rate = rospy.Rate(10)

        self.ctrl_msg              = CtrlCmd()
        self.forward_point         = Point()
        self.current_position      = Point()
        self.astar_real_path       = Path()
        self.is_look_forward_point = False
        self.vehicle_length        = 2.8
        self.lfd                   = 5
        self.min_lfd               = 2
        self.max_lfd               = 30
        self.steering              = 0
        self.local_path            = 0
        self.astar_path            = 0 
        self.global_path           = 0
    
        self.cur_x = 0
        self.cur_y = 0
        
        self.mode = 0 
        self.init_flag = 0
        
        self.goal_pos_x = 0
        self.goal_pos_y = 0

        self.global_out_path = Path()
        self.local_out_path  = Path()    
        
        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
        
        self.des_steering = 0
        self.des_vel      = 0
        
        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0

        self.vel_x      = 0 
        self.target_vel = 0     
        
        rospy.Subscriber('/local_path', Path, self.LocalCB)
        rospy.Subscriber('/astar_path', Path, self.astarCB)
        
        ##=====for Morai======================================================##
        # rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        # rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.egoCB)
        # rospy.Subscriber('/imu', Imu, self.imuCB)   
        
        ##=====for ERP42======================================================##
        rospy.Subscriber('/linear_vel', EgoVehicleStatus, self.egoCB)
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gpsCB)
        rospy.Subscriber('/imuComp',Float32,self.imuCB)    
        
        self.mode_pub = rospy.Publisher('/mode',Int16, queue_size=10)
        self.cmd_pub  = rospy.Publisher('/cmd',CtrlCmd, queue_size=10)
        
    
        
    def LocalCB(self, _data:Path):
        self.local_path = _data
        self.local_status = True
        # print('local on')
        
    def astarCB(self, _data:Path):                         
        
        if self.astar_pub:
            self.astar_path = _data
            length = len(self.astar_path.poses)
            self.goal_pos_x = self.astar_path.poses[length - 2].pose.position.x
            self.goal_pos_y = self.astar_path.poses[length - 2].pose.position.y
            self.astar_pub = False
    
        # print('astar on')
        # print(len(self.astar_path.poses))

    ##======================Morai Callback Func============================##
    
    # def gpsCB(self, _data: GPSMessage):
    #     xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        
    #     self.cur_x = xy_zone[0] - self.x_init
    #     self.cur_y = xy_zone[1] - self.y_init
    #     # print(self.cur_x, self.cur_y)
        
    # def imuCB(self, _data:Imu):
    #     quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
    #     self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환
    #     self.is_status = True
        # print('imu on')
        
    # def egoCB(self, _data: EgoVehicleStatus):
    #     self.vel_x = _data.velocity.x
    #     # print('ego on')
    
    ##======================ERP42 Callback Func============================## 
    def gpsCB(self, _data: NavSatFix):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        # if self.gps_init:
        #     self.x_init = xy_zone[0]
        #     self.y_init = xy_zone[1]
        #     self.gps_init = False
            
        self.x = xy_zone[0] - self.x_init
        self.y = xy_zone[1] - self.y_init

        # self.is_status = True
        
    def imuCB(self, _data:Float32):
        self.yaw = _data.data  
    
    def egoCB(self, _data: EgoVehicleStatus):
        self.vel_x            = _data.velocity.x * (1000/3600)    

    def steering_angle(self): 
        self.current_position.x = self.cur_x
        self.current_position.y = self.cur_y
        current_vel             = self.vel_x
        vehicle_yaw             = self.yaw

        vehicle_position            = self.current_position
        rotated_point               = Point()
        self.is_look_forward_point  = False
        try:
            if self.local_status:
                # print('modeeeeeeeeee:', self.mode)
                if self.mode == 1:
                    # print('local')
                    for k in range(len(self.local_path.poses)):
                        dx = self.local_path.poses[k].pose.position.x - vehicle_position.x ## 변위
                        dy = self.local_path.poses[k].pose.position.y - vehicle_position.y ## 변위
                        
                        # print(self.local_path.poses[k].pose.position.x, vehicle_position.x)
                        # print(self.local_path.poses[k].pose.position.y, vehicle_position.y)
                        # print(atan2(dy, dx)*57.3, vehicle_yaw*57.3)

                        rotated_point.x = cos(vehicle_yaw)*dx + sin(vehicle_yaw)*dy ## 
                        rotated_point.y = sin(vehicle_yaw)*dx - cos(vehicle_yaw)*dy ##

                        if rotated_point.x > 0 :
                            dis = sqrt(pow(rotated_point.x,2) + pow(rotated_point.y,2))  
                            if dis >= self.lfd :     
                                self.lfd = current_vel * 0.65    
        
                                if self.lfd < self.min_lfd :   
                                    self.lfd = self.min_lfd

                                elif self.lfd > self.max_lfd : 
                                    self.lfd = self.max_lfd

                                self.forward_point = self.local_path.poses[k].pose.position

                                self.is_look_forward_point = True
                                
                                break
                            
                    theta=atan2(rotated_point.y,rotated_point.x)
                    
                    # self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)   #### 추종 각도
                    self.steering = atan2(dy, dx) 
                    # print('local:', self.steering * 57.3)
                    return self.steering                                                        #### Steering 반환 

                elif self.mode == 2:
                    # print('astar')
                    # print(len(self.astar_path.poses))
                    for l in range(0, len(self.astar_path.poses)):
                        # print(l)
                        dx = self.astar_path.poses[l + 1].pose.position.x - vehicle_position.x ## 변위
                        dy = self.astar_path.poses[l + 1].pose.position.y - vehicle_position.y ## 변위
                        
                        # dx = self.astar_path.poses[l + 1].pose.position.y - vehicle_position.y ## 변위
                        # dy = self.astar_path.poses[l + 1].pose.position.x - vehicle_position.x ## 변위
                        
                        # print(self.astar_path.poses[l + 1].pose.position.x, vehicle_position.x)
                        # print(self.astar_path.poses[l + 1].pose.position.y, vehicle_position.y)
                        # print(atan2(dy, dx)*57.3, vehicle_yaw*57.3)

                        rotated_point.x = cos(vehicle_yaw)*dx + sin(vehicle_yaw)*dy ## 
                        rotated_point.y = sin(vehicle_yaw)*dx - cos(vehicle_yaw)*dy ##

                        if rotated_point.x > 0 :
                            dis = sqrt(pow(rotated_point.x,2) + pow(rotated_point.y,2))
                            
                            if dis >= self.lfd :     
                                self.lfd = current_vel * 0.65
                                if self.lfd < self.min_lfd :   
                                    self.lfd = self.min_lfd
                                elif self.lfd > self.max_lfd : 
                                    self.lfd = self.max_lfd
                                self.forward_point = self.astar_path.poses[l].pose.position
                                self.is_look_forward_point = True
                                
                                break
                        if l >= len(self.astar_path.poses):
                            break
                    theta=atan2(rotated_point.y,rotated_point.x)
                    
                    # self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)   #### 추종 각도 
                    self.steering = atan2(dy, dx)
                    return self.steering     
                
                else:
                    pass
                
        except:
            pass
            
    def goal_vel(self, target_vel_m):          #### km/h -> m/s 해주는 함수
        target_vel = target_vel_m * (1000/3600)
        return target_vel

    def vel(self):
        if type(self.ctrl_msg.steering)==float:
            self.ctrl_msg.steering = (self.ctrl_msg.steering)
            # if self.is_status:
                # if self.ctrl_msg.steering > abs(0.3):
                #     self.target_vel = self.goal_vel(15)
                # elif self.ctrl_msg.steering > abs(0.6):
                #     self.target_vel = self.goal_vel(15)
                # elif self.ctrl_msg.steering > abs(0.8):         
                #     self.target_vel = self.goal_vel(15)
                # else:
            self.target_vel = self.goal_vel(self.basic_vel)

        return self.target_vel
    
    def main_pure(self):
        pass
            
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _purepursuit = PurePursuit()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)