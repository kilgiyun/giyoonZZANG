#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from importlib.resources import path
import pstats
import sys

from numpy import float64
import rospy
import rospkg
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from std_msgs.msg import Int16


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
        
        self.rate = rospy.Rate(10)
        
        # self.pid                   = pidController()
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
        
        self.goal_pos_x = 0
        self.goal_pos_y = 0

        self.global_out_path = Path()
        self.local_out_path  = Path()    
        
        self.mode = 1
        
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
        rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.egoCB)
        rospy.Subscriber('/imu', Imu, self.imuCB)       
        
        self.mode_pub = rospy.Publisher('/mode',Int16, queue_size=10)
        self.cmd_pub  = rospy.Publisher('/cmd',CtrlCmd, queue_size=10)
        
    def gpsCB(self, _data: GPSMessage):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        
        self.cur_x = xy_zone[0] - self.x_init
        self.cur_y = xy_zone[1] - self.y_init
        # print('gps on')
        
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

    def imuCB(self, _data:Imu):
        quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
        self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환
        self.is_status = True
        # print('imu on')
        
    def egoCB(self, _data: EgoVehicleStatus):
        self.vel_x = _data.velocity.x
        # print('ego on')

    def steering_angle(self): 
        self.current_position.x = self.cur_x
        self.current_position.y = self.cur_y
        current_vel             = self.vel_x
        vehicle_yaw             = self.yaw

        vehicle_position            = self.current_position
        rotated_point               = Point()
        self.is_look_forward_point  = False
        
        if self.local_status:
            if self.mode == 1:
                for k in range(len(self.local_path.poses)):
                    dx = self.local_path.poses[k].pose.position.x - vehicle_position.x ## 변위
                    dy = self.local_path.poses[k].pose.position.y - vehicle_position.y ## 변위

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
                
                self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)   #### 추종 각도 
                # print('local:', self.steering)
                return self.steering                                                        #### Steering 반환 

            elif self.mode == 2:
                # print('astar')
                # print(len(self.astar_path.poses))
                for l in range(0, len(self.astar_path.poses)):
                    # print(l)
                    dx = self.astar_path.poses[l].pose.position.x - vehicle_position.x ## 변위
                    dy = self.astar_path.poses[l].pose.position.y - vehicle_position.y ## 변위

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
                
                self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)   #### 추종 각도 
                
                return self.steering     
            
            else:
                pass
            
    def goal_vel(self, target_vel_m):          #### km/h -> m/s 해주는 함수
        target_vel = target_vel_m * (1000/3600)
        return target_vel

    def vel(self):
        if type(self.ctrl_msg.steering)==float:
            self.ctrl_msg.steering = (-self.ctrl_msg.steering)
            if self.is_status:
                if self.ctrl_msg.steering > abs(0.3):
                    self.target_vel = self.goal_vel(11)
                elif self.ctrl_msg.steering > abs(0.6):
                    self.target_vel = self.goal_vel(8)
                elif self.ctrl_msg.steering > abs(0.8):         
                    self.target_vel = self.goal_vel(5)
                else:
                    self.target_vel = self.goal_vel(15)

        return self.target_vel
    
    def main_pure(self):
        pass
            
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _purepursuit = PurePursuit()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)