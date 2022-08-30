#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from importlib.resources import path
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

class PurePursuit:                                          #### purePursuit 알고리즘 적용 ##
    def pure__init__(self):
        self.x = 0
        self.y = 0
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.local_status = False
        self.astar_on     = True
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
        
        self.cur_x = 0
        self.cur_y = 0
        
        self.mode = 1 
        
        self.global_path = 0
        
        self.goal_pos_x = 0
        self.goal_pos_y = 0

        self.global_out_path = Path()
        self.local_out_path  = Path()    
        
        self.mode = 0 
        
        self.x_init = 0
        self.y_init = 0 

        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0

        self.vel_x =0 
        
        
        self.mode_pub = rospy.Publisher('/mode',Int16, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd',CtrlCmd, queue_size=10)
            
        rospy.Subscriber('/local_path', Path, self.LocalCB)
        rospy.Subscriber('/astar_path', Path, self.astarCB)
        rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.egoCB)
        rospy.Subscriber('/imu', Imu, self.imuCB)
        
        self.main()
        
    def gpsCB(self, _data: GPSMessage):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        if self.gps_init:
            self.x_init = xy_zone[0]
            self.y_init = xy_zone[1]
            self.gps_init = False
            
        self.x = xy_zone[0] - self.x_init
        self.y = xy_zone[1] - self.y_init
        
    def LocalCB(self, _data:Path):
        self.local_path = _data
        self.local_status = True
        
    def astarCB(self, _data:Path):    
        self.astar_path = _data                        
        length = len(self.astar_path.poses)
        if self.astar_pub:
            self.goal_pos_x = self.astar_path.poses[length - 1].pose.position.x
            self.goal_pos_y = self.astar_path.poses[length - 1].pose.position.y
            self.astar_pub = False

    def imuCB(self, _data:Imu):
        quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
        self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환
        self.is_status = True
        
    def egoCB(self, _data: EgoVehicleStatus):
        self.vel_x = _data.velocity.x

    def steering_angle(self): 
        self.current_position.x = self.x
        self.current_position.y = self.y
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
                
                self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi   #### 추종 각도 
                
                return self.steering                                                        #### Steering 반환 
      
            elif self.mode == 2:
                for l in range(len(self.astar_path.poses)):
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
    
                theta=atan2(rotated_point.y,rotated_point.x)
                
                self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi   #### 추종 각도 )
                
                return self.steering                                                        #### Steering 반환 
            else:
                pass
               
    def goal_vel(self, target_vel_m):          #### km/h -> m/s 해주는 함수
        target_vel = target_vel_m * (1000/3600)
        return target_vel

    def vel(self):
        steering_angle = self.steering_angle() 
        
        target_vel = 0
        if type(steering_angle)==float:
            self.ctrl_msg.steering = (-steering_angle) * pi /180
            # print('target angle:', self.ctrl_msg.steering)
        if self.is_status:
            if self.ctrl_msg.steering > abs(0.4):
                target_vel = self.goal_vel(7)
            elif self.ctrl_msg.steering > abs(0.6):
                target_vel = self.goal_vel(7)
            elif self.ctrl_msg.steering > abs(0.8):         
                target_vel = self.goal_vel(7)
            else:
                target_vel = self.goal_vel(20)
                
        # print('target vel: ',target_vel)
        return target_vel
    
    def main(self):
        pass

def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _purepursuit = PurePursuit()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)