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

# class PID(self)
class velocity_PidController :                                                                   #### 속도 제어를 위한 PID
    def __init__(self):                                                                 #### gain 값은 점차 조정 해 가야함 
        self.vel_p_gain      = 0.5   # 0.1                                                #### p 가 높으면 목표값(속도)에 금방 도달, 높아질 수록 불안정해짐 but 너무 낮으면 목표 속도에 도달 못 함
        self.vel_i_gain      = 0.01     # 0.0                                              #### i 가 높으면 목표까지 가는 시간 단축, but 불안정해질 수 있음 (우리가 배운 overshoot 발생)
        self.vel_d_gain      = 0.00    # 0.05                                               #### d 가 높으면 안정성이 높아짐 
        
        self.angle_p_gain    = 0.5
        self.angle_i_gain    = 0.01
        self.angle_d_gain    = 0.00
        self.controlTime = 0.01
        self.vel_prev_error  = 0
        self.vel_i_control   = 0
        
        self.angle_i_control = 0
        self.angle_prev_error  = 0
        
        self.vel_x = 0
        
        self.yaw = 0
        self.yawrate = 0
        
        self.target_vel = 0
        self.steering   = 0
        
        self.cur_steering = 0
        
        self.rad2deg = 180 / pi
        self.deg2rad = pi / 180
        
        self.filtered_des_v = 0
        self.e_int = 0
        self.e_dot_int = 0
        self.e_ddot_int = 0
        
        self.filtered_des_psi = 0
        self.filtered_des_psi_dot = 0
        self.filtered_des_psi_ddot = 0
        
        self.ctrl_msg = CtrlCmd()
        
        self.rate = rospy.Rate(10)
        
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=10)
        # self.mode_pub = rospy.Publisher('/mode',Int16, queue_size=10)
        
        rospy.Subscriber('/cmd', CtrlCmd, self.cmdCB)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.egoCB)
        rospy.Subscriber('/imu', Imu, self.imuCB)
        
        self.main()

    def imuCB(self, _data:Imu):
        quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
        self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환
        self.yawrate = (_data.angular_velocity.z) * self.deg2rad

    def cmdCB(self, _data:CtrlCmd):
        self.target_vel = _data.velocity
        # print('?',_data.velocity)
        self.steering   = _data.steering
        # print('angle:', self.steering)     
          
    def egoCB(self, _data: EgoVehicleStatus):
        self.vel_x = _data.velocity.x
        self.cur_steering = (_data.wheel_angle) * self.deg2rad
        
    def vel_pid(self):
        ############ego_topic###########3
        error           = self.target_vel - self.vel_x  
        ####################################
        p_control           = self.vel_p_gain * error
        self.vel_i_control += self.vel_i_gain * error * self.controlTime
        d_control           = self.vel_d_gain * (error - self.vel_prev_error) / self.controlTime
        output              = p_control + self.vel_i_control + d_control
        self.vel_prev_error = error
        return output
    
    def angle_pid(self):
        error                   = self.steering - self.cur_steering  
        p_control               = self.angle_p_gain * error
        self.angle_i_control    += self.angle_i_gain * error * self.controlTime
        d_control               = self.angle_d_gain * (error - self.angle_prev_error) / self.controlTime
        output                  = p_control + self.angle_i_control + d_control
        self.angle_prev_error   = error
        return output
    
    def angleCheck(self, _angle):
        
        _result = _angle
        
        if abs(_angle) > pi:
            if _angle < 0:
                _result = _angle + 2 * pi
            else:
                _result = _angle - 2 * pi
                        
        return _result
    
    def vel_smc(self, cur_vel, des_vel):
        _result = .0
        _lambda = 1
        _dt = 0.01
        _e_filtered = 0
        _wn = 2
        _k1 = 1
        _eps = 1e-3
        _delta_hat = 0.1
        
        e1 = des_vel - self.filtered_des_v
        
        self.e_dot_int = self.e_dot_int + _dt * (-2 * _wn * self.e_dot_int + _wn**2 * e1)
        self.filtered_des_v = self.filtered_des_v + _dt * self.e_dot_int
        
        self.e_int = self.e_int + _dt * e1
        
        e = self.filtered_des_v - cur_vel
        
        s = e + (_k1 * self.e_int)
        
        _sat = s / _eps
         
        if abs(_sat) > 1:
            if _sat < 0:
                _sat = -1
            else:
                _sat = 1
                
        _u = self.e_dot_int + _k1 * e + _delta_hat * _sat
        
        return _u
    
    def main(self):
        _temp = 0
        
        while not rospy.is_shutdown():
            # print('des_steering : {0}, cur_steering : {1}'.format(self.steering, self.cur_steering))
            vel_input = self.vel_smc(self.vel_x, self.target_vel)
            angle_input = self.angle_pid()
            
            _temp = angle_input
            # self.ctrl_msg.steering = angle_input
            self.ctrl_msg.steering = - angle_input
        
            if vel_input > 0:                                   
                self.ctrl_msg.accel = vel_input                 
                self.ctrl_msg.brake = 0
            else:                                                
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = -vel_input 
            
            print(self.ctrl_msg)
            self.ctrl_pub.publish(self.ctrl_msg) 
            self.rate.sleep()

def main(args):

    rospy.init_node('controller', anonymous=False)
    # _purePursuit = purePursuit()
    _pidcontrol  = velocity_PidController()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)