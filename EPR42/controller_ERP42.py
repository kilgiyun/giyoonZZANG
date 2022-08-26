#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from importlib.resources import path
import pstats
import sys

from numpy import float64
import rospy
import rospkg
from nav_msgs.msg import Path,Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped,Point, Vector3Stamped
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from std_msgs.msg import Float64,Int16,Float32MultiArray
from detection_msgs.msg import BoundingBoxes
from std_srvs.srv import SetBool, SetBoolRequest

from math import cos,sin,sqrt,pow,atan2,pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj
from std_msgs.msg import Float32MultiArray


class purePursuit:                                          #### purePursuit 알고리즘 적용 ##
    def __init__(self):
        self.path_name=''
        self.steering_pub = rospy.Publisher('/steering_angle', CtrlCmd , queue_size=1)
        self.ctrl_pub     = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)

        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # self.stop_service    = rospy.service('/stop', SetBool, self.stopCB)
        
        rospy.Subscriber('/mode', Int16, self.modeCB)
        rospy.Subscriber('/local_path', Path, self.LocalCB)
        rospy.Subscriber('/astar_path', Path, self.astarCB)
        
        ##=====for Morai======================================================##
        # rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        # rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.egoCB)
        # rospy.Subscriber('/imu', Imu, self.imuCB)
        
        ##=====for ERP42======================================================##
        rospy.Subscriber('/linear_vel', EgoVehicleStatus, self.egoCB)
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gpsCB)
        rospy.Subscriber('/imu/rpy',Vector3Stamped,self.imuCB)
        
        
        self.local_status = False
        self.astar_on     = False
        self.is_status    = False
        self.gps_init     = True

        self.pid                   = pidController()
        self.ctrl_msg              = CtrlCmd()
        self.forward_point         = Point()
        self.current_position      = Point()
        self.is_look_forward_point = False
        self.vehicle_length        = 2.8
        self.lfd                   = 5
        self.min_lfd               = 2
        self.max_lfd               = 30
        self.steering              = 0
        self.local_path            = 0
        
        self.astar_path            = 0 
        
        self.x = 0
        self.y = 0
        
        self.mode = 0 
        
        self.x_init = 0
        self.y_init = 0 

        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0

        self.vel_x =0 
        
        self.point0_x=0,
        self.point0_y=0
        
        rospack =   rospkg.RosPack()
        self.file_path  =  rospack.get_path('my_test')
        
        self.main()
        
    def get_fp(self, file_name):
        full_file_name = self.file_path+"/path/"+file_name
        openFile       = open(full_file_name, 'r')
        
        line = openFile.readlines()
        for i in line:
            if i == 0:
                line_lst=i.split()
                self.point0_x = line_lst[0]
                self.point0_y = line_lst[1]    

    def modeCB(self, _data:Int16):
        self.mode = _data.data
        
    def LocalCB(self, _data:Path):
        self.local_path = _data
        # print('local:', type(self.local_path))
        self.local_status = True
        
    def astarCB(self, _data:Path):
        print('astar_path_subscribe')
        self.astar_path = _data
        # print('astar:', type(self.astar_path))?
        
        
    ##======================Morai Callback Func============================##    

    # def gpsCB(self, _data: GPSMessage):
    #     xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
    #     if self.gps_init:
    #         self.x_init = xy_zone[0]
    #         self.y_init = xy_zone[1]
    #         self.gps_init = False
            
    #     self.x = xy_zone[0] - self.x_init
    #     self.y = xy_zone[1] - self.y_init

    #     self.is_status = True

    # def imuCB(self, _data:Imu):
    #     quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
    #     self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환

    #     self.is_status = True
        
    # def egoCB(self, _data: EgoVehicleStatus):
    #     self.vel_x = _data.velocity.x
        
    ##======================ERP42 Callback Func============================## 
       
    def gpsCB(self, data:NavSatFix):
        self.lon = data.longitude      
        self.lat = data.latitude       

        xy_zone= self.proj_UTM(self.lon, self.lat)   

        self.x = xy_zone[0]           
        self.y = xy_zone[1]       
    
    def imuCB(self, _data:Vector3Stamped):
    
        self.yaw = _data.vector.z
        self.get_fp(self.path_name)
        psi_way=atan2(self.point0_y-self.y,self.point0_x-self.x)
        
        if self.init_flag == False:
            self.psi_err = psi_way - self.yaw
            print("Psi_Err: {}".format(self.psi_err))
            self.init_flag = True
        
        self.yaw = self.yaw + self.psi_err
        
        print("Atan2 : {} | Psi : {}".format(psi_way,self.yaw))
        # print('psi_way : {0}\n yaw_imu : {1} \n psi_err : {2}'.format(psi_way, self.yaw, psi_err))
        # print('yaw_init : {}'.format(self.yaw))
        if self.yaw>pi:
            self.yaw=self.yaw-2*pi
        elif self.yaw<-pi:
            self.yaw=self.yaw+2*pi
        else:
             pass      

        self.is_status = True       
        
    def egoCB(self, _data: EgoVehicleStatus):
        self.vel_x            = _data.velocity.x * (1000/3600)                                              ##km/h to m/s   
        
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
                for k in self.local_path.poses:
                    # print('local')
                    path_point = k.pose.position

                    dx = path_point.x - vehicle_position.x ## 변위
                    dy = path_point.y - vehicle_position.y ## 변위

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

                            self.forward_point = path_point

                            self.is_look_forward_point = True
                            
                            break
                        
                theta=atan2(rotated_point.y,rotated_point.x)
                # if self.is_look_forward_point:
                self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi   #### 추종 각도 
                    # print('angle: ',self.steering) ###deg
                return self.steering                                                        #### Steering 반환 
                # else : 
                    # print("no found forward point")
                    # return 0
            elif self.mode == 2:
                for l in self.astar_path.poses:
                    # print('astar')
                    path_point = l.pose.position
                    dx = path_point.x - vehicle_position.x ## 변위
                    dy = path_point.y - vehicle_position.y ## 변위

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
                            self.forward_point = path_point
                            self.is_look_forward_point = True
                            
                            break
                        
                theta=atan2(rotated_point.y,rotated_point.x)
                self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi   #### 추종 각도 )
                a = self.steering/180 * pi
                # print("astar : ", a)
                return self.steering                                                        #### Steering 반환 
            else:
                pass
               

    def goal_vel(self, target_vel_m):          #### km/h -> m/s 해주는 함수
        target_vel = target_vel_m * (1000/3600)
        return target_vel

    def main(self):
        target_vel = 0

        while not rospy.is_shutdown():
            # print('mode: ', self.mode)
            steering_angle = self.steering_angle()
            # print('pub : ',steering_angle)
            if type(steering_angle)==float:
                self.ctrl_msg.steering = -steering_angle/180 * pi

            if self.is_status:
                if self.ctrl_msg.steering > abs(0.4): 
                    target_vel = self.goal_vel(12)
                elif self.ctrl_msg.steering > abs(0.6):
                    target_vel = self.goal_vel(9)
                elif self.ctrl_msg.steering > abs(0.8):                                 #### 이 if문의 목적은 코너링 할때 속도를 줄여주는 건데 아직 미완
                    target_vel = self.goal_vel(7)
                else:
                    target_vel = self.goal_vel(20)

            control_input = self.pid.fnc_pid(target_vel, self.vel_x)

            if control_input > 0:                                   #### error(내 cur_vel 과 target_vel 의 차이로 뽑아낸 error값) 값이 0보다 클 때 [내 cur_vel이 느릴때]
                self.ctrl_msg.accel = control_input                 #### error 값 만큼 accel 밟아주고
                self.ctrl_msg.brake = 0
            else:                                                   #### 내 cur_vel 이 빠를 때
                self.ctrl_msg.accel = 0
                self.ctrl_msg.brake = -control_input                #### brake 밟아주고
            
            print('wheel : ', self.ctrl_msg.steering)
            self.ctrl_pub.publish(self.ctrl_msg) 


class pidController :                                                                   #### 속도 제어를 위한 PID
    def __init__(self):                                                                 #### gain 값은 점차 조정 해 가야함 
        self.p_gain      = 0.1   # 0.1                                                #### p 가 높으면 목표값(속도)에 금방 도달, 높아질 수록 불안정해짐 but 너무 낮으면 목표 속도에 도달 못 함
        self.i_gain      = 0.0     # 0.0                                              #### i 가 높으면 목표까지 가는 시간 단축, but 불안정해질 수 있음 (우리가 배운 overshoot 발생)
        self.d_gain      = 0.05    # 0.05                                               #### d 가 높으면 안정성이 높아짐 
        self.controlTime = 0.033
        self.prev_error  = 0
        self.i_control   = 0
    
    def fnc_pid(self,target_vel,current_vel):
        ############ego_topic###########3
        error           = target_vel - current_vel  ####################3               #### 만약 실제 차로 속도를 받는데 성공하면 current vel 에 그 값을 넣어주면 됩니다.
        ####################################
        # print('error:', round(error,2))
        p_control       = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control       = self.d_gain * (error - self.prev_error) / self.controlTime
        output          = p_control + self.i_control + d_control
        self.prev_error = error
        return output

    def main(self):
        pass

def main(args):

    rospy.init_node('controller', anonymous=False)
    _purePursuit = purePursuit()
    # _pidcontrol  = pidController()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)