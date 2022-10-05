#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from curses.ascii import ctrl
import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage, CtrlCmd
from math import pi,cos,sin,sqrt,pow,atan2
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped,Vector3Stamped
from pyproj import Proj
from std_msgs.msg import Float64,Int16,Float32MultiArray,Float32
from datetime import datetime

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class test :
    def __init__(self):
        self.path_folder_name="path"
        month   = datetime.now().month
        day     = datetime.now().day
        hour    = datetime.now().hour
        minute  = datetime.now().minute
        # print('{0}-{1}/{2}:{3}'.format(month, day, hour, minute))
        # current_time = month/day hour: minute
        # print(current_time)
        # self.make_path_name=input("저장할 파일명을 입력하시오: ")
        # self.make_path_name=rospy.Time()
        self.make_path_name = '{0}월{1}일{2}시{3}분'.format(month, day, hour, minute)

        # rospy.Subscriber("/ublox_gps/fix",NavSatFix, self.gpsCB)

        rospy.init_node('logger', anonymous=False)
        
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.is_status  = False
        self.gps_status = True
        
        self.yaw = 0
        self.des_vel=0
        self.des_steer=0
        self.cur_vel=0
        self.mode=0

        self.x  =   0
        self.y  =   0
        self.init_flag=0
        
        self.prev_x = 0
        self.prev_y = 0

        self.lat = 0
        self.lon = 0

        self.e_o = 0
        self.n_o = 0
        
        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
        
        ##=====for Morai======================================================##
        # rospy.Subscriber("/gps",GPSMessage, self.gpsCB)
        # rospy.Subscriber('/imu',Imu, self.imuCB)
        # rospy.Subscriber('/Ego_topic',EgoVehicleStatus,self.egoCB)
        
        ##=====for ERP42======================================================##
        rospy.Subscriber('/linear_vel', EgoVehicleStatus, self.egoCB)
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gpsCB)
        rospy.Subscriber('/imugen',Float32,self.imuCB)
        
        rospy.Subscriber('/ctrl_cmd',CtrlCmd, self.ctrlCB)
        rospy.Subscriber('/mode',Int16,self.modeCB)
        
        self.global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)        

        rospack =   rospkg.RosPack()
        pkg_path =  rospack.get_path('my_test')
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'       ## linux
        # full_path = pkg_path +'WW'+ self.path_folder_name+'WW'+self.make_path_name+'.txt'       ## wsl 
        self.f =    open(full_path, 'w')

        rate=rospy.Rate(30)
        self.f.write('x\ty\tmode\tdes_steer\taccel\tyaw\tcur_vel\n')
        while not rospy.is_shutdown():
            if self.is_status==True:
                self.path_make()
            rate.sleep()    

        self.f.close()
    
    ##======================Morai Callback Func============================## 
    
    # def egoCB(self, data:EgoVehicleStatus):
    #     self.cur_vel = data.velocity.x * 3.6    
        
    # def gpsCB(self, gps_msg:GPSMessage):
    #     self.lat=gps_msg.latitude
    #     self.lon=gps_msg.longitude

    #     xy_zone= self.proj_UTM(self.lon, self.lat)

    #     # if self.gps_status:
    #     #     self.init_x = xy_zone[0]
    #     #     self.init_y = xy_zone[1]
    #     #     self.gps_status = False
    #     self.x=xy_zone[0] - self.x_init 
    #     self.y=xy_zone[1] - self.y_init

    #     self.is_status=True

    # def imuCB(self, _data:Imu):
    #     quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
    #     self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환
    #     self.is_status = True
    
    ##======================ERP42 Callback Func============================## 
    
    def gpsCB(self, _data: NavSatFix):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        # if self.gps_init:
        #     self.x_init = xy_zone[0]
        #     self.y_init = xy_zone[1]
        #     self.gps_init = False
            
        self.x = xy_zone[0] - self.x_init
        self.y = xy_zone[1] - self.y_init

        self.is_status = True
        
    def imuCB(self, _data:Float32):
        self.yaw = _data.data     
    
    def egoCB(self, _data: EgoVehicleStatus):
        self.cur_vel            = _data.velocity.x
        
    def modeCB(self, data:Int16):
        self.mode = data.data    
    
    def ctrlCB(self, data:CtrlCmd):
        self.des_steer = data.steering
        self.des_vel = data.accel    
        
    def path_make(self):
        x=self.x
        y=self.y
        des_steering=self.des_steer
        accel=self.des_vel
        yaw=self.yaw
        cur_vel=self.cur_vel
        mode = self.mode

        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        
        # print(distance)
        # ('x\ty\tmode\tdes_steer\taccel\tyaw\tcur_vel\n')
        if distance > 1:
            data='{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\n'.format(x,y,mode,des_steering,accel,yaw,cur_vel)
            # print(data)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y

if __name__ == '__main__':
    try:
        test_track=test()
    except rospy.ROSInterruptException:
        pass

