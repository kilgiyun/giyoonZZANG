#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
from morai_msgs.msg import GPSMessage
from math import atan2, pi,sqrt,pow
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj

class generate_imu():
    def __init__(self):
        self.x          = 0
        self.y          = 0
        
        self.prev_x     = 0
        self.prev_y     = 0
        self.yaw        = 0
        self.yaw_real   = 0 
        self.yaw_init   = 0
        self.roll       = 0
        self.pitch      = 0
        
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.is_status  = True
        self.init_flag  = False
        self.gps_flag   = False
        
        self.rate = rospy.Rate(50)
        
        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
        
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gpsCB)
        # rospy.Subscriber('/imu',Imu,self.imuCB)
        
        self.imupub=rospy.Publisher('/imugen',Float32,queue_size=10)
        
        self.main()
        
    def gpsCB(self, _data:NavSatFix):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        
        self.x = xy_zone[0] 
        self.y = xy_zone[1] 
        
        self.gps_flag =True
        
    
    # def imuCB(self, _data:Imu):
    #     quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
    #     self.roll,self.pitch,self.yaw_real = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환
        
    def genimu(self):
        if self.init_flag == False & self.gps_flag == True:
            self.yaw_init = atan2(self.y_init-self.y,self.x_init-self.x)
            # print('start')
            # print(self.x,self.y)
            # print('finish')
            self.yaw=self.yaw_init
            self.prev_x = self.x
            self.prev_y = self.y
            self.init_flag = True
            
        dis=sqrt(pow(self.x-self.prev_x,2)+pow(self.y-self.prev_y,2))    
        if dis > 0.2:    
            self.yaw = atan2(self.y-self.prev_y,self.x-self.prev_x)
            self.prev_x=self.x
            self.prev_y=self.y
               
    
    def main(self):
        while not rospy.is_shutdown():
            self.genimu()
            # print(self.x,self.y)
            self.imupub.publish(self.yaw)
            # print('imu : {} | gimu : {}'.format(self.yaw_real,self.yaw))     
            self.rate.sleep() 

def main(args):

    rospy.init_node('imugenerator', anonymous=False)
    # _purePursuit = purePursuit()
    _imugen  = generate_imu()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)                    