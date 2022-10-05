#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
from math import atan2, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj

class ImuCompensation():
    def __init__(self):
        self.yaw        = 0
        
        self.init_flag  = 0
        
        self.x_init     = 0
        self.y_init     = 0
        self.x          = 0
        self.y          = 0
        self.psi_err    = 0
        
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.rate = rospy.Rate(30)
        
        self.imupub=rospy.Publisher('/imuComp',Float32,queue_size=10)
        
        rospy.Subscriber('/imu/rpy',Vector3Stamped,self.imuCB)
        rospy.Subscriber('/ublox_gps',NavSatFix, self.gpsCB)
        # rospy.Subscriber('/imu', Imu, self.imuCB)
        
        self.main()
        
    # def imuCB(self, _data:Imu):
    #     quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
    #     self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)  
    
    def gpsCB(self, _data: NavSatFix):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)

        self.x = xy_zone[0] - self.x_init
        self.y = xy_zone[1] - self.y_init 
        
    def imuCB(self, _data:Vector3Stamped):
    
        self.yaw = _data.vector.z
        psi_way=atan2(self.y_init-self.y,self.x_init-self.x)
        
        if self.init_flag <30:
            self.init_flag +=1
        elif self.init_flag == 30:
            self.psi_err = psi_way - self.yaw
            self.init_flag +=1
        else:
            pass
             
           
        self.yaw = self.yaw + self.psi_err    
          
        if self.yaw>pi:
            self.yaw=self.yaw-2*pi
        elif self.yaw<-pi:
            self.yaw=self.yaw+2*pi
        else:
             pass      
        print("Atan2 : {} | Psi : {} | err : {} ".format(psi_way,self.yaw,self.psi_err))
        
        self.is_status = True   
    
    def main(self):
        while not rospy.is_shutdown():
            self.imupub.publish(self.yaw)  
            print(self.yaw)  
            self.rate.sleep()
        
           
def main(args):

    rospy.init_node('imuCompensation', anonymous=False)
    # _purePursuit = purePursuit()
    _imucomp  = ImuCompensation()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)        
    