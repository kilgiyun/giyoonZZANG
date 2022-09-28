#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage
from math import pi,cos,sin,pi,sqrt,pow
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from pyproj import Proj
from std_msgs.msg import Float64,Int16,Float32MultiArray

class test :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        self.path_folder_name="path"
        self.make_path_name=input("저장할 파일명을 입력하시오: ")

        rospy.Subscriber("/gps",GPSMessage, self.gpsCB)

        # rospy.Subscriber("/ublox_gps/fix",NavSatFix, self.gpsCB)
        self.global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)

        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.is_status  = False
        self.gps_status = True

        self.prev_x = 0
        self.prev_y = 0

        self.lat = 0
        self.lon = 0

        self.e_o = 0
        self.n_o = 0
        
        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
        
        rospack =   rospkg.RosPack()
        pkg_path =  rospack.get_path('my_test')
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'       ## linux
        # full_path = pkg_path +'WW'+ self.path_folder_name+'WW'+self.make_path_name+'.txt'       ## wsl 
        self.f =    open(full_path, 'w')

        rate=rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.is_status==True:
                self.path_make()
            rate.sleep()    

        self.f.close()

    def gpsCB(self, gps_msg:GPSMessage):
        self.lat=gps_msg.latitude
        self.lon=gps_msg.longitude

        xy_zone= self.proj_UTM(self.lon, self.lat)

        # if self.gps_status:
        #     self.init_x = xy_zone[0]
        #     self.init_y = xy_zone[1]
        #     self.gps_status = False
        self.x=xy_zone[0] - self.x_init 
        self.y=xy_zone[1] - self.y_init

        self.is_status=True

    def path_make(self):
        x=self.x
        y=self.y

        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        # print(distance)
        if distance > 1:
            data='{0}\t{1}\n'.format(x,y)
            print(data)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y

if __name__ == '__main__':
    try:
        test_track=test()
    except rospy.ROSInterruptException:
        pass

