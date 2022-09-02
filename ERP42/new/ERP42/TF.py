#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rospy
import sys
import tf2_ros
import geometry_msgs.msg 

from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu, PointCloud2, LaserScan,NavSatFix
from geometry_msgs.msg import  Vector3Stamped

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj
from math import atan2,pi

class TF():
    def __init__(self):
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        # rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        # rospy.Subscriber('/imu', Imu, self.imuCB)
        
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gpsCB)
        rospy.Subscriber('/imu/rpy',Vector3Stamped,self.imuCB)
        # rospy.Subscriber('/lidar2D', LaserScan, self.lidarCB)
        rospy.Subscriber('/lidar3D', PointCloud2, self.lidarCB)
        
        self.rate = rospy.Rate(30)
        
        self.status = True
        
        self.x = 0
        self.y = 0
        self.init_flag = 0
        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
        
        self.fake_x = 0
        self.fake_y = 0
        
        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0
        
        self.main()
    
    ##======================Morai Callback Func============================##
        
    # def gpsCB(self, _data:GPSMessage):
    #     xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
    #     self.x = xy_zone[0]
    #     self.y = xy_zone[1]
    #     # if self.status:
    #     #     self.x_init = xy_zone[0]
    #     #     self.y_init = xy_zone[1]
    #     #     self.status = False
    #     self.fake_x = self.x - self.x_init
    #     self.fake_y = self.y - self.y_init
    
    # def imuCB(self, _data:Imu):
    #     quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
    #     self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)     
        
    ##======================ERP42 Callback Func============================## 
       
    def gpsCB(self, data:NavSatFix):
        self.lon = data.longitude      
        self.lat = data.latitude       

        xy_zone= self.proj_UTM(self.lon, self.lat)
        # if self.gps_init:
        #     self.x_init = xy_zone[0]
        #     self.y_init = xy_zone[1]
        #     self.gps_init = False   

        self.x = xy_zone[0] - self.x_init           
        self.y = xy_zone[1] - self.y_init       
    
        self.is_status = True    
    
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
    
    def lidarCB(self, _data:LaserScan):
        pass

    def main(self):
        br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            br.sendTransform((self.fake_x, self.fake_y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                            rospy.Time.now(),
                            "/base_link", # 아들
                            "/map") # 아엄 
            
            br.sendTransform((0, 0, 0),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "gps", # 아들
                            "/base_link") # 아엄 
            
            br.sendTransform((0, 0, 0),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "imu", # 아들
                            "/base_link") # 아엄 
            
            br.sendTransform((3.85, 0, 0.3),
                            tf.transformations.quaternion_from_euler(0, 0, 3.14),
                            rospy.Time.now(),
                            "velodyne", # 아들
                            "/base_link") # 아엄 
            
            br.sendTransform((3.85, 0, 0.3),
                            tf.transformations.quaternion_from_euler(0, 0, 3.14),
                            rospy.Time.now(),
                            "laser_mount_link", # 아들
                            "/base_link") # 아엄 
            # br.sendTransform((3.9, 0, 0.4),
            #                 tf.transformations.quaternion_from_euler(0, 0, 0),
            #                 rospy.Time.now(),
            #                 "1", # 아들
            #                 "/base_link") # 아엄 

            self.rate.sleep()

def main(args):

    rospy.init_node('tf_tf', anonymous=False)
    _tf = TF()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)