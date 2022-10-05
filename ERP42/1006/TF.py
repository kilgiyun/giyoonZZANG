#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rospy
import sys
import tf2_ros
from geometry_msgs.msg import Vector3Stamped 

from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu, PointCloud2, LaserScan,NavSatFix
from std_msgs.msg import Float32

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj
from math import atan2, pi

class TF():
    def __init__(self):
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        # rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        # rospy.Subscriber('/imu', Imu, self.imuCB)
        
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gpsCB)
        rospy.Subscriber('/imugen',Float32,self.imuCB)
        
        # rospy.Subscriber('/lidar2D', LaserScan, self.lidarCB)
        rospy.Subscriber('/lidar3D', PointCloud2, self.lidarCB)
        
        self.rate = rospy.Rate(30)
        
        self.status = True
        
        self.x = 0
        self.y = 0
        
        self.init_flag = 0
        
        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
        
        self.fake_x_init = 302473.5122667786
        self.fake_y_init = 4123735.6543077542
        
        self.fake_x = 0
        self.fake_y = 0
        
        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0
        
        self.main()
        
    def gpsCB(self, _data:NavSatFix):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        self.x = xy_zone[0]
        self.y = xy_zone[1]
        if self.status:
            self.fake_x = self.x - self.fake_x_init
            self.fake_y = self.y - self.fake_y_init
            self.status = False
        self.x = self.x - self.x_init
        self.y = self.y - self.y_init
        
    
    # def imuCB(self, _data:Imu):
    #     quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
    #     self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)     
        
    def imuCB(self, _data:Float32):
        self.yaw = _data.data               
    
    def lidarCB(self, _data:LaserScan):
        pass

    def main(self):
        br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            # br.sendTransform((self.fake_x, self.fake_y, 0),
            br.sendTransform((self.x - self.fake_x, self.y - self.fake_y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                            rospy.Time.now(),
                            "/base_link", # 아들
                            "/fake_base_link") # 아엄
            
            br.sendTransform((self.fake_x, self.fake_y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "/fake_base_link", # 아들
                            "/odom") # 아엄
            
            # br.sendTransform((0, 0, 0),
            #                 tf.transformations.quaternion_from_euler(0, 0, 0),
            #                 rospy.Time.now(),
            #                 "gps", # 아들
            #                 "/base_link") # 아엄 
            
            # br.sendTransform((0, 0, 0),
            #                 tf.transformations.quaternion_from_euler(0, 0, 0),
            #                 rospy.Time.now(),
            #                 "imu", # 아들
            #                 "/base_link") # 아엄 
            
            # br.sendTransform((3.9, 0, 1),
            #                 tf.transformations.quaternion_from_euler(0, 0, 3.14),
            #                 rospy.Time.now(),
            #                 "Camera", # 아들
            #                 "/base_link") # 아엄 
            
            # br.sendTransform((3.95, 0, 1.01),
            #                 tf.transformations.quaternion_from_euler(0, 0, 3.14),
            #                 rospy.Time.now(),
            #                 "zed_left_camera_frame", # 아들
            #                 "/base_link") # 아엄 
            
            # br.sendTransform((3.95, 0, 0.4),
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