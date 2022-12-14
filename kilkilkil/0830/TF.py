#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rospy
import sys
import tf2_ros
import geometry_msgs.msg 

from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu, PointCloud2, LaserScan

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj

class TF():
    def __init__(self):
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        rospy.Subscriber('/imu', Imu, self.imuCB)
        # rospy.Subscriber('/lidar2D', LaserScan, self.lidarCB)
        rospy.Subscriber('/lidar3D', PointCloud2, self.lidarCB)
        
        self.rate = rospy.Rate(30)
        
        self.status = True
        
        self.x = 0
        self.y = 0
        
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
        
    def gpsCB(self, _data:GPSMessage):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        self.x = xy_zone[0]
        self.y = xy_zone[1]
        if self.status:
            self.fake_x = self.x - self.fake_x_init
            self.fake_y = self.y - self.fake_y_init
            self.status = False
        self.x = self.x - self.x_init
        self.y = self.y - self.y_init
        
    
    def imuCB(self, _data:Imu):
        quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
        self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)     
    
    def lidarCB(self, _data:LaserScan):
        pass

    def main(self):
        br = tf.TransformBroadcaster()
        while not rospy.is_shutdown():
            # br.sendTransform((self.fake_x, self.fake_y, 0),
            br.sendTransform((self.x - self.fake_x, self.y - self.fake_y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                            rospy.Time.now(),
                            "/base_link", # ??????
                            "/fake_base_link") # ??????
            
            br.sendTransform((self.fake_x, self.fake_y, 0),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            "/fake_base_link", # ??????
                            "/odom") # ??????
            
            # br.sendTransform((0, 0, 0),
            #                 tf.transformations.quaternion_from_euler(0, 0, 0),
            #                 rospy.Time.now(),
            #                 "gps", # ??????
            #                 "/base_link") # ?????? 
            
            # br.sendTransform((0, 0, 0),
            #                 tf.transformations.quaternion_from_euler(0, 0, 0),
            #                 rospy.Time.now(),
            #                 "imu", # ??????
            #                 "/base_link") # ?????? 
            
            # br.sendTransform((3.9, 0, 1),
            #                 tf.transformations.quaternion_from_euler(0, 0, 3.14),
            #                 rospy.Time.now(),
            #                 "Camera", # ??????
            #                 "/base_link") # ?????? 
            
            # br.sendTransform((3.95, 0, 1.01),
            #                 tf.transformations.quaternion_from_euler(0, 0, 3.14),
            #                 rospy.Time.now(),
            #                 "zed_left_camera_frame", # ??????
            #                 "/base_link") # ?????? 
            
            # br.sendTransform((3.95, 0, 0.4),
            #                 tf.transformations.quaternion_from_euler(0, 0, 0),
            #                 rospy.Time.now(),
            #                 "1", # ??????
            #                 "/base_link") # ?????? 

            self.rate.sleep()

def main(args):

    rospy.init_node('tf_tf', anonymous=False)
    _tf = TF()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)