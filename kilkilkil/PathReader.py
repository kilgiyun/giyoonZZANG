#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from importlib.resources import path
import pstats
import sys
import rospy
import rospkg

from nav_msgs.msg import Path,Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from std_msgs.msg import Float64,Int16,Float32MultiArray
from detection_msgs.msg import BoundingBoxes
from tf2_msgs.msg import TFMessage
from std_srvs.srv import SetBool, SetBoolRequest

from math import cos,sin,sqrt,pow,atan2,pi
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj




class pathReader():                                         
    def __init__(self):

        self.path_name = '0822.txt'
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub  = rospy.Publisher('/local_path',Path, queue_size=1)
        self.tf_pub          = rospy.Publisher('/tf', TFMessage, queue_size=1)

        # self.stop_service=rospy.ServiceProxy('/stop',SetBool)
        self.close_waypoint = True
        self.gpsinit = True
                
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        rospy.Subscriber('/imu', Imu, self.imuCB)
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("my_test")
        
        self.rate = rospy.Rate(30)

        self.global_path = 0
        self.local_path  = 0

        self.current_waypoint = 0

        self.roll   = 0
        self.pitch  = 0
        self.yaw    = 0
         
        self.i = 0 

        self.x = 0
        self.y = 0
        
        self.x_init = 0 
        self.y_init = 0 

        self.main()

    def gpsCB(self, _data: GPSMessage):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        if self.gpsinit:
            self.x_init = xy_zone[0]
            self.y_init = xy_zone[1]
            self.gpsinit = False
            
        self.x = xy_zone[0] - self.x_init
        self.y = xy_zone[1] - self.y_init        
        # print(self.x, self.y)
               
    def imuCB(self, _data:Imu):
        quaternion = (_data.orientation.x, _data.orientation.y, _data.orientation.z, _data.orientation.w)
        self.roll,self.pitch,self.yaw = euler_from_quaternion(quaternion)           #### roll, pitch, yaw 로 변환

    def read_txt(self, file_name):
        full_file_name = self.file_path+"/path/"+file_name
        openFile       = open(full_file_name, 'r')
        
        out_path = Path()
        out_path.header.frame_id = 'map' ## idk

        line = openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x    = float(tmp[0])
            read_pose.pose.position.y    = float(tmp[1]) 
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)

        openFile.close()
        
        return out_path  
    
    def findLocalPath(self):
        out_path = Path()
        min_dis = float('inf')      
        
        # rospy.sleep(0.1)
        global_len = int(len(self.global_path.poses))

        for i in range(global_len):
            dx = self.x - self.global_path.poses[i].pose.position.x 
            dy = self.y - self.global_path.poses[i].pose.position.y         #### //
            dis = sqrt(pow(dx,2)+pow(dy,2))     #### 변위
            if dis < min_dis :
                min_dis = dis
                self.current_waypoint = i
                
        if self.current_waypoint + 16 > len(self.global_path.poses) : #현재 웨이 포인트+50이 len(ref_path)보다 크면
            last_local_waypoint = len(self.global_path.poses)      #last_local_waypoint는 reh_path
        else :
            last_local_waypoint = self.current_waypoint + 16  
        
        # if self.close_waypoint:
        #     global i 
        #     global list_dis
        #     list_dis = []
        #     rospy.sleep(0.1)
        #     global_len = int(len(self.global_path.poses)/2)
        #     # print('global len : ',global_len)
        #     # print(type(global_len))

        #     for i in range(0, global_len):
        #         dx = self.x - self.global_path.poses[i].pose.position.x 
        #         dy = self.y - self.global_path.poses[i].pose.position.y         #### //
        #         dis = sqrt(pow(dx,2)+pow(dy,2))     #### 변위
        #         list_dis.append(dis)

        #     tmp = min(list_dis)
        #     i = list_dis.index(tmp) + 1
        #     # print("fuck")
        #     self.close_waypoint = False

        # print('i  : ' , i)
        # dx = self.x - self.global_path.poses[i + 1].pose.position.x
        # dy = self.y - self.global_path.poses[i + 1].pose.position.y
        # # print("ssisa")
        # dis = sqrt(pow(dx,2)+pow(dy,2))
        
        # # print('dis: ' ,round(dis,2))
        # # print('=================================')

        # if dis <= 1:
        #     # print('next')
        #     i                     += 1
        #     self.current_waypoint = i
            
        
        # last_local_waypoint = self.current_waypoint + 20
        
        # print(self.current_waypoint)
        # print(last_local_waypoint)

        out_path.header.frame_id = 'map'
        
        for j in range(self.current_waypoint, last_local_waypoint):               #### 현재 waypoint 부터 last local waypoint 까지 x,y 값을 넣어서 out_path 를 만들어주는 거
            tmp_pose                    = PoseStamped()
            tmp_pose.pose.position.x    = self.global_path.poses[j].pose.position.x
            tmp_pose.pose.position.y    = self.global_path.poses[j].pose.position.y
            tmp_pose.pose.orientation.x = 0
            tmp_pose.pose.orientation.y = 0
            tmp_pose.pose.orientation.z = 0
            tmp_pose.pose.orientation.w = 1
            out_path.poses.append(tmp_pose)

        return out_path

    def main(self):
        br = tf.TransformBroadcaster()
        self.global_path = self.read_txt(self.path_name)
        while not rospy.is_shutdown():
            print(self.global_path)
            self.local_path  = self.findLocalPath()
            self.global_path_pub.publish(self.global_path)
            self.local_path_pub.publish(self.local_path)
            self.rate.sleep()
            
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _pathreader = pathReader()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)