#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import rospkg

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import GPSMessage
from std_msgs.msg import Int16

from math import cos,sin,sqrt,pow,atan2,pi
from pyproj import Proj




class pathReader():                                         
    def __init__(self):
        #### 예선
        self.path_name = 'pre_0929.txt'
        ####
        
        #### 본선
        # self.path_name = 'final_0916.txt'
        ####
        
        #### bag 
        # self.path_name = 'bag04.txt'
        ####
        
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("my_test")

        self.close_waypoint = True
        self.gpsinit = True
                
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542

        self.rate = rospy.Rate(10)

        self.global_path = 0
        self.local_path  = 0

        self.current_waypoint = 0

        self.roll   = 0
        self.pitch  = 0
        self.yaw    = 0
        
        self.x = 0
        self.y = 0
        
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.local_path_pub  = rospy.Publisher('/local_path',Path, queue_size=1)    
        self.waypoint_pub = rospy.Publisher('/waypoint',Int16, queue_size=10)
            
        # rospy.Subscriber('/gps', GPSMessage, self.gpsCB)    
        
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gpsCB)
            
        self.main()

    def gpsCB(self, _data: NavSatFix):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)

        self.x = xy_zone[0] - self.x_init
        self.y = xy_zone[1] - self.y_init 
        # print(_data.longitude, _data.latitude)

    def read_txt(self, file_name):
        full_file_name = self.file_path+"/path/"+file_name
        openFile       = open(full_file_name, 'r')
        
        out_path = Path()
        out_path.header.frame_id = 'map' 
        # out_path.header.frame_id = 'fake_base_link' 

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

        global_len = int(len(self.global_path.poses))
        dis_array = []
        for i in range(global_len):
            dx = self.x - self.global_path.poses[i].pose.position.x 
            dy = self.y - self.global_path.poses[i].pose.position.y         #### //
            dis = sqrt(pow(dx,2)+pow(dy,2))     #### 변위
            dis_array.append(dis)

        min(dis_array)
        dis_array.index(min(dis_array))    
        self.current_waypoint = dis_array.index(min(dis_array))
        print('current :', self.current_waypoint)
        # print(min(dis_array))
        
        if self.current_waypoint + 16 > len(self.global_path.poses) : #현재 웨이 포인트+50이 len(ref_path)보다 크면
            last_local_waypoint = len(self.global_path.poses)      #last_local_waypoint는 reh_path
            # if self.current_waypoint == len(self.global_path.poses) - 1:
            #     # self.ctrl_cmd
        else :
            last_local_waypoint = self.current_waypoint + 14

        out_path.header.frame_id = 'map'
        # out_path.header.frame_id = 'fake_base_link'
        
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
        self.global_path = self.read_txt(self.path_name)
        while not rospy.is_shutdown():
            # print(self.global_path)
            # print('1111')
            self.local_path  = self.findLocalPath()
            self.global_path_pub.publish(self.global_path)
            self.local_path_pub.publish(self.local_path)
            self.waypoint_pub.publish(self.current_waypoint)
            self.rate.sleep()
            
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _pathreader = pathReader()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)