#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import numpy as np
from PIL import Image
from pyproj import Proj
from matplotlib import path
from matplotlib.pyplot import pause
from math import sqrt

from visualization_msgs.msg import Marker
from std_msgs.msg import Int16

import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from gpp_astar.srv import Astar


import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

class SubLocalMap():
    def __init__(self):

        self.bReceived           = False
        self.bCalcLocalPathDone  = False
        self.bSubGPS             = False
        self.bSubEgoState        = False
        self.bastarState         = False
        self.okaaaaay            = True
        
        self.resultPath        = 0

        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        #### preliminary
        self.obstacle_lines = [136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151,
                            152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166]
        ####
        #### move obstacle : 102 ~ 115
        #### obstacle      : 186 ~208
        #### 
        #### final
        # self.obstacle_lines = [180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194,
        #                     195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
        #                     211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 
        #                     227, 228, 229, 230, 231]
        ####
        
        self.cur_x = 0
        self.cur_y = 0

        self.Mor_x = 0 #### ego_topic - gps 기준 utm 
        self.Mor_y = 0

        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
    
        self.fResolution = 0
        self.iWidth      = 0
        self.iHeight     = 0
        self.arrayMap    = 0

        self.dis_obstacle = 7
        ############### 1pixel = 0.5 m
        ############### nemely, 40 pixel = 20 m 
        
        self.local_path_real_x          = []
        self.local_path_real_y          = []
        self.line_pos_x                 = []
        self.line_pos_y                 = []
        self.realrealreal_LocalPath_x   = []
        self.realrealreal_LocalPath_y   = []
        self.arrayRealLocalPath_x       = []
        self.arrayRealLocalPath_y       = []
                
        self.local_path       = 0
        self.twodimension_map = 0

        self.localmap_start_x = 0
        self.localmap_start_y = 0

        self.pos_x = 0
        self.pos_y = 0

        self.path_length = 0



        self.local_map_marker_1 = Marker()
        self.local_map_marker_2 = Marker()
        self.local_map_marker_3 = Marker()

        rospy.Subscriber('/local_map', OccupancyGrid, self.fnc_subLocalMap) 
        rospy.Subscriber('/local_path', Path, self.fnc_subLocalPath)
        rospy.Subscriber('/gps', GPSMessage, self.fnc_subGpsCB)
        rospy.Subscriber('/waypoint', Int16, self.waypointCB)
        #####

        self.srvAstar = rospy.ServiceProxy('/gpp_Astar', Astar)
        #####
        self.pub_local_map_marker_1 = rospy.Publisher('/local_map_marker_1',Marker, queue_size=1)
        self.pub_local_map_marker_2 = rospy.Publisher('/local_map_marker_2',Marker, queue_size=1)
        self.pub_local_map_marker_3 = rospy.Publisher('/local_map_marker_3',Marker, queue_size=1)

        self.pub_astar_path         = rospy.Publisher('/astar_path', Path, queue_size=1)
    
        self.main()
        
        
    def waypointCB(self, _data: Int16):
        self.current_waypoint =_data.data
        
    def fnc_subGpsCB(self, _data:GPSMessage):

        xy_zone = self.proj_UTM(_data.longitude, _data.latitude)
        
        # if self.gpsDataFirst:
        #     self.origin_gpspos_x = xy_zone[0]    ### 30000
        #     self.origin_gpspos_y = xy_zone[1]    ### 40000
        #     # print('subgpsData')
        #     self.gpsDataFirst = False

        self.cur_x = xy_zone[0] - self.x_init  
        self.cur_y = xy_zone[1] - self.y_init   
        self.bSubGPS = True

    def fnc_subLocalMap(self, _data:OccupancyGrid):
        if self.bSubGPS:
            self.fResolution = _data.info.resolution
            self.iWidth      = _data.info.width
            self.iHeight     = _data.info.height
            self.arrayMap    = _data.data

            self.localmap_start_x = _data.info.origin.position.x   ### local map 초기 위치 
            self.localmap_start_y = _data.info.origin.position.y
            
            a = np.array(self.arrayMap, dtype=np.uint8) 

            self.twodimension_map = a.reshape(self.iWidth, self.iHeight)
            self.bReceived = True
    
    def fnc_subLocalPath(self, _data: Path):
        self.local_path = _data.poses
        self.fnc_CalclocalPath()
    
    def pub_marker(self, _x1, _y1, _x2, _y2, _x3, _y3):
        _marker_type = 1
        _marker_size = 0.5
        self.local_map_marker_1.header.frame_id="map"
        self.local_map_marker_1.header.stamp=rospy.Time()
        self.local_map_marker_1.ns ="test"
        self.local_map_marker_1.id=0
        self.local_map_marker_1.type=_marker_type
        self.local_map_marker_1.action=0
        self.local_map_marker_1.pose.position.x = _x1
        self.local_map_marker_1.pose.position.y = _y1
        self.local_map_marker_1.pose.orientation.x = 0.0
        self.local_map_marker_1.pose.orientation.y = 0.0
        self.local_map_marker_1.pose.orientation.z = 0.0
        self.local_map_marker_1.pose.orientation.w = 1
        self.local_map_marker_1.scale.x=_marker_size
        self.local_map_marker_1.scale.y=_marker_size
        self.local_map_marker_1.scale.z=_marker_size
        self.local_map_marker_1.color.a=1
        self.local_map_marker_1.color.r=0
        self.local_map_marker_1.color.g=1
        self.local_map_marker_1.color.b=0

        self.local_map_marker_2.header.frame_id="map"
        self.local_map_marker_2.header.stamp=rospy.Time()
        self.local_map_marker_2.ns ="test"
        self.local_map_marker_2.id=0
        self.local_map_marker_2.type=_marker_type
        self.local_map_marker_2.action=0
        self.local_map_marker_2.pose.position.x = _x2
        self.local_map_marker_2.pose.position.y = _y2
        self.local_map_marker_2.pose.orientation.x = 0.0
        self.local_map_marker_2.pose.orientation.y = 0.0
        self.local_map_marker_2.pose.orientation.z = 0.0
        self.local_map_marker_2.pose.orientation.w = 1
        self.local_map_marker_2.scale.x=_marker_size
        self.local_map_marker_2.scale.y=_marker_size
        self.local_map_marker_2.scale.z=_marker_size
        self.local_map_marker_2.color.a=1
        self.local_map_marker_2.color.r=0
        self.local_map_marker_2.color.g=1
        self.local_map_marker_2.color.b=0

        self.local_map_marker_3.header.frame_id="map"
        self.local_map_marker_3.header.stamp=rospy.Time()
        self.local_map_marker_3.ns ="test"
        self.local_map_marker_3.id=0
        self.local_map_marker_3.type=_marker_type
        self.local_map_marker_3.action=0
        self.local_map_marker_3.pose.position.x = _x3
        self.local_map_marker_3.pose.position.y = _y3
        self.local_map_marker_3.pose.orientation.x = 0.0
        self.local_map_marker_3.pose.orientation.y = 0.0
        self.local_map_marker_3.pose.orientation.z = 0.0
        self.local_map_marker_3.pose.orientation.w = 1
        self.local_map_marker_3.scale.x=_marker_size
        self.local_map_marker_3.scale.y=_marker_size
        self.local_map_marker_3.scale.z=_marker_size
        self.local_map_marker_3.color.a=1
        self.local_map_marker_3.color.r=0
        self.local_map_marker_3.color.g=1
        self.local_map_marker_3.color.b=0

        self.pub_local_map_marker_1.publish(self.local_map_marker_1)
        self.pub_local_map_marker_2.publish(self.local_map_marker_2)
        self.pub_local_map_marker_3.publish(self.local_map_marker_3)

    def fnc_CalclocalPath(self):
        # f = open('data.txt', 'w')
        if self.bReceived:
            self.line_pos_x = []
            self.line_pos_y = []
            for i in range(0, self.local_path.__len__()):
                pos_x = self.local_path[i].pose.position.x
                pos_y = self.local_path[i].pose.position.y
                self.line_pos_x.append(pos_x)
                self.line_pos_y.append(pos_y)
                
            # print(self.line_pos_x)

            ##### covert to 1 m unit
            clean_pos_x = []
            clean_pos_y = []
            for j in range(0, len(self.line_pos_x) - 1):
                dis = sqrt(pow((self.line_pos_x[j+1] - self.line_pos_x[j]),2)+pow((self.line_pos_y[j+1] - self.line_pos_y[j]),2))
                if dis >= 1:
                    clean_pos_x.append(self.line_pos_x[j])
                    clean_pos_y.append(self.line_pos_y[j])
                    
            # print(clean_pos_x)
            # print(clean_pos_y)
            # print('================================================')
            ###### utm 2 pixel (local_map)
            self.local_path_real_x = []
            self.local_path_real_y = []
            for k in range(0, len(clean_pos_x)):
                self.local_path_real_x.append((clean_pos_x[k] - (self.localmap_start_x)) / self.fResolution)
                self.local_path_real_y.append((clean_pos_y[k] - (self.localmap_start_y)) / self.fResolution)
            
            # print(self.local_path_real_x)
            # print(self.local_path_real_y)
            # print('================================================')
            #### 정수로 바꿔줌
            self.arrayRealLocalPath_x = []
            self.arrayRealLocalPath_y = []
            for l in range(0, len(self.local_path_real_x)):
                # print('111111')
                if self.local_path_real_x[l] <= self.iWidth and self.local_path_real_y[l] <= self.iHeight:
                    # print('22222222')
                    self.arrayRealLocalPath_x.append(round(abs(self.local_path_real_x[l])))
                    self.arrayRealLocalPath_y.append(round(abs(self.local_path_real_y[l])))
            
            # print(self.arrayRealLocalPath_x)
            # print(self.arrayRealLocalPath_y)
            # print('================================================')
            # self.path_length = len(self.arrayRealLocalPath_x)
            self.bCalcLocalPathDone = True

            #### 다시 바꾼 실제 local paths
            self.realrealreal_LocalPath_x = []
            self.realrealreal_LocalPath_y = []
            for n in range(0, len(self.arrayRealLocalPath_x)):            
                self.realrealreal_LocalPath_x.append(((self.arrayRealLocalPath_x[n] * self.fResolution) + self.localmap_start_x))
                self.realrealreal_LocalPath_y.append(((self.arrayRealLocalPath_y[n] * self.fResolution) + self.localmap_start_y))
                
            # print(self.realrealreal_LocalPath_x)
            # print(self.realrealreal_LocalPath_y)
            # print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
            # print("sibal")
            self.path_length = len(self.arrayRealLocalPath_x)
            # print(self.path_length)
            # print(self.realrealreal_LocalPath_x[0])
            # print(self.realrealreal_LocalPath_y[0])

            # print(self.arrayRealLocalPath_x[0])
            
            if self.path_length >= 1:
                self.pub_marker((self.arrayRealLocalPath_x[0] * self.fResolution) + self.localmap_start_x , (self.arrayRealLocalPath_y[0] * self.fResolution) + self.localmap_start_y, 
                                (self.arrayRealLocalPath_x[self.dis_obstacle]* self.fResolution) + self.localmap_start_x , (self.arrayRealLocalPath_y[self.dis_obstacle] * self.fResolution) + self.localmap_start_y,
                                (self.arrayRealLocalPath_x[self.path_length - 1]* self.fResolution) + self.localmap_start_x , (self.arrayRealLocalPath_y[self.path_length - 1] * self.fResolution) + self.localmap_start_y)
            
            # print('fucking', (self.arrayRealLocalPath_x[0] * self.fResolution) + self.localmap_start_x ,(self.arrayRealLocalPath_y[0] * self.fResolution) + self.localmap_start_y, \
            #                 (self.arrayRealLocalPath_x[self.dis_obstacle]* self.fResolution) + self.localmap_start_x , (self.arrayRealLocalPath_y[self.dis_obstacle] * self.fResolution) + self.localmap_start_y,\
            #                     (self.arrayRealLocalPath_x[self.path_length - 1]* self.fResolution) + self.localmap_start_x , (self.arrayRealLocalPath_y[self.path_length - 1] * self.fResolution) + self.localmap_start_y)
            
            # print(self.arrayRealLocalPath_x[0], self.arrayRealLocalPath_y[0],self.arrayRealLocalPath_x[self.dis_obstacle], self.arrayRealLocalPath_y[self.dis_obstacle], self.arrayRealLocalPath_x[self.path_length - 1], self.arrayRealLocalPath_y[self.path_length - 1])
    
    def main(self):
        while not rospy.is_shutdown():        
            try:
                # if self.current_waypoint in self.obstacle_lines:
                if self.bReceived and self.bCalcLocalPathDone:
                    
                    _data_check_x = self.arrayRealLocalPath_x
                    _data_check_y = self.arrayRealLocalPath_y
    
                    _data1 = self.twodimension_map[self.arrayRealLocalPath_y[self.dis_obstacle]][self.arrayRealLocalPath_x[self.dis_obstacle]]
                    _data2 = self.twodimension_map[self.arrayRealLocalPath_y[self.dis_obstacle] - 1][self.arrayRealLocalPath_x[self.dis_obstacle]]
                    _data3 = self.twodimension_map[self.arrayRealLocalPath_y[self.dis_obstacle] + 1][self.arrayRealLocalPath_x[self.dis_obstacle]]
                    _data4 = self.twodimension_map[self.arrayRealLocalPath_y[self.dis_obstacle] ][self.arrayRealLocalPath_x[self.dis_obstacle] - 1]
                    _data5 = self.twodimension_map[self.arrayRealLocalPath_y[self.dis_obstacle] ][self.arrayRealLocalPath_x[self.dis_obstacle] + 1]
                    
                    #############################################################
                    ###### 100 이면 장애물 있는거, 50이면모르는 거 , 0 이면 빈 공간  x,y 좌표 반대임
                    #############################################################
                    print(_data1, _data2, _data3, _data4, _data5)############ 다시 켜주고
                    
                    ##################           

                    if _data1 >= 100 or _data2 >= 100 or _data3 >= 100 or _data4 >= 100 or _data5 >= 100:
                        # _start_pos_x = _data_check_x[int(self.path_length/5)]
                        # _start_pos_y = _data_check_y[int(self.path_length/5)]
                        _start_pos_x    = _data_check_x[2]
                        _start_pos_y    = _data_check_y[2]
                        _goal_pos_x     = _data_check_x[self.path_length - 2]
                        _goal_pos_y     = _data_check_y[self.path_length - 2]
                        _obstacle_pos_x = _data_check_x[int(self.path_length / 2)]
                        _obstacle_pos_y = _data_check_y[int(self.path_length / 2)]
                        self.resultPath = self.srvAstar(_start_pos_x, _start_pos_y, _goal_pos_x, _goal_pos_y, _obstacle_pos_x, _obstacle_pos_y)
                        print('astar publish')
                        self.bastarState = True
                    rospy.sleep(0.1)
            except:
                pass
            self.astar_pub()
            # self.pub_astar_path.publish(self.astar_path)

    
    def astar_pub(self):
        astar_path                 = Path()
        astar_path.header.frame_id = 'map' ## idk\
        if self.resultPath != 0:
            for i in range(0, len(self.resultPath.path_x)):
                read_pose                    = PoseStamped()
                read_pose.pose.position.x    = (self.resultPath.path_x[i] * self.fResolution) + self.localmap_start_x
                read_pose.pose.position.y    = (self.resultPath.path_y[i] * self.fResolution) + self.localmap_start_y
                # read_pose.pose.position.x    = self.resultPath.path_x[i] 
                # read_pose.pose.position.y    = self.resultPath.path_y[i]
                read_pose.pose.orientation.x = 0
                read_pose.pose.orientation.y = 0
                read_pose.pose.orientation.z = 0
                read_pose.pose.orientation.w = 1
                astar_path.poses.append(read_pose)
                # print(astar_path.poses[i].pose.position)
            # print(astar_path)
            
            if self.bastarState:

                self.pub_astar_path.publish(astar_path)
                self.bastarState =False
            return astar_path
        

def main(args):

    rospy.init_node('localmap_ver2', anonymous=False)

    _sublocalmap = SubLocalMap()

    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)
