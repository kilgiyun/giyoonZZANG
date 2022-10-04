#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from locale import locale_encoding_alias
import sys
import rospy
import rospkg
from morai_msgs.msg import EgoVehicleStatus,CtrlCmd, GPSMessage
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj
from nav_msgs.msg import Path
from math import pi,cos,sin,pi,sqrt,pow
import pandas as pd
from datetime import datetime
from std_msgs.msg import Int16

class data_frame:
    def __init__(self):
        self.x = 0
        self.y = 0

        self.local_path = Path()
        
        self.astar_pub = True
        
        self.rad2deg = 180 / pi
        self.deg2rad = pi / 180
        
        self.current_waypoint = 0
        
        self.heading = 0
        
        self.pre_waypoint = 0
        self.mode   = 0
        
        self.pos_x_list         = []
        self.pos_y_list         = []
        
        self.path_x_list        = []
        self.path_y_list        = []
        
        self.mode_list          = []
        
        self.heading_list       = []
        self.des_heading_list   = []
        
        self.local_path_x_list  = []
        self.local_path_y_list  = []
        
        self.astar_path_x_list  = []
        self.astar_path_y_list  = []
        
        self.astar_path_x  = []
        self.astar_path_y  = []
        
        self.target_vel     = 0
        self.des_steering   = 0
        self.des_brake      = 0
        
        self.next_local_path_x  = 0
        self.next_local_path_y  = 0
        
        self.last_astar_path_x = 0
        self.last_astar_path_y = 0
        
        self.astar_path = 0
        
        self.prev_x = 0
        self.prev_y = 0
        
        self.x_init = 302473.5122667786
        self.y_init = 4123735.6543077542
        
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        rospy.Subscriber('/gps', GPSMessage, self.gpsCB)
        rospy.Subscriber('/local_path', Path, self.LocalCB)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.egoCB)
        rospy.Subscriber('/cmd', CtrlCmd, self.cmdCB)
        rospy.Subscriber('/waypoint', Int16, self.waypointCB)
        rospy.Subscriber('/mode',Int16, self.modeCB)
        rospy.Subscriber('/astar_path', Path, self.astarCB)
        
        self.main()
        
    def waypointCB(self, _data: Int16):
        self.current_waypoint =_data.data
        
    def modeCB(self, _data: Int16):
        self.mode = _data.data
        
    def astarCB(self, _data:Path):                         
        self.astar_path = _data
        _len = len(self.astar_path.poses)
        if self.astar_pub and self.astar_path:
            for a in range(_len):
                self.astar_path_x.append(self.astar_path.poses[a].pose.position.x)
                self.astar_path_y.append(self.astar_path.poses[a].pose.position.y)
                
            self.last_astar_path_x = self.astar_path.poses[_len - 1].pose.position.x
            self.last_astar_path_y = self.astar_path.poses[_len - 1].pose.position.y
            
            # print(self.astar_path_x)
            # print(self.astar_path_y)
            # print(self.last_astar_path_x)
            # print(self.last_astar_path_y)
            self.astar_pub = False
        
    def gpsCB(self, _data:GPSMessage):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        
        self.x = xy_zone[0] - self.x_init
        self.y = xy_zone[1] - self.y_init      
        
    def cmdCB(self, _data:CtrlCmd):
        self.target_vel     = _data.velocity
        self.des_steering   = _data.steering
        self.des_brake      = _data.brake
        
    def egoCB(self, _data: EgoVehicleStatus):
        self.heading = _data.heading
        
    def LocalCB(self, _data:Path):
        self.local_path = _data
        self.next_local_path_x = _data.poses[0].pose.position.x
        self.next_local_path_y = _data.poses[0].pose.position.y
        # print(self.next_local_path_x, self.next_local_path_y)
        # for k in range(len(self.local_path.poses)):
            # pass
        # print(self.local_path.poses[0])
        
    def save_csv(self, state):
        
        rospack =   rospkg.RosPack()
        pkg_path =  rospack.get_path('my_test')
        self.path_folder_name = 'csv'
        month   = datetime.now().month
        day     = datetime.now().day
        hour    = datetime.now().hour
        minute  = datetime.now().minute
        self.make_path_name = '{0}m{1}d{2}h{3}m'.format(month, day, hour, minute)
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name
        state.to_csv(full_path + '.csv', index=False)
        
    def main(self):
        i = 0
        n = 0
        while not rospy.is_shutdown():            
            dis = sqrt(pow(self.x-self.prev_x,2)+pow(self.y-self.prev_y,2))
            
            if dis > 1:
                # print(self.x, self.prev_x, self.y, self.prev_y)
                self.pos_x_list.append(self.x)
                self.pos_y_list.append(self.y)
                self.local_path_x_list.append(self.next_local_path_x)
                self.local_path_y_list.append(self.next_local_path_y)
                self.heading_list.append(self.heading)
                self.des_heading_list.append(self.des_steering * self.rad2deg)
                self.mode_list.append(self.mode)
                if not self.astar_path_x or self.mode ==1:
                    print('is empty')
                    # self.astar_path_x_list.append(self.x)
                    # self.astar_path_y_list.append(self.y)
                    self.astar_path_x_list.append(0)
                    self.astar_path_y_list.append(0)
                elif self.astar_path_x and self.mode == 2 :
                    # print(self.astar_path_x[0])
                    if n >= len(self.astar_path_x) :
                        self.astar_path_x_list.append(0)
                        self.astar_path_y_list.append(0)
                    else:
                        self.astar_path_x_list.append(self.astar_path_x[n])
                        self.astar_path_y_list.append(self.astar_path_y[n])
                        n+=1
                
                self.prev_x = self.x
                self.prev_y = self.y
                
            
            dict = {'pos_x': self.pos_x_list,
                    'pos_y': self.pos_y_list,
                    'path_x': self.local_path_x_list,
                    'path_y': self.local_path_y_list,
                    'heading': self.heading_list,
                    'des_heading': self.des_heading_list,
                    'mode': self.mode_list,
                    'astar_x': self.astar_path_x_list,
                    'astar_y': self.astar_path_y_list
                    }
            
            # print(dict)
            
            # self.pre_waypoint = self.current_waypoint
            
            # print(self.current_waypoint)
            if self.current_waypoint >= 50:
                # print(len(self.pos_x_list))
                # state = pd.DataFrame(list(zip(self.pos_x_list, self.pos_y_list)))
                state = pd.DataFrame(dict)
                # print('save file')
                self.save_csv(state)
            
def main(args):

    rospy.init_node('data_frame', anonymous=False)
    _data_frame = data_frame()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)
