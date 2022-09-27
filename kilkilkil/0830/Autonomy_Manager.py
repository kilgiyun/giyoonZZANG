#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from curses.ascii import ctrl
import sys
sys.path.append("/home/giyun/catkin_ws/src/my_test/include")

import rospy
from math import cos,sin,sqrt,pow,atan2,pi

from h_pure_pursuit import PurePursuit
from h_yolo import Yolo

import matplotlib.pyplot as plt

class Start_planner(PurePursuit, Yolo):
    def __init__(self):       
        self.pure__init__()
        self.yolo__init__()
        self.rate = rospy.Rate(30)
        
        self.main()              
            
    def main(self):
        while not rospy.is_shutdown():
            self.main_yolo()
            ##########
            # print('sijac')
            if self.red_staus:
                self.ctrl_msg.steering = 0
                self.ctrl_msg.velocity = -4
                # self.ctrl_msg.brake = 1
                print('stop')
            else:
                self.ctrl_msg.steering = self.steering_angle() ## deg
                self.ctrl_msg.velocity = self.vel()
                print('go')
            
            # print(self.ctrl_msg.velocity)
            # print(self.current_waypoint)
            
            if self.local_path:
                self.mode = 1
                if self.astar_path:
                    self.mode = 2
                    # print('umm', self.goal_pos_x, self.goal_pos_y)
                    # print('cur', self.x, self.y)
                    dis = sqrt(pow(self.goal_pos_x - self.cur_x,2) + pow(self.goal_pos_y - self.cur_y, 2))
                    print("dis:", dis)
                    print('==============================')
                    if dis <= 2:
                        self.astar_path = False
                        
            print('self.mode:', self.mode)
            # print(self.ctrl_msg)
            if self.ctrl_msg.steering!= None:
                self.cmd_pub.publish(self.ctrl_msg)  #### field steering must be float type
                self.mode_pub.publish(self.mode)
                
            plt.plot([1, 2, 3, 4], [1, 4, 9, 16])
            plt.show()
            self.rate.sleep()
            
def main(args):

    rospy.init_node('Autonomy_Manager', anonymous=False)

    _start_planner = Start_planner()

    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)