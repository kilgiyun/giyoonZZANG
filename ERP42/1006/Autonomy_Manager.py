#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from curses.ascii import ctrl
import sys
import os
sys.path.append("/home/catkin_ws/src/my_test/include")

from std_msgs.msg import Int16
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
        
        self.stop_data = 0
        self.stop_status = False
        rospy.Subscriber('/stop',Int16, self.stopCB)
        
        self.main()              
        
    def stopCB(self, _data:Int16):
        self.stop_data = _data.data
            
    def main(self):
        while not rospy.is_shutdown():
            self.main_yolo()
            ##########
            if self.red_staus or self.stop_data == 1:
                self.ctrl_msg.steering = 0
                self.ctrl_msg.velocity = -4
                # self.ctrl_msg.brake = 1
                # print('stop')
            else:
                self.ctrl_msg.steering = self.steering_angle() ## deg
                self.ctrl_msg.velocity = self.vel()
                # print('go')

            if self.local_path:
                self.mode = 1
                if self.astar_path:
                    self.mode = 2
                    dis = sqrt(pow(self.goal_pos_x - self.cur_x,2) + pow(self.goal_pos_y - self.cur_y, 2))
                    # print("dis:", dis)
                    # print('==============================')
                    if dis <= 1.5:
                        self.astar_path = False

            if self.ctrl_msg.steering!= None:
                self.cmd_pub.publish(self.ctrl_msg)  #### field steering must be float type
                self.mode_pub.publish(self.mode)
                
            self.rate.sleep()
        
            
def main(args):

    rospy.init_node('Autonomy_Manager', anonymous=False)

    _start_planner = Start_planner()

    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)