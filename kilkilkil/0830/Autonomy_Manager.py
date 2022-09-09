#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
sys.path.append("/home/giyun/catkin_ws/src/my_test/include")

import rospy
from math import cos,sin,sqrt,pow,atan2,pi

from h_pure_pursuit import PurePursuit
from h_yolo import Yolo

class Start_planner(PurePursuit, Yolo):
    def __init__(self):       
        self.pure__init__()
        self.yolo__init__()
        
        self.main()               
        
    def main(self):
        while not rospy.is_shutdown():
            self.ctrl_msg.steering = self.steering_angle() ## deg
            if self.yolo_pub:
                self.ctrl_msg.velocity = self.yolo_vel
                # print('yolo_on')
                if self.yolo_off:
                    self.ctrl_msg.velocity = self.vel()
                    # print('yolo_status')
                    
            if self.local_path:
                self.mode = 1
                if self.astar_path:
                    self.mode = 2
                    # print('umm', self.goal_pos_x, self.goal_pos_y)
                    # print('cur', self.x, self.y)
                    dis = sqrt(pow(self.goal_pos_x - self.cur_x,2) + pow(self.goal_pos_y - self.cur_y, 2))
                    # print("dis:", dis)
                    # print('==============================')
                    if dis <= 2:
                        self.astar_path = False
                        
            # print(self.mode)
            # print(self.ctrl_msg)
            self.cmd_pub.publish(self.ctrl_msg)
            self.mode_pub.publish(self.mode)
            
            self.rate.sleep()
            
def main(args):

    rospy.init_node('Autonomy_Manager', anonymous=False)

    _start_planner = Start_planner()

    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)