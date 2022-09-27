#!/usr/bin/env python3
import roslib
roslib.load_manifest('gpp_astar')
import sys
sys.path.append("/home/syk/catkin_ws/src/simul_pkg/gpp/gpp_astar/include/")
sys.path.append("/home/syk/catkin_ws/src/simul_pkg/gpp/gpp_astar/map_img/")

import rospy
import os
import time
import numpy as np
import cv2

import csv

from pprint import pprint
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_srvs.srv import Empty
from gpp_astar.srv import Astar, AstarResponse
from threading import Thread
from math import pow,atan2,sqrt,sin,cos
from math import hypot

from matplotlib import pyplot as plt

from h_A_star import h_A_star

class c_A_star(h_A_star):
    def __init__(self):
        self.init_variables()
        self.init_srv()
        
        self.astar_main()

    def astar_main(self):
        pass

def main(args):
    rospy.init_node('gpp_A_star', anonymous = True)
    start_gpp = c_A_star()

    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
