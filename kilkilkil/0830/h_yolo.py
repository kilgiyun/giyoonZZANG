#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from xml.etree.ElementPath import get_parent_map
import rospy

from std_msgs.msg import Int16
from morai_msgs.msg import CtrlCmd
from detection_msgs.msg import BoundingBoxes

from math import cos,sin,sqrt,pow,atan2,pi

class Yolo:                                        
    def yolo__init__(self):
        
        self.yolo_pub = True
        self.yolo_off = True  
        
        self.red_staus   = False
        self.green_staus = False
        
        self.current_waypoint = 0
        
        self._yolo_data = 0 
        
        self.lines = [64, 65, 66, 67, 68, 69, 107, 108, 109, 110, 111, 112, 212, 213, 214, 215,
                        216, 217, 218, 295, 296, 297, 298, 299, 300, 301, 302, 522, 523, 524, 525,
                        526, 623, 624, 625, 626, 627, 628, 659, 660, 661, 662, 663]
        
        self.yolo_vel = 0
        self.ctrl_msg = CtrlCmd()
        rospy.Subscriber('/waypoint', Int16, self.waypointCB)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
    
    def waypointCB(self, _data: Int16):
        self.current_waypoint =_data.data
        ####
        # 1번째 : 64 - 69
        # 2번째 : 107- 112
        # 3번째 : 212- 218
        # 4번째 : 295 - 302
        # 5번째 : U 턴
        # 6번째 : 522 - 526
        # 7번째 : 623 - 628
        # 8번째 : 659 - 663 
        ####
        
    def yoloCB(self, _data: BoundingBoxes):

        size = []
        if self.current_waypoint in self.lines:
            # print('111')
            for i in range(len(_data.bounding_boxes)):
                xmax = _data.bounding_boxes[i].xmax
                xmin = _data.bounding_boxes[i].xmin
                ymax = _data.bounding_boxes[i].ymax
                ymin = _data.bounding_boxes[i].ymin
            
                size.append(self.box_size(xmin, xmax, ymin, ymax))
                
            max_index = size.index(max(size))  
            self._yolo_data = _data.bounding_boxes[max_index].Class
        else:
            self._yolo_data = 0
        
        # print(_yolo_data)
    
    def box_size(self, x1, x2, y1, y2):
        return ((x2 - x1) *(y2 - y1))
    
    def main_yolo(self):

        if self._yolo_data == '4_red' or self._yolo_data == '3_red':
            print('reddddd')
            self.red_staus   = True
            self.green_staus = False
            
        elif self._yolo_data == '4_green' or self._yolo_data == '3_green':
            print('grennnn')
            self.red_staus   = False
            self.green_staus = True
        else:
            print('nothing')
            self.red_staus   = False
            self.green_staus = False
        
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _yolo = Yolo()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)