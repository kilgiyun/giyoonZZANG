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
        
        self.lines = [93, 94, 95, 96, 135, 136, 137, 138, 236, 237, 238, 239, 240, 241,
                    323, 324, 325, 326, 550, 551, 552, 553, 554, 555, 657, 658, 659, 660, 693, 694, 695, 696]
        
        self.yolo_vel = 0
        self.ctrl_msg = CtrlCmd()
        rospy.Subscriber('/waypoint', Int16, self.waypointCB)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
    
    def waypointCB(self, _data: Int16):
        self.current_waypoint =_data.data
        ####
        # 1번째 : 69 - 71
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
            
            if size:
                max_index = size.index(max(size))  
                self._yolo_data = _data.bounding_boxes[max_index].Class
        else:
            self._yolo_data = 0
    
    def box_size(self, x1, x2, y1, y2):
        return ((x2 - x1) *(y2 - y1))
    
    def main_yolo(self):
        
        ############## Moari
        if self._yolo_data == '4_red' or self._yolo_data == '3_red' or self._yolo_data == '4_yellow':
            self.red_staus   = True
            self.green_staus = False
            
        elif self._yolo_data == '4_green' or self._yolo_data == '3_green' or self._yolo_data == '4_str_left' :
            self.red_staus   = False
            self.green_staus = True
        else:
            pass
        ##############
        
        ############## Real
        if self._yolo_data == 'red' or self._yolo_data == 'yellow':
            self.red_staus   = True
            self.green_staus = False
            
        elif self._yolo_data == 'straight' or self._yolo_data == 'left_straight' or self._yolo_data == 'left':
            self.red_staus   = False
            self.green_staus = True
        else:
            pass
        ##############
        # print('red', self.red_staus)
        # print('green', self.green_staus)
        
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _yolo = Yolo()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)