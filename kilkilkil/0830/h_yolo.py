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
        
        self.yolo_real = False    ### Morai
        # self.yolo_real = True     ### real
        
        # self.yolo_final = True
        self.yolo_final = False
        
        self.current_waypoint = 0
        
        self._yolo_data = 0 
        
        if self.yolo_real == False:
            if self.yolo_final:
                self.lines = [92, 93, 94, 95, 96, 135, 136, 137, 138, 236, 237, 238, 239, 240, 241,             #### 1, 2, 3, 7 번의 정지선 앞 
                    548, 549, 550, 551, 552, 553, 554, 555, 656, 657, 658, 659, 660, 693, 694, 695, 696]
                self.lines2 = [323, 324, 325, 326] 
                
            elif self.yolo_final == False:
                self.lines = [93, 94, 95, 96, 97, 98, 99, 100]
                self.lines2 = []
                
                
        ########
        # 323 ~ 326 : 빨좌일때만 (left)
        ########
        self.yolo_vel = 0
        self.ctrl_msg = CtrlCmd()
        rospy.Subscriber('/waypoint', Int16, self.waypointCB)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
    
    def waypointCB(self, _data: Int16):
        self.current_waypoint =_data.data
        
    def yoloCB(self, _data: BoundingBoxes):

        size = []
        if self.current_waypoint in self.lines or self.current_waypoint in self.lines2:
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
        
        ############## Real
        if self.yolo_real:
            if self.current_waypoint in self.lines:
                if self._yolo_data == 'red' or self._yolo_data == 'yellow':
                    self.green_staus = False
                    
                elif self._yolo_data == 'straight' or self._yolo_data == 'left_straight':
                    self.red_staus   = False
                    self.green_staus = True
            
            elif self.current_waypoint in self.lines2:
                if self._yolo_data == 'left':    
                    self.red_staus   = False
                    self.green_staus = True
                elif self._yolo_data == 'red' or self._yolo_data == 'yellow' or self._yolo_data == 'straight' or self._yolo_data == 'left_straight':
                    self.red_staus   = True
                    self.green_staus = False
                    
            else:
                pass
        ##############
        
        ############## Moari
        else:
            if self.current_waypoint in self.lines:
                # print('normal')
                if self._yolo_data == '4_red' or self._yolo_data == '3_red' or self._yolo_data == '4_yellow':
                    self.red_staus   = True
                    self.green_staus = False
                    
                elif self._yolo_data == '4_green' or self._yolo_data == '3_green' or self._yolo_data == '4_str_left' :
                    self.red_staus   = False
                    self.green_staus = True
            
            elif self.current_waypoint in self.lines2:
                if self._yolo_data == '4_red':    
                    self.red_staus   = False
                    self.green_staus = True
                elif self._yolo_data == '4_yellow' or self._yolo_data == '4_green' or self._yolo_data == '4_left'or self._yolo_data == '4_str_left'or self._yolo_data == '4_yel_green'or self._yolo_data == '3_red'or self._yolo_data == '3_yellow'or self._yolo_data == '3_green':
                    self.red_staus   = True
                    self.green_staus = False
                    
            else:
                pass
        ##############
        
        print(self._yolo_data)
        print('red', self.red_staus)
        print('green', self.green_staus)
        
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _yolo = Yolo()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)