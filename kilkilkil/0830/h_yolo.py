#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy

from morai_msgs.msg import CtrlCmd
from detection_msgs.msg import BoundingBoxes

from math import cos,sin,sqrt,pow,atan2,pi

class Yolo:                                        
    def yolo__init__(self):
        
        self.yolo_pub = True
        self.yolo_off = True  
        
        self.yolo_vel = 0
        self.ctrl_msg = CtrlCmd()
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.yoloCB)
        
    def box_size(self, x1, x2, y1, y2):
        return ((x2 - x1) *(y2 - y1))
    
    def yoloCB(self, _data: BoundingBoxes):
        # print(len(_data.bounding_boxes)==0)
        # print(type(len(_data.bounding_boxes)==0))
        
        
        size = []
        for i in range(len(_data.bounding_boxes)):
            xmax = _data.bounding_boxes[i].xmax
            xmin = _data.bounding_boxes[i].xmin
            ymax = _data.bounding_boxes[i].ymax
            ymin = _data.bounding_boxes[i].ymin
        
            size.append(self.box_size(xmin, xmax, ymin, ymax))
            
        # print(size)
        # print(size.index(max(size)))
        max_index = size.index(max(size))
        # print(max_index)
        
        _yolo_data = _data.bounding_boxes[max_index].Class
        
        print(_yolo_data)
        
        if len(_yolo_data) == 0:
            self.yolo_off = True  
            # print('yolo_offffff')
            # print(self.yolo_status)                       
        elif self.yolo_pub and len(_yolo_data) > 0:
            self.yolo_off = False
            # print('yolo_onnnnnnnc')
            if _yolo_data == "4_red":                        #### 빨간불이면 엑셀:0 , 브레이크: 1
                # print("red")
                self.yolo_vel = 0
            elif _yolo_data == "4_green":
                pass
        
        else:
            pass
        #     elif Class == "Yellow":      
        #         print("yellow")
        #         self.ctrl_msg.accel = 0
        #         self.ctrl_msg.brake = 1
                
        #     elif Class == "left":       
        #         print("left")
        #         # if 내 앞 좌표 3~5개가 좌회전 하는 path라면 gogo
        #         self.ctrl_msg.accel = 0
        #         self.ctrl_msg.brake = 1        
                
        #     elif Class == "straight":
        #         print("straight")
        #         pass
        #     elif Class == "left_straight":
        #         print("left_straight")
        #         pass
            
        # self.yolo_pub = False
        
        return self.ctrl_msg
    
    def main_pure(self):
        pass
            
def main(args):

    rospy.init_node('path_reader', anonymous=False)
    _yolo = Yolo()
    rospy.spin()

if __name__ == '__main__':

    _run = main(sys.argv)