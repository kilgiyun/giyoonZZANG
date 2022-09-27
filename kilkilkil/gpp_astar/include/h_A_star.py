#!/usr/bin/env python3

import roslib
roslib.load_manifest('gpp_astar')
import sys
sys.path.append("/home/giyun/catkin_ws/src/gpp_astar/src")

sys.path.append("/home/syk/catkin_ws/src/simul_pkg_201221/gpp_pkg/gpp_astar/map_img/")

import rospy
import os
import time
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


import csv

from pprint import pprint
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_srvs.srv import Empty
from gpp_astar.srv import Astar, AstarResponse
from threading import Thread
from nav_msgs.msg import *
from math import pow,atan2,sqrt,sin,cos
from math import hypot

from matplotlib import pyplot as plt

class h_A_star():

    def __init__(self):
        self.init_variables()
        self.init_srv()
        self.init_sub()
    
    def init_variables(self):
        # _s_img_name = rospy.get_param("/gpp_astar/map_img")
        _b_map_plot = rospy.get_param("/map_plot")
        self.localmap_sub = rospy.get_param("/localmap_topic")
        
        self.f_heuristic_weight = rospy.get_param('/heuristic_weight')
        self.i_resize_x = rospy.get_param('/resize_grid_x')
        self.i_resize_y = rospy.get_param('/resize_grid_y')

        self.b_map_plot = _b_map_plot

        self.fResolution      = 0
        self.iWidth           = 0
        self.iHeight          = 0
        self.arrayMap         = 0
        self.twodimension_map = 0
        self.bReceived = False

    def init_pub(self):
        pass

    def init_sub(self):
        _sub1 = rospy.Subscriber(self.localmap_sub, OccupancyGrid, self.fnc_subLocalMap)

    def init_srv(self):
        self.srv_server_gpp = rospy.Service('/gpp_Astar', Astar, self.calc_gpp_astar)
        # print(self.srv_server_gpp)

    def fnc_subLocalMap(self, _data:OccupancyGrid):
    
        self.fResolution = _data.info.resolution
        self.iWidth      = _data.info.width
        self.iHeight     = _data.info.height
        self.arrayMap    = _data.data
        # print(self.fResolution)
        # print(self.iWidth)
        # print(self.iHeight)
        # print(self.arrayMap)
        # , self.iWidth, self.iHeight, self.arrayMap)
        a = np.array(self.arrayMap, dtype=np.uint8) 
        # print(a)
        self.twodimension_map = a.reshape(self.iWidth, self.iHeight)
        # print(self.fResolution)
        self.bReceived = True

    def fncObstacleCheck(self, _map):
        pass
    
    def Dist_chk(self, x, x1, y, y1):
        return sqrt()
    def calc_gpp_astar(self, _req: Astar):

        _start_x    = _req.start_pos_x
        _start_y    = _req.start_pos_y
        _goal_x     = _req.goal_pos_x
        _goal_y     = _req.goal_pos_y
        _obstacle_x = _req.obstacle_pos_x
        _obstacle_y = _req.obstacle_pos_y
        # print(_req)
        _weight = sqrt(self.f_heuristic_weight)
        # print('w:', _weight)
        _pos = [_goal_x, _goal_y]
        
        _Map = self.twodimension_map

        rospy.sleep(0.1)
        _height = len(_Map)
        _width = len(_Map[0])
        

        _xmax = _width      ## 40 
        _ymax = _height     ## 40

        _b_solved = False   
        _i = 0

        _solved_count = 0

        _path_x = []
        _path_y = []

        _closed_nodes = []

        _H = np.zeros((_height, _width))        ### 전부 0
        _G = np.zeros((_height, _width))        ### 전부 0    
        _F = np.zeros((_height, _width))        ### 전부 0
        _O = np.zeros((_height, _width))        ### 장애물 cost
        
        np.set_printoptions(threshold=sys.maxsize)
        # print(_H)
        # print(_G)
        # print(_F)
        _open_nodes = []
        _dis        = []
        _pos_start      = [_start_x, _start_y]        ### 시작 점
        _pos_goal       = [_goal_x, _goal_y]          ### 도착 점
        _pos_obstacle   = [_obstacle_x, _obstacle_y]  ### 장애물 점
        
        # print(_pos_start)
        # print(_pos_goal)
        # print(_pos_obstacle)
        
        for i in range(len(_Map)):              ### 40
            for j in range(len(_Map[0])):       ### 40
                if _Map[i][j] != 100:           ### 장애물이 아니면
                    _dist_x = _pos_goal[0] - j  ### MAP[0][0] 부터 MAP[40][40] 까지 도착 점이랑 좌표랑 dis계산
                    _dist_y = _pos_goal[1] - i
                    _dist_obs_x = _pos_obstacle[0] - j
                    _dist_obs_y = _pos_obstacle[0] - i
                    _H[i][j] = _weight * np.sqrt(_dist_x * _dist_x + _dist_y * _dist_y)  ### H = heuristic_weight * 거리 (도착지점 까지의 COST)
                    _G[i][j] = float('inf')     ### 이건 걍 inf
                    # _O[i][j] =  1 / ((sqrt(_dist_obs_x * _dist_obs_x + _dist_obs_y * _dist_obs_y)) + 1)
                    # print('{0}, {1} : '.format(i,j), _O[i][j])
                    # _dis.append(np.sqrt(_dist_obs_x * _dist_obs_x + _dist_obs_y * _dist_obs_y))
        
        for q in range (_pos_obstacle[1] - 2, _pos_obstacle[1] + 2):
            for w in range (_pos_obstacle[0] - 2, _pos_obstacle[0] + 2):    
                _O[q][w] = 50
        
        print(_O)
        _G[_pos_start[1]][_pos_start[0]] = 0    ### 출발 지점(y,x)의 COST = 0 (근데 왜 y랑 x 반전이지)
        _F[_pos_start[1]][_pos_start[0]] = _H[_pos_start[1]][_pos_start[0]]  ### 출발 점 F 는 출발 점 H
        print('first',_O)
        _open_nodes = [[_pos_start[0], _pos_start[1], _G[_pos_start[1]][_pos_start[0]],\
                            _F[_pos_start[1]][_pos_start[0]], 0]]      ### [[14, 10, 0.0, 196.468827, 0]]
        # print(len(_open_nodes))      ### 시작 점 X, 시작 점 Y, 시작 점 H의 COST, 시작 점 F의 COST

        ### 5개씩 들어가는데
        while not rospy.is_shutdown():
            _A = []
            _temp = []

            for i in range(len(_open_nodes)):       ### open nodes의 길이? (1로 나오던데)
                _temp.append(_open_nodes[i][3])     ### open nodes[i][3] = > _G[_pos_start[1]][_pos_start[0]] 이것만 계속 확인 => 시작점의 G 값?

            _A = [min(_temp)]

            _i = _temp.index(_A[0])     ### temp 의 index 중 _A[0] 인 index = > _A[0] 인 것은 196.4688
            
            _current = []
        
            _current = [_open_nodes[_i][0],_open_nodes[_i][1],_open_nodes[_i][2],_open_nodes[_i][3],_open_nodes[_i][4]]
            # print(_current)
            if _current[0] == _pos_goal[0] and _current[1] == _pos_goal[1]:     ### _open_nodes[_i][0] 이 도착 점이랑 좌표가 맞으면 (맞을때까지 찾는거네)
                _closed_nodes.append(_current)                                  ### 여태까지의 _current 를 전부 추가 

                _b_solved = True
                break
            del _open_nodes[_i]                                                 ### _open_nodes[_i] 를 삭제
            
            _closed_nodes.append(_current)                                      ### 왜 또? / 마지막 줄이 없네
            
            for x in range(_current[1] - 1, _current[1] + 2):                   ### 원래는 x랑 y가 뒤바껴
                for y in range(_current[0] - 1, _current[0] + 2):

                    if x < 1 or x > _xmax+1 or y < 1 or y > _ymax+1:
                        continue

                    if x <= _xmax and y <= _ymax:
                        if _Map[x][y] == 100:
                            continue

                    if x == _current[1] and y == _current[0]:
                        continue

                    _b_skip = False

                    for i in range(0 , len(_closed_nodes)):
                        if x == _closed_nodes[i][1] and y == _closed_nodes[i][0]:
                            _b_skip = True
                            break
                    
                    if _b_skip == True:
                        continue

                    _A = [-1]

                    if _open_nodes:
                        for i in range(0, len(_open_nodes)):
                            if x == _open_nodes[i][1] and y == _open_nodes[i][0]:
                                _A = [i]
                                break

                    _diff_temp_x = _current[0] - y
                    _diff_temp_y = _current[1] - x
                    _newG = _G[_current[1]][_current[0]] + round(sqrt(_diff_temp_x * _diff_temp_x + _diff_temp_y * _diff_temp_y) , 1)

                    if _A[0] == -1:
                        _G[x][y] = _newG
            
                        # _newF = _G[x][y] + _H[x][y] + 2 
                        _newF = _G[x][y] + _H[x][y] + 2 + _O[x][y]
                        _new_node = [y, x, _G[x][y], _newF, len(_closed_nodes)]
                        _open_nodes.append(_new_node)
                        # print('x,y', x, y, _G[x][y], _H[x][y], _O[x][y], _newF)
                        continue

                    if _newG >= _G[x][y]:
                        continue

                    _G[x][y] = _newG
                    _newF = _newG + _H[x][y] + _O[x][y]
                    _open_nodes[_A[0]][2] = _newG
                    _open_nodes[_A[0]][3] = _newF 
                    _open_nodes[_A[0]][4] = len(_closed_nodes)
                    # print('hihihi')
            
        if _b_solved == True:
            _b_solved = False
            _pre_x = []
            _pre_y = []
            _solved_count += 1
            j = len(_closed_nodes)
            j -=1

            while j >= 0:
                x=_closed_nodes[j][0]
                y=_closed_nodes[j][1]
                j=_closed_nodes[j][4]
                j = j-1
                _pre_x.append(int(x))
                _pre_y.append(int(y))

            for i in reversed(_pre_x):
                _path_x.append(i)
            for j in reversed(_pre_y):
                _path_y.append(j)
            
            print("Astar_FINISH")
            return AstarResponse(_path_x, _path_y)
        
        

    # def fnc_load_map(self):
    #     _map_img = cv2.imread(self.s_dir_map_img + self.s_img_fname, cv2.IMREAD_COLOR)
    #     _gray = cv2.cvtColor(_map_img, cv2.COLOR_BGR2GRAY)
    #     _ret, _dst = cv2.threshold(_gray, 200, 255, cv2.THRESH_BINARY)
    #     _re_dst = cv2.resize(_dst,dsize=(self.i_resize_x, self.i_resize_y), interpolation=cv2.INTER_AREA)
        
    #     _width, _height = _re_dst.shape[:2]
    #     _Map = np.zeros((_height, _width))

    #     _obs_pos = self.fnc_obstacle_pos(_re_dst)

    #     for i in range(len(_obs_pos[0])):
    #         _Map[_obs_pos[0][i]][_obs_pos[1][i]] = float('inf')

    #     if self.b_map_plot == True:
    #         plt.imshow(_Map)
    #         plt.show()

    #     return _Map
    #     ##########################
    #     # view TEST
    
    # def fnc_obstacle_pos(self, _img_data):
    #     _obs_position = []
    #     _obs_position.append([])
    #     _obs_position.append([])

    #     for i in range(len(_img_data)):
    #         for j in range(len(_img_data[0])):
    #             if(_img_data[i,j] == 0):
    #                 _obs_position[0].append(i)
    #                 _obs_position[1].append(j)

    #     # plt.scatter(_obs_position[0], _obs_position[1])
    #     # plt.show()
    #     self.temp_map_x = _obs_position[1]
    #     self.temp_map_y = _obs_position[0]

    #     return _obs_position

def main(args):
    rospy.init_node('gpp_astar', anonymous = True)
    start_am = h_A_star()

    rospy.spin()

if __name__=='__main__':
    main(sys.argv)
