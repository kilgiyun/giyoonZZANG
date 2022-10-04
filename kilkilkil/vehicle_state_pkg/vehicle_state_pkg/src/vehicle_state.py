#!/usr/bin/env python3
##ref code from
##https://github.com/engcang/husky/tree/master/Python-Kinematic-Position-Control

import roslib
roslib.load_manifest('vehicle_state_pkg')
import sys
import rospy
from math import pow,atan2,sqrt,sin,cos
import numpy as np
import utm
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from squaternion import Quaternion
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from sensor_msgs.msg import Imu
from pyproj import Proj


class c_vehicle_state_():

    def __init__(self):
        super(c_vehicle_state_, self).__init__()

        self.pi = 3.141592654

        self.bRecvCheck_MoraiVehState = False
        self.bRecvCheck_MoraiImu = False
        self.bPubDone = False
        self.bFirst = True
        self.gps_init = True

        self.dataStamp = 0
        self.fMoraiVehHeanding = 0
        self.Clock = 0

        self.vehicle_state_out = Odometry()
        self.fMoraiVehState = Odometry()
        self.fMoraiVehImu = Imu()

        self.x_init = 0
        self.y_init = 0
        
        self.rate = rospy.Rate(40)

        self.child_frame_id = rospy.get_param("/vehicle_state_node/child_frame_id")
        self.vehicle_state_in_ = rospy.get_param("/vehicle_state_node/vehicle_state_in_topic")
        self.vehicle_imu_in_ = rospy.get_param("/vehicle_state_node/vehicle_imu_in_topic")
        self.vehicle_state_out_ = rospy.get_param("/vehicle_state_node/vehicle_state_out_topic")

        # self.sub_clock = rospy.Subscriber('/clock', Clock, self.callback_Clock)
        # self.sub_curpos_ugv = rospy.Subscriber(self.vehicle_state_in_, Odometry, self.odom_callback)
        # self.pub_curpos_ugv = rospy.Publisher(self.vehicle_state_out_, Twist, queue_size = 1)
        self.proj_UTM= Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.sub_moraiVehState = rospy.Subscriber(self.vehicle_state_in_, EgoVehicleStatus, self.fnc_MoraiVehState)
        self.sub_moraiVehImu = rospy.Subscriber(self.vehicle_imu_in_, Imu, self.fnc_MoraiVehImu)
        self.sub_moraiVehGPS = rospy.Subscriber('/gps', GPSMessage, self.fnc_MoraiVehGPS)
        self.pub_curpos_ugv = rospy.Publisher(self.vehicle_state_out_, Odometry, queue_size = 1)

        self.fnc_PubMoraiVehState()
    
    def callback_Clock(self, _data:Clock):
        self.Clock = _data.clock
        
    def fnc_MoraiVehGPS(self, _data: GPSMessage):
        xy_zone= self.proj_UTM(_data.longitude, _data.latitude)
        if self.gps_init:
            self.x_init = xy_zone[0]
            self.y_init = xy_zone[1]
            self.gps_init = False
            
        self.fMoraiVehState.pose.pose.position.x = xy_zone[0] - self.x_init
        self.fMoraiVehState.pose.pose.position.y = xy_zone[1] - self.y_init
        self.fMoraiVehState.pose.pose.position.z = 0

    def fnc_MoraiVehState(self, _data:EgoVehicleStatus):
        self.fMoraiVehImu.linear_acceleration.x = _data.acceleration.x
        self.fMoraiVehImu.linear_acceleration.y = _data.acceleration.y
        self.fMoraiVehImu.linear_acceleration.z = _data.acceleration.z
        
        self.fMoraiVehState.twist.twist.linear.x = _data.velocity.x
        self.fMoraiVehState.twist.twist.linear.y = _data.velocity.y
        self.fMoraiVehState.twist.twist.linear.z = _data.velocity.z
        
        # self.fMoraiVehState.pose.pose.position.x = _data.position.x
        # self.fMoraiVehState.pose.pose.position.y = _data.position.y
        # self.fMoraiVehState.pose.pose.position.z = _data.position.z

        self.fMoraiVehHeanding = _data.heading
        self.dataStamp = _data.header.stamp

        self.bRecvCheck_MoraiVehState = True

    def fnc_MoraiVehImu(self, _data:Imu):
        
        self.fMoraiVehState.twist.twist.angular.x = _data.angular_velocity.x
        self.fMoraiVehState.twist.twist.angular.y = _data.angular_velocity.y
        self.fMoraiVehState.twist.twist.angular.z = _data.angular_velocity.z
        
        self.fMoraiVehImu.linear_acceleration.x = _data.linear_acceleration.x
        self.fMoraiVehImu.linear_acceleration.y = _data.linear_acceleration.y
        self.fMoraiVehImu.linear_acceleration.z = _data.linear_acceleration.z
        
        self.fMoraiVehState.pose.pose.orientation.x = _data.orientation.x
        self.fMoraiVehState.pose.pose.orientation.y = _data.orientation.y
        self.fMoraiVehState.pose.pose.orientation.z = _data.orientation.z
        self.fMoraiVehState.pose.pose.orientation.w = _data.orientation.w

        self.bRecvCheck_MoraiImu = True

    def fnc_PubMoraiVehState(self):
        while not rospy.is_shutdown():
            if self.bRecvCheck_MoraiVehState and self.bRecvCheck_MoraiImu:

                self.vehicle_state_out.header.stamp = self.dataStamp

                _w = self.fMoraiVehState.pose.pose.orientation.w
                _x = self.fMoraiVehState.pose.pose.orientation.x
                _y = self.fMoraiVehState.pose.pose.orientation.y
                _z = self.fMoraiVehState.pose.pose.orientation.z

                _q = Quaternion(_w, _x, _y, _z)
                _e = _q.to_euler()

                _roll = _e[0]
                _pitch = _e[1]
                _yaw = _e[2]

                _n_dot = self.fMoraiVehState.twist.twist.linear.x
                _e_dot = self.fMoraiVehState.twist.twist.linear.y
                _d_dot = self.fMoraiVehState.twist.twist.linear.z

                _MoraiVehBodyVel_x, _MoraiVehBodyVel_y, _MoraiVehBodyVel_z  = self.fnc_calc_bodydot(_roll, _pitch, _yaw, _n_dot, _e_dot, _d_dot)

                self.vehicle_state_out.pose.pose.position.x = self.fMoraiVehState.pose.pose.position.x
                self.vehicle_state_out.pose.pose.position.y = self.fMoraiVehState.pose.pose.position.y
                self.vehicle_state_out.pose.pose.position.z = self.fMoraiVehState.pose.pose.position.z

                self.vehicle_state_out.twist.twist.linear.x = _MoraiVehBodyVel_x
                self.vehicle_state_out.twist.twist.linear.y = _MoraiVehBodyVel_y
                self.vehicle_state_out.twist.twist.linear.z = _MoraiVehBodyVel_z

                self.vehicle_state_out.twist.twist.angular.x = self.fMoraiVehState.twist.twist.angular.x
                self.vehicle_state_out.twist.twist.angular.y = self.fMoraiVehState.twist.twist.angular.y
                self.vehicle_state_out.twist.twist.angular.z = self.fMoraiVehState.twist.twist.angular.z

                self.vehicle_state_out.pose.pose.orientation.x = _roll
                self.vehicle_state_out.pose.pose.orientation.y = _pitch
                self.vehicle_state_out.pose.pose.orientation.z = _yaw
                self.vehicle_state_out.pose.pose.orientation.w = self.fMoraiVehHeanding

                self.pub_curpos_ugv.publish(self.vehicle_state_out)

            self.rate.sleep()

    
    def tf_callback(self, tf_data):

        if tf_data.transforms[0].child_frame_id == self.child_frame_id:

            self.cur_pos_vehicle = tf_data.transforms[0].transform.translation
            self.cur_rot_vehicle = tf_data.transforms[0].transform.rotation

            # self.cur_pos_vehicle.x = round(self.cur_pos_vehicle.x, 4)
            # self.cur_pos_vehicle.y = round(self.cur_pos_vehicle.y, 4)

            # self.cur_rot_list = [self.cur_rot_vehicle.x, self.cur_rot_vehicle.y, self.cur_rot_vehicle.z, self.cur_rot_vehicle.w] 

            _q = Quaternion(self.cur_rot_vehicle.w, self.cur_rot_vehicle.x, self.cur_rot_vehicle.y, self.cur_rot_vehicle.z)
            _e = _q.to_euler()

            roll = _e[0]
            pitch = _e[1]
            yaw = _e[2]

            self.vehicle_state_out.linear.x = self.cur_pos_vehicle.x
            self.vehicle_state_out.linear.y = self.cur_pos_vehicle.y
            self.vehicle_state_out.angular.z = yaw
            
            self.pub_curpos_ugv.publish(self.vehicle_state_out)

    def odom_callback(self, odom_data):

            self.vehicle_state_out.header.stamp = self.Clock
            self.cur_pos_vehicle = odom_data.pose.pose.position
            self.cur_rot_vehicle = odom_data.pose.pose.orientation
            self.cur_vel = odom_data.twist.twist.linear
            self.cur_rate_angular = odom_data.twist.twist.angular

            self.cur_pos_vehicle.x = round(self.cur_pos_vehicle.x, 6)
            self.cur_pos_vehicle.y = round(self.cur_pos_vehicle.y, 6)
            self.cur_pos_vehicle.z = round(self.cur_pos_vehicle.z, 6)

            _q = Quaternion(self.cur_rot_vehicle.w, self.cur_rot_vehicle.x, self.cur_rot_vehicle.y, self.cur_rot_vehicle.z)
            _e = _q.to_euler()

            roll = _e[0]
            pitch = _e[1]
            yaw = _e[2]

            _body_dot = self.fnc_calc_bodydot(roll, pitch, yaw,\
                                    self.cur_vel.x, self.cur_vel.y, self.cur_vel.z)

            self.vehicle_vx = _body_dot[0]
            self.vehicle_vy = _body_dot[1]
            self.vehicle_vz = _body_dot[2]

            theta = yaw - self.pi

            if abs(theta) > self.pi:
                if theta > 0:
                    theta = theta - 2 * self.pi
                else:
                    theta = theta + 2 * self.pi

            self.vehicle_state_out.pose.pose.position.x = self.cur_pos_vehicle.x
            self.vehicle_state_out.pose.pose.position.y = self.cur_pos_vehicle.y
            self.vehicle_state_out.pose.pose.position.z = self.cur_pos_vehicle.z
            self.vehicle_state_out.pose.pose.orientation.x = roll
            self.vehicle_state_out.pose.pose.orientation.y = pitch
            self.vehicle_state_out.pose.pose.orientation.z = yaw
            self.vehicle_state_out.pose.pose.orientation.w = -theta
            self.vehicle_state_out.twist.twist.linear.x = self.vehicle_vx
            self.vehicle_state_out.twist.twist.linear.y = self.vehicle_vy
            self.vehicle_state_out.twist.twist.linear.z = self.vehicle_vz
            self.vehicle_state_out.twist.twist.angular.x = self.cur_rate_angular.x
            self.vehicle_state_out.twist.twist.angular.y = self.cur_rate_angular.y
            self.vehicle_state_out.twist.twist.angular.z = self.cur_rate_angular.z

            
            self.pub_curpos_ugv.publish(self.vehicle_state_out)

    def fnc_calc_bodydot(self, _roll, _pitch, _yaw, _N_dot, _E_dot, _h_dot):

        try:
            R = np.matrix([[cos(_pitch)*cos(_yaw), -cos(_pitch)*sin(_yaw)+sin(_roll)*sin(_pitch)*cos(_yaw), sin(_roll)*sin(_yaw)+cos(_roll)*sin(_pitch)*cos(_yaw)],\
                [cos(_pitch)*sin(_yaw), cos(_roll)*cos(_yaw)+sin(_roll)*sin(_pitch)*sin(_yaw), -sin(_roll)*cos(_yaw)+cos(_roll)*sin(_pitch)*sin(_yaw)],\
                [sin(_pitch), -sin(_roll)*cos(_pitch), -cos(_roll)*cos(_pitch)]])

            R_ = np.linalg.inv(R)

            vx, vy, vz = R_ * np.matrix([[_N_dot],[_E_dot],[_h_dot]])

            pre_vx = np.array2string(vx)
            pre_vy = np.array2string(vy)
            pre_vz = np.array2string(vz)
            _vx_len = len(pre_vx)
            _vy_len = len(pre_vy)
            _vz_len = len(pre_vz)
            result_vx = float(pre_vx[2 : _vx_len - 2])
            result_vy = float(pre_vy[2 : _vy_len - 2])
            result_vz = float(pre_vz[2 : _vz_len - 2])

            return result_vx, result_vy, result_vz

        except Exception as e:
            print("calc_body_dot error"+e)
            _temp = 1
            pass

def main(args):
    rospy.init_node('vehicle_state_node', anonymous = True)

    start_vehicle = c_vehicle_state_()

    rospy.spin()

if __name__=='__main__':
    main(sys.argv)

