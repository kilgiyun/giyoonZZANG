#!/usr/bin/env python3
import rospy
import serial
import time
import numpy
from geometry_msgs.msg import Twist
from autonomy_msgs.msg import ERP42_mode
from std_msgs.msg import Float64
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus

bytesize=serial.EIGHTBITS,
parity = serial.PARITY_NONE,
stopbits = serial.STOPBITS_ONE,
xonxoff = False
ser = serial.Serial()
ser.baudrate = 115200
ser.port = '/dev/ttyUSB0'
ser.open()

#now cmd_vel.linear.x = 1


class Car:

    def __init__(self, speed=0, steer1=0, steer0=0, vel=0, direction=0):
        #Sub
        rospy.init_node("cmd_vel_to_serial", anonymous=True)
        rospy.Subscriber('/ctrl_cmd', CtrlCmd ,self.cmd_vel_callback)
            
        #Pub
        self.vel_pub=rospy.Publisher('/linear_vel',EgoVehicleStatus, queue_size=1)
        self.rate=rospy.Rate(10)
            
        if ser.isOpen():
            print(ser.name + ' is connected...')
            print("Serial details parameters: ", ser)
        self.speed = speed
        self.steer1 = steer1
        self.steer0 = steer0
        self.vel = vel
        self.dir = direction
        self.erp_command_mode = 1
        
        self.linear_x=EgoVehicleStatus()
        
        # self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        # self.erp_mode_sub = rospy.Subscriber("/vehicle_mode", ERP42_mode, self.erp_mode_callback)
        self.rate = rospy.Rate(1)
        self.send_serial()

    def cmd_vel_callback(self, data:CtrlCmd):
        
        # self.vel = int(data.accel * 3.6) # m/s to kph
        self.vel = 1 * 3.6 # m/s to kph
        # self.dir = int(round(data.angular.z * 57.3 * 30)) # radian to degree to dir
        self.dir = int(round(data.steering * 57.3 * 30))
        # print("vel / dir {} {}".format(self.vel, self.dir))
        
    def acceleration(self):
        y = self.vel * 10
        # print('accel : {0} |vel :{1}'.format(self.vel,y))
        if 200 < y:
            # self.cmd_acc = (self.cmd_vel - self.vel)
            self.speed = 0
            # print("Speed value error!")
        else:
            sp = (int(y))
            self.speed = sp

    def direction(self):
        direction = self.dir
        # print(direction)
        bits = 16
        if dir == 0:
            self.steer1 = 0
            self.steer0 = 0
            degree = direction / 71
            print("Direction :{} Deg".format(degree))
        elif 0 < direction <= 255:
            st0 = 0
            a = hex((0 << bits) + direction)
            a = a[2:]
            a = a.zfill(len(a) + len(a) % 2)
            b = ''.join(a[i:i + 1] for i in range(0, len(a), 1))
            st1 = (int(b, 16))
            degree = (st0 * 256 + st1) / 71
            self.steer1 = st1
            self.steer0 = st0
            print("Steering :{} Right Deg".format(degree))
        elif 256 <= direction <= 2000:
            a = hex((0 << bits) + direction)
            nn = hex((0 << bits) + direction)
            a = a[3:]
            nn = nn[1:3]
            a = a.zfill(len(a) + len(a) % 2)
            b = ''.join(a[i:i + 3] for i in range(0, len(a), 3))
            st00 = ''.join(nn[i:i + 1] for i in range(1, len(nn), 3))
            st1 = (int(b, 16))
            st0 = (int(st00))
            degree = (st0 * 256 + st1) / 71
            self.steer1 = st1
            self.steer0 = st0
            print("Steering :{} Right Deg".format(degree))
        elif 2000 < direction or direction < -2000:
            self.steer1 = 0
            self.steer0 = 0
            print("Steer value error!")
        elif -2000 <= direction < 0:
            def tohex(val, nbits):
                return hex((val + (1 << nbits)) % (1 << nbits))
            minus = (tohex(direction, 16))
            # steering 1
            minus1 = minus[4:]
            st1 = (int(minus1, 16))
            # steering 0
            minus0 = minus[2:4]
            st0 = (int(minus0, 16))
            self.steer1 = st1
            self.steer0 = st0
            degree = direction / 71
            # print("Steering :{} Left Deg".format(degree))

    def send_serial(self):
        # ERP42 Packet 
        # S    T    X    Auto/Manual E-Stop    Gear Speed0 Speed1 Steer0 Steer1 Brake Alive ETX0 ETX1
        # 0x53 0x54 0x58 0x00/0x01   0x00/0x01 0~2     0 ~ 200    -2000 ~ 2000  1~33  0~255 0x0D 0x0A
        byte = ([0x53, 0x54, 0x58, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0D, 0x0A])
        number = byte[0:14]
        data = 0x00
        while not rospy.is_shutdown():
            self.direction()
            self.acceleration()
            number[11] = data
            number[8] = self.steer0
            number[9] = self.steer1
            number[6] = 0
            number[7] = self.speed    

            if(data < 255):
                data += 1
            else:
                data = 0            
            send = ser.write(serial.to_bytes(number))
            if ser.readable():
                read = ser.readline()
                if(read[0]==83 and len(read) == 18):
                    # self.vel = read[6]/10 # km/h
                    # print(self.vel)
                    self.linear_x.velocity.x=read[6]/10
                    self.vel_pub.publish(self.linear_x)
                    self.rate.sleep
                    
            # print("Send =", number)
            # print("Read =", read)

if __name__ == '__main__':
    try:
        print("cmd_vel_to_serial")
        my_car = Car()
    except rospy.ROSInterruptException:
        print("serial Close")
        ser.close()  