#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
from std_msgs.msg import String, Int16

def talker():
    
    pub = rospy.Publisher('/steer', Int16, queue_size=10)
    rospy.init_node('steer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0
    
    while not rospy.is_shutdown():
        steer = 0
        while True :
            if count%2 == 0:
                steer +=1
                time.sleep(0.05)
                if steer == 2000:
                    count +=1
            elif count%2 == 1:
                steer -= 1
                time.sleep(0.05)
                if steer == -2000:
                    count +=1
        
            pub.publish(steer)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass