#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16
import time

class rob_id:
    def __init__(self):
        self.id_pub = rospy.Publisher('rob_id', Int16, queue_size=1)
    def rob_pub_id(self,msg):
        while True:
            if self.id_pub.get_num_connections() > 0 :
                self.id_pub.publish(msg)
                break

if __name__ == '__main__':
    rospy.init_node('rob_id_publisher', anonymous=True)
    A = rob_id()
    for rob in range(2): # No. indicates the no. of robots
        rob_id = Int16()
        rob_id.data = rob + 1
        A.rob_pub_id(rob_id)
        time.sleep(0.1)
