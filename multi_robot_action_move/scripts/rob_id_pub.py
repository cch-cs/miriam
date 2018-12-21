#! /usr/bin/env python

import rospy
from multi_robot_action_move.msg import rob_id
import time

class robo_id:
    def __init__(self):
        self.id_pub = rospy.Publisher('rob_id', rob_id, queue_size=1)
    def rob_pub_id(self,msg):
        while True:
            if self.id_pub.get_num_connections() > 0 :
                self.id_pub.publish(msg)
                break

if __name__ == '__main__':
    rospy.init_node('rob_id_publisher', anonymous=True)
    A = robo_id()
    rob_id = rob_id()
    for rob in range(2): # No. indicates the no. of robots
        rob_id.rob_id.append(True)
    A.rob_pub_id(rob_id)
