#! /usr/bin/env python

import sys
import sys
import rospy
from std_msgs.msg import Bool
from multi_robot_action_move.msg import rob_id

from multiprocessing import Process

class robot_id:
    def __init__(self):
        rospy.Subscriber("rob_id", rob_id, self.publisher_id)
        self.id_pub_1 = rospy.Publisher(sys.argv[1] + "/robot_id", Bool, queue_size=1)
        self.id_pub_2 = rospy.Publisher(sys.argv[2] + "/robot_id", Bool, queue_size=1)
        self._robot_id = Bool()
        self._robot_id.data = True
    def publisher_id(self,msg):
        _rob_id = msg.rob_id
        self.rob_id_pub(_rob_id)

    def rob_id_pub(self,msg):
        if msg[0] == True:
            while True:
                if self.id_pub_1.get_num_connections() > 0 :
                    self.id_pub_1.publish(self._robot_id)
                    print("rob1")
                    break
        if msg[1] == True:
            while True:
                if self.id_pub_2.get_num_connections() > 0 :
                    self.id_pub_2.publish(self._robot_id)
                    print("rob2")
                    break

if __name__ == '__main__':
    rospy.init_node('id_publisher', anonymous=True)
    A = robot_id()
    rospy.spin()

