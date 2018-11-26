#! /usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

class robot_id:
    def __init__(self):
        print(sys.argv)
        rospy.Subscriber("rob_id", Int16, self.publisher_id)
        self.id_pub_1 = rospy.Publisher(sys.argv[1] + "/robot_id", String, queue_size=1)
        self.id_pub_2 = rospy.Publisher(sys.argv[2] + "/robot_id", String, queue_size=1)
        #self.id_pub_1 = rospy.Publisher('/turtlebot1/robot_id', String, queue_size=1) # turtlesim
        #self.id_pub_2 = rospy.Publisher('/turtlebot2/robot_id', String, queue_size=1)
        #self.id_pub_1 = rospy.Publisher('/robot1/robot_id', String, queue_size=1) # gazebo
        #self.id_pub_2 = rospy.Publisher('/robot2/robot_id', String, queue_size=1)
        #self.id_pub_1 = rospy.Publisher('/robot_0/robot_id', String, queue_size=1) # stage_ros
        #self.id_pub_2 = rospy.Publisher('/robot_1/robot_id', String, queue_size=1)
        self._robot_id = "true"
    def publisher_id(self,msg):
        _rob_id = Int16()
        _rob_id = msg.data
        if _rob_id == 1:
            while True:
                if self.id_pub_1.get_num_connections() > 0 :
                    self.id_pub_1.publish(self._robot_id)
                    break
        if _rob_id == 2:
            while True:
                if self.id_pub_2.get_num_connections() > 0 :
                    self.id_pub_2.publish(self._robot_id)
                    break

if __name__ == '__main__':
    rospy.init_node('id_publisher', anonymous=True)
    A = robot_id()
    rospy.spin()
