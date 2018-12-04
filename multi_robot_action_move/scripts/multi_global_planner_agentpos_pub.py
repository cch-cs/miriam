#! /usr/bin/env python

import sys
import numpy as np
import rospy
from multi_robot_action_move.msg import robot_pose, robot_pose_array

class agents_pos_publisher:
    def __init__(self):
        self._agent_pos = robot_pose_array()
        self.i = 0
        self.pos_dup = False
        _start_sub_1 = rospy.Subscriber(sys.argv[1] + "/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback)
        _start_sub_2 = rospy.Subscriber(sys.argv[2] + "/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback)
        self._agents_pos_pub = rospy.Publisher("agents_pos",robot_pose_array,queue_size=1)

    def agent_pos_Subscriber_callback(self,msg):
        _start = robot_pose()
        _start = msg
        if len(self._agent_pos.robot_pose_array) < 1:
            self._agent_pos.robot_pose_array.append(_start)
        else:
            for pose in self._agent_pos.robot_pose_array:
                if ((round(_start.robot_pose.pose.position.x,1) == round(pose.robot_pose.pose.position.x,1)) and (round(_start.robot_pose.pose.position.y,1) == round(pose.robot_pose.pose.position.y,1))):
                    self.pos_dup = True
                    break
            if not self.pos_dup:
                self._agent_pos.robot_pose_array.append(_start)


            else:
                rospy.logwarn("The Robot start position is already appended to agent_pos %s",_start)
                self.pos_dup = False
        print(self._agent_pos)
        while len(self._agent_pos.robot_pose_array) == 2: # give the no. w.r.t the robots
            if (self._agents_pos_pub.get_num_connections() > 0):
                self._agents_pos_pub.publish(self._agent_pos)
                self._agent_pos = robot_pose_array()
                self.i = 0
                print("self._agent_pos_none")
                print(self._agent_pos)
                break

if __name__ == "__main__":
    rospy.init_node('agent_pos_Subscriber_Publisher')
    A=agents_pos_publisher()
    rospy.spin()




