#! /usr/bin/env python

import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from real_robot_controller.msg import Plan

class agents_pos_publisher:
    def __init__(self):
        self._agent_pos = Plan()
        self.i = 0
        self.pos_dup = False
        _start_sub_1 = rospy.Subscriber(sys.argv[1] + "/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback)
        _start_sub_2 = rospy.Subscriber(sys.argv[2] + "/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback)
        #_start_sub_1 = rospy.Subscriber("/turtlebot1/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback) # turtlesim
        #_start_sub_2 = rospy.Subscriber("/turtlebot2/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback)
        #_start_sub_1 = rospy.Subscriber("/robot1/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback) # gazebo
        #_start_sub_2 = rospy.Subscriber("/robot2/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback)
        #_start_sub_1 = rospy.Subscriber("/robot_0/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback) # stage_ros
        #_start_sub_2 = rospy.Subscriber("/robot_1/agent_pos", PoseStamped, self.agent_pos_Subscriber_callback)
        self._agents_pos_pub = rospy.Publisher("agents_pos",Plan,queue_size=1)

    def agent_pos_Subscriber_callback(self,msg):
        _start = PoseStamped()
        _start = msg
        if len(self._agent_pos.plan) < 1:
            self._agent_pos.plan.append(_start)
        else:
            for pose in self._agent_pos.plan:
#                if _start.pose != pose.pose :
#                    self.i = self.i + 1
#            if len(self._agent_pos.plan) == self.i:
#                self._agent_pos.plan.append(_start)
                if ((round(_start.pose.position.x,1) == round(pose.pose.position.x,1)) and (round(_start.pose.position.y,1) == round(pose.pose.position.y,1))):
                    self.pos_dup = True
                    break
            if not self.pos_dup:
                self._agent_pos.plan.append(_start)


            else:
                rospy.logwarn("The Robot start position is already appended to agent_pos %s",_start)
                self.pos_dup = False
        print(self._agent_pos)
        while len(self._agent_pos.plan) == 2: # give the no. w.r.t the robots
            if (self._agents_pos_pub.get_num_connections() > 0):
                self._agents_pos_pub.publish(self._agent_pos)
                self._agent_pos = Plan()
                self.i = 0
                print("self._agent_pos_none")
                print(self._agent_pos)
                #_start_sub.unregister()<
                break
                #rospy.signal_shutdown("got all agent_pos")

    def agent_pos_Subscriber_callback_2(self,msg):
        _start = PoseStamped()
        _start = msg
        print(self._agent_pos)
        if len(self._agent_pos.plan) < 1:
            self._agent_pos.plan.append(_start)
        else:
            for pose in self._agent_pos.plan:
                if _start.pose != pose.pose :
                    self.i = self.i + 1
            if len(self._agent_pos.plan) == self.i:
                self._agent_pos.plan.append(_start)
            else:
                rospy.logwarn("The Robot start position is already appended to agent_pos %s",_start)
        print(self._agent_pos)
        while len(self._agent_pos.plan) == 2: # give the no. w.r.t the robots
            if (self._agents_pos_pub.get_num_connections() > 0):
                self._agents_pos_pub.publish(self._agent_pos)
                self._agent_pos = Plan()
                self.i = 0
                print("self._agent_pos_none")
                print(self._agent_pos)
                #_start_sub.unregister()
                break
                #rospy.signal_shutdown("got all agent_pos")

if __name__ == "__main__":
    rospy.init_node('agent_pos_Subscriber_Publisher')
    A=agents_pos_publisher()
    rospy.spin()




