#! /usr/bin/env python
import copy
import rospy
from std_msgs.msg import String
from real_robot_controller.msg import Path_array_agentjob
from multi_robot_action_move.msg import robot_pose
from nav_msgs.msg import Path
import tf
import sys

class pose_publisher:

    def __init__(self):
        rospy.Subscriber("robot_id", String,self.robot_pose_goal_plan_pub)
        self._start_pub = rospy.Publisher("agent_pos",PoseStamped,queue_size=1)

    def robot_pose_goal_plan_pub(self,msg):
        robot_id = String()
        robot_id = msg.data
        self.agent_pose = robot_pose()
        listener_ = tf.TransformListener()
        listener_.waitForTransform('/map',sys.argv[1] + "/base_link",rospy.Time(),rospy.Duration(1.0))
        robot_pose_position,robot_pose_quaternion = listener_.lookupTransform('/map',sys.argv[1] + "/base_link",rospy.Time())
        self.agent_pose.robot_name.data = sys.argv[1]
        self.agent_pose.robot_pose.header.frame_id = "map"
        self.agent_pose.robot_pose.pose.position.x = robot_pose_position[0]
        self.agent_pose.robot_pose.pose.position.y = robot_pose_position[1]
        self.agent_pose.robot_pose.pose.position.z = robot_pose_position[2]
        self.agent_pose.robot_pose.pose.orientation.x = robot_pose_quaternion[0]
        self.agent_pose.robot_pose.pose.orientation.y = robot_pose_quaternion[1]
        self.agent_pose.robot_pose.pose.orientation.z = robot_pose_quaternion[2]
        self.agent_pose.robot_pose.pose.orientation.w = robot_pose_quaternion[3]
        print("agent_pos")
        print(self.agent_pose)
        plan_made = rosparam.get_param('plan', False)
        if (robot_id == 'true' and plan_made == False):
            while True:
                if (self._start_pub.get_num_connections() > 0):
                    self._start_pub.publish(self.agent_pose)
                    print(sys.argv[1], "start postion is published")
                    break

if __name__ == "__main__":
    rospy.init_node('multi_goal_publisher', anonymous=True)
    A = pose_publisher()
    rospy.spin()




