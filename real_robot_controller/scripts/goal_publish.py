#! /usr/bin/env python
import copy
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from real_robot_controller.msg import Path_array_agentjob
from nav_msgs.msg import Path
import tf
import sys

class pose_goal_plan_publisher:

    def __init__(self):
        self.global_path = None
        self.start_published_ = False
        self.sub_times = 0
        self._agent = None
        self._agent_dup = None
        self.agent_assigned = False
        rospy.Subscriber("robot_id", String,self.robot_pose_goal_plan_pub)
        rospy.Subscriber("/gui_global_path",Path_array_agentjob,self.gui_global_path_subscriber)
        self._start_pub = rospy.Publisher("agent_pos",PoseStamped,queue_size=1)
        self._plan_pub = rospy.Publisher("robot_plan",Path,queue_size=1)
        self._goal_pub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=1)

    def gui_global_path_subscriber(self,msg):
        self.global_path = Path_array_agentjob()
        self.global_path = msg
        self.sub_times = 1

    def robot_pose_goal_plan_pub(self,msg):
        robot_id = String()
        robot_id = msg.data
        self.agent_pose = PoseStamped()
        listener_ = tf.TransformListener()
        now = rospy.Time.now()
        listener_.waitForTransform('/map',sys.argv[1] + "/base_link", rospy.Time(), rospy.Duration(1.0))
        robot_pose_position,robot_pose_quaternion = listener_.lookupTransform('/map',sys.argv[1] + "/base_link", rospy.Time())
        self.agent_pose.header.frame_id = "map"
        self.agent_pose.pose.position.x = robot_pose_position[0]
        self.agent_pose.pose.position.y = robot_pose_position[1]
        self.agent_pose.pose.position.z = robot_pose_position[2]
        self.agent_pose.pose.orientation.x = robot_pose_quaternion[0]
        self.agent_pose.pose.orientation.y = robot_pose_quaternion[1]
        self.agent_pose.pose.orientation.z = robot_pose_quaternion[2]
        self.agent_pose.pose.orientation.w = robot_pose_quaternion[3]
        print("agent_pos")
        print(self.agent_pose)

        if (self._agent is not None):
            for i in range(len(self.global_path.agent_job)): # no. defines no. of robots
                for j in range(len(self.global_path.path_array[i].agent_paths)):
                    self.global_path.path_array[i].agent_paths[j].header.seq = 0 # path.path_array[0] the index would change depending on the robot or consider all agents
            for j in range(len(self._agent.agent_paths)):
                self._agent.agent_paths[j].header.seq = 0
        if (robot_id == 'true' and self.sub_times == 0):
            while True:
                if (self._start_pub.get_num_connections() > 0):
                    self._start_pub.publish(self.agent_pose)
                    print(sys.argv[1], "start postion is published")
                    break
        while (self.sub_times > 0):
            if self._agent in self.global_path.path_array:
                self.agent_assigned = True
            if (self.agent_assigned):
                for path in self._agent_dup.agent_paths:
                    if (round(self.agent_pose.pose.position.x,0) == round(path.poses[0].pose.position.x,0) and round(self.agent_pose.pose.position.y,0) == round(path.poses[0].pose.position.y,0)):
                        if (path.poses[0].pose.position.x == path.poses[-1].pose.position.x) and (path.poses[0].pose.position.y == path.poses[-1].pose.position.y):
                            print("All assigned jobs for this robot are completed")
                            self.sub_times = 0
                            break
                        else:
                            goal = PoseStamped()
                            goal.header.frame_id = "map"
                            goal.pose.position.x = path.poses[-1].pose.position.x
                            goal.pose.position.y = path.poses[-1].pose.position.y
                            goal.pose.orientation.w = 1
                            while True:
                                if (self._goal_pub.get_num_connections() > 0):
                                    self._goal_pub.publish(goal)
                                    break

                            while True:
                                if (self._plan_pub.get_num_connections() > 0):
                                    self._agent_dup.agent_paths.pop(self._agent_dup.agent_paths.index(path))
                                    self._plan_pub.publish(path)
                                    if (len(self._agent_dup.agent_paths) < 1):
                                        self.agent_assigned = False
                                        print("jobs done")
                                        self.agent_assigned = False
                                        self.sub_times = 0
                                        self._agent = None
                                        self._agent_dup = None

                                    break
                        break

                break

            else:
                for agent in self.global_path.path_array:
                    for path in agent.agent_paths:
                        if round(self.agent_pose.pose.position.x,0) == round(path.poses[0].pose.position.x,0) and round(self.agent_pose.pose.position.y,0) == round(path.poses[0].pose.position.y,0):
                            if (path.poses[0].pose.position.x == path.poses[-1].pose.position.x) and (path.poses[0].pose.position.y == path.poses[-1].pose.position.y):
                                print("All assigned jobs for this robot are completed")
                                self.sub_times = 0
                                break

                            else:
                                self._agent = copy.deepcopy(agent)
                                self._agent_dup = copy.deepcopy(agent)
                                goal = PoseStamped()
                                goal.header.frame_id = "map"
                                goal.pose.position.x = path.poses[-1].pose.position.x
                                goal.pose.position.y = path.poses[-1].pose.position.y
                                goal.pose.orientation.w = 1
                                while True:
                                    if (self._goal_pub.get_num_connections() > 0):
                                        self._goal_pub.publish(goal)
                                        break

                                while True:
                                    if (self._plan_pub.get_num_connections() > 0):
                                        self._agent_dup.agent_paths.pop(self._agent_dup.agent_paths.index(path))
                                        self._plan_pub.publish(path)
                                        break
                            break
                    else:
                        continue
                    break
                break

if __name__ == "__main__":
    rospy.init_node('multi_goal_publisher', anonymous=True)
    A = pose_goal_plan_publisher()
    rospy.spin()




