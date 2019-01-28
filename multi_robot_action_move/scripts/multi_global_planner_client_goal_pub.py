#! /usr/bin/env python

import sys
import rospy
from multiprocessing import Process
from multi_robot_action_move.srv import multiplannergreedy,multiplannergreedyResponse
from multi_robot_action_move.msg import robot_pose, robot_pose_array, robo_goal
from real_robot_controller.msg import Job
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class multiglobalplannerclient:

    def __init__(self):
        self._jobs = []
        self._fname = sys.argv[3]
        self._agent_pos = None
        self.robo_status = []
        self.robo_goal = []
        self.goal_pub = [None] * (len(sys.argv) - 1)
        self._agent_pos = robot_pose_array()
        self.pos = 0
        self.pos_dup = False
        rospy.Subscriber(sys.argv[1] + "/agent_pos", robot_pose, self.agent_pos_Subscriber_callback)
        rospy.Subscriber(sys.argv[2] + "/agent_pos", robot_pose, self.agent_pos_Subscriber_callback)
        rospy.Subscriber("job",Job,self.job_subscriber)
        rospy.Subscriber(sys.argv[1] + "/move_base/result", MoveBaseActionResult, self.move_base_status)
        rospy.Subscriber(sys.argv[2] + "/move_base/result", MoveBaseActionResult, self.move_base_status)
        self.goal_pub[0] = rospy.Publisher(sys.argv[1] + "/move_base_simple/goal",PoseStamped,queue_size=1)
        self.goal_pub[1] = rospy.Publisher(sys.argv[2] + "/move_base_simple/goal",PoseStamped,queue_size=1)

    def agent_pos_Subscriber_callback(self,msg):
        _start = robot_pose()
        _start = msg
        if len(self._agent_pos.robot_name_pose) < 1:
            self._agent_pos.robot_name_pose.append(_start)
        else:
            for pose in self._agent_pos.robot_name_pose:
                if ((round(_start.robot_pose.pose.position.x,1) == round(pose.robot_pose.pose.position.x,1)) and (round(_start.robot_pose.pose.position.y,1) == round(pose.robot_pose.pose.position.y,1))):
                    self.pos_dup = True
                    break
            if not self.pos_dup:
                self._agent_pos.robot_name_pose.append(_start)


            else:
                rospy.logwarn("The Robot start position is already appended to agent_pos %s",_start)
                self.pos_dup = False
        if len(self._agent_pos.robot_name_pose) == 2: # give the no. w.r.t the robots
            self.robo_status = []
            self.multi_planner_client(self._agent_pos)
            self._agent_pos = robot_pose_array()
            self.pos = 0


    def multi_planner_client(self,msg):
        self._agent_pos = msg
        self.robos = len(self._agent_pos.robot_name_pose)
        if len(self._jobs) > 0:
            rospy.wait_for_service('multi_planner_greedy')

            try:
                multi_planner_greedy = rospy.ServiceProxy('multi_planner_greedy',multiplannergreedy)
                resp1 = multi_planner_greedy(self._agent_pos,self._jobs,self._fname)
                self._jobs[:] = []
                self.paths_jobs = resp1.gui_path_array.path_array
                self.robo_count = len(resp1.gui_path_array.agent_job)
                self.robo_jobs = resp1.gui_path_array.agent_job
                self.job_size = resp1.gui_path_array.max_job_size
                rospy.set_param('plan', True)
                self.move_base_goal()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        else:
            rospy.loginfo("There are no jobs in the queue")

    def move_base_status(self,msg):
        result = msg
        status = msg.status.status
        goal_id = msg.status.goal_id.id
        print("Status!!!!!!!")
        print(status)
        # print(msg.header.seq)
        print(goal_id)
        if self.agents_name_nojobs == []:
            if status == 3:
                self.robo_status.append(True)
               # print("robo_status_append")
               # print(self.robo_status)
            else:
                self.robo_status.append(False)
                print(result, "failed to reach the goal")
                print(result.status.status)
        else:
            for i in range(len(self.agents_name_nojobs)):
                if self.agents_name_nojobs[i] not in goal_id:
                    if status == 3:
                        self.robo_status.append(True)
                    else:
                        self.robo_status.append(False)
                        print(result, "failed to reach the goal")
                        print(result.status.status)
                    break


    def move_base_goal(self):
        agents = self.robo_count
        agents_jobs = []
        for i in range(len(self.robo_jobs)):
            agents_jobs.append(list(self.robo_jobs[i].agent_robo_job))
        print(agents_jobs)
        _start = True
        _jobs = True
        _break = False
        new_job = [None] * agents
        jobs_count_array = [0] * agents
        goals_count_array = [0] * agents
        self.agents_name_nojobs = []
        agents_nojobs = []
        agents_nostatus = []
        while _jobs:
            agents_count = 0
            goal_array = [None] * agents
            agent_jobs_done = []
            while(agents_count < agents):
                while(jobs_count_array[agents_count] < len(self.paths_jobs[agents_count].agent_paths)):
                    while(goals_count_array[agents_count] < len(self.paths_jobs[agents_count].agent_paths[jobs_count_array[agents_count]].poses)):
                        goal_pub = robo_goal()
                        goal_pub.robot_name = self.paths_jobs[agents_count].robot_name
                        goal_pub.robot_goal.header.frame_id = self.paths_jobs[agents_count].agent_paths[jobs_count_array[agents_count]].poses[goals_count_array[agents_count]].header.frame_id
                        goal_pub.robot_goal.pose = self.paths_jobs[agents_count].agent_paths[jobs_count_array[agents_count]].poses[goals_count_array[agents_count]].pose
                        goal_array[agents_count] = goal_pub
                        break
                    if (goals_count_array[agents_count] < len(self.paths_jobs[agents_count].agent_paths[jobs_count_array[agents_count]].poses)):
                        # print("breaked !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        goals_count_array[agents_count] = goals_count_array[agents_count] + 1
                        break
                    else:
                        # print("OOOOOOps")
                        new_job[agents_count] = True
                        jobs_count_array[agents_count] = jobs_count_array[agents_count] + 1
                        goals_count_array[agents_count] = 0
                        # print("jobs_count_array")
                        # print(jobs_count_array[agents_count])
                        if jobs_count_array[agents_count]%2 == 0:
                            del agents_jobs[agents_count][0]
                        if len(agents_jobs[agents_count]) == 0:
                            self.agents_name_nojobs.append(self.paths_jobs[agents_count].robot_name)
                            agents_nojobs.append(agents_count)
                            agents_nostatus.append(agents_count)
                            # print("agents_nojobs")
                            # print(agents_nojobs)
                            # print("agents_nostatus")
                            # print(agents_nostatus)
                            break
                        else:
                            # print("continue")
                            continue

                agents_count = agents_count + 1
                while (agents_count in agents_nojobs):
                    agents_count = agents_count + 1

            if _start == True:
                # print("if")
                self.goal_publisher(goal_array)
                _start = False
            else:
                # print("else")
                while True:
                    if len(self.robo_status) == (agents - len(agents_nostatus)):
                        # print("self.robo_status")
                        # print(self.robo_status)
                        if all(self.robo_status):
                            self.goal_publisher(goal_array)
                            self.robo_status = []
                            break
                        else:
                            print("some of the robots failed to reach the goal, So holding all the goals")
                            _break = True
                            break
                if _break:
                    _jobs = False
                    break

            for jobs in agents_jobs:
                # print(jobs)
                if not jobs:
                    agent_jobs_done.append(True)
            if len(agent_jobs_done) == len(agents_jobs):
                rospy.loginfo("jobs done by all agents")
                self.agents_name_nojobs = []
                _jobs = False
                rospy.set_param('plan', False)

    def goal_publisher(self,msg):
        for i in range(len(msg)):
            if msg[i] is not None:
                # print("goal")
                # print(i)
                j = 0
                while True:
                    if msg[i].robot_name == sys.argv[j + 1]:
                        while True:
                            if self.goal_pub[j].get_num_connections() > 0 :
                                self.goal_pub[j].publish(msg[i].robot_goal)
                                break
                        break
                    else:
                        j = j + 1
                        continue

    def job_subscriber(self,msg):
        _job = msg
        job = [_job.job_start.pose.position.x, _job.job_start.pose.position.y, _job.job_goal.pose.position.x, _job.job_goal.pose.position.y, _job.job_time]
        print('job: ',job)
        self._jobs.extend(job)
        print(self._jobs)

if __name__ == "__main__":
    rospy.init_node('multi_global_planner_client')
    A = multiglobalplannerclient()
    rospy.spin()



