#! /usr/bin/env python

import sys

import rospy
from multiprocessing import Process
from multi_robot_action_move.srv import multiplannergreedy,multiplannergreedyResponse
from multi_robot_action_move.msg import robot_pose_array, robo_goal
from real_robot_controller.msg import Job
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped

class multiglobalplannerclient:

    def __init__(self):
        self._jobs = []
        self._fname = 'MessLabor_lab_sample.pkl'
        self._agent_pos = None
        self.robo_status = []
        self.robo_goal = []
        self.goal_pub = []
        rospy.Subscriber("agents_pos", robot_pose_array, self.multi_planner_client)
        rospy.Subscriber("job",Job,self.job_subscriber)
        for i in (len(sys.argv) - 1)
            rospy.Subscriber(sys.argv[i + 1] + "/move_base/result",MoveBaseActionResult,self.move_base_status, i + 1)
            self.goal_pub.append(rospy.Publisher(sys.argv[i + 1] + "/move_base_simple/goal",PoseStamped,queue_size=1))


    def multi_planner_client(self,msg):
        self._agent_pos = msg.robot_pose_array
        self.robos = len(self._agent_pos)
        print(self._agent_pos)
        rosparam.set_param('plan',True)
       # while True:
        if len(self._jobs) > 0:
                #print(self._jobs)
            rospy.wait_for_service('multi_planner_greedy')

            try:
                multi_planner_greedy = rospy.ServiceProxy('multi_planner_greedy',multiplannergreedy)
                resp1 = multi_planner_greedy(self._agent_pos,self._jobs,self._fname)
                self._jobs[:] = []
                self.paths_jobs = resp1.gui_path_array.path_array
                self.robo_count = len(resp1.gui_path_array.agent_job)
                self.robo_jobs = resp1.gui_path_array.agent_job
                self.job_size = resp1.gui_path_array.max_job_size
                rosparam.set_param('plan', True)
                print(self.paths_jobs)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        else:
            rospy.loginfo("There are no jobs in the queue")

    def move_base_goal(self):
        agents = self.robo_count
        agents_jobs = self.robo_jobs
        jobs = True
        while jobs:
            _start = 0
            agents_count = 0
            jobs_count = 0
            goals_count = 0
            agents_nojobs = []
            processes = [None] * agents
            jobs_count_array = [agents]
            goals_count_array = [agents]
            goal_array = [None] * agents
            while(agents_count < agents):
                goal_pub = robo_goal()
                goal_pub.robot_name = self.paths_jobs[agents_count].robot_name
                while(jobs_count[agents_count] < len(self.paths_jobs[agents_count])):
                    while(goals_count[agents_count] < len(self.paths_jobs[agents_count][jobs_count[agents_count]])):
                        goal_pub = robot_goal()
                        goal_pub.robot_goal.header.frame_id = self.paths_jobs[agents_count][jobs_count[agents_count]][goals_count[agents_count]].header.frame_id
                        goal_pub.robot_goal.pose = self.paths_jobs[agents_count][jobs_count[agents_count]][goals_count[agents_count]].pose
                        goal_array[agents_count] = goal_pub
                        goals_count[agents_count] = goals_count[agents_count] + 1
                        break
                    if (goals_count[agents_count] < len(self.paths_jobs[agents_count][jobs_count[agents_count]])):
                        break
                    else:
                        jobs_count[agents_count] = jobs_count[agents_count] + 1
                        goals_count[agents_count] = 0
                        agents_jobs[agents_count].pop(0)
                        if len(agents_jobs[agents_count]) == 0:
                            agents_nojobs.append(agents_count)
                            break
                        continue

                agents_count = agents_count + 1
                while (agents_count in agents_nojobs):
                    agents_count = agents_count + 1
            _start = _start + 1
            for i in len(processes):
                if _start == 1:
                    processes[i] = Process(target = self.goal_publisher, args= goal_array[i])
                    processes[i].start
                else:
                    robo_status_array = []
                    robo_result_array = []
                    for j in (len(sys.argv) - 1):
                        robo_result_array[j] = rospy.wait_for_message(sys.argv[j + 1] + "/move_base/result",MoveBaseActionResult)
                        if robo_result_array[j].status.status == 3:
                            robo_status_array[j] = True
                        else:
                            ros_info(robo_result_array[j].status.goal_id.id, "failed to reach the goal")
                            break
                    if all(robo_status_array):
                        processes[i] = Process(target = self.goal_publisher, args= goal_array[i])
                        processes[i].start




    def move_base_status(self,msg,args):
        robot_status = msg
        if robot_status.status.status == 3:

            if sys.argv[1] in robot_status.status.goal_id.id:
            if sys.argv[2] in robot_status.status.goal_id.id:
        else:
            ros_info("The robot failed to go the goal")


    def goal_publisher(self,msg):
        for i in (len(sys.argv) - 1)
            if msg.robot_name == sys.argv[i + 1]:
                while True:
                    if self.goal_pub[i].get_num_connections() > 0 :
                        self.goal_pub[i].publish(msg.robot_goal)
                        break
                break

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



