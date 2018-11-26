#! /usr/bin/env python

import numpy as np
import rospy
from multirobotglobalplanner.msg import Path_array,Path_array_agentjob
from multirobotglobalplanner.srv import multiplannerjobserver,multiplannerjobserverResponse
from multirobotglobalplanner.msg import Plan
from multirobotglobalplanner.msg import Job
from geometry_msgs.msg import PoseStamped

class multiglobalplannerclient:

    def __init__(self):
        self._jobs = []
        self._fname = 'MessLabor_lab_sample.pkl'
        self.pub_times = None
        self._agent_pos = None
        rospy.Subscriber("agents_pos", Plan, self.multi_planner_client)
        rospy.Subscriber("job",Job,self.job_subscriber)
        self._gui_path_array = rospy.Publisher("gui_global_path",Path_array_agentjob,queue_size=1)


    def multi_planner_client(self,msg):
        self._agent_pos = msg.plan
        print(self._agent_pos)
       # while True:
        if len(self._jobs) > 0:
                #print(self._jobs)
            rospy.wait_for_service('multi_planner_jobserver')

            try:
                multi_planner_jobserver = rospy.ServiceProxy('multi_planner_jobserver',multiplannerjobserver)
                resp1 = multi_planner_jobserver(self._agent_pos,self._jobs,self._fname)
                self._jobs[:] = []
                self.paths_jobs = resp1.gui_path_array
                self.job_size = self.paths_jobs.max_job_size
                print(self.paths_jobs)
                while True:
                    if (self._gui_path_array.get_num_connections() > 1):
                        self._gui_path_array.publish(self.paths_jobs)
                        break

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        else:
            rospy.loginfo("There are no jobs in the queue")

    def job_subscriber(self,msg):
        _job = Job()
        _job = msg

        job = [_job.job_start.pose.position.x, _job.job_start.pose.position.y, _job.job_goal.pose.position.x, _job.job_goal.pose.position.y, _job.job_time]
        print('job: ',job)
        self._jobs.extend(job)
        print(self._jobs)

if __name__ == "__main__":
    rospy.init_node('multi_global_planner_client')
    A = multiglobalplannerclient()
    rospy.spin()



