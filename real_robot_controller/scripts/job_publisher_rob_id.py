#! /usr/bin/env python

import time
import rospy
from std_msgs.msg import String, Int16
from real_robot_controller.msg import Job

class job_rob_pub:
    def __init__(self):
        self.pub = rospy.Publisher('job', Job, queue_size=1)
        self.id_pub = rospy.Publisher('rob_id', Int16, queue_size=1)
    def job(self):
        job = Job()
        job.job_start.header.frame_id = "map"
        job.job_start.pose.position.x = 6 # 10.5
        job.job_start.pose.position.y = 5 # 12.2
        job.job_start.pose.orientation.w = 1
        job.job_goal.header.frame_id = "map"
        job.job_goal.pose.position.x = 4 # 11.4
        job.job_goal.pose.position.y = 5 # 12.7
        job.job_goal.pose.orientation.w = 1
        job.job_time = 1
        while True:
            if self.pub.get_num_connections() > 0 :
                self.pub.publish(job)
                break
        time.sleep(0.01)
        job = Job()
        job.job_start.header.frame_id = "map"
        job.job_start.pose.position.x = 3 # 10.5
        job.job_start.pose.position.y = 2 # 12.2
        job.job_start.pose.orientation.w = 1
        job.job_goal.header.frame_id = "map"
        job.job_goal.pose.position.x = 5 # 11.4
        job.job_goal.pose.position.y = 2 # 12.7
        job.job_goal.pose.orientation.w = 1
        job.job_time = 2
        while True:
            if self.pub.get_num_connections() > 0 :
                self.pub.publish(job)
                break
        time.sleep(0.01)
        job = Job()
        job.job_start.header.frame_id = "map"
        job.job_start.pose.position.x = 4 # 10.5
        job.job_start.pose.position.y = 6 # 12.2
        job.job_start.pose.orientation.w = 1
        job.job_goal.header.frame_id = "map"
        job.job_goal.pose.position.x = 2 # 11.4
        job.job_goal.pose.position.y = 6 # 12.7
        job.job_goal.pose.orientation.w = 1
        job.job_time = 3
        while True:
            if self.pub.get_num_connections() > 0 :
                self.pub.publish(job)
                break
        time.sleep(0.01)

    def rob_pub_id(self,msg):
        while True:
            if self.id_pub.get_num_connections() > 0 :
                self.id_pub.publish(msg)
                break

if __name__ == '__main__':
    rospy.init_node('job_rob_publisher', anonymous=True)
    A = job_rob_pub()
    try:
        A.job()
        for rob in range(2): # No. indicates the no. of robots
            rob_id = Int16()
            rob_id.data = rob + 1
            A.rob_pub_id(rob_id)
            time.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
