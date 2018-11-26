#! /usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from multirobotglobalplanner.msg import Job

def job():
    pub = rospy.Publisher('job', Job, queue_size=1)
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
        if pub.get_num_connections() > 0 :
            pub.publish(job)
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
        if pub.get_num_connections() > 0 :
            pub.publish(job)
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
        if pub.get_num_connections() > 0 :
            pub.publish(job)
            break
    time.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('job_publisher', anonymous=True)
    try:
        job()
    except rospy.ROSInterruptException:
        pass
