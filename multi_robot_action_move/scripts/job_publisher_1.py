#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from real_robot_controller.msg import Job
import time

def job():
    pub = rospy.Publisher('job', Job, queue_size=1)
    job = Job()
    job.job_start.header.frame_id = "map"
    job.job_start.pose.position.x = 2 # 10.5
    job.job_start.pose.position.y = 5 # 12.2
    job.job_start.pose.orientation.w = 1
    job.job_goal.header.frame_id = "map"
    job.job_goal.pose.position.x = 3 # 11.4
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
    job.job_start.pose.position.x = 6 # 10.5
    job.job_start.pose.position.y = 2 # 12.2
    job.job_start.pose.orientation.w = 1
    job.job_goal.header.frame_id = "map"
    job.job_goal.pose.position.x = 4 # 11.4
    job.job_goal.pose.position.y = 4 # 12.7
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
    job.job_start.pose.position.y = 3 # 12.2
    job.job_start.pose.orientation.w = 1
    job.job_goal.header.frame_id = "map"
    job.job_goal.pose.position.x = 2 # 11.4
    job.job_goal.pose.position.y = 5 # 12.7
    job.job_goal.pose.orientation.w = 1
    job.job_time = 3
    while True:
        if pub.get_num_connections() > 0 :
            pub.publish(job)
            break
    time.sleep(0.01)
    job = Job()
    job.job_start.header.frame_id = "map"
    job.job_start.pose.position.x = 3 # 10.5
    job.job_start.pose.position.y = 6 # 12.2
    job.job_start.pose.orientation.w = 1
    job.job_goal.header.frame_id = "map"
    job.job_goal.pose.position.x = 1 # 11.4
    job.job_goal.pose.position.y = 4 # 12.7
    job.job_goal.pose.orientation.w = 1
    job.job_time = 4
    while True:
        if pub.get_num_connections() > 0 :
            pub.publish(job)
            break
    # time.sleep(0.1)
    # job = Job()
    # job.job_start.header.frame_id = "map"
    # job.job_start.pose.position.x = 5 # 10.5
    # job.job_start.pose.position.y = 1 # 12.2
    # job.job_start.pose.orientation.w = 1
    # job.job_goal.header.frame_id = "map"
    # job.job_goal.pose.position.x = 2 # 11.4
    # job.job_goal.pose.position.y = 2 # 12.7
    # job.job_goal.pose.orientation.w = 1
    # job.job_time = 5
    # while True:
    #     if pub.get_num_connections() > 0 :
    #         pub.publish(job)
    #         break
    # job = Job()
    # job.job_start.header.frame_id = "map"
    # job.job_start.pose.position.x = 2.5 # 10.5
    # job.job_start.pose.position.y = 4.2 # 12.2
    # job.job_start.pose.orientation.w = 1
    # job.job_goal.header.frame_id = "map"
    # job.job_goal.pose.position.x = 5.4 # 11.4
    # job.job_goal.pose.position.y = 7.7 # 12.7
    # job.job_goal.pose.orientation.w = 1
    # job.job_time = 3
    # while True:
    #     if pub.get_num_connections() > 0 :
    #         pub.publish(job)
    #         break

if __name__ == '__main__':
    rospy.init_node('job_publisher', anonymous=True)
    try:
        job()
    except rospy.ROSInterruptException:
        pass
