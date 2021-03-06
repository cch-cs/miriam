#!/usr/bin/env python

import numpy as np
import os
import sys
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
sys.path.append(os.path.realpath(os.path.join(__file__,"../../../")))
from tools import load_map
from planner.tcbs.plan import generate_config
from planner.greedy.greedy import plan_greedy
from multi_robot_action_move.msg import agent_paths_id, Path_array_agents_id
from real_robot_controller.msg import agent_job,agent_paths
from multi_robot_action_move.srv import multiplannergreedy,multiplannergreedyResponse
from nav_msgs.msg import Path
import rospy

def handle_multi_planner(req):
    _map = load_map(os.path.realpath(os.path.join(__file__,"../../../")) + '/multi_robot_action_move/map/' + sys.argv[1])
    map_resolution = 1
#    _map = rospy.wait_for_message("/map",OccupancyGrid) # change the topic based on the namespace
#    map_resolution = round(_map.info.resolution,2)
#    map_width = _map.info.width
#    map_height = _map.info.height
#    map = np.array(_map.data)
#    map = np.reshape(map,(map_height,map_width))
    agent_pos = []
    _agent = 0
    for pos in req.start.robot_name_pose:
        agent_pos.append((round(pos.robot_pose.pose.position.x/map_resolution,0),round(pos.robot_pose.pose.position.y/map_resolution,0))) #round based on map resolution
    it_jobs =[ round(elem,2) for elem in req.jobs]
    print(it_jobs)
    list_jobs = [it_jobs[i:i+5] for i in range(0, len(it_jobs), 5)]
    print(list_jobs)
    jobs = []
    for sublist in list_jobs:
        tuple_start = (round(sublist[0]/map_resolution,0),round(sublist[1]/map_resolution,0))
        tuple_goal = (round(sublist[2]/map_resolution,0),round(sublist[3]/map_resolution,0))
        job = (tuple_start,tuple_goal,sublist[4])
        print(job)
        jobs.append(job)
    grid = np.repeat(_map[:, ::2, np.newaxis], 100, axis=2)
#    grid = np.repeat(map[:, ::, np.newaxis], 100, axis=2)

    config = generate_config()
    config['filename_pathsave'] = req.fname
    config['finished_agents_block'] = True
    print("Problem:")
    print(str(jobs))
    print(str(agent_pos))
    print("GREEDY")
    gui_path_array = Path_array_agents_id()
    res_agent_job, tuple_paths = plan_greedy(agent_pos, jobs, grid, config)
    max_job_size = max([len(i) for i in res_agent_job]) * 2
    gui_path_array.max_job_size = max_job_size

    for robo in res_agent_job:
        agent_robo_job = agent_job()
        for jobs in robo:
            agent_robo_job.agent_robo_job.append(jobs)
        gui_path_array.agent_job.append(agent_robo_job)
    print("tuple_paths")
    print(tuple_paths)
    for agent in tuple_paths:
        _seq = 0
        _agent_paths = agent_paths_id()
        for job in agent:
            gui_path = Path()
            Poses = []
            for pos in job:
                pose = PoseStamped()
                pose.header.seq = _seq
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = pos[0]*map_resolution
                pose.pose.position.y = pos[1]*map_resolution
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0
                Poses.append(pose)
                _seq = _seq + 1
            gui_path.header.frame_id = "map"
            gui_path.header.stamp = rospy.Time.now()
            gui_path.poses = Poses
            if len(gui_path.poses) == 1:
                _agent_paths.agent_paths.append(gui_path)
            elif gui_path.poses[0].pose != gui_path.poses[-1].pose:
                _agent_paths.agent_paths.append(gui_path)
        _agent_paths.robot_name = req.start.robot_name_pose[_agent].robot_name
        _agent = _agent + 1
        gui_path_array.path_array.append(_agent_paths)
    _agent = 0
    return gui_path_array

def multirobot_planner_server():
    rospy.init_node('multi_planner_server')
    s = rospy.Service('multi_planner_greedy',multiplannergreedy,handle_multi_planner)
    print ("Planning the path for the robots")
    rospy.spin()

if __name__ == '__main__':
    multirobot_planner_server()

