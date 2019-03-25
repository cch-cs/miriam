# real_robot_controller Package

real_robot_controller is a package for the multi robot navigation by providing the plan of each job to the jobs, and then carries out the next job. when it receives input from the user.

## USAGE
## Launch Files:
'''
“gazebo_multiplanner.launch” launches the gazebo empty world with two robots.  
“stage_multiplanner.launch” launches stage with two robots.  
“turtlesim_multiplanner.launch”, launches two turtlesim robots.  
'''
## To Start Multi Robot Navigation:

After Launching any one of the above launch file, run the “rob_id_pub.py” file to start the job.   
Once the job is completed, rerun the file mentioned in the above line to start the new job, which is preassigned. Rerun the file still all the jobs are completed.  

## Note:

Timestamp of the plan of the robots was not considered, which can lead to collision.  
The plan is made by greedy planner for the jobs preassigned, and the plan for each job is given to the robots.  
