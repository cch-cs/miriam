# multi_robot_action_move package

multi_robot_action_move is a package for the multi robot navigation by each goal at a time for all robots for the preprocessed plan for the jobs.

## USAGE
## Launch Files:

“gazebo_multiplanner_goal_pub.launch” launches the gazebo empty world with two robots.
“stage_multiplanner_goal_pub.launch” launches stage with two robots.
“turtlesim_multiplanner_goal_pub.launch”, launches two turtlesim robots.


## Note:
Timestamp of the plan of the robots was considered.
All the jobs are carried out as per the plan with respect to the timestamp made by greedy planner.
