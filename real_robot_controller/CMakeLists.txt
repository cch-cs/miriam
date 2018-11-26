cmake_minimum_required(VERSION 2.8.3)
project(real_robot_controller)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  nav_msgs
  geometry_msgs
  message_generation
  nav_core
  roscpp
  rospy
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Plan.msg
   Path_array_agentjob.msg
   Job.msg
   agent_job.msg
   agent_paths.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   multiplannerjobserver.srv
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   real_robot_controller
   geometry_msgs
   std_msgs
   nav_msgs
)


include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a C++ library
  add_library(multiglobalplannerfinal_lib scripts/multi_global_planner_final.cpp)


add_executable(multi_global_planner_final scripts/multi_global_planner_final.cpp)
target_link_libraries(multi_global_planner_final ${catkin_LIBRARIES})

