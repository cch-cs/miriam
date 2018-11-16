#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Config.h>


using std::string;

#ifndef MULTIGLOBALPLANNER_FINAL_ROS_CPP
#define MULTIGLOBALPLANNER_FINAL_ROS_CPP

namespace multi_global_planner_final {

class MultiGlobalPlannerFinal : public nav_core::BaseGlobalPlanner {
	costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
	bool initialized_;
	ros::Publisher plan_pub_;
	boost::shared_ptr<nav_msgs::Path const> _gui_plan;
	nav_msgs::Path gui_plan;
			
	
public:
	
	MultiGlobalPlannerFinal();
	MultiGlobalPlannerFinal(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	void DynamicUpdateParameter(geometry_msgs::PoseStamped _goal);
	};
};
#endif



