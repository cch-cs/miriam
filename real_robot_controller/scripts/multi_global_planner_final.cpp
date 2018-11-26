#include <real_robot_controller/multi_global_planner_final.h>
#include <pluginlib/class_list_macros.h>



namespace multi_global_planner_final
{
	MultiGlobalPlannerFinal::MultiGlobalPlannerFinal()
	: costmap_ros_(NULL), initialized_(false)
	{
	}	
	
	MultiGlobalPlannerFinal::MultiGlobalPlannerFinal(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	: costmap_ros_(NULL), initialized_(false)
	{
		initialize(name, costmap_ros);
	}
	
	
/*	void MultiGlobalPlanner::Subscribercallback(multirobotglobalplanner::Path_array msg)
	{
		this->gui_globalpath = msg;		
	}
*/	
	
	
	
	void MultiGlobalPlannerFinal::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		if (!initialized_)
		{
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();
			
			ros::NodeHandle private_nh("~/" + name);
			
			plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);
			
			initialized_ = true;
			
		}
	}
	
	void MultiGlobalPlannerFinal::DynamicUpdateParameter(geometry_msgs::PoseStamped _goal)
	{
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::StrParameter str_param;
		dynamic_reconfigure::Config conf;

		str_param.name = "base_global_planner";
		str_param.value = "global_planner/GlobalPlanner";
		conf.strs.push_back(str_param);

		srv_req.config = conf;

		if (ros::service::call("/move_base/set_parameters", srv_req, srv_resp))
		{	
			ROS_INFO("call to set base_global_planner as global_planer/GlobalPlanner succeeded");
		}
		else
		{
			ROS_INFO("call to set base_global_planner as global_planer/GlobalPlanner failed");
		}
	}
		
	bool MultiGlobalPlannerFinal::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
	{
		if (!initialized_)
		{
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return false;
		}
		
		ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
		
		plan.clear();
		
		if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
		{
			ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
			costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
			return false;
		}
		this-> _gui_plan = ros::topic::waitForMessage<nav_msgs::Path>("robot_plan");
		this-> gui_plan = *_gui_plan;
		if (this-> gui_plan.poses.size() > 0)
		{
			plan = this-> gui_plan.poses;
			plan_pub_.publish(this-> gui_plan);
			return true;
		}	
		else
			{
				ROS_WARN("There is no plan for this robot in the path planned, so the robot is moving to the goal position");
				DynamicUpdateParameter(goal);
				return false;
							
			}
		}
		

		


};

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(multi_global_planner_final::MultiGlobalPlannerFinal, nav_core::BaseGlobalPlanner)


