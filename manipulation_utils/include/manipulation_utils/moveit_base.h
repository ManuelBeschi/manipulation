#include <ros/ros.h>

#include <manipulation_utils.h>
#include <rosdyn_core/primitives.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

namespace manipulation
{
	class MoveItBase
	{
		protected:
			std::string world_frame="world";

			std::vector<std::string> m_group_names;
			std::vector<std::string> m_request_adapters;

			robot_model::RobotModelPtr m_kinematic_model;

			std::map<std::string,bool> m_use_single_goal;
			std::map<std::string,std::string> m_tool_names;

			std::map<std::string,double> m_fjt_result;
			std::map<std::string,int> m_max_ik_goal_number;
			std::map<std::string,rosdyn::ChainPtr> m_chains; 

			std::map<std::string,moveit::core::JointModelGroup*> m_joint_models;
			std::map<std::string,moveit::planning_interface::MoveGroupInterfacePtr> m_groups;

			std::map<std::string,planning_pipeline::PlanningPipelinePtr> m_planning_pipeline;
			std::map<std::string,std::shared_ptr<planning_scene::PlanningScene>> m_planning_scene;

			bool ik(const std::string& group_name,
							const Eigen::Affine3d& T_w_a, std::vector<Eigen::VectorXd >& sols, unsigned int ntrial=N_ITER);
	};
}