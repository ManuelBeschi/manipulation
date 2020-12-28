#pragma once

/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>

#include <manipulation_utils.h>
#include <rosdyn_core/primitives.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>



namespace manipulation
{
	class MoveItBase
	{
		protected:
			std::string world_frame="world";

			ros::Publisher m_display_publisher;

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