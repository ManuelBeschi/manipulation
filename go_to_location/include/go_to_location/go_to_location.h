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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <pluginlib/class_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/AddBox.h>
#include <manipulation_msgs/GoToAction.h>
#include <manipulation_msgs/ListOfObjects.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/SetBool.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


namespace manipulation_skills
{
class GoToLocation
{
protected:


  robot_model::RobotModelPtr m_kinematic_model;
  std::shared_ptr<planning_scene::PlanningScene> m_planning_scene;
  planning_pipeline::PlanningPipelinePtr m_planning_pipeline;

  std::vector<std::string> m_group_names;
  std::map<std::string,moveit::planning_interface::MoveGroupInterfacePtr> m_groups;
  std::map<std::string,moveit::core::JointModelGroup*> m_joint_models;
  std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::GoToAction>>> m_as;
  std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;

  std::string m_planner_plugin_name;
  std::vector<std::string> m_request_adapters;
  std::map<std::string,double> m_fjt_result;


  ros::Publisher m_target_pub;
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  bool m_init=false;
  bool execute(const std::string& group_name,
                                                      const moveit::planning_interface::MoveGroupInterface::Plan& plan);


  bool wait(const std::string& group_name);
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const control_msgs::FollowJointTrajectoryResultConstPtr& result,
              const std::string& group_name);

public:
  GoToLocation(const ros::NodeHandle& nh,
                 const ros::NodeHandle& pnh);

  bool init();


  void gotoGoalCb(const manipulation_msgs::GoToGoalConstPtr& goal,
                         const std::string& group_name);


  moveit::planning_interface::MoveGroupInterface::Plan planToLocation(const std::string& group_name,
                                                                  const std::string& place_id,
                                                                      const Eigen::VectorXd& starting_jconf,
                                                                      const Eigen::VectorXd& location_jconf,
                                                                  moveit::planning_interface::MoveItErrorCode& result);


  friend std::ostream& operator<<  (std::ostream& os, const GoToLocation& goto_location);
};
}

