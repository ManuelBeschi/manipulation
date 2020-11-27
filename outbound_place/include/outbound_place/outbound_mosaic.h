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
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_msgs/ListOfObjects.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/SetBool.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <object_loader_msgs/detachObject.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rosdyn_core/primitives.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define N_ITER 30
#define N_MAX_ITER 2000
#define TOLERANCE 1e-6


#define ROS_PROTO(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)


namespace pickplace {

class OutboundMosaic
{

protected:


  robot_model::RobotModelPtr m_kinematic_model;
  std::map<std::string,planning_pipeline::PlanningPipelinePtr> m_planning_pipeline;
  std::map<std::string,std::shared_ptr<planning_scene::PlanningScene>> m_planning_scene;
  std::vector<std::string> m_group_names;
  std::map<std::string,moveit::planning_interface::MoveGroupInterfacePtr> m_groups;
  std::map<std::string,moveit::core::JointModelGroup*> m_joint_models;
  std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>> m_as;
  std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;
  std::map<std::string,rosdyn::ChainPtr> m_chains;

  std::string world_frame="world";
  std::map<std::string,std::string> m_tool_names;

  std::vector<std::string> m_request_adapters;
  std::map<std::string,double> m_fjt_result;

  ros::ServiceClient m_grasp_srv;
  ros::ServiceClient m_detach_object_srv;
  ros::ServiceServer m_reset_srv;
  ros::Publisher m_target_pub;
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  std::map<std::string, Eigen::Affine3d, std::less<std::string>,
                    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > > m_slot_map;
  std::map<std::string, Eigen::Affine3d, std::less<std::string>,
                    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > > m_approach_slot_map;

  std::map<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>> m_slot_configurations;
  std::map<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>> m_approach_slot_configurations;

  std::map<std::string,bool> m_slot_busy;

  bool m_init=false;
  bool ik(const std::string& group_name,
          const Eigen::Affine3d& T_w_a, std::vector<Eigen::VectorXd >& sols, unsigned int ntrial=N_ITER);


  bool execute(const std::string& group_name,
                                                      const moveit::planning_interface::MoveGroupInterface::Plan& plan);


  bool wait(const std::string& group_name);
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const control_msgs::FollowJointTrajectoryResultConstPtr& result,
              const std::string& group_name);



public:
  OutboundMosaic(const ros::NodeHandle& nh,
                 const ros::NodeHandle& pnh);

  bool init();


  void placeObjectGoalCb(const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                         const std::string& group_name);


  moveit::planning_interface::MoveGroupInterface::Plan planToSlot(const std::string& group_name,
                                                                  const std::string& place_id,
                                                                  const Eigen::VectorXd& starting_jconf,
                                                                  moveit::planning_interface::MoveItErrorCode& result,
                                                                  Eigen::VectorXd& slot_jconf);

  moveit::planning_interface::MoveGroupInterface::Plan planToApproachSlot(const std::string& group_name,
                                                                          const std::string& place_id,
                                                                          const Eigen::VectorXd& starting_jconf,
                                                                          moveit::planning_interface::MoveItErrorCode& result,
                                                                          Eigen::VectorXd& slot_jconf);



  bool resetCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  friend std::ostream& operator<<  (std::ostream& os, const OutboundMosaic& pick_objs);

};


}
