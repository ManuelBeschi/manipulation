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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/AddBox.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_msgs/ListOfObjects.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/SetBool.h>
#include <rosparam_utilities/rosparam_utilities.h>
#define N_ITER 20
#define TOLERANCE 1e-6


#define ROS_PROTO(...) ROS_LOG(::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)


namespace pickplace {

class OutboundPallet
{

protected:

  moveit::planning_interface::MoveGroupInterfacePtr m_group;
  std::shared_ptr<planning_scene::PlanningScene> m_planning_scene;
  robot_model::RobotModelPtr m_kinematic_model;
  moveit::core::JointModelGroup* m_jmg;
  std::string m_group_name;
  std::string tool_name="tip";

  planning_pipeline::PlanningPipelinePtr m_planning_pipeline;
  std::string m_planner_plugin_name;

  ros::ServiceClient m_grasp_srv;
  std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> m_as;
  ros::Publisher m_target_pub;
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  Eigen::Affine3d m_T_w_f; // world <- first slot
  Eigen::Affine3d m_T_w_a; // world <- approach
  Eigen::Affine3d m_T_w_s; // world <- slot

  std::vector<Eigen::VectorXd> m_approach_sols;

  Eigen::VectorXd m_gaps;
  std::vector<unsigned int> m_sizes;
  std::vector<unsigned int> m_indexes;

  bool m_init=false;
  bool m_full=false;
  bool ik(const Eigen::Affine3d& T_w_a, std::vector<Eigen::VectorXd >& sols, unsigned int ntrial=N_ITER);

  bool ikForTheSlot(std::vector<Eigen::VectorXd >& sols);
public:
  OutboundPallet(const ros::NodeHandle& nh,
                 const ros::NodeHandle& pnh);

  bool init(const std::string& group_name,
            const Eigen::Affine3d& T_w_f,
            const Eigen::Vector3d& gaps,
            const std::vector<unsigned int>& sizes,
            const Eigen::Vector3d& approach_distance);

  bool init();

  void placeObjectGoalCb(const manipulation_msgs::PlaceObjectsGoalConstPtr& goal);


  moveit::planning_interface::MoveGroupInterface::Plan planToApproach(moveit::planning_interface::MoveItErrorCode& result,
                                                                      Eigen::VectorXd& approach_jconf);


  moveit::planning_interface::MoveGroupInterface::Plan planToSlot(const Eigen::VectorXd& starting_jconf,
                                                                  moveit::planning_interface::MoveItErrorCode& result,
                                                                  Eigen::VectorXd& slot_jconf);

  moveit::planning_interface::MoveGroupInterface::Plan planToReturnToApproach(const Eigen::VectorXd& starting_jconf,
                                                                              const Eigen::VectorXd& approach_jconf,
                                                                              moveit::planning_interface::MoveItErrorCode& result);


  moveit::planning_interface::MoveItErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
  void wait();

  friend std::ostream& operator<<  (std::ostream& os, const OutboundPallet& pick_objs);

};


}
