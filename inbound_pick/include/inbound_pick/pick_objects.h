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
#include <inbound_pick/inbound_pick.h>
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
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/ListOfObjects.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/SetBool.h>
#include <object_loader_msgs/AttachObject.h>
#include <object_loader_msgs/DetachObject.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <mutex>
#include <rosdyn_core/primitives.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit_planning_helper/manage_trajectories.h>
#include <moveit_msgs/DisplayTrajectory.h>

#define N_MAX_ITER 20000
#define N_TRIAL 200
#define TOLERANCE 1e-3

namespace pickplace {

typedef std::multimap<std::string,
                      Eigen::Affine3d,
                      std::less<std::string>,
                      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d>>>  PosesMap;
typedef std::pair<std::string,Eigen::Affine3d>  PosesPair;

class PickObjects
{

protected:
  std::map<std::string,InboundBoxPtr> m_boxes;

  std::map<std::string,std::shared_ptr<planning_scene::PlanningScene>> m_planning_scene;
  robot_model::RobotModelPtr m_kinematic_model;
  std::map<std::string,planning_pipeline::PlanningPipelinePtr> m_planning_pipeline;
  std::vector<std::string> m_request_adapters;

  ros::Publisher m_display_publisher;
  int m_ik_sol_number=200;

  std::map<std::string,bool> m_use_single_goal;

  std::mutex m_mtx;
  std::mutex m_scene_mtx;
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  std::vector<std::string> m_group_names;
  std::map<std::string,moveit::planning_interface::MoveGroupInterfacePtr> m_groups;
  std::map<std::string,moveit::core::JointModelGroup*> m_joint_models;
  std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>>> m_pick_servers;
  std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;
  std::map<std::string,std::string> m_tool_names;
  std::map<std::string,double> m_fjt_result;
  std::map<std::string,rosdyn::ChainPtr> m_chains;

  std::map<std::string,int> m_max_ik_goal_number;
  std::string world_frame="world";

  tf::TransformBroadcaster m_broadcaster;
  std::map<std::string,tf::Transform> m_tf;

  ros::ServiceServer m_add_obj_srv;
  ros::ServiceServer m_add_box_srv;
  ros::ServiceServer m_list_objects_srv;
  ros::ServiceClient m_grasp_srv;
  ros::ServiceClient m_attach_obj_;
  ros::Publisher m_target_pub;
  ros::ServiceServer m_reset_box_srv;

  Eigen::Affine3d m_T_w_as; // world <- approach to slot
  Eigen::Affine3d m_T_w_s;  // world <- slot
  std::map<std::string,Eigen::VectorXd> m_preferred_configuration;
  std::map<std::string,Eigen::VectorXd> m_preferred_configuration_weight;


  bool ik(const std::string& group_name,
          Eigen::Affine3d T_w_a, std::vector<Eigen::VectorXd >& sols, unsigned int ntrial=N_TRIAL);



  std::map<std::string,InboundBoxPtr> searchBoxWithType(const std::string& type_name);
  std::map<std::string,InboundBoxPtr> searchBoxWithTypes(const std::vector<std::string>& type_names);

  moveit::planning_interface::MoveGroupInterface::Plan planToApproachSlot(const std::string& group_name,
                                                                          const Eigen::VectorXd& starting_jconf,
                                                                          const Eigen::Affine3d& approach_pose,
                                                                          const ObjectPtr& selected_object,
                                                                          const GraspPosePtr& selected_grasp_pose,
                                                                          moveit::planning_interface::MoveItErrorCode& result,
                                                                          Eigen::VectorXd& slot_jconf);

  moveit::planning_interface::MoveGroupInterface::Plan planToBestBox(const std::string& group_name,
                                                                     const std::map<std::string,InboundBoxPtr>& possible_boxes,
                                                                     moveit::planning_interface::MoveItErrorCode& result,
                                                                     InboundBoxPtr& selected_box,
                                                                     Eigen::VectorXd& jconf);


  moveit::planning_interface::MoveGroupInterface::Plan planToObject(const std::string& group_name,
                                                                    const std::vector<std::string>& type_name,
                                                                    const InboundBoxPtr& selected_box,
                                                                    const Eigen::VectorXd& starting_jconf,
                                                                    const std::string& tool_name,
                                                                    moveit::planning_interface::MoveItErrorCode& result,
                                                                    ObjectPtr& selected_object,
                                                                    GraspPosePtr& selected_grasp_pose
                                                                    );


  /* return false if already present
   */
  bool createInboundBox(const std::string& box_name,
                        const Eigen::Affine3d& T_w_box,
                        const double heigth);

  std::map<std::string,InboundBoxPtr>::iterator findBox(const std::string& box_name);

  /* return false if box does not exist
   */
  bool removeInboundBox(const std::string& box_name);

  ObjectPtr createObject(const std::string& type, const std::string& id,
                         const std::string& box_name,
                         const PosesMap& poses);

  bool removeObject(const std::string& type,
                    const std::string& box_name);


  bool execute(const std::string& group_name,
               const moveit::planning_interface::MoveGroupInterface::Plan& plan);
  bool wait(const std::string& group_name);

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const control_msgs::FollowJointTrajectoryResultConstPtr& result,
              const std::string& group_name);
public:
  PickObjects(const ros::NodeHandle& nh,
              const ros::NodeHandle& pnh);

  bool init();

  bool addObjectCb(manipulation_msgs::AddObjects::Request& req,
                    manipulation_msgs::AddObjects::Response& res);
  bool addBoxCb(manipulation_msgs::AddBox::Request& req,
              manipulation_msgs::AddBox::Response& res);
  bool listObjects(manipulation_msgs::ListOfObjects::Request& req,
                   manipulation_msgs::ListOfObjects::Response& res);

  void pickObjectGoalCb(const manipulation_msgs::PickObjectsGoalConstPtr& goal,
                        const std::string& group_name);

  bool resetBoxesCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  friend std::ostream& operator<<  (std::ostream& os, const PickObjects& pick_objs);

  void publishTF();

  void updatePlanningScene(const moveit_msgs::PlanningScene &scene);

};


}
