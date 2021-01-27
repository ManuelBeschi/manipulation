#pragma once
/*
Copyright (c) 2020, Manuel Beschi 
CARI Joint Research Lab
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
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

#include <actionlib/server/simple_action_server.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <manipulation_msgs/Location.h>
#include <manipulation_msgs/AddLocations.h>
#include <manipulation_msgs/RemoveLocations.h>

#include <pluginlib/class_loader.h>

#include <rosdyn_core/primitives.h>

#include <rosparam_utilities/rosparam_utilities.h>

#include <tf_conversions/tf_eigen.h>

#define N_ITER 30
#define N_MAX_ITER 2000
#define TOLERANCE 1e-6

namespace manipulation 
{

class LocationManager;

class Location
{
public:
//  enum Status { empty, full, error};
  enum Destination { Approach, To, Leave};
  Location(const std::string& name,
           const Eigen::Affine3d& T_w_location,
           const Eigen::Affine3d& T_w_approach,
           const Eigen::Affine3d& T_w_leave);

  Location(const manipulation_msgs::Location& msg);

  friend class LocationManager;

  void addLocationIk( const std::string& group_name, const std::vector<Eigen::VectorXd>& solutions);
  void addApproachIk( const std::string& group_name, const std::vector<Eigen::VectorXd>& solutions);
  void addReturnIk(   const std::string& group_name, const std::vector<Eigen::VectorXd>& solutions);

  bool canBePickedBy(const std::string& group_name);
  
  Eigen::Affine3d getLocation(){return m_T_w_location;}
  Eigen::Affine3d getApproach(){return m_T_w_approach;}
  Eigen::Affine3d getReturn  (){return m_T_w_return;}

  std::vector<Eigen::VectorXd> getLocationIk(const std::string& group_name){return m_location_configurations.at(group_name);}
  std::vector<Eigen::VectorXd> getApproachIk(const std::string& group_name){return m_approach_location_configurations.at(group_name);}
  std::vector<Eigen::VectorXd> getReturnIk  (const std::string& group_name){return m_return_location_configurations.at(group_name);}

protected:
//  Status m_status;
  std::string m_name;
  std::string m_frame;
  Eigen::Affine3d m_T_w_location;  // world <- location
  Eigen::Affine3d m_T_w_approach;  // world <- approach
  Eigen::Affine3d m_T_w_return;    // world <- return

  std::map<std::string,std::vector<Eigen::VectorXd>> m_location_configurations;
  std::map<std::string,std::vector<Eigen::VectorXd>> m_approach_location_configurations;
  std::map<std::string,std::vector<Eigen::VectorXd>> m_return_location_configurations;

};
typedef shared_ptr_namespace::shared_ptr<Location> LocationPtr;

class LocationManager
{
public:

  LocationManager(const ros::NodeHandle& nh);

  bool init();

  bool addLocationsCb(manipulation_msgs::AddLocations::Request& req,
                      manipulation_msgs::AddLocations::Response& res);

  bool removeLocationsCb( manipulation_msgs::RemoveLocations::Request& req,
                          manipulation_msgs::RemoveLocations::Response& res);

  bool addLocationsFromMsg(const std::vector<manipulation_msgs::Location>& locations);

  bool removeLocations(const std::vector<std::string>& location_names);

  moveit::planning_interface::MoveGroupInterface::Plan planTo(const std::string& group_name,
                                                              const std::vector<std::string>& location_names,
                                                              const Location::Destination& destination,
                                                              const Eigen::VectorXd& starting_jconf,
                                                              moveit::planning_interface::MoveItErrorCode& result,
                                                              Eigen::VectorXd& final_configuration,
                                                              std::string& location_name );

protected:
  ros::NodeHandle m_nh;

  std::map<std::string,LocationPtr> m_locations;

  robot_model::RobotModelPtr m_kinematic_model;
  std::map<std::string,planning_pipeline::PlanningPipelinePtr> m_planning_pipeline;
  std::map<std::string,std::shared_ptr<planning_scene::PlanningScene>> m_planning_scene;
  std::map<std::string,moveit::planning_interface::MoveGroupInterfacePtr> m_groups;
  std::map<std::string,moveit::core::JointModelGroup*> m_joint_models;

  std::string world_frame;
  std::vector<std::string> m_group_names;
  std::vector<std::string> m_request_adapters;

  std::map<std::string,bool> m_use_single_goal;
  std::map<std::string,std::string> m_tool_names;

  std::map<std::string,double> m_fjt_result;
  std::map<std::string,int> m_max_ik_goal_number;
  std::map<std::string,rosdyn::ChainPtr> m_chains; 

  std::map<std::string,Eigen::VectorXd> m_preferred_configuration;
  std::map<std::string,Eigen::VectorXd> m_preferred_configuration_weight;

  ros::Publisher m_display_publisher;
  ros::ServiceServer m_add_locations_srv;
  ros::ServiceServer m_remove_locations_srv;

  bool addLocationFromMsg(const manipulation_msgs::Location& location);

  bool removeLocation(const std::string& location_name);

  std::vector<Eigen::VectorXd> getIkSolForLocation( const std::string& location_name,
                                                    const Location::Destination& destination,
                                                    const std::string& group_name);

  double computeDistanceBetweenLocations( const std::string& location_name,
                                          const std::string& group_name,
                                          const Location::Destination& destination,
                                          const Eigen::VectorXd& jconf);

  bool ik(const std::string& group_name,
          const Eigen::Affine3d& T_w_a,
          std::vector<Eigen::VectorXd >& sols,
          unsigned int ntrial = N_ITER);

};

}  // end namespace manipulation
