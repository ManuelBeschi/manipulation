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

#pragma once
#include <rosdyn_core/primitives.h>



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
#include <eigen_conversions/eigen_msg.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <tf_conversions/tf_eigen.h>
#include <manipulation_msgs/Location.h>
#include <manipulation_msgs/AddLocations.h>
#include <manipulation_msgs/RemoveLocations.h>

#define N_ITER 30
#define N_MAX_ITER 2000
#define TOLERANCE 1e-6

namespace pickplace {



class LocationManager;

class Location
{
public:
//  enum Status { empty, full, error};
  enum Destination { Approach, Slot, Leave};
  Location(const std::string& name,
           const Eigen::Affine3d& T_w_location,
           const Eigen::Affine3d& T_w_approach,
           const Eigen::Affine3d& T_w_leave);

  Location(const manipulation_msgs::Location& msg);

  friend class LocationManager;

  void addLocationIk( const std::string& group_name, const std::vector<Eigen::VectorXd>& solutions);
  void addApproachIk( const std::string& group_name, const std::vector<Eigen::VectorXd>& solutions);
  void addReturnIk(    const std::string& group_name, const std::vector<Eigen::VectorXd>& solutions);

  bool canBePickedBy(const std::string& group_name);
  std::vector<Eigen::VectorXd> getLocationIk(const std::string& group_name){return m_location_configurations.at(group_name);}
  std::vector<Eigen::VectorXd> getApproachIk(const std::string& group_name){return m_approach_location_configurations.at(group_name);}
  std::vector<Eigen::VectorXd> getReturnIk  (const std::string& group_name){return m_return_location_configurations.at(group_name);}

protected:
//  Status m_status;
  std::string m_name;
  Eigen::Affine3d m_T_w_location;  // world <- location
  Eigen::Affine3d m_T_w_approach;  // world <- approach
  Eigen::Affine3d m_T_w_return;    // world <- return

  std::map<std::string,std::vector<Eigen::VectorXd>> m_location_configurations;
  std::map<std::string,std::vector<Eigen::VectorXd>> m_approach_location_configurations;
  std::map<std::string,std::vector<Eigen::VectorXd>> m_return_location_configurations;

};
typedef shared_ptr_namespace::shared_ptr< Location   > LocationPtr;


}  // end namespace pickplace
