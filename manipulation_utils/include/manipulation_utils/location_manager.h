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

#include <manipulation_utils/location.h>

#define N_ITER 30
#define N_MAX_ITER 2000
#define TOLERANCE 1e-6

namespace pickplace {

class LocationManager
{
public:

  LocationManager( const std::map<std::string,std::shared_ptr<planning_scene::PlanningScene>>& planning_scene,
                   const std::map<std::string,moveit::planning_interface::MoveGroupInterfacePtr>& groups,
                   const std::map<std::string,moveit::core::JointModelGroup*>& joint_models,
                   const ros::NodeHandle& nh);
  bool addLocation(LocationPtr& location);
  bool removeLocation(const std::string& location_name);
  //bool setLocationStatus(const std::string& location_name, const Location::Status& status);
  //Location::Status getLocationStatus(const std::string& location_name);
  //bool setLocationStatusCb();

  bool addLocationsCb(manipulation_msgs::AddLocations::Request& req,
                      manipulation_msgs::AddLocations::Response& res);

  bool removeLocationsCb(manipulation_msgs::RemoveLocations::Request& req,
                         manipulation_msgs::RemoveLocations::Request& res);


  bool getConfigurationForGroup(const std::string& group_name,
                                const std::vector<std::string>& location_names,
                                std::vector<Eigen::VectorXd>& location_configurations,
                                std::vector<Eigen::VectorXd>& approach_configurations,
                                std::vector<Eigen::VectorXd>& leave_configurations);


  moveit::planning_interface::MoveGroupInterface::Plan planTo(const std::string& group_name,
                                                              const std::string& location_name,
                                                              const Location::Destination& destination,
                                                              const Eigen::VectorXd& starting_jconf,
                                                              moveit::planning_interface::MoveItErrorCode& result,
                                                              Eigen::VectorXd& final_configuration);

protected:
  ros::NodeHandle m_nh;
  ros::ServiceServer m_add_loc_srv;
  ros::ServiceServer m_remove_loc_srv;


  std::map<std::string,LocationPtr> m_locations;

  robot_model::RobotModelPtr m_kinematic_model;
  std::map<std::string,planning_pipeline::PlanningPipelinePtr> m_planning_pipeline;
  std::map<std::string,std::shared_ptr<planning_scene::PlanningScene>> m_planning_scene;
  std::map<std::string,moveit::planning_interface::MoveGroupInterfacePtr> m_groups;
  std::map<std::string,moveit::core::JointModelGroup*> m_joint_models;

  bool ik(const std::string& group_name,
          const Eigen::Affine3d& T_w_a,
          std::vector<Eigen::VectorXd >& sols,
          unsigned int ntrial=N_ITER);

};

}  // end namespace pickplace
