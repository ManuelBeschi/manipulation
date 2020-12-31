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

#include <location.h>

#include <manipulation_msgs/ListOfObjects.h>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>


namespace manipulation
{  
  class SkillBase
  {
    protected:

      bool m_init;

      ros::NodeHandle m_nh;
      ros::NodeHandle m_pnh; 

      ros::Publisher m_target_pub;
      ros::ServiceClient m_grasp_srv;

      LocationManager m_loc_man;

      std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;

      //std::map<std::string,InboundBoxPtr> searchLocationName(const std::string& location_name);

      bool removeLocation(const std::string& element_name);

      bool execute( const std::string& group_name,
                    const moveit::planning_interface::MoveGroupInterface::Plan& plan);

      bool wait(const std::string& group_name);

      void doneCb(const actionlib::SimpleClientGoalState& state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result,
                  const std::string& group_name);

    public:
      SkillBase(const ros::NodeHandle& nh,
                const ros::NodeHandle& pnh);

      bool init();

      bool addObjectsCb(manipulation_msgs::AddObjects::Request& req,
                        manipulation_msgs::AddObjects::Response& res);

      bool addBoxesCb(manipulation_msgs::AddBoxes::Request& req,
                      manipulation_msgs::AddBoxes::Response& res);

      bool addLocationsCb(manipulation_msgs::AddLocations::Request& req,
                          manipulation_msgs::AddLocations::Response& res); // to be checked if it is necessary

      bool listObjectsCb( manipulation_msgs::ListOfObjects::Request& req,
                          manipulation_msgs::ListOfObjects::Response& res);

      bool resetBoxesCb(std_srvs::SetBool::Request& req, 
                        std_srvs::SetBool::Response& res); // to be evaluated
                        
      bool removeLocationsCb( manipulation_msgs::RemoveLocations::Request& req,
                              manipulation_msgs::RemoveLocations::Response& res);

  };

} // end namespace manipulation