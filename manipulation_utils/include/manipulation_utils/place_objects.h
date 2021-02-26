

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

#include <manipulation_msgs/AddSlots.h>
#include <manipulation_msgs/RemoveSlots.h>
#include <manipulation_msgs/RemoveObjectsFromSlot.h>
#include <manipulation_msgs/ResetSlots.h>
#include <manipulation_msgs/PlaceObjectsAction.h>

#include <manipulation_utils/skill_base.h>
#include <manipulation_utils/manipulation_utils.h>

#include <std_srvs/SetBool.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace manipulation
{
  class PlaceObjects: public SkillBase
  {
    protected:
      
      ros::NodeHandle m_nh;
      ros::NodeHandle m_pnh;

      ros::ServiceServer m_add_slots_srv;
      ros::ServiceServer m_remove_slots_srv;
      ros::ServiceServer m_remove_obj_from_slot_srv;      
      ros::ServiceServer m_reset_slots_srv;
      ros::ServiceClient m_detach_object_srv;

      std::map<std::string,SlotPtr> m_slots;

      tf::TransformBroadcaster m_broadcaster;
      std::map<std::string,tf::Transform> m_tf;

      std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>> m_place_servers;

    public:
      PlaceObjects( const ros::NodeHandle& m_nh,
                    const ros::NodeHandle& m_pnh);
  
      bool init();

      bool addSlotsCb(manipulation_msgs::AddSlots::Request& req, 
                      manipulation_msgs::AddSlots::Response& res);

      bool removeSlotsCb( manipulation_msgs::RemoveSlots::Request& req, 
                          manipulation_msgs::RemoveSlots::Response& res);

      bool removeObjectFromSlotCb(manipulation_msgs::RemoveObjectsFromSlot::Request& req, 
                                  manipulation_msgs::RemoveObjectsFromSlot::Response& res);

      bool resetSlotsCb(manipulation_msgs::ResetSlots::Request& req, 
                        manipulation_msgs::ResetSlots::Response& res);

      void placeObjectGoalCb( const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                              const std::string& group_name);

      void publishTF();
 
  };


}