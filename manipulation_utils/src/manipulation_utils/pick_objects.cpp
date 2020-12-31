
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

#include <manipulation_utils/pick_objects.h>

#include <object_loader_msgs/attachObject.h>

namespace manipulation
{
  PickObjects::PickObjects( const ros::NodeHandle& nh, 
                            const ros::NodeHandle& pnh):
                            m_nh(nh),
                            m_pnh(pnh),
                            SkillBase(nh,pnh)
  {

  }

  bool PickObjects::init()
  {
    if (!SkillBase::init())
    {
      m_init = false;
      return m_init;
    }
    
    m_add_obj_srv = m_nh.advertiseService("add_objects",&PickObjects::addObjectCb,this);
    m_add_box_srv = m_nh.advertiseService("add_box",&PickObjects::addBoxCb,this);
    m_list_objects_srv = m_nh.advertiseService("list_objects",&PickObjects::listObjects,this);
    m_reset_srv = m_nh.advertiseService("inbound/reset_box",&PickObjects::resetBoxesCb,this);

    m_attach_object_srv = m_nh.serviceClient<object_loader_msgs::attachObject>("attach_object_to_link");

    for (const std::string& group_name: m_group_names)
    { 
      //////////// To be Checked //////
      std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>> as;
      as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>( m_nh,
                                                                                        group_name+"/pick",
                                                                                        boost::bind(&PickObjects::pickObjectGoalCb,
                                                                                        this,
                                                                                        _1,
                                                                                        group_name),
                                                                                        false));
      m_pick_servers.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>>>(group_name,as));

      std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_ac;
      fjt_ac.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+group_name+"/follow_joint_trajectory",true));
      m_fjt_clients.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>>(group_name,fjt_ac));
      ////////////////////////////


      m_pick_servers.at(group_name)->start();
     
    }
    
    m_init = true;
    return m_init;
  }

}