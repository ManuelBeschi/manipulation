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

#include <manipulation_utils/skill_base.h>

namespace manipulation
{
  SkillBase::SkillBase( const ros::NodeHandle& nh,
                        const ros::NodeHandle& pnh,
                        const std::string& name):
  m_nh(nh),
  m_pnh(pnh),
  m_name(name)
  {

  }

  bool SkillBase::init()
  {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); // può essere definto come membro di MoveItBase?!?
    m_kinematic_model = robot_model_loader.getModel();

    m_display_publisher = m_nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); // verificare se il nome del messaggio può essere sposta in file di configurazione 
  

    if (!m_pnh.getParam("request_adapters", m_request_adapters))
    {
      ROS_ERROR_STREAM("Could not find request_adapters in namespace " << m_nh.getNamespace());
      return false;
    }


    if (!m_pnh.getParam("groups",m_tool_names))
    {
      ROS_ERROR("parameter %s/groups is not defined",m_pnh.getNamespace().c_str());
      if (m_pnh.hasParam("groups"))
      {
        ROS_ERROR("parameter %s/groups is not wrong",m_pnh.getNamespace().c_str());
      }
      return false;
    }
    for (const std::pair<std::string,std::string>& p: m_tool_names)
    {
      m_group_names.push_back(p.first);
    }


    // create groups
    for (const std::string& group_name: m_group_names)
    {
      std::string planner_plugin_name;
      if (!m_pnh.getParam(group_name+"/planning_plugin", planner_plugin_name))
      {
        ROS_ERROR_STREAM("Could not find planner plugin name");
        return false;
      }

      bool use_single_goal;
      if (!m_pnh.getParam(group_name+"/use_single_goal",use_single_goal))
      {
        ROS_INFO("parameter %s/use_single_goal is not defined, use false (multi goal enable)",m_pnh.getNamespace().c_str());
        use_single_goal=false;
      }
      m_use_single_goal.insert(std::pair<std::string,bool>(group_name,use_single_goal));

      m_planning_scene.insert(std::pair<std::string,std::shared_ptr<planning_scene::PlanningScene>>(group_name,std::make_shared<planning_scene::PlanningScene>(m_kinematic_model)));
      collision_detection::AllowedCollisionMatrix acm=m_planning_scene.at(group_name)->getAllowedCollisionMatrixNonConst();
      std::vector<std::string> allowed_collisions;
      bool use_disable_collisions;
      if (m_pnh.getParam(group_name+"/use_disable_collisions",use_disable_collisions))
      {
        if (!m_pnh.getParam(group_name+"/disable_collisions",allowed_collisions))
        {
          ROS_INFO("parameter %s/%s/disable_collisions is not defined, use default",m_pnh.getNamespace().c_str(),group_name.c_str());
        }
        else
        {
          for (const std::string& link: allowed_collisions)
          {
            ROS_INFO("Disable collision detection for group %s and link %s",group_name.c_str(),link.c_str());
            acm.setEntry(link,true);
          }
        }
      }
      else
      {
        if (m_pnh.getParam(group_name+"/disable_collisions",allowed_collisions))
        {
          ROS_WARN("in group %s/%s you set disable_collisions but not use_disable_collisions, it is ignored",m_pnh.getNamespace().c_str(),group_name.c_str());
        }
      }

    }







    // To be finished

    return true;
  }


} //