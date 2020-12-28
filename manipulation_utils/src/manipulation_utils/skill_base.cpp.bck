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

namespace pickplace {


Skill::Skill(const ros::NodeHandle& nh,
             const ros::NodeHandle& pnh,
             const std::string& name):
  m_nh(nh),
  m_pnh(pnh),
  m_name(name)
{

}
/*
bool Skill::init()
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  m_kinematic_model = robot_model_loader.getModel();

  if (!m_pnh.getParam("request_adapters", m_request_adapters))
    ROS_ERROR_STREAM("Could not find request_adapters in namespace " << m_pnh.getNamespace());

  if (!m_pnh.getParam("groups",m_tool_names))
  {
    ROS_ERROR("parameter %s/groups is not defined",m_pnh.getNamespace().c_str());
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

    m_planning_scene.insert(std::pair<std::string,std::shared_ptr<planning_scene::PlanningScene>>(group_name,std::make_shared<planning_scene::PlanningScene>(m_kinematic_model)));

    planning_pipeline::PlanningPipelinePtr planning_pipeline=std::make_shared<planning_pipeline::PlanningPipeline>(m_kinematic_model, m_nh, planner_plugin_name, m_request_adapters);
    m_planning_pipeline.insert(std::pair<std::string,planning_pipeline::PlanningPipelinePtr>(group_name,planning_pipeline));

    moveit::planning_interface::MoveGroupInterfacePtr group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
    if (!group->startStateMonitor(3))
    {
      ROS_ERROR("unable to get robot state for group %s",group_name.c_str());
      return false;
    }
    group->setStartState(*group->getCurrentState());
    m_groups.insert(std::pair<std::string,moveit::planning_interface::MoveGroupInterfacePtr>(group_name,group));


    moveit::core::JointModelGroup* jmg = m_kinematic_model->getJointModelGroup(group_name);
    m_joint_models.insert(std::pair<std::string,moveit::core::JointModelGroup*>(group_name,jmg));

    size_t n_joints=jmg->getActiveJointModelNames().size();
    std::vector<double> tmp;
    if (m_pnh.getParam(group_name+"/preferred_configuration",tmp))
    {
      assert(tmp.size()==n_joints);
      Eigen::VectorXd preferred_position(n_joints);
      for (size_t idof=0;idof<n_joints;idof++)
        preferred_position(idof)=tmp.at(idof);

      ROS_INFO_STREAM("preferred configuration of "<<group_name<<" is " << preferred_position.transpose());
      m_preferred_configuration.insert(std::pair<std::string,Eigen::VectorXd>(group_name,preferred_position));
      if (m_pnh.getParam(group_name+"/preferred_configuration_weight",tmp))
      {
        assert(tmp.size()==n_joints);
        Eigen::VectorXd preferred_position_weight(n_joints);
        for (size_t idof=0;idof<n_joints;idof++)
          preferred_position_weight(idof)=tmp.at(idof);
        ROS_INFO_STREAM("preferred configuration weight of "<<group_name<<" is " << preferred_position_weight.transpose());

        m_preferred_configuration_weight.insert(std::pair<std::string,Eigen::VectorXd>(group_name,preferred_position_weight));
      }
      else
      {
        Eigen::VectorXd preferred_position_weight(n_joints,1);
        ROS_INFO_STREAM("preferred configuration weight of "<<group_name<<" is " << preferred_position_weight.transpose());
        m_preferred_configuration_weight.insert(std::pair<std::string,Eigen::VectorXd>(group_name,preferred_position_weight));
      }
    }
    else
    {
      ROS_WARN("no preferred configuration for group %s",group_name.c_str());
      ROS_WARN("to do it set parameters %s/%s/preferred_configuration and %s/%s/preferred_configuration_weight",m_pnh.getNamespace().c_str(),group_name.c_str(),m_pnh.getNamespace().c_str(),group_name.c_str());
    }

    int max_ik_goal_number;
    if (!m_pnh.getParam(group_name+"/max_ik_goal_number",max_ik_goal_number))
    {
      max_ik_goal_number=N_TRIAL;
    }
    m_max_ik_goal_number.insert(std::pair<std::string,int>(group_name,max_ik_goal_number));

    std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>> as;
    as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>(m_nh,group_name+"/pick",
                                                                                    boost::bind(&PickObjects::pickObjectGoalCb,this,_1,group_name),
                                                                                    false));
    m_pick_servers.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>>>(group_name,as));

    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_ac;
    fjt_ac.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+group_name+"/follow_joint_trajectory",true));
    m_fjt_clients.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>>(group_name,fjt_ac));

    m_fjt_result.insert(std::pair<std::string,double>(group_name,0));


  }

  m_add_obj_srv=m_nh.advertiseService("add_objects",&PickObjects::addObjectCb,this);
  m_add_box_srv=m_nh.advertiseService("add_box",&PickObjects::addBoxCb,this);
  m_list_objects_srv=m_nh.advertiseService("list_objects",&PickObjects::listObjects,this);

  m_attach_obj_=m_nh.serviceClient<object_loader_msgs::attachObject>("attach_object_to_link");

  m_target_pub=m_nh.advertise<geometry_msgs::PoseStamped>("target",1);

  m_grasp_srv=m_nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");

  for (const std::string& group_name: m_group_names)
  {
    m_pick_servers.at(group_name)->start();
  }
  return true;
}
*/
}  // end namespace pickplace
