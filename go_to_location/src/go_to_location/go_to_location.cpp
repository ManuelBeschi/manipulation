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

#include <go_to_location/go_to_location.h>


namespace manipulation_skills
{

GoToLocation::GoToLocation(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
{
  m_nh=nh;
  m_pnh=pnh;
}

bool GoToLocation::init()
{
  Eigen::Affine3d T_frame_slot;
  Eigen::Affine3d T_w_frame;
  Eigen::Affine3d T_w_slot;


  std::string frame;

  if (!m_pnh.getParam("group_names",m_group_names))
  {
    ROS_ERROR("parameter %s/group_names is not defined",m_pnh.getNamespace().c_str());
    return false;
  }


  if (!m_pnh.getParam("request_adapters", m_request_adapters))
  {
    ROS_ERROR_STREAM("Could not find request_adapters in namespace " << m_nh.getNamespace());
    return false;
  }
  m_planner_plugin_name= "ha_planner/DgacoPlannerManager";
  if (!m_pnh.getParam("planning_plugin", m_planner_plugin_name))
  {
    ROS_ERROR_STREAM("Could not find planner plugin name");
    return false;
  }

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  m_kinematic_model = robot_model_loader.getModel();
  m_planning_scene=std::make_shared<planning_scene::PlanningScene>(m_kinematic_model);
  m_planning_pipeline=std::make_shared<planning_pipeline::PlanningPipeline>(m_kinematic_model, m_nh, m_planner_plugin_name, m_request_adapters);

  // create groups
  for (const std::string& group_name: m_group_names)
  {
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

    std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::GoToAction>> as;
    as.reset(new actionlib::SimpleActionServer<manipulation_msgs::GoToAction>(m_nh,group_name+"/goto",
                                                                              boost::bind(&GoToLocation::gotoGoalCb,this,_1,group_name),
                                                                              false));
    m_as.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::GoToAction>>>(group_name,as));

    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_ac;
    fjt_ac.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+group_name+"/follow_joint_trajectory",true));
    m_fjt_clients.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>>(group_name,fjt_ac));

    m_fjt_result.insert(std::pair<std::string,double>(group_name,0));
  }

  m_target_pub=m_nh.advertise<geometry_msgs::PoseStamped>("target",1);

  m_init=true;
  for (const std::string& group_name: m_group_names)
  {
    m_as.at(group_name)->start();
  }

  return true;

}

void GoToLocation::gotoGoalCb(const manipulation_msgs::GoToGoalConstPtr& goal,
                              const std::string& group_name)
{
  std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::GoToAction>> as=m_as.at(group_name);

  manipulation_msgs::GoToResult action_res;
  std::string place_id=goal->location_id;
  moveit::planning_interface::MoveItErrorCode result;


  if (!m_init)
  {
    ROS_ERROR("goto server is not initialized well");
    action_res.result=manipulation_msgs::GoToResult::NotInitialized;
    as->setAborted(action_res,"not initialized");
    return;
  }

  std::vector<double> configuration;
  if (!m_pnh.getParam("locations/"+group_name+"/"+place_id,configuration))
  {
    ROS_ERROR("slot %s/locations/%s/%s is not managed by the goto server ",m_pnh.getNamespace().c_str(),group_name.c_str(), place_id.c_str());
    action_res.result=manipulation_msgs::GoToResult::NotInitialized;
    as->setAborted(action_res,"slot not managed");
    return;
  }

  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  if (!group->startStateMonitor(2))
  {
    ROS_ERROR("unable to get actual state",m_pnh.getNamespace().c_str());
    action_res.result=manipulation_msgs::GoToResult::SceneError;
    as->setAborted(action_res,"unable to get actual state");
    return;
  }
  group->setStartState(*group->getCurrentState());
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);


  Eigen::VectorXd actual_jconf;
  group->getCurrentState()->copyJointGroupPositions(group_name,actual_jconf);

  if (configuration.size()!=actual_jconf.size())
  {
    ROS_ERROR("slot %s has wrong dimensions: %zu instead of %zu",place_id.c_str(),configuration.size(),actual_jconf.size());
    action_res.result=manipulation_msgs::GoToResult::NotInitialized;
    as->setAborted(action_res,"slot not managed");
    return;
  }
  Eigen::VectorXd location_jconf(actual_jconf.size());
  for (size_t iax=0;iax<configuration.size();iax++)
  {
    location_jconf(iax)=configuration.at(iax);
  }

  moveit::planning_interface::MoveGroupInterface::Plan approac_pick_plan=planToLocation(group_name,
                                                                                        place_id,
                                                                                        actual_jconf,
                                                                                        location_jconf,
                                                                                        result);

  if (!result)
  {
    action_res.result=manipulation_msgs::GoToResult::NoAvailableTrajectories;
    ROS_ERROR("error in plan for placing slot, code = %d",result.val);
    as->setAborted(action_res,"error in planning for placing");
    return;
  }

  execute(group_name,approac_pick_plan);
  if (!wait(group_name))
  {
    action_res.result=manipulation_msgs::GoToResult::TrajectoryError;
    ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
    as->setAborted(action_res,"error in trajectory execution");
    return;
  }

  action_res.result=manipulation_msgs::GoToResult::Success;
  as->setSucceeded(action_res,"ok");

  return;

}




moveit::planning_interface::MoveGroupInterface::Plan GoToLocation::planToLocation(const std::string& group_name,
                                                                                  const std::string& place_id,
                                                                                  const Eigen::VectorXd& starting_jconf,
                                                                                  const Eigen::VectorXd& location_jconf,
                                                                                  moveit::planning_interface::MoveItErrorCode& result)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);

  robot_state::RobotState state = *group->getCurrentState();
  state.setJointGroupPositions(jmg,starting_jconf);

  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name=group_name;
  req.start_state=plan.start_state_;

  req.allowed_planning_time=5;
  robot_state::RobotState goal_state(m_kinematic_model);


  goal_state.setJointGroupPositions(jmg, location_jconf);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
  req.goal_constraints.push_back(joint_goal);


  if (!m_planning_pipeline->generatePlan(m_planning_scene, req, res))
  {
    ROS_ERROR("Could not compute plan successfully");
    result= res.error_code_;
    return plan;
  }
  plan.planning_time_=res.planning_time_;

  res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);

  result= res.error_code_;

  return plan;
}




bool GoToLocation::execute(const std::string& group_name,
                           const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory=plan.trajectory_.joint_trajectory;

  auto cb=boost::bind(&manipulation_skills::GoToLocation::doneCb,this,_1,_2,group_name);
  m_fjt_clients.at(group_name)->sendGoal(goal,
                                         cb);

  m_fjt_result.at(group_name)=std::nan("1");;

  return true;
}

bool GoToLocation::wait(const std::string& group_name)
{
  while (std::isnan(m_fjt_result.at(group_name)))
    ros::Duration(0.01).sleep();
  return !std::isnan(m_fjt_result.at(group_name));
}

void GoToLocation::doneCb(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &group_name)
{
  m_fjt_result.at(group_name)=result->error_code;
  if (result->error_code<0)
  {
    ROS_ERROR("error executing %s/follow_joint_trajectory: %s",group_name.c_str(),result->error_string.c_str());
  }
  return;
}


}
