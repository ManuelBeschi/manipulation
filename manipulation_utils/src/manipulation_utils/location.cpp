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

#include <manipulation_utils/location.h>



namespace pickplace {

Location::Location(const std::string& name,
         const Eigen::Affine3d& T_w_location,
         const Eigen::Affine3d& T_w_approach,
         const Eigen::Affine3d& T_w_leave,
         const Status& status):
  m_name(name),
  m_T_w_location(T_w_location),
  m_T_w_approach(T_w_approach),
  m_T_w_leave(T_w_leave),
  m_status(status)
{

}

bool Location::canBePickedBy(const std::string &group_name)
{
  return ( m_location_configurations.find(group_name) != m_location_configurations.end() );
}
void Location::addLocationIk(const std::string &group_name, const std::vector<Eigen::VectorXd> &solutions)
{
  if ( m_location_configurations.find(group_name) == m_location_configurations.end() )
    m_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_location_configurations.at(group_name)=solutions;
}
void Location::addApproachIk(const std::string &group_name, const std::vector<Eigen::VectorXd> &solutions)
{
  if ( m_approach_location_configurations.find(group_name) == m_approach_location_configurations.end() )
    m_approach_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_approach_location_configurations.at(group_name)=solutions;
}
void Location::addLeaveIk(const std::string &group_name, const std::vector<Eigen::VectorXd> &solutions)
{
  if ( m_leave_location_configurations.find(group_name) == m_leave_location_configurations.end() )
    m_leave_location_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(group_name, solutions));
  else
    m_leave_location_configurations.at(group_name)=solutions;
}

LocationManager::LocationManager(const std::map<std::string, std::shared_ptr<planning_scene::PlanningScene> > &planning_scene,
                                 const std::map<std::string, moveit::planning_interface::MoveGroupInterfacePtr> &groups,
                                 const std::map<std::string, moveit::core::JointModelGroup *> &joint_models):
  m_planning_scene(planning_scene),
  m_groups(groups),
  m_joint_models(joint_models)
{

}

bool LocationManager::addLocation(LocationPtr &location)
{
  if ( m_locations.find(location->m_name) != m_locations.end() )
  {
    ROS_ERROR("Location %s is already present",location->m_name.c_str());
    return false;
  }
  bool added=false;
  for (const std::pair<std::string,moveit::planning_interface::MoveGroupInterfacePtr>& group: m_groups)
  {
    std::vector<Eigen::VectorXd> sols;
    std::vector<std::vector<double>> sols_stl;


    if (!m_nh.hasParam("slot_ik/"+location->m_name+"/"+group.first))
    {

      if (!ik(group.first,location->m_T_w_location,sols))
      {
        ROS_INFO("Location %s cannot be reached by group %s",location->m_name.c_str(),group.first.c_str());
        continue;
      }

      sols_stl.resize(sols.size());
      for (size_t isolution=0;isolution<sols.size();isolution++)
      {
        sols_stl.at(isolution).resize(sols.at(isolution).size());
        for (size_t iax=0;iax<sols.at(isolution).size();iax++)
          sols_stl.at(isolution).at(iax)=sols.at(isolution)(iax);
      }
      rosparam_utilities::setParam(m_nh,"slot_ik/"+location->m_name+"/"+group.first,sols_stl);
    }
    else
    {
      if (!rosparam_utilities::getParamMatrix(m_nh,"slot_ik/"+location->m_name+"/"+group.first,sols_stl))
      {
        ROS_ERROR("parameter %s/slot_ik/%s/%s is not correct",m_nh.getNamespace().c_str(),location->m_name.c_str(),group.first.c_str());
        return false;
      }
      sols.resize(sols_stl.size());
      for (size_t isolution=0;isolution<sols.size();isolution++)
      {
        sols.at(isolution).resize(sols_stl.at(isolution).size());
        for (size_t iax=0;iax<sols.at(isolution).size();iax++)
          sols.at(isolution)(iax)=sols_stl.at(isolution).at(iax);
      }
    }
    location->addLocationIk(group.first,sols);


    if (!ik(group.first,location->m_T_w_approach,sols))
    {
      ROS_INFO("Location %s cannot be reached by group %s",location->m_name.c_str(),group.first.c_str());
      continue;
    }
    location->addApproachIk(group.first,sols);

    if (!ik(group.first,location->m_T_w_leave,sols))
    {
      ROS_INFO("Location %s cannot be reached by group %s",location->m_name.c_str(),group.first.c_str());
      continue;
    }
    location->addLeaveIk(group.first,sols);

    added=true;
  }
  m_locations.insert(std::pair<std::string,LocationPtr>(location->m_name,location));
  return added;
}

bool LocationManager::ik(const std::string& group_name,
                         const Eigen::Affine3d& T_w_a, std::vector<Eigen::VectorXd >& sols, unsigned int ntrial)
{
  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
  std::map<double,Eigen::VectorXd> solutions;
  robot_state::RobotState state = *group->getCurrentState();
  Eigen::VectorXd actual_configuration;
  state.copyJointGroupPositions(group_name,actual_configuration);
  unsigned int n_seed=sols.size();
  bool found=false;

  for (unsigned int iter=0;iter<N_MAX_ITER;iter++)
  {
    if (solutions.size()>=3*N_ITER)
      break;
    if (iter<n_seed)
    {
      state.setJointGroupPositions(jmg,sols.at(iter));
    }
    else if (iter==n_seed)
    {
      state = *group->getCurrentState();
    }
    else
    {
      state.setToRandomPositions();
    }

    if (state.setFromIK(jmg,(Eigen::Isometry3d)T_w_a.matrix()))
    {
      if (!state.satisfiesBounds())
        continue;
      state.updateCollisionBodyTransforms();
      if (!m_planning_scene.at(group_name)->isStateValid(state))
        continue;
      Eigen::VectorXd js;
      state.copyJointGroupPositions(group_name,js);
      double dist=(js-actual_configuration).norm();
      if (solutions.size()==0)
      {
        solutions.insert(std::pair<double,Eigen::VectorXd>(dist,js));
        found=true;
      }
      else
      {
        bool is_diff=true;
        for (const std::pair<double,Eigen::VectorXd>& sol: solutions)
        {
          if ((sol.second-js).norm()<TOLERANCE)
          {
            is_diff=false;
            break;
          }
        }
        if (is_diff)
        {
          solutions.insert(std::pair<double,Eigen::VectorXd>(dist,js));
          found=true;
        }
      }
    }
    else
    {
      ROS_DEBUG("unable to find solutions for this seed");
    }
  }

  sols.clear();
  for (const std::pair<double,Eigen::VectorXd>& sol: solutions)
  {
    if (sols.size()>=N_ITER)
      break;
    sols.push_back(sol.second);
  }

  return found;
}

bool LocationManager::setLocationStatus(const std::string& location_name, const Location::Status& status)
{
  if ( m_locations.find(location_name) == m_locations.end() )
  {
    ROS_ERROR("Location %s is not present",location_name.c_str());
    return false;
  }
  m_locations.at(location_name)->m_status=status;
  return true;
}

bool LocationManager::removeLocation(const std::string& location_name)
{
  if ( m_locations.find(location_name) == m_locations.end() )
  {
    ROS_ERROR("Location %s is not present",location_name.c_str());
    return false;
  }
  m_locations.erase(m_locations.find(location_name));
  return true;
}



moveit::planning_interface::MoveGroupInterface::Plan LocationManager::planTo(const std::string& group_name,
                                                                             const std::string& location_name,
                                                                             const Destination& destination,
                                                                             const Eigen::VectorXd& starting_jconf,
                                                                             moveit::planning_interface::MoveItErrorCode& result,
                                                                             Eigen::VectorXd& final_configuration)
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


  std::vector<Eigen::VectorXd> sols;
  switch (destination)
  {
    case Location::Approach:
      sols=m_locations.at(location_name)->getApproachIk(group_name);
      break;
    case Location::Slot:
      sols=m_locations.at(location_name)->getSlotIk(group_name);
      break;
    case Location::Leave:
      sols=m_locations.at(location_name)->getLeaveIk(group_name);
      break;
  }

  for (const Eigen::VectorXd& goal: sols)
  {

    goal_state.setJointGroupPositions(jmg, goal);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
    req.goal_constraints.push_back(joint_goal);

  }
  ROS_DEBUG("Found %zu solution",sols.size());

  if (!m_planning_pipeline.at(group_name)->generatePlan(m_planning_scene.at(group_name), req, res))
  {
    ROS_ERROR("Could not compute plan successfully");
    result= res.error_code_;
    return plan;
  }
  plan.planning_time_=res.planning_time_;

  res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);
  if (res.trajectory_->getWayPointCount()==0)
  {
    ROS_WARN("trajectory with 0 waypoint");
  }
  else
    res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,final_configuration);
  result= res.error_code_;

  return plan;
}


}  // end namespace pickplace

