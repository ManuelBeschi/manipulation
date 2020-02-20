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


#include <outbound_place/outbound_place.h>


namespace pickplace
{

  OutboundPallet::OutboundPallet(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  {
    m_nh=nh;
    m_pnh=pnh;
  }

  bool OutboundPallet::init(const std::string &group_name,
                            const Eigen::Affine3d &T_w_f,
                            const Eigen::Vector3d &gaps,
                            const std::vector<unsigned int> &sizes,
                            const Eigen::Vector3d &approach_distance)
  {
    m_group_name=group_name;
    m_T_w_f=T_w_f;
    m_gaps=gaps;
    m_approach_distance=approach_distance;
    assert(sizes.size()==3);
    m_sizes=sizes;
    m_indexes.resize(3,0);
    m_group=std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);
    m_group->setStartState(*m_group->getCurrentState());

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    m_kinematic_model = robot_model_loader.getModel();
    m_jmg = m_kinematic_model->getJointModelGroup(group_name);



    m_planning_scene=std::make_shared<planning_scene::PlanningScene>(m_kinematic_model);

    m_planner_plugin_name= "ha_planner/DgacoPlannerManager";
    if (!m_nh.getParam("planning_plugin", m_planner_plugin_name))
      ROS_ERROR_STREAM("Could not find planner plugin name");

    std::vector<std::string> request_adapters;
    if (!m_nh.getParam("request_adapters", request_adapters))
      ROS_ERROR_STREAM("Could not find request_adapters in namespace " << m_nh.getNamespace());

    m_planning_pipeline=std::make_shared<planning_pipeline::PlanningPipeline>(m_kinematic_model, m_nh, m_planner_plugin_name, request_adapters);


    m_T_w_a=m_T_w_f;
    m_T_w_a.translation()+=approach_distance;

    if (!ik(m_T_w_a,m_approach_sols))
    {
      ROS_ERROR_STREAM("no IK solutions found for approach pose: \n"<<m_T_w_a.matrix());
      return false;
    }

    m_target_pub=m_nh.advertise<geometry_msgs::PoseStamped>("target",1);
    m_grasp_srv=m_nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");

    m_as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>(m_pnh,"place",
                                                                                        boost::bind(&OutboundPallet::placeObjectGoalCb,this,_1),
                                                                                        false));
    m_init=true;
    m_as->start();

    return true;
  }

  bool OutboundPallet::init()
  {
    std::string group_name;
    Eigen::Affine3d T_w_f;
    Eigen::Vector3d gaps;
    std::vector<unsigned int> sizes(3,0);
    Eigen::Vector3d approach_distance;

    if (!m_pnh.getParam("group_name",group_name))
    {
      ROS_ERROR("parameter %s/group_name is not defined",m_pnh.getNamespace().c_str());
      return false;
    }

    std::vector<double> gaps_d;
    if (!m_pnh.getParam("gaps",gaps_d))
    {
      ROS_ERROR("parameter %s/gaps is not defined",m_pnh.getNamespace().c_str());
      return false;
    }
    assert(gaps_d.size()==3);
    gaps(0)=gaps_d.at(0);
    gaps(1)=gaps_d.at(1);
    gaps(2)=gaps_d.at(2);

    std::vector<int> sizes_i;
    if (!m_pnh.getParam("sizes",sizes_i))
    {
      ROS_ERROR("parameter %s/sizes is not defined",m_pnh.getNamespace().c_str());
      return false;
    }
    assert(sizes_i.size()==3);
    sizes.at(0)=sizes_i.at(0);
    sizes.at(1)=sizes_i.at(1);
    sizes.at(2)=sizes_i.at(2);

    std::vector<double> approach_distance_d;
    if (!m_pnh.getParam("approach_distance",approach_distance_d))
    {
      ROS_ERROR("parameter %s/approach_distance is not defined",m_pnh.getNamespace().c_str());
      return false;
    }
    assert(approach_distance_d.size()==3);
    approach_distance(0)=approach_distance_d.at(0);
    approach_distance(1)=approach_distance_d.at(1);
    approach_distance(2)=approach_distance_d.at(2);


    std::vector<double> position;
    XmlRpc::XmlRpcValue first_slot;
    if (!m_pnh.getParam("first_slot",first_slot))
    {
      ROS_ERROR("parameter %s/first_slot is not defined",m_pnh.getNamespace().c_str());
      return false;
    }

    if( !rosparam_utilities::getParamVector(first_slot,"position",position) )
    {
      ROS_WARN("first_slot has not the field 'position'");
      return false;
    }
    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParamVector(first_slot,"quaternion",quaternion) )
    {
      ROS_WARN("first_slot has not the field 'quaternion'");
      return false;
    }
    assert(quaternion.size()==4);

    Eigen::Quaterniond q(quaternion.at(3),
                         quaternion.at(0),
                         quaternion.at(1),
                         quaternion.at(2));

    T_w_f=q;
    T_w_f.translation()(0)=position.at(0);
    T_w_f.translation()(1)=position.at(1);
    T_w_f.translation()(2)=position.at(2);



    return init(group_name,T_w_f,gaps,sizes,approach_distance);

  }

  void OutboundPallet::placeObjectGoalCb(const manipulation_msgs::PlaceObjectsGoalConstPtr& goal)
  {

    manipulation_msgs::PlaceObjectsResult action_res;
    std::string type_name=goal->object_type;

    moveit::planning_interface::MoveItErrorCode result;


    if (!m_init)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NotInitialized;
      ROS_ERROR("outbound pallet is not initialized well, code = %d",result.val);
      m_as->setAborted(action_res,"not initialized");
      return;
    }
    if (m_full)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::Full;
      ROS_ERROR("pallet is full, code = %d",result.val);
      m_as->setAborted(action_res,"pallet is full");
      return;
    }


    Eigen::VectorXd approach_jconf;
    moveit::planning_interface::MoveGroupInterface::Plan plan=planToApproach(result,approach_jconf);


    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan to approach place pose, code = %d",result.val);
      m_as->setAborted(action_res,"error in planning to pallet");
      return;
    }
    ROS_PROTO("plan to approach in %f second",plan.planning_time_);

    geometry_msgs::PoseStamped target;
    target.header.frame_id="world";
    target.header.stamp=ros::Time::now();
    tf::poseEigenToMsg(m_T_w_a,target.pose);
    m_target_pub.publish(target);
    execute(plan);


    Eigen::VectorXd approach_slot_jconf;
    moveit::planning_interface::MoveGroupInterface::Plan approac_pick_plan=planToApproachSlot(approach_jconf,
                                                                                              result,
                                                                                              approach_slot_jconf);

    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan for placing slot, code = %d",result.val);
      m_as->setAborted(action_res,"error in planning for placing");
      return;
    }

    wait();
    tf::poseEigenToMsg(m_T_w_as,target.pose);
    m_target_pub.publish(target);
    execute(approac_pick_plan);


    Eigen::VectorXd slot_jconf;
    moveit::planning_interface::MoveGroupInterface::Plan pick_plan=planToSlot(approach_slot_jconf,
                                                                              result,
                                                                              slot_jconf);

    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan for placing slot, code = %d",result.val);
      m_as->setAborted(action_res,"error in planning for placing");
      return;
    }

    wait();
    tf::poseEigenToMsg(m_T_w_s,target.pose);
    m_target_pub.publish(target);
    execute(pick_plan);


    std_srvs::SetBool grasp_req;
    grasp_req.request.data=0;
    m_grasp_srv.call(grasp_req);
    ros::Duration(1).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan return_plan=planToReturnToApproach(slot_jconf,
                                                                                            approach_slot_jconf,
                                                                                            result
                                                                                            );


    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan black to box, code = %d",result.val);
      m_as->setAborted(action_res,"error in planning back to box");
      return;
    }

    wait();
    tf::poseEigenToMsg(m_T_w_a,target.pose);
    m_target_pub.publish(target);
    execute(return_plan);

    action_res.result=manipulation_msgs::PlaceObjectsResult::Success;
    m_as->setSucceeded(action_res,"ok");

    m_indexes.at(0)++;
    if (m_indexes.at(0)>=m_sizes.at(0))
    {
      m_indexes.at(0)=0;
      m_indexes.at(1)++;
      if (m_indexes.at(1)>=m_sizes.at(1))
      {
        m_indexes.at(1)=0;
        m_indexes.at(2)++;
        if (m_indexes.at(2)>=m_sizes.at(2))
        {
          m_indexes.at(2)=0;
          ROS_INFO("Pallet is full, stop accepting jobs");
          m_full=true;
        }
      }
    }
    return;

  }



  bool OutboundPallet::ik(const Eigen::Affine3d& T_w_a, std::vector<Eigen::VectorXd>& sols, unsigned int ntrial)
  {
    std::vector<Eigen::VectorXd> solutions;
    robot_state::RobotState state = *m_group->getCurrentState();
    unsigned int n_seed=sols.size();
    bool found=false;

    for (unsigned int iter=0;iter<ntrial;iter++)
    {
      if (iter<n_seed)
      {
        state.setJointGroupPositions(m_jmg,sols.at(iter));
      }
      else if (iter==n_seed)
      {
        state = *m_group->getCurrentState();
      }
      else
      {
        state.setToRandomPositions();
      }

      if (state.setFromIK(m_jmg,(Eigen::Isometry3d)T_w_a.matrix()))
      {
        Eigen::VectorXd js;
        state.copyJointGroupPositions(m_group_name,js);
        if (solutions.size()==0)
        {
          solutions.push_back(js);
          found=true;
        }
        else
        {
          bool is_diff=true;
          for (const Eigen::VectorXd& sol: solutions)
          {
            if ((sol-js).norm()<TOLERANCE)
            {
              is_diff=false;
              break;
            }
          }
          if (is_diff)
          {
            solutions.push_back(js);
            found=true;
          }
        }
      }
      else
      {
        ROS_DEBUG("unable to find solutions for this seed");
      }
    }

    sols=solutions;
    return found;
  }

  bool OutboundPallet::ikForTheSlot(std::vector<Eigen::VectorXd >& sols)
  {
    m_T_w_s=m_T_w_f;
    for (unsigned int idx=0;idx<3;idx++)
    {
      ROS_INFO("position(%u)=%u (%f m)",idx,m_indexes.at(idx),m_indexes.at(idx)*m_gaps(idx));
      m_T_w_s.translation()(idx)=m_T_w_f.translation()(idx)+m_indexes.at(idx)*m_gaps(idx);
    }
    sols=m_approach_sols;
    return ik(m_T_w_s,sols);
  }

  bool OutboundPallet::ikForTheApproachSlot(std::vector<Eigen::VectorXd >& sols)
  {
    m_T_w_as=m_T_w_f;
    for (unsigned int idx=0;idx<3;idx++)
    {
      m_T_w_as.translation()(idx)=m_T_w_f.translation()(idx)+m_approach_distance(idx)+m_indexes.at(idx)*m_gaps(idx);
    }
    sols=m_approach_sols;
    return ik(m_T_w_as,sols);
  }




  moveit::planning_interface::MoveGroupInterface::Plan OutboundPallet::planToApproach(moveit::planning_interface::MoveItErrorCode& result,
                                                                                      Eigen::VectorXd& approach_jconf)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robot_state::RobotState state = *m_group->getCurrentState();

    moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name=m_group_name;
    req.start_state=plan.start_state_;

    robot_state::RobotState goal_state(m_kinematic_model);

    for (const Eigen::VectorXd goal: m_approach_sols)
    {
      goal_state.setJointGroupPositions(m_jmg, goal);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, m_jmg);
      req.goal_constraints.push_back(joint_goal);
    }
    ROS_PROTO("number of possible goals = %zu",req.goal_constraints.size());

    if (!m_planning_pipeline->generatePlan(m_planning_scene, req, res))
    {
      ROS_ERROR("Could not compute plan successfully");
      result= res.error_code_;
      return plan;

    }
    plan.planning_time_=res.planning_time_;
    ROS_PROTO("solved");

    res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);
    res.trajectory_->getLastWayPoint().copyJointGroupPositions(m_jmg,approach_jconf);
    result= res.error_code_;

    ROS_PROTO("find a solution in = %f",res.planning_time_);
    return plan;
  }

  moveit::planning_interface::MoveGroupInterface::Plan OutboundPallet::planToSlot(const Eigen::VectorXd& starting_jconf, moveit::planning_interface::MoveItErrorCode& result, Eigen::VectorXd& slot_jconf)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    robot_state::RobotState state = *m_group->getCurrentState();
    state.setJointGroupPositions(m_jmg,starting_jconf);
    moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name=m_group_name;
    req.start_state=plan.start_state_;

    robot_state::RobotState goal_state(m_kinematic_model);

    std::vector<Eigen::VectorXd> sols;
    if (!ikForTheSlot(sols))
    {
      ROS_ERROR("No Ik solution for the slot");
      result=moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION;
      return plan;
    }

    for (const Eigen::VectorXd& goal: sols)
    {

      goal_state.setJointGroupPositions(m_jmg, goal);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, m_jmg);
      req.goal_constraints.push_back(joint_goal);

    }
    ROS_PROTO("Found %zu solution",sols.size());

    if (!m_planning_pipeline->generatePlan(m_planning_scene, req, res))
    {
      ROS_ERROR("Could not compute plan successfully");
      result= res.error_code_;
      return plan;
    }
    plan.planning_time_=res.planning_time_;

    res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);

    res.trajectory_->getLastWayPoint().copyJointGroupPositions(m_jmg,slot_jconf);
    result= res.error_code_;

    return plan;
  }


  moveit::planning_interface::MoveGroupInterface::Plan OutboundPallet::planToApproachSlot(const Eigen::VectorXd& starting_jconf, moveit::planning_interface::MoveItErrorCode& result, Eigen::VectorXd& slot_jconf)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    robot_state::RobotState state = *m_group->getCurrentState();
    state.setJointGroupPositions(m_jmg,starting_jconf);
    moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name=m_group_name;
    req.start_state=plan.start_state_;

    robot_state::RobotState goal_state(m_kinematic_model);

    std::vector<Eigen::VectorXd> sols;
    if (!ikForTheApproachSlot(sols))
    {
      ROS_ERROR("No Ik solution for the slot");
      result=moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION;
      return plan;
    }

    for (const Eigen::VectorXd& goal: sols)
    {

      goal_state.setJointGroupPositions(m_jmg, goal);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, m_jmg);
      req.goal_constraints.push_back(joint_goal);

    }
    ROS_PROTO("Found %zu solution",sols.size());

    if (!m_planning_pipeline->generatePlan(m_planning_scene, req, res))
    {
      ROS_ERROR("Could not compute plan successfully");
      result= res.error_code_;
      return plan;
    }
    plan.planning_time_=res.planning_time_;

    res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);

    res.trajectory_->getLastWayPoint().copyJointGroupPositions(m_jmg,slot_jconf);
    result= res.error_code_;

    return plan;
  }


  moveit::planning_interface::MoveGroupInterface::Plan OutboundPallet::planToReturnToApproach(const Eigen::VectorXd& starting_jconf,
                                                                                              const Eigen::VectorXd& approach_jconf,
                                                                                              moveit::planning_interface::MoveItErrorCode& result)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robot_state::RobotState state = *m_group->getCurrentState();
    state.setJointGroupPositions(m_jmg,starting_jconf);
    moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name=m_group_name;
    req.start_state=plan.start_state_;

    robot_state::RobotState goal_state(m_kinematic_model);

    goal_state.setJointGroupPositions(m_jmg, approach_jconf);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, m_jmg);
    req.goal_constraints.push_back(joint_goal);

    if (!m_planning_pipeline->generatePlan(m_planning_scene, req, res))
    {
      ROS_ERROR("Could not compute plan successfully");
      result= res.error_code_;
      return plan;

    }
    plan.planning_time_=res.planning_time_;

    res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);

    return plan;
  }



  moveit::planning_interface::MoveItErrorCode OutboundPallet::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    return m_group->execute(plan);
  }

  void  OutboundPallet::wait()
  {

  }

}
