#include <inbound_pick/pick_objects.h>


namespace pickplace
{

PickObjects::PickObjects(const std::string& group_name):
  m_group_name(group_name)
{
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

  m_add_obj_srv=m_nh.advertiseService("add_objects",&PickObjects::addObjectCb,this);
  m_add_box_srv=m_nh.advertiseService("add_box",&PickObjects::addBoxCb,this);
  m_list_objects_srv=m_nh.advertiseService("list_objects",&PickObjects::listObjects,this);

  m_attach_obj_=m_nh.serviceClient<object_loader_msgs::attachObject>("attach_object_to_link");

  m_target_pub=m_nh.advertise<geometry_msgs::PoseStamped>("target",1);

  m_grasp_srv=m_nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");
  m_as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>("inbound_pick",
                                                                                     boost::bind(&PickObjects::pickObjectGoalCb,this,_1),
                                                                                     false));
  m_as->start();

}


bool PickObjects::listObjects(manipulation_msgs::ListOfObjects::Request &req, manipulation_msgs::ListOfObjects::Response &res)
{
  for (const std::pair<std::string,InboundBoxPtr>& box: m_boxes)
  {
    for (const ObjectPtr& obj: box.second->getAllObjects())
    {
      res.type.push_back(obj->getType());
      res.id.push_back(obj->getId());
      res.box.push_back(box.second->getName());
    }
  }
  return true;

}
bool PickObjects::addObjectCb(manipulation_msgs::AddObjects::Request& req,
                              manipulation_msgs::AddObjects::Response& res)
{
  for (const manipulation_msgs::Object& obj: req.add_objects)
  {
    ROS_DEBUG("adding object %s of type %s in box %s",obj.id.c_str(),obj.type.c_str(),req.inbound_box_name.c_str());
    pickplace::PosesMap poses;
    for (const manipulation_msgs::Grasp& g: obj.grasping_poses)
    {
      Eigen::Affine3d T;
      tf::poseMsgToEigen(g.pose,T);
      PosesPair p(g.tool_name,T);
      poses.insert(p);
    }
    ObjectPtr obj_ptr=createObject(obj.type,obj.id,req.inbound_box_name,poses);
    if (!obj_ptr)
      continue;

  }
  res.results=manipulation_msgs::AddObjects::Response::Success;
  return true;
}

bool PickObjects::addBoxCb(manipulation_msgs::AddBox::Request &req, manipulation_msgs::AddBox::Response &res)
{

  std::map<std::string,InboundBoxPtr>::iterator it=findBox(req.inbound_box_name);
  if (it!=m_boxes.end())
  {
    ROS_ERROR("A box names %s is already present",req.inbound_box_name.c_str());
    return false;
  }

  Eigen::Affine3d T_w_box;
  tf::poseMsgToEigen(req.box_pose,T_w_box);
  ROS_DEBUG_STREAM("Creating a box named "<<req.inbound_box_name << " in position\n" << req.box_pose << "\n with approaching heigth ="<<req.height);
  bool ok=createInboundBox(req.inbound_box_name,T_w_box,req.height);
  if (!ok)
    ROS_ERROR("unable to create the box");
  return ok;
}




moveit::planning_interface::MoveGroupInterface::Plan PickObjects::planToApproachSlot(const Eigen::VectorXd& starting_jconf, moveit::planning_interface::MoveItErrorCode& result, Eigen::VectorXd& slot_jconf)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  robot_state::RobotState state = *m_group->getCurrentState();
  state.setJointGroupPositions(m_jmg,starting_jconf);
  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name=m_group_name;
  req.start_state=plan.start_state_;
  req.allowed_planning_time=5;
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



void PickObjects::pickObjectGoalCb(const manipulation_msgs::PickObjectsGoalConstPtr &goal)
{

  manipulation_msgs::PickObjectsResult action_res;
  std::vector<std::string> type_names=goal->object_types;
  std::map<std::string,pickplace::InboundBoxPtr> possible_boxes=searchBoxWithTypes(type_names);

  ROS_PROTO("find %zu boxes",possible_boxes.size());
  if (possible_boxes.size()==0)
  {
    action_res.result=manipulation_msgs::PickObjectsResult::NoInboundBoxFound;
    ROS_ERROR("No objects found");
    m_as->setAborted(action_res,"no objects found");
    return;
  }

  moveit::planning_interface::MoveItErrorCode result;
  pickplace::InboundBoxPtr selected_box;
  Eigen::VectorXd approach_jconf;
  moveit::planning_interface::MoveGroupInterface::Plan plan=planToBestBox(possible_boxes,
                                                                          result,
                                                                          selected_box,
                                                                          approach_jconf);


  if (!result)
  {
    action_res.result=manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
    ROS_ERROR("error in plan to best box, code = %d",result.val);
    m_as->setAborted(action_res,"error in planning to box");
    return;
  }
  ROS_PROTO("plan to approach in %f second",plan.planning_time_);

  geometry_msgs::PoseStamped target;
  target.header.frame_id=world_frame;
  target.header.stamp=ros::Time::now();
  tf::poseEigenToMsg(selected_box->getApproachPose(),target.pose);
  m_target_pub.publish(target);
  execute(plan);

  pickplace::ObjectPtr selected_object;
  pickplace::GraspPosePtr selected_grasp_pose;

  moveit::planning_interface::MoveGroupInterface::Plan pick_plan=planToObject(type_names,
                                                                              selected_box,
                                                                              approach_jconf,
                                                                              tool_name,
                                                                              result,
                                                                              selected_object,
                                                                              selected_grasp_pose
                                                                              );

  if (!result)
  {
    action_res.result=manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
    ROS_ERROR("error in plan to best object in the box, code = %d",result.val);
    m_as->setAborted(action_res,"error in planning to object");
    return;
  }

  wait();

  tf::poseEigenToMsg(selected_grasp_pose->getPose(),target.pose);
  m_target_pub.publish(target);
  execute(pick_plan);
  wait();

  ros::Duration(0.5).sleep();

  object_loader_msgs::attachObject attach_srv;
  attach_srv.request.obj_id=selected_object->getId();
  attach_srv.request.link_name=selected_grasp_pose->getToolName();;
  if (!m_attach_obj_.call(attach_srv))
  {
    action_res.result=manipulation_msgs::PickObjectsResult::NoObjectsFound;
    ROS_ERROR("unaspected error calling %s service",m_attach_obj_.getService().c_str());
    m_as->setAborted(action_res,"unaspected error calling attach server");
    return;
  }
  if (!attach_srv.response.success)
  {
    action_res.result=manipulation_msgs::PickObjectsResult::NoObjectsFound;
    ROS_ERROR("unable to attach object");
    m_as->setAborted(action_res,"unable to attach object");
    return;
  }

  ROS_PROTO("attached collision object %s to tool %s",attach_srv.request.obj_id.c_str(),attach_srv.request.link_name.c_str());

  std_srvs::SetBool grasp_req;
  grasp_req.request.data=1;
  m_grasp_srv.call(grasp_req);
  ros::Duration(1).sleep();


  moveit::planning_interface::MoveGroupInterface::Plan return_plan=planToReturnToApproach(type_names,
                                                                                          selected_box,
                                                                                          approach_jconf,
                                                                                          selected_grasp_pose,
                                                                                          result
                                                                                          );


  if (!result)
  {
    action_res.result=manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
    ROS_ERROR("error in plan black to box, code = %d",result.val);
    m_as->setAborted(action_res,"error in planning back to box");
    return;
  }
    wait();
    tf::poseEigenToMsg(selected_box->getApproachPose(),target.pose);
    m_target_pub.publish(target);
    execute(return_plan);

  if (!selected_box->removeObject(selected_object->getId()))
    ROS_WARN("unable to remove object");

  action_res.object_type=selected_object->getType();
  action_res.object_id=selected_object->getId();
  action_res.result=manipulation_msgs::PickObjectsResult::Success;
  m_as->setSucceeded(action_res,"ok");
  return;

}

bool PickObjects::createInboundBox(const std::string& box_name, const Eigen::Affine3d& T_w_box, const double heigth)
{
  std::vector<Eigen::VectorXd> sols;
  InboundBoxPtr box=std::make_shared<InboundBox>(box_name,T_w_box,heigth);
  if (!ik(T_w_box,sols))
    return false;

  if (!ik(box->getApproachPose(),sols,sols.size()))
  {
    ROS_ERROR("found %zu ik solutions for approach pose of %s",sols.size(),box_name.c_str());
    return false;
  }

  box->setConfigurations(sols);
  m_boxes.insert(std::pair<std::string,InboundBoxPtr>(box_name,box));

  geometry_msgs::Pose box_pose;
  tf::poseEigenToMsg(T_w_box,box_pose);
  return true;

}

std::map<std::string,InboundBoxPtr>::iterator PickObjects::findBox(const std::string &box_name)
{
  std::map<std::string,InboundBoxPtr>::iterator it;
  it=m_boxes.find(box_name);
  return it;
}
bool PickObjects::removeInboundBox(const std::string& box_name)
{
  std::map<std::string,InboundBoxPtr>::iterator it=findBox(box_name);
  if (it==m_boxes.end())
    return false;
  m_boxes.erase(it);
  return true;

}

bool PickObjects::removeObject(const std::string& type,
                               const std::string& box_name)
{
  return false;
}

ObjectPtr PickObjects::createObject(const std::string& type,
                                    const std::string& id,
                                    const std::string& box_name,
                                    const PosesMap& poses)
{
  ObjectPtr obj = std::make_shared<Object>(type);
  obj->setId(id);

  std::vector<std::string> object_ids;
  object_ids.push_back(id);



  std::map<std::string,InboundBoxPtr>::iterator it=findBox(box_name);
  if (it==m_boxes.end())
  {
    ROS_WARN("box %s is not managed by this inbound_pick",box_name.c_str());
    obj.reset();
    return obj;
  }

  InboundBoxPtr box=findBox(box_name)->second;
  for (const auto& pose : poses)
  {
    std::vector<Eigen::VectorXd> sols=box->getConfigurations();
    if (ik(pose.second,sols))
    {
      for (const Eigen::VectorXd sol: sols)
      {
        GraspPosePtr grasp_pose=std::make_shared<GraspPose>(sol,pose.first,pose.second);
        obj->add(grasp_pose);
      }
    }
  }
  if (box->addObject(obj))
    obj.reset();
  return obj;

}



std::map<std::string,InboundBoxPtr> PickObjects::searchBoxWithType(const std::string& type_name)
{
  std::map<std::string,InboundBoxPtr> possible_boxes;

  for (const std::pair<std::string,InboundBoxPtr>& box: m_boxes)
  {
    if (box.second->getObjectsByType(type_name).size())
    {
      possible_boxes.insert(box);
    }
  }

  return possible_boxes;
}

std::map<std::string,InboundBoxPtr> PickObjects::searchBoxWithTypes(const std::vector<std::string>& type_names)
{
  std::map<std::string,InboundBoxPtr> possible_boxes;

  for (const std::pair<std::string,InboundBoxPtr>& box: m_boxes)
  {
    if (box.second->getObjectsByTypes(type_names).size())
    {
      possible_boxes.insert(box);
    }
  }
  return possible_boxes;
}

/*
bool PickObjects::ik(Eigen::Affine3d T_w_a, std::vector<Eigen::VectorXd>& sols, unsigned int ntrial)
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
      if (!state.satisfiesBounds())
      {
        ROS_DEBUG("unable to find IK solutions for this trial");
        continue;
      }
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
      ROS_DEBUG("unable to find IK solutions for this trial");
    }
  }

  sols=solutions;
  return found;
}
*/
bool PickObjects::ik(Eigen::Affine3d T_w_a, std::vector<Eigen::VectorXd>& sols, unsigned int ntrial)
{
  std::map<double,Eigen::VectorXd> solutions;
  robot_state::RobotState state = *m_group->getCurrentState();
  Eigen::VectorXd actual_configuration;
  state.copyJointGroupPositions(m_group_name,actual_configuration);
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
    sols.push_back(sol.second);
  }

  return found;
}


moveit::planning_interface::MoveGroupInterface::Plan PickObjects::planToBestBox(const std::map<std::string,InboundBoxPtr>& possible_boxes,
                                                                                moveit::planning_interface::MoveItErrorCode& result,
                                                                                InboundBoxPtr& selected_box,
                                                                                Eigen::VectorXd& jconf)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  robot_state::RobotState state = *m_group->getCurrentState();
  //  m_planning_scene->getCurrentState()

  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;

  planning_interface::MotionPlanResponse res;
  ROS_PROTO("Group name = %s",m_group_name.c_str());

  req.group_name=m_group_name;
  req.start_state=plan.start_state_;
  req.allowed_planning_time=5;

  robot_state::RobotState goal_state(m_kinematic_model);

  for (const std::pair<std::string,InboundBoxPtr>& box: possible_boxes)
  {
    std::vector<Eigen::VectorXd> box_goals=box.second->getConfigurations();
    ROS_PROTO("box %s has %zu configuration",box.first.c_str(),box_goals.size());
    for (const Eigen::VectorXd& goal: box_goals)
    {
      goal_state.setJointGroupPositions(m_jmg, goal);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, m_jmg);
      req.goal_constraints.push_back(joint_goal);
    }
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
  res.trajectory_->getLastWayPoint().copyJointGroupPositions(m_jmg,jconf);
  result= res.error_code_;

  ROS_PROTO("find a solution in = %f",res.planning_time_);

  bool found=false;
  for (const std::pair<std::string,InboundBoxPtr>& box: possible_boxes)
  {
    std::vector<Eigen::VectorXd> box_goals=box.second->getConfigurations();
    for (const Eigen::VectorXd& goal: box_goals)
    {
      if ((goal-jconf).norm()<TOLERANCE)
      {
        ROS_INFO("selected box = %s", box.first.c_str());
        selected_box=box.second;
        found=true;
        break;
      }
    }
    if (found)
      break;
  }
  if (!found)
  {
    ROS_PROTO("find a plan, but not the destination, what??");
    result=moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN;
  }
  return plan;
}

moveit::planning_interface::MoveGroupInterface::Plan PickObjects::planToObject(const std::vector<std::string>& type_name,
                                                                               const InboundBoxPtr& selected_box,
                                                                               const Eigen::VectorXd& starting_jconf,
                                                                               const std::string& tool_name,
                                                                               moveit::planning_interface::MoveItErrorCode& result,
                                                                               ObjectPtr& selected_object,
                                                                               GraspPosePtr& selected_grasp_pose
                                                                               )
{
  std::vector<ObjectPtr> objects=selected_box->getObjectsByTypes(type_name);
  ROS_PROTO("possible object %zu", objects.size());
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (objects.size()==0)
  {
    result=moveit::planning_interface::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
    return plan;
  }

  robot_state::RobotState state = *m_group->getCurrentState();
  state.setJointGroupPositions(m_jmg,starting_jconf);
  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name=m_group_name;
  req.start_state=plan.start_state_;
  req.allowed_planning_time=5;
  robot_state::RobotState goal_state(m_kinematic_model);

  for (const ObjectPtr& obj: objects)
  {

    for (const GraspPosePtr& grasp_pose: obj->getGraspPoses())
    {
      if (!grasp_pose->getToolName().compare(tool_name))
      {
        goal_state.setJointGroupPositions(m_jmg, grasp_pose->getConfiguration());
        moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, m_jmg);
        req.goal_constraints.push_back(joint_goal);
      }
    }
  }

  if (!m_planning_pipeline->generatePlan(m_planning_scene, req, res))
  {
    ROS_ERROR("Could not compute plan successfully");
    result= res.error_code_;
    return plan;
  }
  plan.planning_time_=res.planning_time_;

  res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);

  Eigen::VectorXd jconf;
  res.trajectory_->getLastWayPoint().copyJointGroupPositions(m_jmg,jconf);
  result= res.error_code_;

  bool found=false;
  for (const ObjectPtr& obj: objects)
  {
    for (const GraspPosePtr& grasp_pose: obj->getGraspPoses())
    {
      if ((grasp_pose->getConfiguration()-jconf).norm()<TOLERANCE)
      {
        ROS_INFO("selected object = %s, id = %s", obj->getType().c_str(),obj->getId().c_str());
        selected_object=obj;
        selected_grasp_pose=grasp_pose;
        found=true;
        break;
      }
    }
    if (found)
      break;
  }

  return plan;
}


moveit::planning_interface::MoveGroupInterface::Plan PickObjects::planToReturnToApproach(const std::vector<std::string>& type_name,
                                                                                         const InboundBoxPtr& selected_box,
                                                                                         const Eigen::VectorXd& approach_jconf,
                                                                                         const GraspPosePtr& selected_grasp_pose,
                                                                                         moveit::planning_interface::MoveItErrorCode& result
                                                                                         )
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  robot_state::RobotState state = *m_group->getCurrentState();
  state.setJointGroupPositions(m_jmg,selected_grasp_pose->getConfiguration());
  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name=m_group_name;
  req.start_state=plan.start_state_;
  req.allowed_planning_time=5;
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



moveit::planning_interface::MoveItErrorCode PickObjects::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  return m_group->execute(plan);
}

void  PickObjects::wait()
{

}


}
