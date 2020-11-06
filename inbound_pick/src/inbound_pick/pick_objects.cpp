#include <inbound_pick/pick_objects.h>


namespace pickplace
{

PickObjects::PickObjects(const ros::NodeHandle& nh, const ros::NodeHandle& pnh):
  m_nh(nh),
  m_pnh(pnh)
{

}

bool PickObjects::init()
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
  m_reset_box_srv=m_nh.advertiseService("inbound/reset_box",&PickObjects::resetBoxesCb,this);

  m_attach_obj_=m_nh.serviceClient<object_loader_msgs::attachObject>("attach_object_to_link");

  m_target_pub=m_nh.advertise<geometry_msgs::PoseStamped>("target",1);

  m_grasp_srv=m_nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");

  for (const std::string& group_name: m_group_names)
  {
    m_pick_servers.at(group_name)->start();
  }
  return true;
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

moveit::planning_interface::MoveGroupInterface::Plan PickObjects::planToApproachSlot(const std::string& group_name,
                                                                                     const Eigen::VectorXd& starting_jconf,
                                                                                     const Eigen::Affine3d& approach_pose,
                                                                                     const ObjectPtr& selected_object,
                                                                                     const GraspPosePtr& selected_grasp_pose,
                                                                                     moveit::planning_interface::MoveItErrorCode& result,
                                                                                     Eigen::VectorXd& slot_jconf)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
  int max_ik_goal_number=m_max_ik_goal_number.at(group_name);


  if (!group->startStateMonitor(2))
  {
    ROS_ERROR("unable to get actual state",m_pnh.getNamespace().c_str());
    result=moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA;
    return plan;
  }
  robot_state::RobotState state = *group->getCurrentState();
  state.setJointGroupPositions(jmg,starting_jconf);
  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name=group_name;
  req.start_state=plan.start_state_;
  req.allowed_planning_time=5;
  robot_state::RobotState goal_state(m_kinematic_model);


  std::vector<Eigen::VectorXd> sols=selected_grasp_pose->getApproachConfiguration();
  //  for (const GraspPosePtr& grasp_pose: selected_object->getGraspPoses(group_name))
  //  {
  //    if (!grasp_pose->getToolName().compare(tool_name))
  //    {
  //      goal_state.setJointGroupPositions(jmg, grasp_pose->getConfiguration());
  //      goal_state.updateCollisionBodyTransforms();
  //      if (!m_planning_scene.at(group_name)->isStateValid(goal_state))
  //        continue;
  //      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
  //      req.goal_constraints.push_back(joint_goal);
  //      if (ik_goal++>=max_ik_goal_number)
  //        break;

  //    }
  //  }

  if (!ik(group_name,approach_pose,sols))
  {
    ROS_ERROR("No Ik solution for approach");
    result=moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION;
    return plan;
  }

  m_mtx.lock();
  planning_scene::PlanningScenePtr planning_scene= planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));
  m_mtx.unlock();
  for (const Eigen::VectorXd& goal: sols)
  {
    if (req.goal_constraints.size()>=max_ik_goal_number)
      break;
    goal_state.setJointGroupPositions(jmg, goal);
    goal_state.updateCollisionBodyTransforms();
    m_mtx.lock();
    if (!planning_scene->isStateValid(goal_state))
    {
      m_mtx.unlock();
      continue;
    }
    m_mtx.unlock();
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
    req.goal_constraints.push_back(joint_goal);

  }
  ROS_DEBUG("Found %zu solutions for approach slots",sols.size());
  if (req.goal_constraints.size()==0)
  {
    ROS_ERROR("Inbound server: no valid goals");
    result= res.error_code_;
    return plan;

  }

  ROS_PROTO("Plan approach for group %s",group_name.c_str());
  m_mtx.lock();
  if (!m_planning_pipeline.at(group_name)->generatePlan(planning_scene, req, res))
  {
    ROS_ERROR("Could not compute plan successfully");
    result= res.error_code_;
    m_mtx.unlock();
    return plan;
  }
  m_mtx.unlock();

  plan.planning_time_=res.planning_time_;

  res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);

  res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,slot_jconf);
  result= res.error_code_;

  return plan;
}

void PickObjects::pickObjectGoalCb(const manipulation_msgs::PickObjectsGoalConstPtr &goal,
                                   const std::string& group_name)
{

  manipulation_msgs::PickObjectsResult action_res;
  std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>> as=m_pick_servers.at(group_name);
  try
  {
    ros::Time t_start=ros::Time::now();

    std::vector<std::string> type_names=goal->object_types;
    m_mtx.lock();
    std::map<std::string,pickplace::InboundBoxPtr> possible_boxes=searchBoxWithTypes(type_names);
    m_mtx.unlock();
    std::string tool_name=m_tool_names.at(group_name);

    ROS_PROTO("find %zu boxes",possible_boxes.size());
    if (possible_boxes.size()==0)
    {
      action_res.result=manipulation_msgs::PickObjectsResult::NoInboundBoxFound;
      ROS_ERROR("No objects found");
      as->setAborted(action_res,"no objects found");
      return;
    }


    moveit::planning_interface::MoveItErrorCode result;
    pickplace::InboundBoxPtr selected_box;
    Eigen::VectorXd approach_jconf;

    ros::Time t_planning_init=ros::Time::now();

    moveit::planning_interface::MoveGroupInterface::Plan plan=planToBestBox(group_name,
                                                                            possible_boxes,
                                                                            result,
                                                                            selected_box,
                                                                            approach_jconf);



    if (!result)
    {
      action_res.result=manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan to best box, code = %d",result.val);
      as->setAborted(action_res,"error in planning to box");
      return;
    }
    ROS_PROTO("Group=%s: plan to approach in %f second",group_name.c_str(),plan.planning_time_);
    ros::Time t_planning=ros::Time::now();
    action_res.planning_duration+=(t_planning-t_planning_init);
    action_res.expected_execution_duration+=plan.trajectory_.joint_trajectory.points.back().time_from_start;

    geometry_msgs::PoseStamped target;
    target.header.frame_id=world_frame;
    target.header.stamp=ros::Time::now();
    tf::poseEigenToMsg(selected_box->getApproachPose(),target.pose);
    m_target_pub.publish(target);
    execute(group_name,
            plan);

//    if (!wait(group_name))
//    {
//      action_res.result=manipulation_msgs::PickObjectsResult::TrajectoryError;
//      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
//      as->setAborted(action_res,"error in trajectory execution");
//      // readd object to box
//      return;
//    }

    pickplace::ObjectPtr selected_object;
    pickplace::GraspPosePtr selected_grasp_pose;

    t_planning_init=ros::Time::now();
    selected_box->getMutex().lock();
    moveit::planning_interface::MoveGroupInterface::Plan pick_plan=planToObject(group_name,
                                                                                type_names,
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
      ROS_ERROR("Group=%s: error in plan to best object in the box, code = %d",group_name.c_str(),result.val);
      as->setAborted(action_res,"error in planning to object");
      selected_box->getMutex().unlock();
      return;
    }
    t_planning=ros::Time::now();
    action_res.planning_duration+=(t_planning-t_planning_init);
    action_res.expected_execution_duration+=pick_plan.trajectory_.joint_trajectory.points.back().time_from_start;

    if (!selected_box->removeObject(selected_object->getId()))
    {
      ROS_WARN("unable to remove object");
    }
    selected_box->getMutex().unlock();

// POSSIBILE FONTE DEL FAULT?
    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PickObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      // readd object to box
      selected_box->addObject(selected_object);
      return;
    }

    tf::poseEigenToMsg(selected_grasp_pose->getPose(),target.pose);
    m_target_pub.publish(target);
    execute(group_name,
            pick_plan);

    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PickObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      // readd object to box
      selected_box->addObject(selected_object);
      return;
    }

    ros::Time t_grasp_init=ros::Time::now();
    ros::Duration(0.5).sleep();

    object_loader_msgs::attachObject attach_srv;
    attach_srv.request.obj_id=selected_object->getId();
    attach_srv.request.link_name=selected_grasp_pose->getToolName();;
    if (!m_attach_obj_.call(attach_srv))
    {
      action_res.result=manipulation_msgs::PickObjectsResult::NoObjectsFound;
      ROS_ERROR("unaspected error calling %s service",m_attach_obj_.getService().c_str());
      as->setAborted(action_res,"unaspected error calling attach server");
      return;
    }
    if (!attach_srv.response.success)
    {
      action_res.result=manipulation_msgs::PickObjectsResult::NoObjectsFound;
      ROS_ERROR("unable to attach object");
      as->setAborted(action_res,"unable to attach object");
      return;
    }

    ROS_PROTO("Group=%s: attached collision object %s to tool %s",group_name.c_str(),attach_srv.request.obj_id.c_str(),attach_srv.request.link_name.c_str());

    std_srvs::SetBool grasp_req;
    grasp_req.request.data=1;
    m_grasp_srv.call(grasp_req);
    ros::Duration(1).sleep();

    action_res.grasping_object_duration=(ros::Time::now()-t_grasp_init);

    Eigen::Affine3d T_w_approach=selected_grasp_pose->getPose();
    T_w_approach.translation()(2)+=selected_box->getHeight();

    t_planning_init=ros::Time::now();
    ROS_PROTO("planning to approach. Group=%s",group_name.c_str());
    moveit::planning_interface::MoveGroupInterface::Plan return_plan=planToApproachSlot(group_name,
                                                                                        selected_grasp_pose->getConfiguration(),
                                                                                        T_w_approach,
                                                                                        selected_object,
                                                                                        selected_grasp_pose,
                                                                                        result,
                                                                                        approach_jconf);


    if (!result)
    {
      action_res.result=manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan black to box, code = %d",result.val);
      as->setAborted(action_res,"error in planning back to box");
      return;
    }
    t_planning=ros::Time::now();
    action_res.planning_duration+=(t_planning-t_planning_init);
    action_res.expected_execution_duration+=return_plan.trajectory_.joint_trajectory.points.back().time_from_start;


//    ROS_PROTO("waiting to execute trj");
//    if (!wait(group_name))
//    {
//      action_res.result=manipulation_msgs::PickObjectsResult::TrajectoryError;
//      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
//      as->setAborted(action_res,"error in trajectory execution");
//      return;
//    }
    tf::poseEigenToMsg(T_w_approach,target.pose);
    m_target_pub.publish(target);
    ROS_PROTO("execute trj");
    execute(group_name,
            return_plan);


    ROS_PROTO("waiting to execute trj");
    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PickObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      return;
    }
    action_res.object_type=selected_object->getType();
    action_res.object_id=selected_object->getId();
    action_res.result=manipulation_msgs::PickObjectsResult::Success;

    action_res.actual_duration+=(ros::Time::now()-t_start);
    as->setSucceeded(action_res,"ok");
    return;
  }
  catch( std::exception& ex)
  {
    ROS_ERROR("exception: %s",ex.what());
    action_res.result=manipulation_msgs::PickObjectsResult::UnexpectedError;
    as->setAborted(action_res,"exeception");
    return;
  }
}

bool PickObjects::createInboundBox(const std::string& box_name, const Eigen::Affine3d& T_w_box, const double heigth)
{
  InboundBoxPtr box=std::make_shared<InboundBox>(box_name,T_w_box,heigth);
  for (const std::string& group_name: m_group_names)
  {
    std::vector<Eigen::VectorXd> sols;
    std::vector<std::vector<double>> sols_stl;

    if (!m_pnh.hasParam("box_ik/"+box_name+"/"+group_name))
    {

      if (!ik(group_name,T_w_box,sols))
      {
        ROS_WARN("found %zu ik solutions of pose of %s for group %s",sols.size(),box_name.c_str(),group_name.c_str());
        sols.clear();
      }
      else if (!ik(group_name,box->getApproachPose(),sols,sols.size()))
      {
        ROS_WARN("found %zu ik solutions of approach pose of %s for group %s",sols.size(),box_name.c_str(),group_name.c_str());
        sols.clear();
      }

      sols_stl.resize(sols.size());
      for (size_t isolution=0;isolution<sols.size();isolution++)
      {
        sols_stl.at(isolution).resize(sols.at(isolution).size());
        for (size_t iax=0;iax<sols.at(isolution).size();iax++)
          sols_stl.at(isolution).at(iax)=sols.at(isolution)(iax);
      }
      rosparam_utilities::setParam(m_pnh,"box_ik/"+box_name+"/"+group_name,sols_stl);
    }
    else
    {
      if (!rosparam_utilities::getParamMatrix(m_pnh,"box_ik/"+box_name+"/"+group_name,sols_stl))
      {
        ROS_ERROR("parameter %s/box_ik/%s/%s is not correct",m_pnh.getNamespace().c_str(),box_name.c_str(),group_name.c_str());
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

    box->setConfigurations(group_name,sols);
  }

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
  for (const std::string& group_name: m_group_names)
  {
    ROS_INFO("adding object %s to box %s for group %s",obj->getType().c_str(),box->getName().c_str(),group_name.c_str());
    for (const auto& pose : poses)
    {
      if (m_tool_names.at(group_name).compare(pose.first))
        continue;

      Eigen::Affine3d T_w_approach=pose.second;
      T_w_approach.translation()(2)+=box->getHeight();

      std::vector<Eigen::VectorXd> approach_sols=box->getConfigurations(group_name);
      if (!ik(group_name,T_w_approach,approach_sols,approach_sols.size()))
      {
        ROS_INFO("no solutions to approach slots");
        continue;
      }
      std::vector<Eigen::VectorXd> sols=approach_sols;

      if (ik(group_name,pose.second,sols,sols.size()))
      {
        for (const Eigen::VectorXd sol: sols)
        {

          GraspPosePtr grasp_pose=std::make_shared<GraspPose>(sol,approach_sols,pose.first,pose.second,T_w_approach);
          obj->add(group_name,grasp_pose);


        }
      }
    }
  }
  if (!box->addObject(obj))
    obj.reset();

  ROS_INFO("add object %s to box %s",obj->getType().c_str(),box->getName().c_str());
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

bool PickObjects::ik(const std::string& group_name, Eigen::Affine3d T_w_a, std::vector<Eigen::VectorXd>& sols, unsigned int ntrial)
{
  std::map<double,Eigen::VectorXd> solutions;
  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
  robot_state::RobotState state = *group->getCurrentState();
  m_mtx.lock();
  planning_scene::PlanningScenePtr planning_scene= planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));
  m_mtx.unlock();

  Eigen::VectorXd preferred_configuration(jmg->getActiveJointModels().size());
  Eigen::VectorXd preferred_configuration_weight(jmg->getActiveJointModels().size(),1);

  if (m_preferred_configuration.count(group_name)>0)
  {
    preferred_configuration=m_preferred_configuration.at(group_name);
    preferred_configuration_weight=m_preferred_configuration_weight.at(group_name);
  }
  else
  {
    state.copyJointGroupPositions(group_name,preferred_configuration);
  }

  unsigned int n_seed=sols.size();
  bool found=false;

  for (unsigned int iter=0;iter<N_MAX_ITER;iter++)
  {
    if (solutions.size()>=ntrial)
      break;
    if (solutions.size()==0 && iter>ntrial*5)
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
      m_mtx.lock();
      if (!planning_scene->isStateValid(state))
      {
        m_mtx.unlock();
        continue;
      }
      m_mtx.unlock();

      Eigen::VectorXd js;
      state.copyJointGroupPositions(group_name,js);
      double dist=(preferred_configuration_weight.cwiseProduct(js-preferred_configuration)).norm();
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

moveit::planning_interface::MoveGroupInterface::Plan PickObjects::planToBestBox(const std::string& group_name,
                                                                                const std::map<std::string,InboundBoxPtr>& possible_boxes,
                                                                                moveit::planning_interface::MoveItErrorCode& result,
                                                                                InboundBoxPtr& selected_box,
                                                                                Eigen::VectorXd& jconf)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);

  if (!group->startStateMonitor(2))
  {
    ROS_ERROR("unable to get actual state",m_pnh.getNamespace().c_str());
    result=moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA;
    return plan;
  }

  robot_state::RobotState state = *group->getCurrentState();
  int max_ik_goal_number=m_max_ik_goal_number.at(group_name);
  //  m_planning_scene->getCurrentState()

  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;

  planning_interface::MotionPlanResponse res;
  ROS_PROTO("Group name = %s",group_name.c_str());

  req.group_name=group_name;
  req.start_state=plan.start_state_;
  req.allowed_planning_time=5;

  robot_state::RobotState goal_state(m_kinematic_model);

  m_mtx.lock();
  planning_scene::PlanningScenePtr planning_scene= planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));
  m_mtx.unlock();

  for (const std::pair<std::string,InboundBoxPtr>& box: possible_boxes)
  {
    std::vector<Eigen::VectorXd> box_goals=box.second->getConfigurations(group_name);
    ROS_PROTO("box %s has %zu configuration",box.first.c_str(),box_goals.size());
    int ik_goal=0;
    for (const Eigen::VectorXd& goal: box_goals)
    {
      if (ik_goal++>=max_ik_goal_number)
        break;
      goal_state.setJointGroupPositions(jmg, goal);
      goal_state.updateCollisionBodyTransforms();
      m_mtx.lock();
      if (!planning_scene->isStateValid(goal_state))
      {
        m_mtx.unlock();
        continue;
      }
      m_mtx.unlock();
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
      req.goal_constraints.push_back(joint_goal);
    }
  }
  if (req.goal_constraints.size()==0)
  {
    ROS_ERROR("Inbound server: no valid goals");
    result= res.error_code_;
    return plan;

  }

  ROS_PROTO("number of possible goals = %zu",req.goal_constraints.size());

  m_mtx.lock();
  if (!m_planning_pipeline.at(group_name)->generatePlan(planning_scene, req, res))
  {
    ROS_ERROR("Could not compute plan successfully");
    m_mtx.unlock();
    result=moveit::planning_interface::MoveItErrorCode::FAILURE;
    return plan;

  }
  m_mtx.unlock();
  plan.planning_time_=res.planning_time_;
  ROS_PROTO("solved");

  res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);
  res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,jconf);
  result= res.error_code_;

  ROS_PROTO("find a solution in = %f",res.planning_time_);

  bool found=false;
  for (const std::pair<std::string,InboundBoxPtr>& box: possible_boxes)
  {
    std::vector<Eigen::VectorXd> box_goals=box.second->getConfigurations(group_name);
    for (const Eigen::VectorXd& goal: box_goals)
    {
      if ((goal-jconf).norm()<TOLERANCE)
      {
        ROS_DEBUG("selected box = %s", box.first.c_str());
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

moveit::planning_interface::MoveGroupInterface::Plan PickObjects::planToObject(const std::string& group_name,
                                                                               const std::vector<std::string>& type_name,
                                                                               const InboundBoxPtr& selected_box,
                                                                               const Eigen::VectorXd& starting_jconf,
                                                                               const std::string& tool_name,
                                                                               moveit::planning_interface::MoveItErrorCode& result,
                                                                               ObjectPtr& selected_object,
                                                                               GraspPosePtr& selected_grasp_pose
                                                                               )
{
  std::vector<ObjectPtr> objects=selected_box->getObjectsByTypes(type_name);
  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
  ROS_PROTO("possible object %zu", objects.size());
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  int max_ik_goal_number=m_max_ik_goal_number.at(group_name);


  if (!group->startStateMonitor(2))
  {
    ROS_ERROR("unable to get actual state",m_pnh.getNamespace().c_str());
    result=moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA;
    return plan;
  }
  if (objects.size()==0)
  {
    result=moveit::planning_interface::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS;
    return plan;
  }

  robot_state::RobotState state = *group->getCurrentState();
  state.setJointGroupPositions(jmg,starting_jconf);
  moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name=group_name;
  req.start_state=plan.start_state_;
  req.allowed_planning_time=5;
  robot_state::RobotState goal_state(m_kinematic_model);

  m_mtx.lock();
  planning_scene::PlanningScenePtr planning_scene= planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));
  m_mtx.unlock();

  for (const ObjectPtr& obj: objects)
  {
    int ik_goal=0;
    for (const GraspPosePtr& grasp_pose: obj->getGraspPoses(group_name))
    {
      if (!grasp_pose->getToolName().compare(tool_name))
      {
        goal_state.setJointGroupPositions(jmg, grasp_pose->getConfiguration());
        goal_state.updateCollisionBodyTransforms();
        m_mtx.lock();
        if (!planning_scene->isStateValid(goal_state))
        {
          m_mtx.unlock();
          continue;
        }
        m_mtx.unlock();

        moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
        req.goal_constraints.push_back(joint_goal);
        if (ik_goal++>=max_ik_goal_number)
          break;

      }
    }
  }
  if (req.goal_constraints.size()==0)
  {
    ROS_ERROR("Inbound server: no valid goals");
    result= res.error_code_;
    return plan;

  }

  m_mtx.lock();
  if (!m_planning_pipeline.at(group_name)->generatePlan(planning_scene, req, res))
  {
    ROS_ERROR("Could not compute plan successfully");
    result= res.error_code_;
    m_mtx.unlock();
    return plan;
  }
  m_mtx.unlock();
  plan.planning_time_=res.planning_time_;

  res.trajectory_->getRobotTrajectoryMsg(plan.trajectory_);

  Eigen::VectorXd jconf;
  res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,jconf);
  result= res.error_code_;

  bool found=false;
  for (const ObjectPtr& obj: objects)
  {
    for (const GraspPosePtr& grasp_pose: obj->getGraspPoses(group_name))
    {
      if ((grasp_pose->getConfiguration()-jconf).norm()<TOLERANCE)
      {
        ROS_DEBUG("selected object = %s, id = %s, in box %s", obj->getType().c_str(),obj->getId().c_str(),selected_box->getName().c_str());
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



bool PickObjects::execute(const std::string& group_name, const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory=plan.trajectory_.joint_trajectory;

  auto cb=boost::bind(&pickplace::PickObjects::doneCb,this,_1,_2,group_name);
  m_fjt_clients.at(group_name)->sendGoal(goal,
                                         cb);

  m_fjt_result.at(group_name)=std::nan("1");;

  return true;
}

void PickObjects::doneCb(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &group_name)
{
  m_fjt_result.at(group_name)=result->error_code;
  if (result->error_code<0)
  {
    ROS_ERROR("error executing %s/follow_joint_trajectory: %s",group_name.c_str(),result->error_string.c_str());
  }
  return;
}

bool  PickObjects::wait(const std::string& group_name)
{
  ros::Time t0=ros::Time::now();
  while (std::isnan(m_fjt_result.at(group_name)))
  {
    ros::Duration(0.01).sleep();

    if ((ros::Time::now()-t0).toSec()>600)
    {
      ROS_ERROR("%s is waiting more than 10 minutes, stop it",group_name.c_str());
      return false;
    }
    if ((ros::Time::now()-t0).toSec()>10)
      ROS_WARN_THROTTLE(10,"%s is waiting for %f seconds",group_name.c_str(),(ros::Time::now()-t0).toSec());
  }
  return !std::isnan(m_fjt_result.at(group_name));
}

bool PickObjects::resetBoxesCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (req.data)
  {
    for (std::pair<std::string,InboundBoxPtr> box: m_boxes)
    {
      box.second->getMutex().lock();
      box.second->removeAllObjects();
      box.second->getMutex().unlock();
    }
  }
  return true;
}

}
