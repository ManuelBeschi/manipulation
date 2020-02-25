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
  planning_scene_diff_publisher = m_nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

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
  ROS_INFO_STREAM("add objects\n"<<req);
  for (const manipulation_msgs::Object& obj: req.add_objects)
  {
    ROS_INFO_STREAM("object:\n"<<obj);
    pickplace::PosesMap poses;
    for (const manipulation_msgs::Grasp& g: obj.grasping_poses)
    {
      Eigen::Affine3d T;
      tf::poseMsgToEigen(g.pose,T);
      PosesPair p(g.tool_name,T);
      poses.insert(p);
    }
    ObjectPtr obj_ptr=createObject(obj.type,req.inbound_box_name,poses);
    if (!addCollisionObject(obj_ptr,obj.pose))
    {
      ROS_WARN("Unable to add collision object");
    }
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
  ROS_INFO_STREAM("Creating a box named "<<req.inbound_box_name << " in position\n" << req.box_pose << "\n with approaching heigth ="<<req.height);
  bool ok=createInboundBox(req.inbound_box_name,T_w_box,req.height);
  if (!ok)
    ROS_ERROR("unable to create the box");
  return ok;
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
  if (!selected_box->removeObject(selected_object->getId()))
    ROS_WARN("unable to remove object");

  wait();
  tf::poseEigenToMsg(selected_box->getApproachPose(),target.pose);
  m_target_pub.publish(target);
  execute(return_plan);

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
    ROS_ERROR("found %zu ik solutions for approach",sols.size());
    return false;
  }

  box->setConfigurations(sols);
  m_boxes.insert(std::pair<std::string,InboundBoxPtr>(box_name,box));

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
                                    const std::string& box_name,
                                    const PosesMap& poses)
{
  ObjectPtr obj = std::make_shared<Object>(type);
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
  box->addObject(obj);
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
      ROS_WARN("unable to find any solutions");
    }
  }

  sols=solutions;
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
  req.group_name=m_group_name;
  req.start_state=plan.start_state_;

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

bool PickObjects::addCollisionObject(const ObjectPtr& obj, const geometry_msgs::Pose& obj_pose)
{
  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.05;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = 0.05;

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = world_frame;
  attached_object.object.header.frame_id = world_frame;
  attached_object.object.id = obj->getId();


  attached_object.object.primitives.push_back(primitive);


  attached_object.object.primitive_poses.push_back(obj_pose);

  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  return true;
}



}
