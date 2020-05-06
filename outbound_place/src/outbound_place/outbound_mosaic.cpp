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


#include <outbound_place/outbound_mosaic.h>


namespace pickplace
{

  OutboundMosaic::OutboundMosaic(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  {
    m_nh=nh;
    m_pnh=pnh;
  }

  bool OutboundMosaic::init()
  {
    Eigen::Affine3d T_frame_slot;
    Eigen::Affine3d T_w_frame;
    Eigen::Affine3d T_w_slot;


    std::string frame;
    if (!m_pnh.getParam("frame",frame))
    {
      ROS_ERROR("parameter %s/frame is not defined",m_pnh.getNamespace().c_str());
      return false;
    }



    if (!m_pnh.getParam("group_names",m_group_names))
    {
      ROS_ERROR("parameter %s/group_names is not defined",m_pnh.getNamespace().c_str());
      return false;
    }

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0=ros::Time::now();
    if (!listener.waitForTransform("world",frame,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame.c_str());
      return false;
    }

    try{
      listener.lookupTransform("world",frame,
                               t0, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }

    tf::poseTFToEigen(transform,T_w_frame);



    XmlRpc::XmlRpcValue slots_config;
    if (!m_pnh.getParam("slots",slots_config))
    {
      ROS_ERROR("parameter %s/slots is not defined",m_pnh.getNamespace().c_str());
      return false;
    }

    if (slots_config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The param is not a list of slots" );
      return false;
    }
    ROS_DEBUG("there are %zu slots",slots_config.size());
    for(size_t i=0; i < slots_config.size(); i++)
    {
      XmlRpc::XmlRpcValue slot = slots_config[i];
      if( slot.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("The element #%zu is not a struct", i);
        continue;
      }


      if( !slot.hasMember("name") )
      {
        ROS_WARN("The element #%zu has not the field 'name'", i);
        return false;
      }
      std::string name=rosparam_utilities::toString(slot["name"]);


      std::vector<double> approach_distance_d;
      if( !rosparam_utilities::getParamVector(slot,"approach_distance",approach_distance_d) )
      {
        ROS_WARN("slot %s has not the field 'approach_distance'",name.c_str());
        return false;
      }
      assert(approach_distance_d.size()==3);
      Eigen::Vector3d approach_distance_in_frame;
      approach_distance_in_frame(0)=approach_distance_d.at(0);
      approach_distance_in_frame(1)=approach_distance_d.at(1);
      approach_distance_in_frame(2)=approach_distance_d.at(2);



      std::vector<double> position;

      if( !rosparam_utilities::getParamVector(slot,"position",position) )
      {
        ROS_WARN("slot %s has not the field 'position'",name.c_str());
        return false;
      }
      assert(position.size()==3);

      std::vector<double> quaternion;
      if( !rosparam_utilities::getParamVector(slot,"quaternion",quaternion) )
      {
        ROS_WARN("slot %s has not the field 'quaternion'",name.c_str());
        return false;
      }
      assert(quaternion.size()==4);

      Eigen::Quaterniond q(quaternion.at(3),
                           quaternion.at(0),
                           quaternion.at(1),
                           quaternion.at(2));

      T_frame_slot=q;
      T_frame_slot.translation()(0)=position.at(0);
      T_frame_slot.translation()(1)=position.at(1);
      T_frame_slot.translation()(2)=position.at(2);
      T_w_slot=T_w_frame*T_frame_slot;

      Eigen::Vector3d approach_distance_in_world=T_w_frame.linear()*approach_distance_in_frame;

      Eigen::Affine3d T_w_approach=T_w_slot;
      T_w_approach.translation()+=approach_distance_in_world;


      m_slot_map.insert(std::pair<std::string,Eigen::Affine3d>(name,T_w_slot));
      m_approach_slot_map.insert(std::pair<std::string,Eigen::Affine3d>(name,T_w_approach));
      m_slot_busy.insert(std::pair<std::string,bool>(name,false));
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

      std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as;
      as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>(m_nh,group_name+"/place",
                                                                                          boost::bind(&OutboundMosaic::placeObjectGoalCb,this,_1,group_name),
                                                                                          false));
      as->start();
      m_as.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>>(group_name,as));

      std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_ac;
      fjt_ac.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+group_name+"/follow_joint_trajectory",true));
      m_fjt_clients.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>>(group_name,fjt_ac));

      m_fjt_result.insert(std::pair<std::string,double>(group_name,0));

      std::map<std::string,std::vector<Eigen::VectorXd>> slot_configurations;
      std::map<std::string,std::vector<Eigen::VectorXd>> approach_slot_configurations;

      for (const std::pair<std::string, Eigen::Affine3d>& p: m_slot_map)
      {
        std::string place_id=p.first;
        ROS_DEBUG("Compute ik for slot %s, group=%s",place_id.c_str(),group_name.c_str());

        Eigen::Affine3d T_w_as; // world <- approach to slot
        Eigen::Affine3d T_w_s; // world <- slot
        T_w_s=m_slot_map.at(place_id);
        T_w_as=m_approach_slot_map.at(place_id);
        std::vector<Eigen::VectorXd> sols;
        std::vector<std::vector<double>> sols_stl;

        if (!m_pnh.hasParam("slot_ik/"+place_id+"/"+group_name))
        {
          // first, search solutions for the final destination
          if (!ik(group_name,T_w_s,sols))
          {
            ROS_WARN("No Ik solution for the slot %s",place_id.c_str());
            sols.clear();
            slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,sols));
            approach_slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,sols));
            continue;
          }


          sols_stl.resize(sols.size());
          for (size_t isolution=0;isolution<sols.size();isolution++)
          {
            sols_stl.at(isolution).resize(sols.at(isolution).size());
            for (size_t iax=0;iax<sols.at(isolution).size();iax++)
              sols_stl.at(isolution).at(iax)=sols.at(isolution)(iax);
          }
          rosparam_utilities::setParam(m_pnh,"slot_ik/"+place_id+"/"+group_name,sols_stl);
          ROS_DEBUG("Find %zu solutions to slot %s (group=%s)",sols.size(),place_id.c_str(),group_name.c_str());
        }
        else
        {
          if (!rosparam_utilities::getParamMatrix(m_pnh,"slot_ik/"+place_id+"/"+group_name,sols_stl))
          {
            ROS_ERROR("parameter %s/slot_ik/%s/%s is not correct",m_pnh.getNamespace().c_str(),place_id.c_str(),group_name.c_str());
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
        slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,sols));


        if (!m_pnh.hasParam("approach_ik/"+place_id+"/"+group_name))
        {


          // then,search solutions for the approach pose
          if (!ik(group_name,T_w_as,sols,sols.size()))
          {
            ROS_WARN("No Ik solution for the approach to slot %s",place_id.c_str());
            sols.clear();
            slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,sols));
            approach_slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,sols));
            continue;
          }
          ROS_DEBUG("Find %zu solutions to approach the slot %s (group=%s)",sols.size(),place_id.c_str(),group_name.c_str());

          sols_stl.resize(sols.size());
          for (size_t isolution=0;isolution<sols.size();isolution++)
          {
            sols_stl.at(isolution).resize(sols.at(isolution).size());
            for (size_t iax=0;iax<sols.at(isolution).size();iax++)
              sols_stl.at(isolution).at(iax)=sols.at(isolution)(iax);
          }
          rosparam_utilities::setParam(m_pnh,"approach_ik/"+place_id+"/"+group_name,sols_stl);
        }
        else
        {
          if (!rosparam_utilities::getParamMatrix(m_pnh,"approach_ik/"+place_id+"/"+group_name,sols_stl))
          {
            ROS_ERROR("parameter %s/approach_ik/%s/%s is not correct",m_pnh.getNamespace().c_str(),place_id.c_str(),group_name.c_str());
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
        approach_slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,sols));

      }
      m_slot_configurations.insert(std::pair<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>>(group_name,slot_configurations));
      m_approach_slot_configurations.insert(std::pair<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>>(group_name,approach_slot_configurations));


    }

    m_target_pub=m_nh.advertise<geometry_msgs::PoseStamped>("target",1);
    m_grasp_srv=m_nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");
    m_detach_object_srv=m_nh.serviceClient<object_loader_msgs::detachObject>("detach_object_to_link");

    m_init=true;


    return true;

  }

  void OutboundMosaic::placeObjectGoalCb(const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                                         const std::string& group_name)
  {
    std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as=m_as.at(group_name);

    manipulation_msgs::PlaceObjectsResult action_res;
    std::string place_id=goal->place_id;
    moveit::planning_interface::MoveItErrorCode result;


    if (!m_init)
    {
      ROS_ERROR("outbound pallet is not initialized well");
      action_res.result=manipulation_msgs::PlaceObjectsResult::NotInitialized;
      as->setAborted(action_res,"not initialized");
      return;
    }

    if (m_slot_map.find(place_id)==m_slot_map.end())
    {
      ROS_ERROR("slot %s is not managed by the outbound_mosaic",place_id.c_str());
      action_res.result=manipulation_msgs::PlaceObjectsResult::NotInitialized;
      as->setAborted(action_res,"slot not managed");
      return;
    }
    if (m_slot_busy.find(place_id)==m_slot_busy.end())
    {
      ROS_ERROR("slot %s is not managed by the outbound_mosaic",place_id.c_str());
      action_res.result=manipulation_msgs::PlaceObjectsResult::NotInitialized;
      as->setAborted(action_res,"slot not managed");
      return;
    }

    if (m_slot_busy.at(place_id))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::Full;
      ROS_ERROR("slot %s is already used",place_id.c_str());
      as->setAborted(action_res,"slot is already used");
      return;
    }
    m_slot_busy.at(place_id)=true;

    moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
    group->setStartState(*group->getCurrentState());
    moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);


    Eigen::Affine3d T_w_as; // world <- approach to slot
    Eigen::Affine3d T_w_s; // world <- slot
    T_w_s=m_slot_map.at(place_id);
    T_w_as=m_approach_slot_map.at(place_id);


    geometry_msgs::PoseStamped target;
    target.header.frame_id="world";
    target.header.stamp=ros::Time::now();

    ROS_PROTO("plannig to approach");

    Eigen::VectorXd actual_jconf;
    group->getCurrentState()->copyJointGroupPositions(group_name,actual_jconf);
    Eigen::VectorXd approach_slot_jconf;
    moveit::planning_interface::MoveGroupInterface::Plan approac_pick_plan=planToApproachSlot(group_name,
                                                                                              place_id,
                                                                                              actual_jconf,
                                                                                              result,
                                                                                              approach_slot_jconf);

    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan for placing slot, code = %d",result.val);
      as->setAborted(action_res,"error in planning for placing");
      return;
    }

    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      return;
    }
    tf::poseEigenToMsg(T_w_as,target.pose);
    m_target_pub.publish(target);
    execute(group_name,approac_pick_plan);


    Eigen::VectorXd slot_jconf;
    moveit::planning_interface::MoveGroupInterface::Plan plan_plan=planToSlot(group_name,
                                                                              place_id,
                                                                              approach_slot_jconf,
                                                                              result,
                                                                              slot_jconf);

    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan for placing slot, code = %d",result.val);
      as->setAborted(action_res,"error in planning for placing");
      return;
    }

    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      return;
    }
    tf::poseEigenToMsg(T_w_s,target.pose);
    m_target_pub.publish(target);
    execute(group_name,plan_plan);
    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      return;
    }

    ros::Duration(0.5).sleep();

    object_loader_msgs::detachObject detach_srv;
    detach_srv.request.obj_id=goal->object_id;
    if (!m_detach_object_srv.call(detach_srv))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::SceneError;
      ROS_ERROR("unaspected error calling %s service",m_detach_object_srv.getService().c_str());
      as->setAborted(action_res,"unaspected error calling detach server");
      return;
    }
    if (!detach_srv.response.success)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::SceneError;
      ROS_ERROR("unable to detach object id %s",goal->object_id.c_str());
      as->setAborted(action_res,"unable to attach object");
      return;
    }

    ROS_PROTO("detached collision object %s ",detach_srv.request.obj_id.c_str());


    std_srvs::SetBool grasp_req;
    grasp_req.request.data=0;
    m_grasp_srv.call(grasp_req);
    ros::Duration(1).sleep();


    moveit::planning_interface::MoveGroupInterface::Plan return_plan=planToApproachSlot(group_name,
                                                                                        place_id,
                                                                                        slot_jconf,
                                                                                        result,
                                                                                        approach_slot_jconf);

    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan black to box, code = %d",result.val);
      as->setAborted(action_res,"error in planning back to box");
      return;
    }

    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      return;
    }
    tf::poseEigenToMsg(T_w_as,target.pose);
    m_target_pub.publish(target);
    execute(group_name,return_plan);

    action_res.result=manipulation_msgs::PlaceObjectsResult::Success;
    as->setSucceeded(action_res,"ok");

    return;

  }



  bool OutboundMosaic::ik(const std::string& group_name,
                          const Eigen::Affine3d& T_w_a,
                          std::vector<Eigen::VectorXd>& sols,
                          unsigned int ntrial)
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
      if (solutions.size()>=N_ITER)
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
        if (!m_planning_scene->isStateValid(state))
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
      sols.push_back(sol.second);
    }

    return found;
  }




  moveit::planning_interface::MoveGroupInterface::Plan OutboundMosaic::planToSlot(const std::string& group_name,
                                                                                  const std::string& place_id,
                                                                                  const Eigen::VectorXd& starting_jconf,
                                                                                  moveit::planning_interface::MoveItErrorCode& result,
                                                                                  Eigen::VectorXd& slot_jconf)
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

    std::vector<Eigen::VectorXd> sols=m_approach_slot_configurations.at(group_name).at(place_id);


    for (const Eigen::VectorXd& goal: sols)
    {

      goal_state.setJointGroupPositions(jmg, goal);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
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

    res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,slot_jconf);
    result= res.error_code_;

    return plan;
  }


  moveit::planning_interface::MoveGroupInterface::Plan OutboundMosaic::planToApproachSlot(const std::string& group_name,
                                                                                          const std::string& place_id,
                                                                                          const Eigen::VectorXd& starting_jconf,
                                                                                          moveit::planning_interface::MoveItErrorCode& result,
                                                                                          Eigen::VectorXd& slot_jconf)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
    moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);

    robot_state::RobotState state = *group->getCurrentState();
    moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name=group_name;
    req.start_state=plan.start_state_;
    req.allowed_planning_time=5;
    robot_state::RobotState goal_state(m_kinematic_model);

    std::vector<Eigen::VectorXd> sols=m_approach_slot_configurations.at(group_name).at(place_id);

    for (const Eigen::VectorXd& goal: sols)
    {

      goal_state.setJointGroupPositions(jmg, goal);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
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
    if (res.trajectory_->getWayPointCount()==0)
    {
      ROS_WARN("trajectory with 0 waypoint");
    }
    else
      res.trajectory_->getLastWayPoint().copyJointGroupPositions(jmg,slot_jconf);
    result= res.error_code_;

    return plan;
  }


  bool OutboundMosaic::execute(const std::string& group_name,
                               const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory=plan.trajectory_.joint_trajectory;

    auto cb=boost::bind(&pickplace::OutboundMosaic::doneCb,this,_1,_2,group_name);
    m_fjt_clients.at(group_name)->sendGoal(goal,
                                           cb);

    m_fjt_result.at(group_name)=std::nan("1");;

    return true;
  }

  bool OutboundMosaic::wait(const std::string& group_name)
  {
    while (std::isnan(m_fjt_result.at(group_name)))
      ros::Duration(0.01).sleep();
    return !std::isnan(m_fjt_result.at(group_name));
  }

  void OutboundMosaic::doneCb(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &group_name)
  {
    m_fjt_result.at(group_name)=result->error_code;
    if (result->error_code<0)
    {
      ROS_ERROR("error executing %s/follow_joint_trajectory: %s",group_name.c_str(),result->error_string.c_str());
    }
    return;
  }


}
