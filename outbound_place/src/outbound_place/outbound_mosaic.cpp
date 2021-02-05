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

    m_display_publisher = m_nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    std::string frame;
    if (!m_pnh.getParam("frame",frame))
    {
      ROS_ERROR("parameter %s/frame is not defined",m_pnh.getNamespace().c_str());
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


    if (!m_pnh.getParam("ik_sol_number",ik_sol_number))
    {
      ROS_ERROR("parameter %s/ik_sol_number is not defined, use 200",m_pnh.getNamespace().c_str());
      ik_sol_number=200;
    }
    if (!m_pnh.getParam("finite_slot",m_finite_slot))
    {
      ROS_INFO("parameter %s/finite_slot is not defined, set true",m_pnh.getNamespace().c_str());
      m_finite_slot=true;
    }

    if (slots_config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The param is not a list of slots" );
      return false;
    }
    ROS_DEBUG("there are %d slots",slots_config.size());
    for(int i=0; i < slots_config.size(); i++)
    {
      XmlRpc::XmlRpcValue slot = slots_config[i];
      if( slot.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("The element #%d is not a struct", i);
        continue;
      }


      if( !slot.hasMember("name") )
      {
        ROS_WARN("The element #%d has not the field 'name'", i);
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

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    m_kinematic_model = robot_model_loader.getModel();

    // create groups
    for (const std::string& group_name: m_group_names)
    {

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
      std::string planner_plugin_name;
      if (!m_pnh.getParam(group_name+"/planning_plugin", planner_plugin_name))
      {
        ROS_ERROR_STREAM("Could not find planner plugin name");
        return false;
      }

      int max_ik_goal_number;
      if (!m_pnh.getParam(group_name+"/max_ik_goal_number",max_ik_goal_number))
      {
        max_ik_goal_number=N_ITER;
      }
      m_max_ik_goal_number.insert(std::pair<std::string,int>(group_name,max_ik_goal_number));


      bool use_single_goal;
      if (!m_pnh.getParam(group_name+"/use_single_goal",use_single_goal))
      {
        ROS_DEBUG("parameter %s/use_single_goal is not defined, use multi goal",m_pnh.getNamespace().c_str());
        use_single_goal=false;
      }
      m_use_single_goal.insert(std::pair<std::string,bool>(group_name,use_single_goal));

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


      Eigen::Vector3d gravity;
      gravity << 0,0,-9.806;
      rosdyn::ChainPtr chain=rosdyn::createChain(*robot_model_loader.getURDF(),world_frame,m_tool_names.at(group_name),gravity);
      chain->setInputJointsName(jmg->getActiveJointModelNames());
      m_chains.insert(std::pair<std::string,rosdyn::ChainPtr>(group_name,chain));


      std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as;
      as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>(m_nh,group_name+"/place",
                                                                                          boost::bind(&OutboundMosaic::placeObjectGoalCb,this,_1,group_name),
                                                                                          false));
      m_as.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>>(group_name,as));

      std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_ac;
      fjt_ac.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/"+group_name+"/follow_joint_trajectory",true));
      m_fjt_clients.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>>(group_name,fjt_ac));

      m_fjt_result.insert(std::pair<std::string,double>(group_name,0));

      std::map<std::string,std::vector<Eigen::VectorXd>> slot_configurations;
      std::map<std::string,std::vector<Eigen::VectorXd>> approach_slot_configurations;

      std::vector<Eigen::VectorXd> sols;

      for (const std::pair<std::string, Eigen::Affine3d>& p: m_slot_map)
      {
        std::string place_id=p.first;
        ROS_DEBUG("Compute ik for slot %s, group=%s",place_id.c_str(),group_name.c_str());

        Eigen::Affine3d T_w_as; // world <- approach to slot
        Eigen::Affine3d T_w_s; // world <- slot
        T_w_s=m_slot_map.at(place_id);
        T_w_as=m_approach_slot_map.at(place_id);
        std::vector<std::vector<double>> sols_stl;

        if (!m_pnh.hasParam("slot_ik/"+place_id+"/"+group_name))
        {
          // first, search solutions for the final destination
          if (!ik(group_name,T_w_s,sols,ik_sol_number))
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
            for (long iax=0;iax<sols.at(isolution).size();iax++)
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
            for (long iax=0;iax<sols.at(isolution).size();iax++)
              sols.at(isolution)(iax)=sols_stl.at(isolution).at(iax);
          }
        }
        slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,sols));
        std::vector<Eigen::VectorXd> approach_sols=sols;


        if (!m_pnh.hasParam("approach_ik/"+place_id+"/"+group_name))
        {


          // then,search solutions for the approach pose
          if (!ik(group_name,T_w_as,approach_sols,approach_sols.size()))
          {
            ROS_WARN("No Ik solution for the approach to slot %s",place_id.c_str());
            approach_sols.clear();
            slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,approach_sols));
            approach_slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,approach_sols));
            continue;
          }
          ROS_DEBUG("Find %zu solutions to approach the slot %s (group=%s)",approach_sols.size(),place_id.c_str(),group_name.c_str());

          sols_stl.resize(approach_sols.size());
          for (size_t isolution=0;isolution<approach_sols.size();isolution++)
          {
            sols_stl.at(isolution).resize(approach_sols.at(isolution).size());
            for (long iax=0;iax<approach_sols.at(isolution).size();iax++)
              sols_stl.at(isolution).at(iax)=approach_sols.at(isolution)(iax);
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
          approach_sols.resize(sols_stl.size());
          for (size_t isolution=0;isolution<approach_sols.size();isolution++)
          {
            approach_sols.at(isolution).resize(sols_stl.at(isolution).size());
            for (long iax=0;iax<approach_sols.at(isolution).size();iax++)
              approach_sols.at(isolution)(iax)=sols_stl.at(isolution).at(iax);
          }
        }
        approach_slot_configurations.insert(std::pair<std::string,std::vector<Eigen::VectorXd>>(place_id,approach_sols));

      }
      m_slot_configurations.insert(std::pair<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>>(group_name,slot_configurations));
      m_approach_slot_configurations.insert(std::pair<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>>(group_name,approach_slot_configurations));


    }

    m_target_pub=m_nh.advertise<geometry_msgs::PoseStamped>("target",1);
    m_grasp_srv=m_nh.serviceClient<std_srvs::SetBool>("/gripper/grasp");
    m_detach_object_srv=m_nh.serviceClient<object_loader_msgs::detachObject>("detach_object_to_link");
    m_remove_object_srv=m_nh.serviceClient<object_loader_msgs::removeObjects>("remove_object_from_scene");
    m_reset_srv=m_nh.advertiseService("outbound/reset",&OutboundMosaic::resetCb,this);
    ROS_WARN("====================================================== RESET ==================================");
    m_init=true;

    for (const std::string& group_name: m_group_names)
    {
      m_as.at(group_name)->start();
    }
    return true;

  }

  void OutboundMosaic::placeObjectGoalCb(const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                                         const std::string& group_name)
  {
    ros::Time t0=ros::Time::now();

    std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as=m_as.at(group_name);

    manipulation_msgs::PlaceObjectsResult action_res;
    std::vector<std::string> tmp_place_ids=goal->place_id;
    moveit::planning_interface::MoveItErrorCode result;


    /* Preliminary check */
    if (!m_init)
    {
      ROS_ERROR("outbound pallet is not initialized well");
      action_res.result=manipulation_msgs::PlaceObjectsResult::NotInitialized;
      as->setAborted(action_res,"not initialized");
      return;
    }

    std::vector<std::string> place_ids;
    for (const std::string& place_id: tmp_place_ids)
    {
      if (m_slot_map.find(place_id)==m_slot_map.end())
      {
        ROS_ERROR("slot %s is not managed by the outbound_mosaic",place_id.c_str());
        action_res.result=manipulation_msgs::PlaceObjectsResult::NotInitialized;
        as->setAborted(action_res,"slot not managed");
        return;
      }
      if (m_slot_busy.find(place_id)==m_slot_busy.end())
      {
        ROS_DEBUG("slot %s is not managed by the outbound_mosaic",place_id.c_str());
        action_res.result=manipulation_msgs::PlaceObjectsResult::NotInitialized;
        as->setAborted(action_res,"slot not managed");
        return;
      }
      if (m_slot_busy.at(place_id))
      {
        continue;
      }
      place_ids.push_back(place_id);
    }

    if (place_ids.size()==0)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::Full;
      ROS_ERROR("error in plan for placing slot, code = %d",result.val);
      as->setAborted(action_res,"error in planning for placing");
      return;
    }


    moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
    if (!group->startStateMonitor(2))
    {
      ROS_ERROR("%s: nable to get actual state",m_pnh.getNamespace().c_str());
      action_res.result=manipulation_msgs::PlaceObjectsResult::SceneError;
      as->setAborted(action_res,"unable to get actual state");
      return;
    }
    group->setStartState(*group->getCurrentState());


    /* ===========================
     * Moving to approach position
     * ===========================*/
    geometry_msgs::PoseStamped target;
    target.header.frame_id="world";
    target.header.stamp=ros::Time::now();

    ROS_PROTO("planning to approach");

    Eigen::VectorXd actual_jconf;
    group->getCurrentState()->copyJointGroupPositions(group_name,actual_jconf);
    Eigen::VectorXd approach_slot_jconf;
    ros::Time t_approach_plan_init=ros::Time::now();
    std::string selected_id;

    moveit::planning_interface::MoveGroupInterface::Plan approac_pick_plan=planToApproachSlot(group_name,
                                                                                              place_ids,
                                                                                              actual_jconf,
                                                                                              result,
                                                                                              approach_slot_jconf,
                                                                                              selected_id);

    ros::Time t_approach_plan=ros::Time::now();
    ROS_DEBUG("plan  approach movement in %f second",(t_approach_plan-t0).toSec());
    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
      ROS_ERROR("error in plan for placing slot, code = %d",result.val);
      as->setAborted(action_res,"error in planning for placing");
      return;
    }
    action_res.planning_duration+=t_approach_plan-t_approach_plan_init;
    action_res.expected_execution_duration+=approac_pick_plan.trajectory_.joint_trajectory.points.back().time_from_start;
    action_res.path_length+=trajectory_processing::computeTrajectoryLength(approac_pick_plan.trajectory_.joint_trajectory);

    action_res.place_id=selected_id;
    Eigen::Affine3d T_w_as; // world <- approach to slot
    Eigen::Affine3d T_w_s; // world <- slot
    T_w_s=m_slot_map.at(selected_id);
    T_w_as=m_approach_slot_map.at(selected_id);

    tf::poseEigenToMsg(T_w_as,target.pose);
    m_target_pub.publish(target);

    moveit_msgs::DisplayTrajectory disp_trj;
    disp_trj.trajectory.push_back(approac_pick_plan.trajectory_);
    disp_trj.model_id=m_kinematic_model->getName();
    disp_trj.trajectory_start=approac_pick_plan.start_state_;
    m_display_publisher.publish(disp_trj);
    execute(group_name,approac_pick_plan);

    ros::Time t_approach_execute=ros::Time::now();
    ROS_DEBUG("execute approach movement in %f second",(t_approach_execute-t_approach_plan).toSec());


    /* ===========================
     * Moving to slot
     * ===========================*/
    Eigen::VectorXd slot_jconf;
    ros::Time t_pick_plan_init=ros::Time::now();
    moveit::planning_interface::MoveGroupInterface::Plan plan_plan=planToSlot(group_name,
                                                                              selected_id,
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
    m_slot_busy.at(selected_id)=m_finite_slot;
    ros::Time t_pick_plan=ros::Time::now();
    action_res.planning_duration+=t_pick_plan-t_pick_plan_init;
    action_res.expected_execution_duration+=plan_plan.trajectory_.joint_trajectory.points.back().time_from_start;
    action_res.path_length+=trajectory_processing::computeTrajectoryLength(plan_plan.trajectory_.joint_trajectory);

    ROS_DEBUG("plan pick movement in %f second",(t_pick_plan-t_approach_execute).toSec());

    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      m_slot_busy.at(selected_id)=false;
      return;
    }
    ros::Time t_approach_wait=ros::Time::now();
    ROS_DEBUG("wait for approach movement finish in %f second",(t_approach_wait-t_pick_plan).toSec());



    tf::poseEigenToMsg(T_w_s,target.pose);
    m_target_pub.publish(target);
    disp_trj.trajectory.at(0)=(plan_plan.trajectory_);
    disp_trj.trajectory_start=plan_plan.start_state_;
    m_display_publisher.publish(disp_trj);

    execute(group_name,plan_plan);
    ros::Time t_pick_execute=ros::Time::now();
    ROS_DEBUG("execute approach movement in %f second",(t_pick_execute-t_approach_wait).toSec());

    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::TrajectoryError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      m_slot_busy.at(selected_id)=false;
      return;
    }
    ros::Time t_pick_wait=ros::Time::now();
    ROS_DEBUG("execute approach movement in %f second",(t_pick_wait-t_pick_execute).toSec());


    /* ===========================
     * Release object
     * ===========================*/
    ros::Time t_release_obj_init=ros::Time::now();
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
      m_slot_busy.at(selected_id)=false;
      return;
    }
    ROS_PROTO("detached collision object %s ",detach_srv.request.obj_id.c_str());

    if (!m_finite_slot)
    {
      object_loader_msgs::removeObjects remove_srv;
      remove_srv.request.obj_ids.push_back(goal->object_id);
      ROS_INFO("REMOVE %s",goal->object_id.c_str());
      if (!m_remove_object_srv.call(remove_srv))
      {
        action_res.result=manipulation_msgs::PlaceObjectsResult::SceneError;
        ROS_ERROR("unaspected error calling %s service",m_remove_object_srv.getService().c_str());
        as->setAborted(action_res,"unaspected error calling removing object server");
        return;
      }
      if (!remove_srv.response.success)
      {
        action_res.result=manipulation_msgs::PlaceObjectsResult::SceneError;
        ROS_ERROR("unable to remove object id %s",goal->object_id.c_str());
        as->setAborted(action_res,"unable to remove object");
        m_slot_busy.at(selected_id)=false;
        return;
      }
      ROS_PROTO("remove collision object %s ",goal->object_id.c_str());
    }



    std_srvs::SetBool grasp_req;
    grasp_req.request.data=0;
    m_grasp_srv.call(grasp_req);
    ros::Duration(0.5).sleep();

    ros::Time t_release_obj=ros::Time::now();

    action_res.release_object_duration=t_release_obj-t_release_obj_init;

    if (!group->startStateMonitor(2))
    {
      ROS_ERROR("%s: unable to get actual state",m_pnh.getNamespace().c_str());
      action_res.result=manipulation_msgs::PlaceObjectsResult::SceneError;
      as->setAborted(action_res,"unable to get actual state");
      m_slot_busy.at(selected_id)=false;
      return;
    }

    /* ===========================
     * Return to approach position
     * ===========================*/
    ros::Time return_time_init=ros::Time::now();
    std::vector<std::string> tmp_ids;tmp_ids.push_back(selected_id);
    moveit::planning_interface::MoveGroupInterface::Plan return_plan=planToApproachSlot(group_name,
                                                                                        tmp_ids,
                                                                                        slot_jconf,
                                                                                        result,
                                                                                        approach_slot_jconf,
                                                                                        selected_id);

    if (!result)
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::ReturnError;
      ROS_ERROR("error in plan black to box, code = %d",result.val);
      as->setAborted(action_res,"error in planning back to box");
      return;
    }
    ros::Time return_time=ros::Time::now();
    action_res.planning_duration+=return_time-return_time_init;
    action_res.expected_execution_duration+=return_plan.trajectory_.joint_trajectory.points.back().time_from_start;
    action_res.path_length+=trajectory_processing::computeTrajectoryLength(return_plan.trajectory_.joint_trajectory);


    tf::poseEigenToMsg(T_w_as,target.pose);
    m_target_pub.publish(target);
    disp_trj.trajectory.at(0)=(return_plan.trajectory_);
    m_display_publisher.publish(disp_trj);
    disp_trj.trajectory_start=return_plan.start_state_;
    execute(group_name,return_plan);

    if (!wait(group_name))
    {
      action_res.result=manipulation_msgs::PlaceObjectsResult::ReturnError;
      ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
      as->setAborted(action_res,"error in trajectory execution");
      return;
    }
    action_res.result=manipulation_msgs::PlaceObjectsResult::Success;
    action_res.actual_duration=ros::Time::now()-t0;
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
    planning_scene::PlanningScenePtr planning_scene= planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));

    Eigen::VectorXd js(actual_configuration.size());
    Eigen::VectorXd seed(actual_configuration.size());

    for (unsigned int iter=0;iter<N_MAX_ITER;iter++)
    {
      if (solutions.size()>=ntrial)
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

      state.copyJointGroupPositions(group_name,seed);
      if (m_chains.at(group_name)->computeLocalIk(js,T_w_a,seed,1e-4,ros::Duration(0.002)))
      {
        state.setJointGroupPositions(group_name,js);
        if (!state.satisfiesBounds())
          continue;

        state.updateCollisionBodyTransforms();
        if (!planning_scene->isStateValid(state,group_name))
          continue;

        if (solutions.size()==0)
        {
          std::vector<Eigen::VectorXd> multiturn=m_chains.at(group_name)->getMultiplicity(js);
          for (const Eigen::VectorXd& tmp: multiturn)
          {
            state.setJointGroupPositions(group_name,tmp);
            if (!state.satisfiesBounds())
              continue;
            double dist=(tmp-actual_configuration).norm();
            solutions.insert(std::pair<double,Eigen::VectorXd>(dist,tmp));
          }
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
            std::vector<Eigen::VectorXd> multiturn=m_chains.at(group_name)->getMultiplicity(js);
            for (const Eigen::VectorXd& tmp: multiturn)
            {
              state.setJointGroupPositions(group_name,tmp);
              if (!state.satisfiesBounds())
                continue;
              double dist=(tmp-actual_configuration).norm();
              solutions.insert(std::pair<double,Eigen::VectorXd>(dist,tmp));
            }
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




  moveit::planning_interface::MoveGroupInterface::Plan OutboundMosaic::planToSlot(const std::string& group_name,
                                                                                  const std::string& place_id,
                                                                                  const Eigen::VectorXd& starting_jconf,
                                                                                  moveit::planning_interface::MoveItErrorCode& result,
                                                                                  Eigen::VectorXd& slot_jconf)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
    moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
    int max_ik_goal_number=m_max_ik_goal_number.at(group_name);

    m_mtx.lock();
    planning_scene::PlanningScenePtr planning_scene= planning_scene::PlanningScene::clone(m_planning_scene.at(group_name));
    m_mtx.unlock();
    planning_scene->getCurrentStateNonConst();
    robot_state::RobotState state = *group->getCurrentState();
    state.setJointGroupPositions(jmg,starting_jconf);
    moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);


    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name=group_name;
    req.start_state=plan.start_state_;
    req.allowed_planning_time=5;
    robot_state::RobotState goal_state(m_kinematic_model);

    std::vector<Eigen::VectorXd> sols=m_slot_configurations.at(group_name).at(place_id);
    std::map<double,Eigen::VectorXd> solutions;

    for (const Eigen::VectorXd& goal: sols)
    {

      goal_state.setJointGroupPositions(jmg, goal);
      goal_state.updateCollisionBodyTransforms();
      if (!planning_scene->isStateValid(goal_state,group_name))
      {
        continue;
      }

      double normsol=(goal-starting_jconf).norm();
      solutions.insert(std::pair<double,Eigen::VectorXd>(normsol,goal));

      if (m_use_single_goal.at(group_name))
        break;
    }

    for (const std::pair<double,Eigen::VectorXd>& p: solutions)
    {
      if (req.goal_constraints.size()>=max_ik_goal_number)
        break;
      goal_state.setJointGroupPositions(jmg, p.second);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
      req.goal_constraints.push_back(joint_goal);
    }
    ROS_PROTO("Found %zu solution",req.goal_constraints.size());


    if (!m_planning_pipeline.at(group_name)->generatePlan(planning_scene, req, res))
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
                                                                                          const std::vector<std::string>& place_ids,
                                                                                          const Eigen::VectorXd& starting_jconf,
                                                                                          moveit::planning_interface::MoveItErrorCode& result,
                                                                                          Eigen::VectorXd& slot_jconf,
                                                                                          std::string& selected_id)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveGroupInterfacePtr group=m_groups.at(group_name);
    moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
    int max_ik_goal_number=m_max_ik_goal_number.at(group_name);


    if (!group->startStateMonitor(2))
    {
      ROS_ERROR("%s: unable to get actual state",m_pnh.getNamespace().c_str());
    }
    robot_state::RobotState state = *group->getCurrentState();
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
    std::map<double,Eigen::VectorXd> solutions;
    for (const std::string& place_id: place_ids)
    {
      std::vector<Eigen::VectorXd> sols=m_approach_slot_configurations.at(group_name).at(place_id);
      for (const Eigen::VectorXd& goal: sols)
      {

        goal_state.setJointGroupPositions(jmg, goal);
        goal_state.updateCollisionBodyTransforms();
        if (!planning_scene->isStateValid(goal_state,group_name))
        {
          planning_scene->isStateValid(goal_state,group_name,true);
          continue;
        }
        double normsol=(goal-starting_jconf).norm();
        solutions.insert(std::pair<double,Eigen::VectorXd>(normsol,goal));


        if (m_use_single_goal.at(group_name))
          break;
      }
      if (m_use_single_goal.at(group_name))
        break;
    }

    for (const std::pair<double,Eigen::VectorXd>& p: solutions)
    {
      if (req.goal_constraints.size()>=max_ik_goal_number)
        break;
      goal_state.setJointGroupPositions(jmg, p.second);
      moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, jmg);
      req.goal_constraints.push_back(joint_goal);
    }

    ROS_PROTO("Found %zu solution",req.goal_constraints.size());

    if (!m_planning_pipeline.at(group_name)->generatePlan(planning_scene, req, res))
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

    ROS_PROTO("Search selected place_id");
    bool found_id=false;

    for (const std::string& place_id: place_ids)
    {
      std::vector<Eigen::VectorXd> sols=m_approach_slot_configurations.at(group_name).at(place_id);
      for (const Eigen::VectorXd& goal: sols)
      {
        if ((goal-slot_jconf).norm()<1e-6)
        {
          selected_id=place_id;
          found_id=true;
          break;
        }
      }
      if (found_id)
        break;
    }
    if (!found_id)
    {
      ROS_ERROR("unable to find the selected place_id");
      throw std::invalid_argument("unable to find the selected place_id");
    }
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
    ros::Time t0=ros::Time::now();
    m_fjt_clients.at(group_name)->waitForResult();
    ros::Duration(0.1).sleep();

    while (std::isnan(m_fjt_result.at(group_name)))
    {
      ros::Duration(0.01).sleep();

      if ((ros::Time::now()-t0).toSec()>600)
      {
        ROS_ERROR("%s is waiting more than ten minutes, stop it",group_name.c_str());
        return false;
      }
      if ((ros::Time::now()-t0).toSec()>10)
        ROS_WARN_THROTTLE(10,"%s is waiting for %f seconds",group_name.c_str(),(ros::Time::now()-t0).toSec());
    }
    return !std::isnan(m_fjt_result.at(group_name));
  }

  void OutboundMosaic::doneCb(const actionlib::SimpleClientGoalState &state, const control_msgs::FollowJointTrajectoryResultConstPtr &result, const std::string &group_name)
  {
    m_fjt_result.at(group_name)=result->error_code;
    ROS_DEBUG("%s has done",group_name.c_str());
    if (result->error_code<0)
    {
      ROS_ERROR("error executing %s/follow_joint_trajectory: %s",group_name.c_str(),result->error_string.c_str());
    }
    return;
  }

  bool OutboundMosaic::resetCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    if (req.data)
    {

      for (const std::pair<std::string,bool>& s: m_slot_busy)
        m_slot_busy.at(s.first)=false;
      ROS_INFO("Outbound mosaic reset");
    }
    return true;
  }

  void OutboundMosaic::updatePlanningScene(const moveit_msgs::PlanningScene& scene)
  {
    m_mtx.lock();
    for (const std::string& group: m_group_names)
    {
      if (!m_planning_scene.at(group)->setPlanningSceneMsg(scene))
        ROS_ERROR("unable to update planning scene");
    }
    m_mtx.unlock();
  }


}
