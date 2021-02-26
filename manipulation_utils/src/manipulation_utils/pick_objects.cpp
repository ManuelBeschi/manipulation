
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

#include <manipulation_msgs/PickObjectsResult.h>
#include <manipulation_utils/pick_objects.h>

#include <object_loader_msgs/attachObject.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_planning_helper/manage_trajectories.h>

namespace manipulation
{
  PickObjects::PickObjects( const ros::NodeHandle& nh, 
                            const ros::NodeHandle& pnh):
                            m_nh(nh),
                            m_pnh(pnh),
                            SkillBase(nh,pnh)
  {
    // nothing to do here     
  }

  bool PickObjects::init()
  {
    if (!SkillBase::init())  
      return false;
    
    m_add_obj_srv = m_pnh.advertiseService("add_objects",&PickObjects::addObjectsCb,this);
    m_add_box_srv = m_pnh.advertiseService("add_boxes",&PickObjects::addBoxesCb,this);
    m_remove_objects_srv = m_pnh.advertiseService("remove_objects",&PickObjects::removeObjectsCb,this);
    m_list_objects_srv = m_pnh.advertiseService("list_objects",&PickObjects::listObjectsCb,this);
    m_reset_srv = m_pnh.advertiseService("inboud/reset_box",&PickObjects::resetBoxesCb,this);

    m_attach_object_srv = m_nh.serviceClient<object_loader_msgs::attachObject>("attach_object_to_link");
    m_attach_object_srv.waitForExistence();

    for (const std::string& group_name: m_group_names)
    { 
      std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>> as;
      as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>( m_pnh,
                                                                                        group_name+"/pick",
                                                                                        boost::bind(&PickObjects::pickObjectGoalCb, this, _1, group_name),
                                                                                        false));
      m_pick_servers.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>>>(group_name,as));

      m_pick_servers.at(group_name)->start();
    }
    
    return true;
  }

  bool PickObjects::addBoxesCb( manipulation_msgs::AddBoxes::Request& req,
                                manipulation_msgs::AddBoxes::Response& res)
  {
    for (const manipulation_msgs::Box& box: req.add_boxes )
    {
      m_boxes.insert(std::pair<std::string,BoxPtr>(box.name,std::make_shared<manipulation::Box>(m_pnh,box)));
      if (!m_boxes.at(box.name)->getIntState())
      {
         m_boxes.erase(m_boxes.find(box.name));
         res.results = manipulation_msgs::AddBoxes::Response::Error;
      }
    }
    res.results = manipulation_msgs::AddBoxes::Response::Success;
    return true;
  }

  bool PickObjects::removeBoxesCb(manipulation_msgs::RemoveBoxes::Request& req,
                                  manipulation_msgs::RemoveBoxes::Response& res)
  {
    for (const std::string& box_name: req.box_names)
    {
      if(m_boxes.find(box_name) != m_boxes.end())
      {
        m_boxes.erase(m_boxes.find(box_name));
        ROS_INFO("The box %s has been removed.", box_name.c_str() );
      }
      else
        ROS_ERROR("Can't remove box %s, the box doesn't exist.", box_name.c_str() );
    }
    return true;
  }

  bool PickObjects::addObjectsCb( manipulation_msgs::AddObjects::Request& req,
                                  manipulation_msgs::AddObjects::Response& res)
  {
    for (const manipulation_msgs::Object& object: req.add_objects )
    {
      if (m_boxes.find(req.box_name) == m_boxes.end())
      {
        ROS_ERROR("Can't add object %s, the box %s does not exist.", object.name.c_str(),
                                                                      req.box_name.c_str());
        return false;
      }

      if(!m_boxes.find(req.box_name)->second->addObject(object))
        return false;  

      ROS_INFO("Added the object %s of the type %s in box %s",object.name.c_str(),
                                                              object.type.c_str(),
                                                              req.box_name.c_str());
    }

    res.results = manipulation_msgs::AddObjects::Response::Success;
    return true;
  }

  bool PickObjects::removeObjectsCb(manipulation_msgs::RemoveObjects::Request& req,
                                    manipulation_msgs::RemoveObjects::Response& res)
  {
    for (const std::string& object_name: req.object_names)
    {
      for (std::map<std::string,BoxPtr>::iterator it = m_boxes.begin(); it != m_boxes.end(); it++)
      {
        if (it->second->findObject(object_name))
        {
          if (m_tf.find("pick/approach/"+object_name) != m_tf.end())
            m_tf.erase(m_tf.find("pick/approach/"+object_name));

          if (m_tf.find("pick/to/"+object_name) != m_tf.end())
            m_tf.erase(m_tf.find("pick/to/"+object_name));

          if (m_tf.find("pick/leave/"+object_name) != m_tf.end())
            m_tf.erase(m_tf.find("pick/leave/"+object_name));

          if (!it->second->removeObject(object_name))
            ROS_ERROR("Can't remove object %s.", object_name.c_str()); 
        }
      }
      ROS_INFO("Removed object %s.", object_name.c_str()); 
    }
    return true;
  }

  bool PickObjects::listObjectsCb(manipulation_msgs::ListOfObjects::Request& req,
                                  manipulation_msgs::ListOfObjects::Response& res)
  {
    std::vector<std::string> object_types, object_names, box_names;
    for (std::map<std::string,BoxPtr>::iterator it = m_boxes.begin(); it != m_boxes.end(); it++)
    {
      std::vector<ObjectPtr> objects = it->second->getAllObjects();
      for(const ObjectPtr& object: objects)
      { 
        object_types.push_back(object->getType());
        object_names.push_back(object->getName());
      }

      box_names.push_back(it->first);
    }

    res.object_types = object_types;
    res.object_names = object_names;
    res.box_names = box_names;

    return true;
  }

  bool PickObjects::resetBoxesCb( manipulation_msgs::ResetBoxes::Request& req, 
                                  manipulation_msgs::ResetBoxes::Response& res) 
  {
    for (const std::string& box_name: req.box_names)
    {
      if (m_boxes.find(box_name) != m_boxes.end())
      {
        m_boxes.find(box_name)->second->removeAllObjects();
        ROS_INFO("Reset the objects in the box %s", box_name.c_str());
      }
      else
        ROS_ERROR("Cannot reset objects in the box %s the specified box does not exist.", box_name.c_str());
    }
    return true;
  }

  void PickObjects::pickObjectGoalCb( const manipulation_msgs::PickObjectsGoalConstPtr& goal,
                                      const std::string& group_name)
  {
    manipulation_msgs::PickObjectsResult action_res;
    std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>> as = m_pick_servers.at(group_name);
    
    try
    {
      ros::Time t_start = ros::Time::now();

      //std::vector<std::string> type_names = goal->object_types;
      
      std::vector<std::string> possible_boxes_names;
      for (const std::string& type_name: goal->object_types)
      {
        ROS_INFO("Adding object %s to the picking list.", type_name.c_str());
        for (std::map<std::string,BoxPtr>::iterator it = m_boxes.begin(); it != m_boxes.end(); it++)
        {
          std::vector<ObjectPtr> objects = it->second->getObjectsByType(type_name);
          if (objects.size() != 0)
            possible_boxes_names.push_back(it->second->getLocationName());
        }
      }
  
      std::string tool_name = m_tool_names.at(group_name);

      ROS_INFO("Found %zu boxes",possible_boxes_names.size());

      if (possible_boxes_names.size() == 0)
      {
        action_res.result = manipulation_msgs::PickObjectsResult::NoInboundBoxFound;
        ROS_ERROR("No objects found");
        as->setAborted(action_res,"no objects found");
        return;
      }

      if (!m_groups.at(group_name)->startStateMonitor(2))
      {
        ROS_ERROR("%s: unable to get actual state",m_pnh.getNamespace().c_str());
        action_res.result = manipulation_msgs::PickObjectsResult::SceneError;
        as->setAborted(action_res,"no objects found");
        return;
      }

      moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
      robot_state::RobotState state = *m_groups.at(group_name)->getCurrentState();
      
      Eigen::VectorXd actual_jconf;
      if (jmg)
        state.copyJointGroupPositions(jmg, actual_jconf);
      
      std::string best_box_name;

      moveit::planning_interface::MoveItErrorCode result;
      Eigen::VectorXd box_approach_jconf;


      /* Planning to approach position for picking object */

      ros::Time t_planning_init = ros::Time::now();
      ROS_INFO("Planning to box approach position for object picking. Group %s",group_name.c_str());
      moveit::planning_interface::MoveGroupInterface::Plan plan = planTo( group_name,
                                                                          possible_boxes_names,
                                                                          Location::Destination::Approach,
                                                                          actual_jconf,
                                                                          actual_jconf,
                                                                          result,
                                                                          box_approach_jconf,
                                                                          best_box_name);

      manipulation::BoxPtr selected_box = m_boxes.at(best_box_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
        ROS_ERROR("error in plan to best box, code = %d",result.val);
        as->setAborted(action_res,"error in planning to the box");
        return;
      }

      ROS_INFO("Group %s: plan to approach in %f second",group_name.c_str(),plan.planning_time_);
      ros::Time t_planning = ros::Time::now();
      action_res.planning_duration += (t_planning-t_planning_init);
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length = trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

      moveit_msgs::DisplayTrajectory disp_trj;
      disp_trj.trajectory.push_back(plan.trajectory_);
      disp_trj.model_id = m_kinematic_model->getName();
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);

      geometry_msgs::PoseStamped target;
      target.header.frame_id = world_frame;
      target.header.stamp = ros::Time::now();
      tf::poseEigenToMsg(m_locations.at(selected_box->getLocationName())->getApproach(),target.pose);
      m_target_pub.publish(target);
      
      ROS_INFO("Execute move to approach position for object picking. Group %s, Box name: %s",group_name.c_str(),best_box_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PickObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      /* Planning to picking object position */

      std::vector<std::string> possible_object_location_names;

      for (const std::string& type_name: goal->object_types)
      {
        std::vector<ObjectPtr> objects = selected_box->getObjectsByType(type_name);
        for(const manipulation::ObjectPtr& object: objects)
        {
          std::vector<std::string> object_location_names = object->getGraspLocationNames();
          if (object_location_names.size() != 0)
            possible_object_location_names.insert(possible_object_location_names.end(),
                                                  object_location_names.begin(),
                                                  object_location_names.end()   );

        }
      }

      if(possible_object_location_names.size()==0)
      {
        action_res.result = manipulation_msgs::PickObjectsResult::NoObjectsFound;
        ROS_ERROR("Can't find any object location names in the location manager.");
        as->setAborted(action_res,"error in planning to the object");
        return;
      }

      std::string best_object_location_name;
      Eigen::VectorXd object_grasp_jconf;
      std::vector<std::string> possible_objects;

      t_planning_init = ros::Time::now();

      ROS_INFO("Planning to object picking position. Group %s, Box name %s",group_name.c_str(), best_box_name.c_str());
      plan = planTo(group_name,
                    possible_object_location_names,
                    Location::Destination::To,
                    box_approach_jconf,
                    box_approach_jconf,
                    result,
                    object_grasp_jconf,
                    best_object_location_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
        ROS_ERROR("Group %s: error in plan to best object in the box %s, code = %d", group_name.c_str(), best_box_name.c_str(), result.val);
        as->setAborted(action_res,"error in planning to object");
        return;
      }

      std::string best_object_name = selected_box->findObjectByGraspingLocation(best_object_location_name);
      manipulation::ObjectPtr selected_object = selected_box->getObject(best_object_name);
      manipulation::GraspPtr selected_grasp_pose = selected_object->getGrasp(best_object_location_name);
      
      tf::Transform transform;
      tf::transformEigenToTF(m_locations.at(selected_grasp_pose->getLocationName())->getApproach(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("pick/approach/"+selected_object->getName(),transform));

      tf::transformEigenToTF(m_locations.at(selected_grasp_pose->getLocationName())->getLocation(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("pick/to/"+selected_object->getName(),transform));

      tf::transformEigenToTF(m_locations.at(selected_grasp_pose->getLocationName())->getLeave(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("pick/leave/"+selected_object->getName(),transform));

      t_planning = ros::Time::now();
      action_res.planning_duration += (t_planning-t_planning_init);
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

      if (!selected_box->removeObject(selected_object->getName()))
        ROS_WARN("Unable to remove object %s form box %s", selected_box->getName().c_str(), selected_object->getName().c_str());
      
      m_fjt_clients.at(group_name)->waitForResult();

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PickObjectsResult::TrajectoryError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        selected_box->addObject(selected_object);
        return;
      }

      disp_trj.trajectory.at(0) = (plan.trajectory_);
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);
      tf::poseEigenToMsg(m_locations.at(best_object_location_name)->getLocation(),target.pose);
      m_target_pub.publish(target);
      
      ROS_INFO("Execute move to object position for picking. Group %s, object name %s",group_name.c_str(), best_object_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PickObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      m_fjt_clients.at(group_name)->waitForResult();

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PickObjectsResult::TrajectoryError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        selected_box->addObject(selected_object);
        return;
      }


      /* Attach object */

      ros::Time t_grasp_init = ros::Time::now();
      ros::Duration(0.5).sleep();

      object_loader_msgs::attachObject attach_srv;
      attach_srv.request.obj_id = selected_object->getName();
      attach_srv.request.link_name = selected_grasp_pose->getToolName();
      if (!m_attach_object_srv.call(attach_srv))
      {
        action_res.result = manipulation_msgs::PickObjectsResult::NoObjectsFound;
        ROS_ERROR("Unaspected error calling %s service",m_attach_object_srv.getService().c_str());
        as->setAborted(action_res,"unaspected error calling attach server");
        return;
      }
      if (!attach_srv.response.success)
      {
        action_res.result = manipulation_msgs::PickObjectsResult::NoObjectsFound;
        ROS_ERROR("Unable to attach object");
        as->setAborted(action_res,"unable to attach object");
        return;
      }

      ROS_INFO("Group %s: attached collision object %s to tool %s",group_name.c_str(),attach_srv.request.obj_id.c_str(),attach_srv.request.link_name.c_str());

      std_srvs::SetBool grasp_req;
      grasp_req.request.data = 1;
      m_grasp_srv.call(grasp_req);
      ros::Duration(1).sleep();

      action_res.grasping_object_duration = (ros::Time::now()-t_grasp_init);

      Eigen::VectorXd object_leave_jconf;
      std::string best_leave_location_name;
      std::vector<std::string> best_object_location_names(1, best_object_location_name);
      state = *m_groups.at(group_name)->getCurrentState();
      if (jmg)
        state.copyJointGroupPositions(jmg, actual_jconf);


      /* Planning to leave position after object picking */

      t_planning_init = ros::Time::now();

      ROS_INFO("Planning to leave position after object picking. Group %s",group_name.c_str());

      plan = planTo(group_name,
                    best_object_location_names,
                    Location::Destination::Leave,
                    actual_jconf,
                    actual_jconf,
                    result,
                    object_leave_jconf,
                    best_leave_location_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PickObjectsResult::NoAvailableTrajectories;
        ROS_ERROR("Error in plan back to box, code %d",result.val);
        as->setAborted(action_res,"Error in planning back to box");
        return;
      }

      t_planning = ros::Time::now();
      action_res.planning_duration += (t_planning-t_planning_init);
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);
      disp_trj.trajectory.at(0) = (plan.trajectory_);
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);

      tf::poseEigenToMsg(m_locations.at(best_leave_location_name)->getLeave(),target.pose);
      m_target_pub.publish(target);

      ROS_INFO("Execute move to leave after object picking. Group %s, Object name %s",group_name.c_str(), best_object_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PickObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      m_fjt_clients.at(group_name)->waitForResult();      

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PickObjectsResult::TrajectoryError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"Error in trajectory execution");
        return;
      }
            
      action_res.object_type = selected_object->getType();
      action_res.object_name = selected_object->getName();
      action_res.result = manipulation_msgs::PickObjectsResult::Success;

      action_res.actual_duration += (ros::Time::now()-t_start);
      as->setSucceeded(action_res,"ok");
      
      ROS_INFO("PickObjectCb ended with success.");

      return;
    }
    catch(const std::exception& ex )
    {
      ROS_ERROR("PickObject::PickObjectGoalCb Exception: %s", ex.what());
      action_res.result = manipulation_msgs::PickObjectsResult::UnexpectedError;
      as->setAborted(action_res,"exception");
      return;
    }

  }

  void PickObjects::publishTF()
  {
    try
    {
      for (const std::pair<std::string,tf::Transform>& t: m_tf)
        m_broadcaster.sendTransform(tf::StampedTransform(t.second, ros::Time::now(), world_frame, t.first)); 
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("PickObject::publishTF Exception thrown while publishing TF: %s",ex.what());
      return;
    }
  }

}