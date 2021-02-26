
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

#include <manipulation_msgs/Slot.h>

#include <manipulation_utils/place_objects.h>

#include <object_loader_msgs/detachObject.h>
#include <moveit_planning_helper/manage_trajectories.h>

namespace manipulation
{
  PlaceObjects::PlaceObjects( const ros::NodeHandle& nh, 
                              const ros::NodeHandle& pnh):
                              m_nh(nh),
                              m_pnh(pnh),
                              SkillBase(nh,pnh)
  {

  }

  bool PlaceObjects::init()
  {
    if (!SkillBase::init())
      return false;
    
    m_add_slots_srv = m_pnh.advertiseService("add_slots",&PlaceObjects::addSlotsCb,this);
    m_remove_slots_srv = m_pnh.advertiseService("remove_slots",&PlaceObjects::removeSlotsCb,this);
    m_remove_obj_from_slot_srv = m_pnh.advertiseService("remove_obj_from_slot",&PlaceObjects::removeObjectFromSlotCb,this);
    m_reset_slots_srv = m_pnh.advertiseService("outbound/reset",&PlaceObjects::resetSlotsCb,this);

    m_detach_object_srv = m_pnh.serviceClient<object_loader_msgs::detachObject>("detach_object_to_link");
     
    for (const std::string& group_name: m_group_names)
    {
      std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as;
      as.reset(new actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>(m_pnh,
                                                                                        group_name+"/place",
                                                                                        boost::bind(&PlaceObjects::placeObjectGoalCb,this,_1,group_name),
                                                                                        false));
      m_place_servers.insert(std::pair<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>>(group_name,as));
      
      m_place_servers.at(group_name)->start();
    }

    return true;
  }

  bool PlaceObjects::addSlotsCb(manipulation_msgs::AddSlots::Request& req, 
                                manipulation_msgs::AddSlots::Response& res)
  {
    bool slot_added = false;
    for (const manipulation_msgs::Slot& slot: req.add_slots )
    {
      m_slots.insert(std::pair<std::string,SlotPtr>(slot.name,std::make_shared<manipulation::Slot>(m_pnh,slot)));
      if (!m_slots.at(slot.name)->getIntState())
      {
         m_slots.erase(m_slots.find(slot.name));
         res.results = manipulation_msgs::AddSlots::Response::Error;
      }
      else
      {
        ROS_INFO("The slot %s has been added.", slot.name.c_str() );
        slot_added = true;
      }
    }

    if (slot_added)
    {
      res.results = manipulation_msgs::AddSlots::Response::Success;
      return true;
    }
    else
    {
      res.results = manipulation_msgs::AddSlots::Response::Error;
      return false;
    }
  }

  bool PlaceObjects::removeSlotsCb( manipulation_msgs::RemoveSlots::Request& req, 
                                    manipulation_msgs::RemoveSlots::Response& res)
  {
    for (const std::string& slot_name: req.slot_names)
    {
      if(m_slots.find(slot_name) != m_slots.end())
      {
          if (m_tf.find("place/approach/"+slot_name) != m_tf.end())
            m_tf.erase(m_tf.find("place/approach/"+slot_name));

          if (m_tf.find("place/to/"+slot_name) != m_tf.end())
            m_tf.erase(m_tf.find("place/to/"+slot_name));

          if (m_tf.find("place/leave/"+slot_name) != m_tf.end())
            m_tf.erase(m_tf.find("place/leave/"+slot_name));

        m_slots.erase(m_slots.find(slot_name));
        ROS_INFO("The slot %s has been removed.", slot_name.c_str() );
      }
      else
      {
        ROS_ERROR("Can't remove slot %s, the slot doesn't exist.", slot_name.c_str() );
        return false;    
      }
        
    }
    return true;
  }

  bool PlaceObjects::removeObjectFromSlotCb(manipulation_msgs::RemoveObjectsFromSlot::Request& req, 
                                            manipulation_msgs::RemoveObjectsFromSlot::Response& res)
  {
    if(m_slots.find(req.slot_name) != m_slots.end())
    {
      for (int iObj=0; iObj<req.obj_to_remove; iObj++)
        m_slots.at(req.slot_name)->removeObjectFromSlot();
      
      ROS_INFO("Removed %d from the slot %s.", req.obj_to_remove, req.slot_name.c_str() );
    }
    else
    {
      ROS_ERROR("The slot %s is not available, can't remove objects.", req.slot_name.c_str() );
      return false;
    }

    return true; 
  }

  bool PlaceObjects::resetSlotsCb(manipulation_msgs::ResetSlots::Request& req, 
                                  manipulation_msgs::ResetSlots::Response& res)
  {
    for (const std::string& slot_name: req.slot_names)
    {
      if (m_slots.find(slot_name) != m_slots.end())
      {
        m_slots.find(slot_name)->second->resetSlot();
        ROS_INFO("Reset the objects in the slot %s", slot_name.c_str());
      }
      else
        ROS_ERROR("Cannot reset objects in the box %s the specified box does not exist.", slot_name.c_str());
    }
    return true;
  }
  
  void PlaceObjects::placeObjectGoalCb( const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                                        const std::string& group_name)
  {
    manipulation_msgs::PlaceObjectsResult action_res;
    std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>> as = m_place_servers.at(group_name);

    try
    {
      ros::Time t_start = ros::Time::now();
    
      /* Check if there is an available slot */

      std::vector<std::string> available_slot_names;
      for (const std::string& slot_name: goal->slot_names)
      {
        if (m_slots.find(slot_name) == m_slots.end())
        {
          ROS_WARN("Slot %s is not available to place the object", slot_name.c_str());
          continue;
        }
          
        if (!m_slots.at(slot_name)->getSlotAvailability())
        {
          ROS_WARN("The slot %s is full", slot_name.c_str());
          continue;
        }
        available_slot_names.push_back(m_slots.at(slot_name)->getLocationName());
      }

      if(available_slot_names.size() == 0)
      {
        ROS_ERROR("No available slot to place objects");
        action_res.result = manipulation_msgs::PlaceObjectsResult::NotInitialized;
        as->setAborted(action_res,"slot not managed");
        return;
      }
      
      if (!m_groups.at(group_name)->startStateMonitor(2))
      {
        ROS_ERROR("%s: enable to get actual state", m_pnh.getNamespace().c_str());
        action_res.result = manipulation_msgs::PlaceObjectsResult::SceneError;
        as->setAborted(action_res,"unable to get actual state");        
        return;
      }

      std::string obj_type = goal->object_type;
      std::string obj_name = goal->object_name;

      moveit::core::JointModelGroup* jmg = m_joint_models.at(group_name);
      robot_state::RobotState state = *m_groups.at(group_name)->getCurrentState();
      
      Eigen::VectorXd actual_jconf;
      if (jmg)
        state.copyJointGroupPositions(jmg, actual_jconf);

      std::string best_slot_name;
      Eigen::VectorXd slot_approach_jconf;
            
      moveit::planning_interface::MoveItErrorCode result;


      /* Planning to approach position */

      ros::Time t_planning_init = ros::Time::now();
      ROS_INFO("Planning to box approach position for object placing. Group %s", group_name.c_str());
      moveit::planning_interface::MoveGroupInterface::Plan plan = planTo( group_name,
                                                                          available_slot_names,
                                                                          Location::Destination::Approach,
                                                                          actual_jconf,
                                                                          actual_jconf,
                                                                          result,
                                                                          slot_approach_jconf,
                                                                          best_slot_name);

       manipulation::SlotPtr selected_slot = m_slots.at(best_slot_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
        ROS_ERROR("Error in plan for placing slot, code = %d",result.val);
        as->setAborted(action_res,"error in planning for placing");
        return;
      }

      ROS_INFO("Group %s: plan to approach in %f second",group_name.c_str(),plan.planning_time_);
      ros::Time t_planning = ros::Time::now();
      action_res.planning_duration += t_planning - t_planning_init;
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

      moveit_msgs::DisplayTrajectory disp_trj;
      disp_trj.trajectory.push_back(plan.trajectory_);
      disp_trj.model_id = m_kinematic_model->getName();
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);
      
      geometry_msgs::PoseStamped target;
      target.header.frame_id = world_frame;
      target.header.stamp = ros::Time::now();
      tf::poseEigenToMsg(m_locations.at(selected_slot->getLocationName())->getApproach(),target.pose);
      m_target_pub.publish(target);

      ROS_INFO("Execute move to approach position for object placing. Group %s, slot name: %s",group_name.c_str(),best_slot_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      /* Planning to slot */

      Eigen::VectorXd object_release_jconf;
      std::vector<std::string> slot_names(1, selected_slot->getLocationName());

      t_planning_init = ros::Time::now();

      ROS_INFO("Planning to object release position. Group %s, Slot name %s",group_name.c_str(), best_slot_name.c_str());
      plan = planTo(group_name,
                    slot_names,
                    Location::Destination::To,
                    slot_approach_jconf,
                    slot_approach_jconf,
                    result,
                    object_release_jconf,
                    best_slot_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::NoAvailableTrajectories;
        ROS_ERROR("Group %s: error in plan to release the object %s in the slot %s, code = %d", group_name.c_str(), obj_name.c_str(), best_slot_name.c_str(), result.val);
        as->setAborted(action_res,"error in planning for placing");
        return;
      }

      tf::Transform transform;
      tf::transformEigenToTF(m_locations.at(selected_slot->getLocationName())->getApproach(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("place/approach/"+selected_slot->getName(),transform));

      tf::transformEigenToTF(m_locations.at(selected_slot->getLocationName())->getLocation(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("place/to/"+selected_slot->getName(),transform));

      tf::transformEigenToTF(m_locations.at(selected_slot->getLocationName())->getLeave(), transform);
      m_tf.insert(std::pair<std::string,tf::Transform>("place/leave/"+selected_slot->getName(),transform));


      t_planning = ros::Time::now();
      action_res.planning_duration += (t_planning-t_planning_init);
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);

      m_fjt_clients.at(group_name)->waitForResult();

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        return;
      }

      disp_trj.trajectory.at(0) = (plan.trajectory_);
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);
      tf::poseEigenToMsg(m_locations.at(selected_slot->getLocationName())->getLocation(),target.pose);
      m_target_pub.publish(target);

      ROS_INFO("Execute move to leave position. Group %s, object name %s",group_name.c_str(), obj_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      m_fjt_clients.at(group_name)->waitForResult();

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        return;
      }


      /* Release object */

      ros::Time t_release_init = ros::Time::now();
      ros::Duration(0.5).sleep();

      object_loader_msgs::detachObject detach_srv;
      detach_srv.request.obj_id = obj_name;
      if (!m_detach_object_srv.call(detach_srv))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReleaseError;
        ROS_ERROR("Unaspected error calling %s service",m_detach_object_srv.getService().c_str());
        as->setAborted(action_res,"unaspected error calling detach server");
        return;
      }
      if (!detach_srv.response.success)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReleaseError;
        ROS_ERROR("Unable to detach object name %s",goal->object_name.c_str());
        as->setAborted(action_res,"unable to detach object");
        return;
      }

      ROS_INFO("Group %s: detached object %s ", group_name.c_str(), detach_srv.request.obj_id.c_str());

      std_srvs::SetBool grasp_req;
      grasp_req.request.data = 0;
      m_grasp_srv.call(grasp_req);
      ros::Duration(0.5).sleep();

      action_res.release_object_duration = ros::Time::now() - t_release_init;

      if (!m_groups.at(group_name)->startStateMonitor(2))
      {
        ROS_ERROR("%s: unable to get actual state", m_pnh.getNamespace().c_str());
        action_res.result = manipulation_msgs::PlaceObjectsResult::SceneError;
        as->setAborted(action_res,"unable to get actual state");
        return;
      }

      m_slots.at(best_slot_name)->addObjectToSlot();


      /* Planning to leave position after object placing */

      Eigen::VectorXd slot_leave_jconf;
      t_planning_init = ros::Time::now();

      ROS_INFO("Planning to leave position after object release. Group %s",group_name.c_str());

      plan = planTo(group_name,
                    slot_names,
                    Location::Destination::Leave,
                    object_release_jconf,
                    object_release_jconf,
                    result,
                    slot_leave_jconf,
                    best_slot_name);

      if (!result)
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReturnError;
        ROS_ERROR("Error in plan back from slot after object release, code = %d",result.val);
        as->setAborted(action_res,"error in planning back from slot after object release");
        return;
      }

      t_planning = ros::Time::now();
      action_res.planning_duration += (t_planning-t_planning_init);
      action_res.expected_execution_duration += plan.trajectory_.joint_trajectory.points.back().time_from_start;
      action_res.path_length += trajectory_processing::computeTrajectoryLength(plan.trajectory_.joint_trajectory);
      disp_trj.trajectory.at(0) = plan.trajectory_;
      disp_trj.trajectory_start = plan.start_state_;
      m_display_publisher.publish(disp_trj);

      tf::poseEigenToMsg(m_locations.at(selected_slot->getLocationName())->getLeave(),target.pose);
      m_target_pub.publish(target);

      ROS_INFO("Execute move to leave after object release. Group %s, Object name %s", group_name.c_str(), obj_name.c_str());
      if(!execute(group_name, plan))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::TrajectoryError;
        ROS_ERROR("Error while executing trajectory");
        as->setAborted(action_res,"error while executing trajectory.");
        return;
      }

      m_fjt_clients.at(group_name)->waitForResult(); 

      if (!wait(group_name))
      {
        action_res.result = manipulation_msgs::PlaceObjectsResult::ReturnError;
        ROS_ERROR("error executing %s/follow_joint_trajectory",group_name.c_str());
        as->setAborted(action_res,"error in trajectory execution");
        return;
      }

      action_res.result = manipulation_msgs::PlaceObjectsResult::Success;
      action_res.actual_duration = ros::Time::now() - t_start;
      as->setSucceeded(action_res,"ok");

      return;

    }
    catch(const std::exception& ex )
    {
      ROS_ERROR("PlaceObject::PlaceObjectGoalCb Exception: %s", ex.what());
      action_res.result = manipulation_msgs::PlaceObjectsResult::UnexpectedError;
      as->setAborted(action_res,"exception");
      return;
    }
  
  }
  
  void PlaceObjects::publishTF()
  {
    try
    {
      for (const std::pair<std::string,tf::Transform>& t: m_tf)
        m_broadcaster.sendTransform(tf::StampedTransform(t.second, ros::Time::now(), world_frame, t.first)); 
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("PlaceObject::publishTF Exception thrown while publishing TF: %s",ex.what());
      return;
    }
  }

}