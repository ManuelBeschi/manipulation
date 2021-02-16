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

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <rosparam_utilities/rosparam_utilities.h>

#include <manipulation_msgs/Box.h>
#include <manipulation_msgs/Grasp.h>
#include <manipulation_utils/inbound_pick_utils.h> 

namespace manipulation 
{

InboundPickFromParam::InboundPickFromParam( const ros::NodeHandle &nh):
                                            nh_(nh)
{
  add_objs_client_ = nh_.serviceClient<manipulation_msgs::AddObjects>("add_objects");
  add_box_client_ = nh_.serviceClient<manipulation_msgs::AddBoxes>("add_boxes");
  add_col_objs_client_ = nh_.serviceClient<object_loader_msgs::addObjects>("/add_object_to_scene");
  ROS_INFO("Scene spawner is waiting %s", add_col_objs_client_.getService().c_str());
  add_col_objs_client_.waitForExistence();
  ROS_INFO("Reading object to spawn");

  ROS_INFO("Waiting for pick server");
  add_box_client_.waitForExistence();
  ROS_INFO("Connection ok");

}

bool InboundPickFromParam::readBoxesFromParam()
{
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("/inbound/boxes",config))
  {
    ROS_ERROR("Unable to find /inboud/boxes");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of boxed" );
    return false;
  }

  std::vector<manipulation_msgs::Box> boxes;

  ROS_INFO("There are %u boxes",config.size());
  for(size_t i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue box = config[i];
    if( box.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%zu is not a struct", i);
      continue;
    }

    if( !box.hasMember("name") )
    {
      ROS_WARN("The element #%zu has not the field 'name'", i);
      continue;
    }
    std::string box_name = rosparam_utilities::toString(box["name"]);
    ROS_INFO("Found box named: %s",box_name.c_str());

    if( !box.hasMember("frame") )
    {
      ROS_WARN("The element #%zu has not the field 'frame'", i);
      continue;
    }
    std::string frame_name = rosparam_utilities::toString(box["frame"]);
    ROS_INFO("Found box frame name: %s",frame_name.c_str());

    if( !box.hasMember("heigth") )
    {
      ROS_WARN("The element #%zu has not the field 'height'", i);
      continue;
    }
    double height = rosparam_utilities::toDouble(box["heigth"]);
    ROS_DEBUG("Box picking heigth %f",height);

    if( !box.hasMember("quaternion") )
    {
      ROS_WARN("The element #%zu has not the field 'name'", i);
      continue;
    }

    std::vector<double> position;
    if( !rosparam_utilities::getParamVector(box,"position",position) )
    {
      ROS_WARN("The element #%zu has not the field 'name'", i);
      continue;
    }
    assert(position.size()==3);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParamVector(box,"quaternion",quaternion) )
    {
      ROS_WARN("The element #%zu has not the field 'name'", i);
      continue;
    }
    assert(quaternion.size()==4);

    Eigen::Quaterniond q(quaternion.at(3),
                         quaternion.at(0),
                         quaternion.at(1),
                         quaternion.at(2));

    Eigen::Affine3d T_frame_box;
    T_frame_box = q;
    T_frame_box.translation()(0) = position.at(0);
    T_frame_box.translation()(1) = position.at(1);
    T_frame_box.translation()(2) = position.at(2);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0 = ros::Time::now();
    if (!listener.waitForTransform("world",frame_name,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame_name.c_str());
      continue;
    }

    try
    {
      listener.lookupTransform("world", frame_name, t0, transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    Eigen::Affine3d T_w_frame;
    tf::poseTFToEigen(transform,T_w_frame);

    Eigen::Affine3d T_w_box = T_w_frame * T_frame_box;

    manipulation_msgs::Box box_;
    box_.name = box_name;
    box_.height = height;
    box_.location.name = box_.name;
    box_.location.frame = "world";
    tf::poseEigenToMsg(T_w_box,box_.location.pose);
    boxes.push_back(box_);
  }

  if (boxes.size()!=0)
  {
    manipulation_msgs::AddBoxes add_boxes_srv;
    add_boxes_srv.request.add_boxes = boxes;

    if (!add_box_client_.call(add_boxes_srv))
      return false;

    ROS_INFO("Added %lu boxes.", boxes.size());  
  }
  else
    ROS_WARN("Can't add any box to the location manager.");
  
  
  return true;
}


bool InboundPickFromParam::readObjectFromParam()
{

  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("/inbound/objects",config))
  {
    ROS_ERROR("Unable to find /inbound/objects");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of boxed" );
    return false;
  }
  ROS_INFO("There are %d objects",config.size());

  object_loader_msgs::addObjects srv;
  std::map<std::string,std::shared_ptr<manipulation_msgs::AddObjects>> add_objs_srv;

  for(size_t i=0; i < config.size(); i++)
  {
    XmlRpc::XmlRpcValue object = config[i];
    if( object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%zu is not a struct", i);
      continue;
    }
    if( !object.hasMember("type") )
    {
      ROS_WARN("The element #%zu has not the field 'type'", i);
      continue;
    }
    std::string type = rosparam_utilities::toString(object["type"]);
    
    XmlRpc::XmlRpcValue type_config;
    if (!nh_.getParam("/" + type,type_config))
    {
      ROS_WARN("Type %s does not exist",type.c_str());
      continue;
    }

    if( !object.hasMember("inbound") )
    {
      ROS_WARN("The element #%zu has not the field 'inbound'", i);
      continue;
    }
    std::string box_name = rosparam_utilities::toString(object["inbound"]);

    if (add_objs_srv.count(box_name)==0)
      add_objs_srv.insert(std::pair<std::string,std::shared_ptr<manipulation_msgs::AddObjects>>(box_name,std::make_shared<manipulation_msgs::AddObjects>()));
    

    add_objs_srv.at(box_name)->request.box_name = box_name;
    manipulation_msgs::Object obj;
    obj.type = type;
    obj.name = type + std::to_string(i);

    if( !object.hasMember("frame") )
    {
      ROS_WARN("The element #%zu has not the field 'frame'", i);
      continue;
    }
    std::string frame_name = rosparam_utilities::toString(object["frame"]);

    ROS_INFO("Object type: %s, box name: %s, frame: %s", type.c_str(),box_name.c_str(),frame_name.c_str());


    std::vector<double> position;
    if( !rosparam_utilities::getParamVector(object,"position",position) )
    {
      ROS_WARN("pose has not the field 'position'");
      continue;
    }

    assert(position.size()==3);
    //////////////// Not anymore necessary /////////////////
    // obj.pose.position.x = position.at(0);
    // obj.pose.position.y = position.at(1);
    // obj.pose.position.z = position.at(2);

    std::vector<double> quaternion;
    if( !rosparam_utilities::getParamVector(object,"quaternion",quaternion) )
    {
      ROS_WARN("pose has not the field 'quaternion'");
      continue;
    }
    assert(quaternion.size()==4);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Time t0 = ros::Time::now();
    if (!listener.waitForTransform("world",frame_name,t0,ros::Duration(10)))
    {
      ROS_WARN("Unable to find a transform from world to %s", frame_name.c_str());
      continue;
    }

    try
    {
      listener.lookupTransform("world",frame_name,t0,transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    Eigen::Affine3d T_w_frame;
    tf::poseTFToEigen(transform,T_w_frame);

    Eigen::Quaterniond q(quaternion.at(3),
                         quaternion.at(0),
                         quaternion.at(1),
                         quaternion.at(2));
    Eigen::Affine3d T_frame_object;
    T_frame_object = q;
    T_frame_object.translation()(0) = position.at(0);
    T_frame_object.translation()(1) = position.at(1);
    T_frame_object.translation()(2) = position.at(2);

    Eigen::Affine3d T_w_object = T_w_frame * T_frame_object;

    manipulation_msgs::Grasp grasp_obj;

    if( !type_config.hasMember("grasp_poses") )
    {
      ROS_WARN("The element #%zu has not the field 'box'", i);
      continue;
    }
    if (type_config["grasp_poses"].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The param is not a list of boxed" );
      return false;
    }
    for (unsigned int ig=0;ig<type_config["grasp_poses"].size();ig++)
    {
      XmlRpc::XmlRpcValue pose = type_config["grasp_poses"][ig];

      if( !pose.hasMember("tool") )
      {
        ROS_WARN("The element #%u has not the field 'tool'", ig);
        continue;
      }
      std::string tool_name=rosparam_utilities::toString(pose["tool"]);


      std::vector<double> position;
      if( !rosparam_utilities::getParamVector(pose,"position",position) )
      {
        ROS_WARN("The element #%u has not the field 'name'", ig);
        continue;
      }
      assert(position.size()==3);

      std::vector<double> quaternion;
      if( !rosparam_utilities::getParamVector(pose,"quaternion",quaternion) )
      {
        ROS_WARN("The element #%u has not the field 'name'", ig);
        continue;
      }
      assert(quaternion.size()==4);

      Eigen::Quaterniond q(quaternion.at(3),
                           quaternion.at(0),
                           quaternion.at(1),
                           quaternion.at(2));

      Eigen::Affine3d T_obj_grasp;
      T_obj_grasp = q;
      T_obj_grasp.translation()(0) = position.at(0);
      T_obj_grasp.translation()(1) = position.at(1);
      T_obj_grasp.translation()(2) = position.at(2);

      Eigen::Affine3d T_w_grasp = T_w_object * T_obj_grasp;
 
      grasp_obj.location.name = obj.name+"_gl"+std::to_string(ig);
      grasp_obj.location.frame = "world";

      tf::poseEigenToMsg(T_w_grasp,grasp_obj.location.pose);
      grasp_obj.tool_name = tool_name;
      obj.grasping_locations.push_back(grasp_obj);
    }

    object_loader_msgs::object col_obj;
    tf::poseEigenToMsg(T_w_object,col_obj.pose.pose);
    col_obj.pose.header.frame_id = "world";
    col_obj.object_type = type;
    srv.request.objects.push_back(col_obj);

    if (!add_col_objs_client_.call(srv))
    {
      ROS_ERROR("sometimes wrong when adding collision object");
      return false;
    }
    if (!srv.response.success)
    {
      ROS_ERROR("sometimes wrong when adding collision object");
      return false;
    }

    add_objs_srv.at(box_name)->request.add_objects.push_back(obj);
    srv.request.objects.clear();
  }

  for (const std::pair<std::string,std::shared_ptr<manipulation_msgs::AddObjects>>& p: add_objs_srv)
    add_objs_client_.call(*p.second);

  return true;
}

}


