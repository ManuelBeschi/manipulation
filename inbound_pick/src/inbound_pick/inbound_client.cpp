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
#include <inbound_pick/inbound_client.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

namespace pickplace {

InboundFromParam::InboundFromParam(const ros::NodeHandle &nh):
  nh_(nh)
{
  add_objs_client_=nh_.serviceClient<manipulation_msgs::AddObjects>("add_objects");
  add_box_client_ =nh_.serviceClient<manipulation_msgs::AddBox>("add_box");

}
bool InboundFromParam::readBoxesFromParam()
{

  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("inbound/boxes",config))
  {
    ROS_ERROR("unable to find inboud/boxed");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of boxed" );
    return false;
  }
  ROS_INFO("there are %zu boxes",config.size());
  ROS_INFO_STREAM("config:\n"<<config);
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

    std::string box_name=rosparam_utilities::toString(box["name"]);
    ROS_INFO("box named =%s",box_name.c_str());
    if( !box.hasMember("heigth") )
    {
      ROS_WARN("The element #%zu has not the field 'heigth'", i);
      continue;
    }

    double heigth=rosparam_utilities::toDouble(box["heigth"]);
    ROS_INFO("box picking heigth %f",heigth);

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

    Eigen::Affine3d T_w_box;
    T_w_box=q;
    T_w_box.translation()(0)=position.at(0);
    T_w_box.translation()(1)=position.at(1);
    T_w_box.translation()(2)=position.at(2);


    manipulation_msgs::AddBox box_srv;
    box_srv.request.inbound_box_name=box_name;
    tf::poseEigenToMsg(T_w_box,box_srv.request.box_pose);
    ROS_INFO_STREAM("Box position\n"<<box_srv.request.box_pose);
    box_srv.request.height=heigth;
    if (!add_box_client_.call(box_srv))
      return false;


  }

  return true;
}



bool InboundFromParam::readObjectFromParam()
{

  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("inbound/objects",config))
  {
    ROS_ERROR("unable to find inboud/boxed");
    return false;
  }

  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The param is not a list of boxed" );
    return false;
  }
  ROS_INFO("there are %zu objects",config.size());
  ROS_INFO_STREAM("config:\n"<<config);
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
      ROS_WARN("The element #%zu has not the field 'name'", i);
      continue;
    }
    std::string type=rosparam_utilities::toString(object["type"]);
    ROS_INFO("Object type = %s",type.c_str());


    if( !object.hasMember("box") )
    {
      ROS_WARN("The element #%zu has not the field 'box'", i);
      continue;
    }
    std::string box_name=rosparam_utilities::toString(object["box"]);
    ROS_INFO("inside box = %s",box_name.c_str());


    manipulation_msgs::AddObjects add_objs_srv;
    add_objs_srv.request.inbound_box_name=box_name;
    manipulation_msgs::Object obj;
    obj.type=type;



    ROS_INFO("object type =%s, box name =%s",type.c_str(),box_name.c_str());


    if( !object.hasMember("grasp_poses") )
    {
      ROS_WARN("The element #%zu has not the field 'box'", i);
      continue;
    }
    if (object["grasp_poses"].getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The param is not a list of boxed" );
      return false;
    }
    for (unsigned int ig=0;ig<object["grasp_poses"].size();ig++)
    {
      XmlRpc::XmlRpcValue pose = object["grasp_poses"][ig];

      if( !pose.hasMember("tool") )
      {
        ROS_WARN("The element #%zu has not the field 'tool'", ig);
        continue;
      }
      std::string tool_name=rosparam_utilities::toString(pose["tool"]);


      std::vector<double> position;
      if( !rosparam_utilities::getParamVector(pose,"position",position) )
      {
        ROS_WARN("The element #%zu has not the field 'name'", ig);
        continue;
      }
      assert(position.size()==3);

      std::vector<double> quaternion;
      if( !rosparam_utilities::getParamVector(pose,"quaternion",quaternion) )
      {
        ROS_WARN("The element #%zu has not the field 'name'", ig);
        continue;
      }
      assert(quaternion.size()==4);

      Eigen::Quaterniond q(quaternion.at(3),
                           quaternion.at(0),
                           quaternion.at(1),
                           quaternion.at(2));

      Eigen::Affine3d T_w_obj;
      T_w_obj=q;
      T_w_obj.translation()(0)=position.at(0);
      T_w_obj.translation()(1)=position.at(1);
      T_w_obj.translation()(2)=position.at(2);


      manipulation_msgs::Grasp grasp_obj;
      tf::poseEigenToMsg(T_w_obj,grasp_obj.pose);
      grasp_obj.tool_name=tool_name;
      obj.grasping_poses.push_back(grasp_obj);
    }

    add_objs_srv.request.add_objects.push_back(obj);
    add_objs_client_.call(add_objs_srv);


  }


  return true;
}

}


