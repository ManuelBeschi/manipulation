#pragma once
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

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#define ROS_PROTO(...) ROS_LOG(::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

namespace pickplace {

class GraspPose
{
protected:
  std::string m_tool_name;
  Eigen::VectorXd m_jconf;
  Eigen::Affine3d m_T_w_g;
public:
  /* jconf: joint configuration to pick the object
   */
  GraspPose(const Eigen::VectorXd& jconf, const std::string& tool_name, const Eigen::Affine3d& T_w_g);
  std::string getToolName(){return m_tool_name;}
  Eigen::VectorXd getConfiguration(){return m_jconf;}
  Eigen::Affine3d getPose(){return m_T_w_g;}

};
typedef std::shared_ptr<GraspPose> GraspPosePtr;

class Object {
protected:
  std::string m_type;
  std::string m_id;
  std::vector<GraspPosePtr> m_grasp_poses;
public:
  Object(const std::string& type);

  std::string getType(){return m_type;}
  std::string getId(){return m_id;}
  void setId(const std::string& id){m_id=id;}
  void add(const GraspPosePtr& grasp_pose){m_grasp_poses.push_back(grasp_pose);}
  std::vector<GraspPosePtr> getGraspPoses(){return m_grasp_poses;}
};

typedef std::shared_ptr<Object> ObjectPtr;

class InboundBox
{
protected:
  std::map<std::string,std::vector<std::string>> m_ids_by_type;
  std::map<std::string,ObjectPtr> m_objects;
  std::string m_name;
  std::vector<Eigen::VectorXd > m_jconfs;
  Eigen::Affine3d m_T_w_box;
  Eigen::Affine3d m_T_w_box_a;
  double m_height;
  unsigned int m_id=0;

  void assignId();
public:
  /* name : name of the inbound box
   * jconfs: joint configurations of the approach pose
   */
  InboundBox(const std::string& name,
             const Eigen::Affine3d T_w_box,
             const double& height);

  /* get inbound-box's name
   */
  std::string getName(){return m_name;}

  /* add an object
   */
  void addObject(const ObjectPtr& object);

  /* add a list of objects
   */
  void addObjects(const std::vector<ObjectPtr>& objects);

  /* remove object with a specific id, return false if the object is not in the inbound box
  */
  bool removeObject(const std::string& object_id);

  /* get all the objects of this  type
  */
  std::vector<ObjectPtr> getObjectsByType(const std::string& object_type);

  /* get all the objects of these types
  */
  std::vector<ObjectPtr> getObjectsByTypes(const std::vector<std::string>& object_types);

  /* get all objects
   */
  std::vector<ObjectPtr> getAllObjects();


  void setConfigurations(const std::vector<Eigen::VectorXd >& sols){m_jconfs=sols;}
  std::vector<Eigen::VectorXd > getConfigurations(){return m_jconfs;}

  Eigen::Affine3d getPose(){return m_T_w_box;}
  Eigen::Affine3d getApproachPose(){return m_T_w_box_a;}


  friend std::ostream& operator<<  (std::ostream& os, const InboundBox& box);

};

typedef std::shared_ptr<InboundBox> InboundBoxPtr;

}
