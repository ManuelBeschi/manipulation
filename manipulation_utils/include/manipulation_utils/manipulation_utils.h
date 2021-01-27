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

#include <manipulation_msgs/Grasp.h>
#include <manipulation_msgs/Object.h>
#include <manipulation_msgs/Box.h>

//#define ROS_PROTO(...) ROS_LOG(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
//#define ROS_PROTO_STREAM(...) ROS_LOG_STREAM(::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

  // To be checked
#define N_MAX_ITER 2000
#define N_ITER 30
#define TOLERANCE 1e-6

namespace manipulation 
{

  /* remove a grasp location from the location manager
  */
  bool removeLocation(const std::string& location_name);
  class Grasp
  {
  protected:
    std::string m_tool_name;
    std::string m_location_name;  // to keep trace about the location inserted in the LocationManager

  public:
    /* constructor
    */
    Grasp(const manipulation_msgs::Grasp& grasp);
      
    /* destructor
    */
    ~Grasp();

    /* get the tool name to grasp the object
    */
    std::string getToolName(){return m_tool_name;}

    /* get the location name
    */
    std::string getLocationName(){return m_location_name;}

  };
  typedef std::shared_ptr<Grasp> GraspPtr;

  class Object 
  {
  protected:
    std::string m_name;
    std::string m_type;

    std::vector<GraspPtr> m_grasp;
  
  public:
    /* constructor
    */
    Object(const manipulation_msgs::Object& object);

    /* destructor
    */
    ~Object();

    /* get the name of the object
    */
    std::string getName(){return m_name;}

    /* get the name of the grasping location
    */
    std::vector<std::string> getGraspLocationNames();

    /* get the grasping location 
    */
    manipulation::GraspPtr getGrasp(const std::string& grasp_location_name);

    /* get the type of the object
    */
    std::string getType(){return m_type;}

  };
  typedef std::shared_ptr<Object> ObjectPtr;

  class Box
  {
  protected:
    std::string m_name;
    double m_height;
    std::string m_location_name;  // to keep trace about the location inserted in the LocationManager

    std::map<std::string,ObjectPtr> m_objects;

  public:
    /* constructor
    */
    Box(const manipulation_msgs::Box& box);

    /* destructor
    */
    ~Box();

    /* get box's name
    */
    std::string getName(){return m_name;}

    /* add an object
    */
    bool addObject(const manipulation_msgs::Object& object);

    /* add an object
    */
    bool addObject(const manipulation::ObjectPtr& object);

    /* remove all objects
    */
    void removeAllObjects();

    /* remove object with a specific name, return false if the object is not in the box
    */
    bool removeObject(const std::string& object_name);
 
    /* find an object with a specific name, return false if the object is not in the box
    */
    bool findObject(const std::string& object_name);

    /* find an object by a grasping location name, return the object name correpondig to a grasping location name
    */
    std::string findObjectByGraspingLocation(const std::string& grasp_location_name);

    /* get a specific object 
    */
    ObjectPtr getObject(const std::string& object_name){return m_objects.at(object_name);}

    /* get all objects
    */
    std::vector<ObjectPtr> getAllObjects();

    /* get all the objects of this type
    */
    std::vector<ObjectPtr> getObjectsByType(const std::string& object_type);

    /* get box height
    */
    const double& getHeight(){return m_height;}

    /* get the location name
    */
    std::string getLocationName(){return m_location_name;}


  };
  typedef std::shared_ptr<Box> BoxPtr;

}