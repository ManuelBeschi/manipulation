#include <algorithm>

#include <ros/ros.h>

#include <manipulation_msgs/AddLocations.h>
#include <manipulation_msgs/RemoveLocations.h>

#include <manipulation_utils/manipulation_utils.h>

namespace manipulation
{

bool removeLocation(const std::string& location_name)
{
  if(!ros::service::exists("remove_locations",true))
  {
    manipulation_msgs::RemoveLocations::Request req;
    manipulation_msgs::RemoveLocations::Response res;
    
    req.location_names.push_back(location_name);

    if(!ros::service::call("remove_locations",req,res))
    {
      ROS_ERROR("Can't remove the location %s from the location manager.",location_name.c_str());
      return false;
    }
    ROS_INFO("Removed the location %s from the location manager.",location_name.c_str());
  }
  else
  {
    ROS_ERROR("The service remove_locations is not available.");
    return false;
  }
  
  return true; 
}

Grasp::Grasp(const manipulation_msgs::Grasp& grasp)
{
  if(!ros::service::exists("add_locations",true))
  {
    manipulation_msgs::AddLocations::Request req;
    manipulation_msgs::AddLocations::Response res;

    req.locations.push_back(grasp.location);

    if(!ros::service::call("add_locations",req,res))
    {
      ROS_ERROR("Can't add the location %s from the location manager.",grasp.location.name.c_str());
      return;
    }
    
    m_tool_name = grasp.tool_name;
    m_location_name = grasp.location.name;
    ROS_INFO("Added the new location %s to the location manager.",grasp.location.name.c_str());  
  }
  else
    ROS_ERROR("The service add_locations is not available.");

  return;
}

Grasp::~Grasp()
{
  if(!removeLocation(m_location_name))
    ROS_ERROR("Can't remove the location %s from location manager.", m_location_name.c_str());
}

Object::Object( const manipulation_msgs::Object& object )
{
  m_name = object.name;
  m_type = object.type;

  for (const manipulation_msgs::Grasp grasp: object.grasping_locations )
    m_grasp.push_back(std::make_shared<Grasp>(grasp));
  
  ROS_INFO("Added the object: %s of the type: %s.", m_name.c_str(), m_type.c_str()); 
}

Object::~Object()
{
  // nothing to do 
}

std::vector<std::string> Object::getGraspLocationNames()
{
  std::vector<std::string> grasp_location_names;

  for (const manipulation::GraspPtr& grasp: m_grasp )
    grasp_location_names.push_back(grasp->getLocationName());

  return grasp_location_names;
}

manipulation::GraspPtr Object::getGrasp(const std::string& grasp_location_name)
{
  for(const manipulation::GraspPtr& grasp: m_grasp)
  {
    if (grasp->getLocationName() == grasp_location_name)
      return grasp;
  }
  
  ROS_ERROR("Can't find grasping location %s in the location manager.", grasp_location_name.c_str());
  return nullptr;
}

Box::Box(const manipulation_msgs::Box& box)
{

  if(!ros::service::exists("add_locations",true))
  {
    manipulation_msgs::AddLocations::Request req;
    manipulation_msgs::AddLocations::Response res;

    req.locations.push_back(box.location);

    if(!ros::service::call("add_locations",req,res))
    {
      ROS_ERROR("Can't add the location %s to the location manager.",box.location.name.c_str());
      return;
    }

    m_name = box.name;
    m_height = box.height;
    m_location_name = box.location.name;

    ROS_INFO("Added the location %s to the location manager.",box.location.name.c_str());
  }
  else
  {
    ROS_ERROR("The service add_locations is not available.");
    return;
  }
    
  for (const manipulation_msgs::Object object: box.objects )
    m_objects.insert(std::pair<std::string,ObjectPtr>(object.name, std::make_shared<Object>(object)));
  
   ROS_INFO("Added the box %s.", m_name.c_str()); 
}

Box::~Box()
{
  if(!removeLocation(m_location_name))
    ROS_ERROR("Can't remove the location %s from location manager.", m_location_name.c_str());
}

bool Box::addObject(const manipulation_msgs::Object& object)
{
  if (m_objects.find(object.name) != m_objects.end())
  {
    ROS_ERROR("The object: %s of the type: %s already exists in the box %s.", object.name.c_str(),object.type.c_str(),m_name.c_str());
    return false;
  }

  m_objects.insert(std::pair<std::string,ObjectPtr>(object.name,std::make_shared<Object>(object)));

  return true;
}

 bool Box::addObject(const manipulation::ObjectPtr& object)
 {  
  if (m_objects.find(object->getName()) != m_objects.end())
  {
    ROS_ERROR("The object: %s of the type: %s already exists in the box %s.", object->getName().c_str(),object->getType().c_str(),m_name.c_str());
    return false;
  }

  m_objects.insert(std::pair<std::string,ObjectPtr>(object->getName(),object));

  return true;

 }

void Box::removeAllObjects()
{
  m_objects.clear();
  ROS_INFO("All the objects in the box %s are removed", m_name.c_str());
}

bool Box::removeObject(const std::string& object_name)
{
  if (m_objects.find(object_name) == m_objects.end())
  {
    ROS_ERROR("The object %s is not in the box %s", object_name.c_str(), m_name.c_str());
    return false;
  }

  m_objects.erase(m_objects.find(object_name));
  ROS_INFO("The object %s has been removed from the box %s", object_name.c_str(), m_name.c_str());

  return true;
}

bool Box::findObject(const std::string& object_name)
{
  if (m_objects.find(object_name) == m_objects.end())
  {
    ROS_INFO("Can't find the object %s in the box %s", object_name.c_str(), m_name.c_str());
    return false;
  }

  ROS_INFO("Found the object %s in the box %s", object_name.c_str(), m_name.c_str());

  return true;
}

std::string Box::findObjectByGraspingLocation(const std::string& grasp_location_name)
{
  for(std::map<std::string,ObjectPtr>::iterator it = m_objects.begin(); it != m_objects.end(); it++)
  {
    std::vector<std::string> location_names = it->second->getGraspLocationNames();
    if (find(location_names.begin(),location_names.end(),grasp_location_name) != location_names.end() )
      return it->first;
  }
  ROS_ERROR("Can't find the corresponding object for grasping location name %s ", grasp_location_name.c_str());
  return std::string();
}

std::vector<ObjectPtr> Box::getAllObjects()
{
  std::vector<ObjectPtr> objects;
  for (const std::pair<std::string,ObjectPtr> obj: m_objects)
    objects.push_back(obj.second);
  return objects;
}


std::vector<ObjectPtr> Box::getObjectsByType(const std::string& object_type)
{
  std::vector<ObjectPtr> objects;

  for (std::map<std::string,ObjectPtr>::iterator it = m_objects.begin(); it != m_objects.end(); it++)
  {
    if ( it->second->getType() == object_type )
      objects.push_back(it->second);
  }  
  return objects;

}

}
