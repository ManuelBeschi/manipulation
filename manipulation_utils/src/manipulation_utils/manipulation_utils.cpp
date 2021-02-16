#include <algorithm>

#include <ros/ros.h>

#include <manipulation_msgs/AddLocations.h>
#include <manipulation_msgs/RemoveLocations.h>

#include <manipulation_utils/manipulation_utils.h>

namespace manipulation
{

bool addLocation( const ros::NodeHandle& nh,
                  const manipulation_msgs::Location& location)
{  
  ros::NodeHandle nh_= nh; 
  ros::ServiceClient add_locations_client = nh_.serviceClient<manipulation_msgs::AddLocations>("add_locations");

  if(add_locations_client.exists())
  {
    manipulation_msgs::AddLocations add_locations;
    add_locations.request.locations.push_back(location);

    if(!add_locations_client.call(add_locations))
      return false;
    
    if (add_locations.response.results != manipulation_msgs::AddLocations::Response::Error)
      ROS_INFO("Added the location %s to the location manager.",location.name.c_str());
  }
  else
  {
    ROS_ERROR("The service %s is not available.",add_locations_client.getService().c_str());
    return false;
  }

  return true;
}

bool removeLocation(const ros::NodeHandle& nh,
                    const std::string& location_name)
{
  ros::NodeHandle nh_= nh; 
  ros::ServiceClient remove_locations_client = nh_.serviceClient<manipulation_msgs::RemoveLocations>("remove_locations");
  
  if(remove_locations_client.exists())
  {
    manipulation_msgs::RemoveLocations remove_location;
    remove_location.request.location_names.push_back(location_name);

    if(!remove_locations_client.call(remove_location))
      return false;
    
    if (remove_location.response.results != manipulation_msgs::RemoveLocations::Response::Error)
      ROS_INFO("Removed the location %s from the location manager.",location_name.c_str());
  }
  else
  {
    ROS_ERROR("The service %s is not available.", remove_locations_client.getService().c_str());
    return false;
  }
  
  return true; 
}

Grasp::Grasp( const ros::NodeHandle& nh,
              const manipulation_msgs::Grasp& grasp):
              m_nh(nh),
              m_int_state(true)
{
  if(!addLocation(m_nh,grasp.location))
  {
    m_int_state = false;
    return;
  }
  
  m_tool_name = grasp.tool_name;
  m_location_name = grasp.location.name;
  ROS_INFO("Added the new location %s to the location manager.",grasp.location.name.c_str());

  return;
}

Grasp::~Grasp()
{
  if (m_int_state)
  {
    if(!removeLocation(m_nh, m_location_name))
      ROS_ERROR("Can't remove the location %s from location manager.", m_location_name.c_str());
  }
}

Object::Object( const ros::NodeHandle& nh,
                const manipulation_msgs::Object& object ):
                m_nh(nh),
                m_int_state(true)
{
  m_name = object.name;
  m_type = object.type;

  for (const manipulation_msgs::Grasp grasp: object.grasping_locations )
  {
    m_grasp.push_back(std::make_shared<manipulation::Grasp>(m_nh,grasp));
    if(!m_grasp.back()->getIntState())
       m_grasp.pop_back();
  }
   
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

Box::Box( const ros::NodeHandle& nh,
          const manipulation_msgs::Box& box):
          m_nh(nh),
          m_int_state(true)
{
  if(!addLocation(m_nh,box.location))
  {
    m_int_state = false;
    return;
  }
 
  m_name = box.name;
  m_height = box.height;
  m_location_name = box.location.name;
    
  for (const manipulation_msgs::Object object: box.objects)
  {
    m_objects.insert(std::pair<std::string,ObjectPtr>(object.name, std::make_shared<manipulation::Object>(m_nh,object)));
    if (!m_objects.at(object.name)->getIntState())
      m_objects.erase(m_objects.find(object.name));
  }
  
  ROS_INFO("Added the box %s.", m_name.c_str()); 
}

Box::~Box()
{
  if (m_int_state)
  {
    if(!removeLocation(m_nh,m_location_name))
      ROS_ERROR("Can't remove the location %s from location manager.", m_location_name.c_str());
  }
}

bool Box::addObject(const manipulation_msgs::Object& object)
{
  if (m_objects.find(object.name) != m_objects.end())
  {
    ROS_ERROR("The object: %s of the type: %s already exists in the box %s.", object.name.c_str(),object.type.c_str(),m_name.c_str());
    return false;
  }

  m_objects.insert(std::pair<std::string,ObjectPtr>(object.name,std::make_shared<Object>(m_nh,object)));

  if (!m_objects.at(object.name)->getIntState())
    m_objects.erase(m_objects.find(object.name));

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
  if (!m_objects.at(object->getName())->getIntState())
      m_objects.erase(m_objects.find(object->getName()));

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
