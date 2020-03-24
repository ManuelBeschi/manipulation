#include <inbound_pick/inbound_pick.h>


namespace pickplace
{

GraspPose::GraspPose(const Eigen::VectorXd& jconf, const std::string &tool_name, const Eigen::Affine3d& T_w_g):
  m_jconf(jconf),
  m_tool_name(tool_name),
  m_T_w_g(T_w_g)
{
}

Object::Object(const std::string &type):
  m_type(type)
{
}


InboundBox::InboundBox(const std::string &name, const Eigen::Affine3d T_w_box, const double& height):
  m_name(name),
  m_T_w_box(T_w_box),
  m_height(height)
{
  m_T_w_box_a=m_T_w_box;
  m_T_w_box_a.translation()(2)+=m_height*1.5;
}

bool InboundBox::addObject(const ObjectPtr &object)
{
  std::string id=object->getId();
  if (m_objects.find(id)!=m_objects.end())
  {
    ROS_ERROR("this id is already used");
    return false;
  }
  m_objects.insert(std::pair<std::string,ObjectPtr>(id,object));

  std::map<std::string,std::vector<std::string>>::iterator it;
  it=m_ids_by_type.find(object->getType());
  if (it==m_ids_by_type.end())
  {
    std::vector<std::string> ids;
    ids.push_back(object->getId());
    ROS_DEBUG("[BOX %s] add object of type %s, id=%s",m_name.c_str(),object->getType().c_str(),object->getId().c_str());
    m_ids_by_type.insert(std::pair<std::string,std::vector<std::string>>(object->getType(),ids));
  }
  else
  {
    ROS_DEBUG("[BOX %s] add object of type %s, id=%s",m_name.c_str(),object->getType().c_str(),object->getId().c_str());
    it->second.push_back(object->getId());
  }
  return true;
}

bool InboundBox::addObjects(const std::vector<ObjectPtr> &objects)
{
  for (const ObjectPtr& object: objects)
    if (!addObject(object))
      return false;
  return true;
}

bool InboundBox::removeObject(const std::string &object_id)
{
  std::map<std::string,ObjectPtr>::iterator it;
  it=m_objects.find(object_id);
  if (it==m_objects.end())
  {
    ROS_ERROR("the object (id=%s) is not in the inbound box (type%s)",object_id.c_str());
    return false;
  }
  m_objects.erase(it);
  std::string type=it->second->getType();

  std::map<std::string,std::vector<std::string>>::iterator type_it;
  type_it=m_ids_by_type.find(type);
  if (type_it==m_ids_by_type.end())
  {
    ROS_ERROR("the object (id=%s) was in the inbound box, but not mapped by type (type%s)",object_id.c_str(),type.c_str());
  }
  else
  {
    std::vector<std::string>& ids=m_ids_by_type.at(type);
    std::vector<std::string>::iterator itr = std::find(ids.begin(), ids.end(), object_id);
    if (itr != ids.end())
      ids.erase(itr);
    else
      ROS_ERROR("the object (id=%s) was in the inbound box, but not mapped by type (type%s)",object_id.c_str(),type.c_str());

  }
  ROS_PROTO("object %s removed from the inbound box",object_id.c_str());

  return true;
}

std::vector<ObjectPtr> InboundBox::getObjectsByType(const std::string& object_type)
{
  std::vector<ObjectPtr> objects;
  std::map<std::string,std::vector<std::string>>::iterator type_it;
  type_it=m_ids_by_type.find(object_type);
  if (type_it==m_ids_by_type.end())
  {
    return objects;
  }
  else
  {
    std::vector<std::string> ids=m_ids_by_type.at(object_type);
    for (const std::string& id: ids)
    {
      std::map<std::string,ObjectPtr>::iterator it;
      it=m_objects.find(id);
      if (it==m_objects.end())
        ROS_ERROR("the object (id=%s, type=%s) was mapped by type, but not in the inbound box list",id.c_str(),object_type.c_str());
      else
        objects.push_back(m_objects.at(id));
    }
  }
  return objects;

}

std::vector<ObjectPtr> InboundBox::getObjectsByTypes(const std::vector<std::string>& object_types)
{
  std::vector<ObjectPtr> objects;
  for (const std::string& type: object_types)
  {
    std::vector<ObjectPtr> objects_type=getObjectsByType(type);
    objects.insert(objects.end(), objects_type.begin(), objects_type.end());
  }
  return objects;
}

std::vector<ObjectPtr> InboundBox::getAllObjects()
{
  std::vector<ObjectPtr> objects;
  for (const std::pair<std::string,ObjectPtr> obj: m_objects)
    objects.push_back(obj.second);
  return objects;
}

std::ostream& operator<<  (std::ostream&os, const InboundBox& box)
{
  os  << std::endl << "box name = " << box.m_name << std::endl;
  os << "box poses number = " << box.m_jconfs.size() << std::endl;
  for (const Eigen::VectorXd& jconf: box.m_jconfs)
  {
    os << "configuration = " <<jconf.transpose() << std::endl;
  }
  os << "list of objects: " << std::endl;
  for (const std::pair<std::string,ObjectPtr> obj: box.m_objects)
  {
    os << "type = " << obj.second->getType() << ", id = " << obj.second->getId() << std::endl;
  }
  return os;
}
}
