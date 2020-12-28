#include <manipulation_utils/manipulation_utils.h>


namespace manipulation
{

GraspPose::GraspPose(const Eigen::VectorXd& jconf, const std::vector<Eigen::VectorXd>& approach_jconf, const std::string &tool_name, const Eigen::Affine3d& T_w_g, const Eigen::Affine3d& T_w_a):
  m_jconf(jconf),
  m_approach_jconf(approach_jconf),
  m_tool_name(tool_name),
  m_T_w_g(T_w_g),
  m_T_w_a(T_w_a)
{
}


Object::Object(const std::string &type):
               m_type(type)
{
}

void Object::add(const std::string& group_name, const GraspPosePtr& grasp_pose)
{
  if (m_grasp_poses.find(group_name)!=m_grasp_poses.end())
  {
    ROS_DEBUG("add grasp pose for object %s id %s and group %s",m_type.c_str(),m_id.c_str(),group_name.c_str());
    m_grasp_poses.at(group_name).push_back(grasp_pose);
  }
  else
  {
    ROS_DEBUG("add first grasp pose for object %s id %s and group %s",m_type.c_str(),m_id.c_str(),group_name.c_str());
    std::vector<GraspPosePtr> v;
    v.push_back(grasp_pose);
    m_grasp_poses.insert(std::pair<std::string,std::vector<GraspPosePtr>>(group_name,v));
  }
}

std::vector<GraspPosePtr> Object::getGraspPoses(const std::string& group_name)
{
  assert(m_grasp_poses.find(group_name)!=m_grasp_poses.end());
  ROS_DEBUG("there are %zu grasp poses for object %s id %s and group %s",m_grasp_poses.at(group_name).size(),m_type.c_str(),m_id.c_str(),group_name.c_str());

  return m_grasp_poses.at(group_name);
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
    ROS_ERROR("this id is already used. id=%s, type=%s",object->getId().c_str(),object->getType().c_str());
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

void InboundBox::removeAllObjects()
{
  m_objects.clear();
  m_ids_by_type.clear();
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

void InboundBox::setConfigurations(const std::string& group_name, const std::vector<Eigen::VectorXd >& sols)
{
  std::map<std::string, std::vector<Eigen::VectorXd >>::iterator it=m_jconfs.find(group_name);
  if (it!=m_jconfs.end())
  {
    it->second=sols;
  }
  else
  {
    m_jconfs.insert(std::pair<std::string,std::vector<Eigen::VectorXd >>(group_name,sols));
  }
}
std::vector<Eigen::VectorXd > InboundBox::getConfigurations(const std::string& group_name)
{
  assert(m_jconfs.find(group_name)!=m_jconfs.end());
  return m_jconfs.at(group_name);
}


std::ostream& operator<<  (std::ostream&os, const InboundBox& box)
{
  os  << std::endl << "box name = " << box.m_name << std::endl;
  os << "box poses number = " << box.m_jconfs.size() << std::endl;

  for (const std::pair<std::string,std::vector<Eigen::VectorXd >>& p: box.m_jconfs)
  {
    os << " group = " << p.first << std::endl;
    for (const Eigen::VectorXd& jconf: p.second)
    {
      os << "configuration = " <<jconf.transpose() << std::endl;
    }
  }
  os << "list of objects: " << std::endl;
  for (const std::pair<std::string,ObjectPtr> obj: box.m_objects)
  {
    os << "type = " << obj.second->getType() << ", id = " << obj.second->getId() << std::endl;
  }
  return os;
}

}
