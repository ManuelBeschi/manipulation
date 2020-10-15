// Generated by gencpp from file pickplace_msgs/AddRemoveObjectsRequest.msg
// DO NOT EDIT!


#ifndef PICKPLACE_MSGS_MESSAGE_ADDREMOVEOBJECTSREQUEST_H
#define PICKPLACE_MSGS_MESSAGE_ADDREMOVEOBJECTSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pickplace_msgs/Object.h>
#include <pickplace_msgs/Object.h>

namespace pickplace_msgs
{
template <class ContainerAllocator>
struct AddRemoveObjectsRequest_
{
  typedef AddRemoveObjectsRequest_<ContainerAllocator> Type;

  AddRemoveObjectsRequest_()
    : inbound_box_name()
    , add_objects()
    , remove_objects()  {
    }
  AddRemoveObjectsRequest_(const ContainerAllocator& _alloc)
    : inbound_box_name(_alloc)
    , add_objects(_alloc)
    , remove_objects(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _inbound_box_name_type;
  _inbound_box_name_type inbound_box_name;

   typedef std::vector< ::pickplace_msgs::Object_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pickplace_msgs::Object_<ContainerAllocator> >::other >  _add_objects_type;
  _add_objects_type add_objects;

   typedef std::vector< ::pickplace_msgs::Object_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pickplace_msgs::Object_<ContainerAllocator> >::other >  _remove_objects_type;
  _remove_objects_type remove_objects;





  typedef boost::shared_ptr< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct AddRemoveObjectsRequest_

typedef ::pickplace_msgs::AddRemoveObjectsRequest_<std::allocator<void> > AddRemoveObjectsRequest;

typedef boost::shared_ptr< ::pickplace_msgs::AddRemoveObjectsRequest > AddRemoveObjectsRequestPtr;
typedef boost::shared_ptr< ::pickplace_msgs::AddRemoveObjectsRequest const> AddRemoveObjectsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pickplace_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'pickplace_msgs': ['/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/pickplace_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "51aa5ad13aecf69d39ca9616fdb01a34";
  }

  static const char* value(const ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x51aa5ad13aecf69dULL;
  static const uint64_t static_value2 = 0x39ca9616fdb01a34ULL;
};

template<class ContainerAllocator>
struct DataType< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pickplace_msgs/AddRemoveObjectsRequest";
  }

  static const char* value(const ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string inbound_box_name\n"
"pickplace_msgs/Object[] add_objects\n"
"pickplace_msgs/Object[] remove_objects\n"
"\n"
"================================================================================\n"
"MSG: pickplace_msgs/Object\n"
"pickplace_msgs/Grasp[] grasping_poses\n"
"string type\n"
"string id\n"
"\n"
"================================================================================\n"
"MSG: pickplace_msgs/Grasp\n"
"geometry_msgs/Pose pose\n"
"string tool_name\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.inbound_box_name);
      stream.next(m.add_objects);
      stream.next(m.remove_objects);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AddRemoveObjectsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pickplace_msgs::AddRemoveObjectsRequest_<ContainerAllocator>& v)
  {
    s << indent << "inbound_box_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.inbound_box_name);
    s << indent << "add_objects[]" << std::endl;
    for (size_t i = 0; i < v.add_objects.size(); ++i)
    {
      s << indent << "  add_objects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pickplace_msgs::Object_<ContainerAllocator> >::stream(s, indent + "    ", v.add_objects[i]);
    }
    s << indent << "remove_objects[]" << std::endl;
    for (size_t i = 0; i < v.remove_objects.size(); ++i)
    {
      s << indent << "  remove_objects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pickplace_msgs::Object_<ContainerAllocator> >::stream(s, indent + "    ", v.remove_objects[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PICKPLACE_MSGS_MESSAGE_ADDREMOVEOBJECTSREQUEST_H
