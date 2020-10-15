// Generated by gencpp from file manipulation_msgs/Object.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MSGS_MESSAGE_OBJECT_H
#define MANIPULATION_MSGS_MESSAGE_OBJECT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <manipulation_msgs/Grasp.h>
#include <geometry_msgs/Pose.h>

namespace manipulation_msgs
{
template <class ContainerAllocator>
struct Object_
{
  typedef Object_<ContainerAllocator> Type;

  Object_()
    : grasping_poses()
    , type()
    , id()
    , pose()  {
    }
  Object_(const ContainerAllocator& _alloc)
    : grasping_poses(_alloc)
    , type(_alloc)
    , id(_alloc)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::manipulation_msgs::Grasp_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::manipulation_msgs::Grasp_<ContainerAllocator> >::other >  _grasping_poses_type;
  _grasping_poses_type grasping_poses;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::manipulation_msgs::Object_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_msgs::Object_<ContainerAllocator> const> ConstPtr;

}; // struct Object_

typedef ::manipulation_msgs::Object_<std::allocator<void> > Object;

typedef boost::shared_ptr< ::manipulation_msgs::Object > ObjectPtr;
typedef boost::shared_ptr< ::manipulation_msgs::Object const> ObjectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::manipulation_msgs::Object_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::manipulation_msgs::Object_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace manipulation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'manipulation_msgs': ['/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::manipulation_msgs::Object_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::manipulation_msgs::Object_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::Object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::Object_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::Object_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::Object_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::manipulation_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bf23ef95064f4210d1bed46aedd97d9f";
  }

  static const char* value(const ::manipulation_msgs::Object_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbf23ef95064f4210ULL;
  static const uint64_t static_value2 = 0xd1bed46aedd97d9fULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "manipulation_msgs/Object";
  }

  static const char* value(const ::manipulation_msgs::Object_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::manipulation_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "manipulation_msgs/Grasp[] grasping_poses\n"
"string type\n"
"string id\n"
"geometry_msgs/Pose pose\n"
"\n"
"================================================================================\n"
"MSG: manipulation_msgs/Grasp\n"
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

  static const char* value(const ::manipulation_msgs::Object_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::manipulation_msgs::Object_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.grasping_poses);
      stream.next(m.type);
      stream.next(m.id);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Object_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manipulation_msgs::Object_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::manipulation_msgs::Object_<ContainerAllocator>& v)
  {
    s << indent << "grasping_poses[]" << std::endl;
    for (size_t i = 0; i < v.grasping_poses.size(); ++i)
    {
      s << indent << "  grasping_poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::manipulation_msgs::Grasp_<ContainerAllocator> >::stream(s, indent + "    ", v.grasping_poses[i]);
    }
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MANIPULATION_MSGS_MESSAGE_OBJECT_H
