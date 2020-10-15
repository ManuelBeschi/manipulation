// Generated by gencpp from file manipulation_msgs/RemoveLocationsResponse.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MSGS_MESSAGE_REMOVELOCATIONSRESPONSE_H
#define MANIPULATION_MSGS_MESSAGE_REMOVELOCATIONSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace manipulation_msgs
{
template <class ContainerAllocator>
struct RemoveLocationsResponse_
{
  typedef RemoveLocationsResponse_<ContainerAllocator> Type;

  RemoveLocationsResponse_()
    {
    }
  RemoveLocationsResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RemoveLocationsResponse_

typedef ::manipulation_msgs::RemoveLocationsResponse_<std::allocator<void> > RemoveLocationsResponse;

typedef boost::shared_ptr< ::manipulation_msgs::RemoveLocationsResponse > RemoveLocationsResponsePtr;
typedef boost::shared_ptr< ::manipulation_msgs::RemoveLocationsResponse const> RemoveLocationsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace manipulation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/usr/local/MATLAB/R2020b/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'manipulation_msgs': ['/home/jacobi/projects/ros_ws/src/manipulation/matlab_msg_gen_ros1/glnxa64/src/manipulation_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "manipulation_msgs/RemoveLocationsResponse";
  }

  static const char* value(const ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
;
  }

  static const char* value(const ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RemoveLocationsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::manipulation_msgs::RemoveLocationsResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // MANIPULATION_MSGS_MESSAGE_REMOVELOCATIONSRESPONSE_H
