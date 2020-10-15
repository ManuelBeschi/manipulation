// Generated by gencpp from file manipulation_msgs/ListOfObjectsResponse.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MSGS_MESSAGE_LISTOFOBJECTSRESPONSE_H
#define MANIPULATION_MSGS_MESSAGE_LISTOFOBJECTSRESPONSE_H


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
struct ListOfObjectsResponse_
{
  typedef ListOfObjectsResponse_<ContainerAllocator> Type;

  ListOfObjectsResponse_()
    : type()
    , id()
    , box()  {
    }
  ListOfObjectsResponse_(const ContainerAllocator& _alloc)
    : type(_alloc)
    , id(_alloc)
    , box(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _type_type;
  _type_type type;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _id_type;
  _id_type id;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _box_type;
  _box_type box;





  typedef boost::shared_ptr< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ListOfObjectsResponse_

typedef ::manipulation_msgs::ListOfObjectsResponse_<std::allocator<void> > ListOfObjectsResponse;

typedef boost::shared_ptr< ::manipulation_msgs::ListOfObjectsResponse > ListOfObjectsResponsePtr;
typedef boost::shared_ptr< ::manipulation_msgs::ListOfObjectsResponse const> ListOfObjectsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "40e4d9b5f6e0112bad855c1d374ac612";
  }

  static const char* value(const ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x40e4d9b5f6e0112bULL;
  static const uint64_t static_value2 = 0xad855c1d374ac612ULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "manipulation_msgs/ListOfObjectsResponse";
  }

  static const char* value(const ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] type\n"
"string[] id\n"
"string[] box\n"
"\n"
;
  }

  static const char* value(const ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.id);
      stream.next(m.box);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ListOfObjectsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::manipulation_msgs::ListOfObjectsResponse_<ContainerAllocator>& v)
  {
    s << indent << "type[]" << std::endl;
    for (size_t i = 0; i < v.type.size(); ++i)
    {
      s << indent << "  type[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type[i]);
    }
    s << indent << "id[]" << std::endl;
    for (size_t i = 0; i < v.id.size(); ++i)
    {
      s << indent << "  id[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id[i]);
    }
    s << indent << "box[]" << std::endl;
    for (size_t i = 0; i < v.box.size(); ++i)
    {
      s << indent << "  box[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.box[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MANIPULATION_MSGS_MESSAGE_LISTOFOBJECTSRESPONSE_H