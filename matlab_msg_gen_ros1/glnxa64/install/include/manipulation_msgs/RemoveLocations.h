// Generated by gencpp from file manipulation_msgs/RemoveLocations.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MSGS_MESSAGE_REMOVELOCATIONS_H
#define MANIPULATION_MSGS_MESSAGE_REMOVELOCATIONS_H

#include <ros/service_traits.h>


#include <manipulation_msgs/RemoveLocationsRequest.h>
#include <manipulation_msgs/RemoveLocationsResponse.h>


namespace manipulation_msgs
{

struct RemoveLocations
{

typedef RemoveLocationsRequest Request;
typedef RemoveLocationsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RemoveLocations
} // namespace manipulation_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::manipulation_msgs::RemoveLocations > {
  static const char* value()
  {
    return "8c80bdc7cee467e52ed1c783520ee4bb";
  }

  static const char* value(const ::manipulation_msgs::RemoveLocations&) { return value(); }
};

template<>
struct DataType< ::manipulation_msgs::RemoveLocations > {
  static const char* value()
  {
    return "manipulation_msgs/RemoveLocations";
  }

  static const char* value(const ::manipulation_msgs::RemoveLocations&) { return value(); }
};


// service_traits::MD5Sum< ::manipulation_msgs::RemoveLocationsRequest> should match 
// service_traits::MD5Sum< ::manipulation_msgs::RemoveLocations > 
template<>
struct MD5Sum< ::manipulation_msgs::RemoveLocationsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::manipulation_msgs::RemoveLocations >::value();
  }
  static const char* value(const ::manipulation_msgs::RemoveLocationsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::manipulation_msgs::RemoveLocationsRequest> should match 
// service_traits::DataType< ::manipulation_msgs::RemoveLocations > 
template<>
struct DataType< ::manipulation_msgs::RemoveLocationsRequest>
{
  static const char* value()
  {
    return DataType< ::manipulation_msgs::RemoveLocations >::value();
  }
  static const char* value(const ::manipulation_msgs::RemoveLocationsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::manipulation_msgs::RemoveLocationsResponse> should match 
// service_traits::MD5Sum< ::manipulation_msgs::RemoveLocations > 
template<>
struct MD5Sum< ::manipulation_msgs::RemoveLocationsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::manipulation_msgs::RemoveLocations >::value();
  }
  static const char* value(const ::manipulation_msgs::RemoveLocationsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::manipulation_msgs::RemoveLocationsResponse> should match 
// service_traits::DataType< ::manipulation_msgs::RemoveLocations > 
template<>
struct DataType< ::manipulation_msgs::RemoveLocationsResponse>
{
  static const char* value()
  {
    return DataType< ::manipulation_msgs::RemoveLocations >::value();
  }
  static const char* value(const ::manipulation_msgs::RemoveLocationsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MANIPULATION_MSGS_MESSAGE_REMOVELOCATIONS_H
