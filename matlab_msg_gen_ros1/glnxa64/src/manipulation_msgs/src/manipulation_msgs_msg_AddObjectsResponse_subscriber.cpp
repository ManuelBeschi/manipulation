// Copyright 2019-2020 The MathWorks, Inc.
// Subscriber for manipulation_msgs/AddObjectsResponse
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267) 
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif
#include "ros/ros.h"
#include "ros/transport_hints.h"
#include "MATLABSubscriberInterface.hpp"
#include "visibility_control.h"
#include "manipulation_msgs/AddObjectsResponse.h"
namespace manipulation_msgs {
namespace matlabhelper {
namespace AddObjectsResponse {
  MDArray_T get_arr(MDFactory_T& factory, const boost::shared_ptr<const manipulation_msgs::AddObjectsResponse>& msg);
} //namespace AddObjectsResponse
} //namespace matlabhelper
} //namespace manipulation_msgs
class MANIPULATION_MSGS_EXPORT manipulation_msgs_msg_AddObjectsResponse_subscriber : public MATLABSubscriberInterface {
    std::shared_ptr<ros::Subscriber> mSub;
    void* mSd;
    SendDataToMATLABFunc_T mSendDataToMATLABFunc;
  public:
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr 
    // for zero-copy transport.
    void callback(const manipulation_msgs::AddObjectsResponse::ConstPtr& msg){
      if(mSd){
        auto outArray = manipulation_msgs::matlabhelper::AddObjectsResponse::get_arr(mFactory, msg);
        appendAndSendToMATLAB(mSd, mSendDataToMATLABFunc, outArray);
      }
    }
    manipulation_msgs_msg_AddObjectsResponse_subscriber()
        : MATLABSubscriberInterface() {
    }
    virtual ~manipulation_msgs_msg_AddObjectsResponse_subscriber() {
    }
    virtual intptr_t createSubscription(const std::string& topic_name,
                                        std::shared_ptr<ros::NodeHandle> n,
                                        void* sd,
                                        SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                        uint32_t buffer_size) {
        mSd = sd;
        mSendDataToMATLABFunc = sendDataToMATLABFunc;
        mSub = std::make_shared<ros::Subscriber>(
                n->subscribe<manipulation_msgs::AddObjectsResponse>(
                            topic_name, 
                            buffer_size, 
                            &manipulation_msgs_msg_AddObjectsResponse_subscriber::callback, 
                            this,
                            ros::TransportHints().reliable().tcpNoDelay()));
        return true;
    }
};
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(manipulation_msgs_msg_AddObjectsResponse_subscriber, MATLABSubscriberInterface)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif
