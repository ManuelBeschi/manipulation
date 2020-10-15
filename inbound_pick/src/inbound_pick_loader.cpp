#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/AddBox.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <inbound_pick/inbound_client.h>
#include <std_srvs/SetBool.h>


std::shared_ptr<pickplace::InboundFromParam> inb;

bool addObjectsCb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
{
  if (!inb->readObjectFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return false;
  }
  ROS_INFO("load objects complete");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_client");
  ros::NodeHandle nh;

  inb=std::make_shared<pickplace::InboundFromParam>(nh);



  if (!inb->readBoxesFromParam())
  {
    ROS_ERROR("Unable to load boxed");
    return 0;
  }
  if (!inb->readObjectFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return 0;
  }

  ROS_INFO("Inbound boxed loaded");

  ros::ServiceServer src=nh.advertiseService("inbound/add_objects",&addObjectsCb);
  ros::spin();
  return 0;
}
