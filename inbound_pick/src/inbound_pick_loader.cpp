#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/AddBox.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <inbound_pick/inbound_client.h>
//bool getInboundBoxedFromParam()

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_client");
  ros::NodeHandle nh;

  ros::ServiceClient add_objs_client=nh.serviceClient<manipulation_msgs::AddObjects>("add_objects");
  ros::ServiceClient add_box_client=nh.serviceClient<manipulation_msgs::AddBox>("add_box");



  pickplace::InboundFromParam inb(nh);
  if (!inb.readBoxesFromParam())
  {
    ROS_ERROR("Unable to load boxed");
    return 0;
  }
  if (!inb.readObjectFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return 0;
  }

  ROS_INFO("Inbound boxed loaded");
  return 0;
}
