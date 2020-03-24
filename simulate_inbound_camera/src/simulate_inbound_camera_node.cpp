#include <ros/ros.h>
#include <object_loader_msgs/addObjects.h>
#include <manipulation_msgs/AddObjects.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulate_inbound_camera_node");
  ros::NodeHandle nh;
  return 0;
}
