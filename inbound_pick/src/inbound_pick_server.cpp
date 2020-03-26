#include <ros/ros.h>
#include <inbound_pick/inbound_pick.h>
#include <inbound_pick/pick_objects.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  pickplace::PickObjects pick(nh,pnh);

  if (!pick.init())
  {
    ROS_ERROR_NAMED(nh.getNamespace(),"unable to load parameters for node %s",pnh.getNamespace().c_str());
    return -1;
  }

  ros::Rate lp(10);
  while (ros::ok())
  {
    lp.sleep();
  }
  return 0;
}
