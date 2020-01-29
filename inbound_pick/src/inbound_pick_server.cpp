#include <ros/ros.h>
#include <inbound_pick/inbound_pick.h>
#include <inbound_pick/pick_objects.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(10);
  spinner.start();

  pickplace::PickObjects pick("ur5_on_guide");

  ros::Rate lp(10);
  while (ros::ok())
  {
    lp.sleep();
  }
  return 0;
}
