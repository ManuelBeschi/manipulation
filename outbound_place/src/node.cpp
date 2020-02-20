#include <ros/ros.h>
#include <outbound_place/outbound_place.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "outbound_pallet");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  ros::AsyncSpinner spinner(4);
  spinner.start();

  pickplace::OutboundPallet pallet(nh,pnh)  ;
  if (!pallet.init())
  {
    ROS_ERROR_NAMED(nh.getNamespace(),"unable to load parameter");
    return -1;
  }


  while (ros::ok())
  {
    ros::Duration(0.1).sleep();
  }
  return 0;
}
