#include <ros/ros.h>
#include <go_to_location/go_to_location.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "goto_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  ros::AsyncSpinner spinner(8);
  spinner.start();

  manipulation_skills::GoToLocation go_to(nh,pnh)  ;
  if (!go_to.init())
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
