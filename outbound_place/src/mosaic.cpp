#include <ros/ros.h>
#include <outbound_place/outbound_mosaic.h>
#include <moveit_msgs/GetPlanningScene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "outbound_mosaic");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ps_client.waitForExistence();
  moveit_msgs::GetPlanningScene ps_srv;

  ros::AsyncSpinner spinner(8);
  spinner.start();

  pickplace::OutboundMosaic pallet(nh,pnh)  ;
  if (!pallet.init())
  {
    ROS_ERROR_NAMED(nh.getNamespace(),"unable to load parameter");
    return -1;
  }


  while (ros::ok())
  {
    if (!ps_client.call(ps_srv))
      ROS_ERROR("call to get_planning_scene srv not ok");
    else
      pallet.updatePlanningScene(ps_srv.response.scene);

    ros::Duration(0.1).sleep();
  }
  return 0;
}
