#include <ros/ros.h>
#include <manipulation_utils/place_objects.h>

#include <moveit_msgs/GetPlanningScene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "outbound_place_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ROS_INFO("Creating PlaceOject server...");
  manipulation::PlaceObjects place(nh,pnh);

  ros::ServiceClient ps_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ps_client.waitForExistence();
  moveit_msgs::GetPlanningScene ps_srv;

  if (!place.init())
  {
    ROS_ERROR_NAMED(nh.getNamespace(),"unable to load parameter");
    return -1;
  }

  ROS_INFO("PlaceOject server initialized.");

  ros::Rate lp(10);
  while (ros::ok())
  {
    if (!ps_client.call(ps_srv))
      ROS_ERROR("Error on  get_planning_scene srv not ok");
    else
      place.updatePlanningScene(ps_srv.response.scene);
  
    lp.sleep();
    place.publishTF();
  }
  return 0;
}
