#include <ros/ros.h>
#include <inbound_pick/inbound_pick.h>
#include <inbound_pick/pick_objects.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPlanningScene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ps_client.waitForExistence();
  moveit_msgs::GetPlanningScene ps_srv;
  pickplace::PickObjects pick(nh,pnh);

  if (!pick.init())
  {
    ROS_ERROR_NAMED(nh.getNamespace(),"unable to load parameters for node %s",pnh.getNamespace().c_str());
    return -1;
  }

  ros::Rate lp(10);
  while (ros::ok())
  {

    if (!ps_client.call(ps_srv))
      ROS_ERROR("call to get_planning_scene srv not ok");
    else
      pick.updatePlanningScene(ps_srv.response.scene);
    lp.sleep();
    pick.publishTF();
  }
  return 0;
}
