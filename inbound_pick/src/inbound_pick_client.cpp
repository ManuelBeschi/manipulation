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

  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac("inbound_pick");
  ROS_INFO("Waiting for pick server");
  pick_ac.waitForServer();
  ROS_INFO("Connection ok");



  manipulation_msgs::PickObjectsGoal pick_goal;
  pick_goal.object_types.push_back("type1");

  pick_ac.sendGoalAndWait(pick_goal);

  pick_ac.sendGoalAndWait(pick_goal);

  ROS_INFO("pick client stopped");
  return 0;
}
