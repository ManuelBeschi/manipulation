#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_utils/manipulation_load_params_utils.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "outbound_place_client");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac("/outbound_place/manipulator/place");
  ROS_INFO("Waiting for place server");
  place_ac.waitForServer();
  ROS_INFO("Connection ok");

  manipulation_msgs::PlaceObjectsGoal place_goal;
  place_goal.object_type = "squadra_piccola";
  place_goal.object_name = "squadra_grande_0";
  place_goal.slot_names.push_back("A1");
  place_goal.slot_names.push_back("A3");
  place_goal.slot_names.push_back("B1");

  place_ac.sendGoalAndWait(place_goal);

  ROS_INFO("Place client stopped");
  return 0;
}
