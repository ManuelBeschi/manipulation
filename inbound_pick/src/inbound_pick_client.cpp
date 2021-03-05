#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_utils/manipulation_load_params_utils.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_pick_client");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac("/inbound_pick_server/manipulator/pick");
  ROS_INFO("Waiting for pick server");
  pick_ac.waitForServer();
  ROS_INFO("Connection ok");

  manipulation_msgs::PickObjectsGoal pick_goal;
  pick_goal.object_types.push_back("squadra_piccola");
  //pick_goal.object_types.push_back("squadra_grande");

  pick_ac.sendGoalAndWait(pick_goal);
  //pick_ac.sendGoalAndWait(pick_goal);

  //pick_ac.sendGoalAndWait(pick_goal);
  //pick_ac.sendGoalAndWait(pick_goal);

  ROS_INFO("pick client stopped");
  return 0;
}
