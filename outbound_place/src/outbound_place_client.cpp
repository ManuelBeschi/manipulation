#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_utils/manipulation_load_params_utils.h>


int main(int argc, char **argv)
{
  std::string object_name, object_type;
  if( argc == 3) 
  {
    object_name = argv[1];    
    object_type = argv[2];
  }
  else
  {
    ROS_ERROR("Wrong number of input, the input needed are: object_name object_type.");
    return -1;
  }
  
  ros::init(argc, argv, "outbound_place_client");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac("/outbound_place_server/manipulator/place");
  ROS_INFO("Waiting for place server");
  place_ac.waitForServer();
  ROS_INFO("Connection ok");

  manipulation_msgs::PlaceObjectsGoal place_goal;
  place_goal.object_name = object_name;
  place_goal.object_type = object_type;
  place_goal.slot_names.push_back("A1");
  place_goal.slot_names.push_back("A3");
  place_goal.slot_names.push_back("B1");

  place_ac.sendGoalAndWait(place_goal);

  ROS_INFO("Place client stopped");
  return 0;
}
