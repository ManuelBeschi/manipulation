#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <manipulation_msgs/PickObjectsAction.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <manipulation_msgs/GoToAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rosparam_utilities/rosparam_utilities.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_recipe");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("Node %s has not a parameter named group_name",pnh.getNamespace().c_str());
    return -1;
  }

  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac(group_name+"/pick");
  actionlib::SimpleActionClient<manipulation_msgs::PlaceObjectsAction> place_ac(group_name+"/place");
  actionlib::SimpleActionClient<manipulation_msgs::GoToAction> goto_ac(group_name+"/goto");

  ROS_INFO("Waiting for pick server");
  pick_ac.waitForServer();
  ROS_INFO("Connection ok");
  ROS_INFO("Waiting for place server");
  place_ac.waitForServer();
  ROS_INFO("Connection ok");
  ROS_INFO("Waiting for goto server");
  goto_ac.waitForServer();
  ROS_INFO("Connection ok");

  std::vector<std::pair<std::string,std::vector<std::string>>> recipe;
  XmlRpc::XmlRpcValue param;
  if (!pnh.getParam("recipe",param))
  {
    ROS_ERROR("Node %s has not a parameter named recipe",pnh.getNamespace().c_str());
    return -1;
  }


  if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("recipe is not a list" );
    return 0;
  }

  for(int i=0; i < param.size(); i++)
  {
    XmlRpc::XmlRpcValue object = param[i];
    if( object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%d is not a struct", i);
      continue;
    }
    if( !object.hasMember("action") )
    {
      ROS_WARN("The element #%d has not the field 'type'", i);
      continue;
    }
    std::string action=rosparam_utilities::toString(object["action"]);

    if( !object.hasMember("description") )
    {
      ROS_WARN("The element #%d has not the field 'frame'", i);
      continue;
    }
    if( object.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_WARN("The element #%d is not a struct", i);
      continue;
    }
    XmlRpc::XmlRpcValue description=object["description"];
    if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("description is not a list" );
      continue;
    }

    std::vector<std::string> descr;
    for(int id=0; id < description.size(); id++)
    {
      descr.push_back(rosparam_utilities::toString(description[id]));
      ROS_INFO("Add skill -> %s:%s",action.c_str(),descr.back().c_str());
    }
    recipe.push_back(std::pair<std::string,std::vector<std::string>>(action,descr));
  }


  manipulation_msgs::PlaceObjectsGoal place_goal;
  manipulation_msgs::PickObjectsGoal pick_goal;
  manipulation_msgs::GoToGoal goto_goal;

  for (const std::pair<std::string,std::vector<std::string>>& skill: recipe)
  {

    ROS_INFO("[Group %s] skill -> %s:",pnh.getNamespace().c_str(),skill.first.c_str());
    for (const std::string& s: skill.second)
      ROS_INFO("[Group %s] \t\t\t: %s",pnh.getNamespace().c_str(),s.c_str());

    if (skill.first.compare("pick")==0)
    {
      pick_goal.object_types=skill.second;

      pick_ac.sendGoalAndWait(pick_goal);


      if (pick_ac.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to pick",pnh.getNamespace().c_str());
        return 0;
      }
      ROS_INFO("[Group %s] well done! I picked it, id=%s tye=%s",pnh.getNamespace().c_str(),pick_ac.getResult()->object_id.c_str(),pick_ac.getResult()->object_type.c_str());
      place_goal.object_type=pick_ac.getResult()->object_type;
      place_goal.object_id=pick_ac.getResult()->object_id;

    }
    else if (skill.first.compare("place")==0)
    {
      place_goal.place_id=skill.second;
      place_ac.sendGoalAndWait(place_goal);

      if (place_ac.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to place, stop it",pnh.getNamespace().c_str());
        return 0;
      }
      ROS_INFO("[Group %s] well done! placed in %s",pnh.getNamespace().c_str(),place_ac.getResult()->place_id.c_str());
    }
    else if (skill.first.compare("goto")==0)
    {
      goto_goal.location_id=skill.second.at(0);
      goto_ac.sendGoalAndWait(goto_goal);

      if (goto_ac.getResult()->result<0)
      {
        ROS_ERROR("[Group %s] unable to place, stop it",pnh.getNamespace().c_str());
        return 0;
      }
      ROS_INFO("[Group %s] well done! ",pnh.getNamespace().c_str());
    }
    else
    {
      ROS_ERROR("unable to execute action %s",skill.first.c_str());
      return 0;
    }

    ros::Duration(0.1).sleep();
  }


  ROS_INFO("[Group %s] recipe executed stopped",pnh.getNamespace().c_str());
  return 0;
}
