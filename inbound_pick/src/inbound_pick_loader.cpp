#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <manipulation_utils/manipulation_load_params_utils.h>



std::shared_ptr<manipulation::InboundPickFromParam> inb;

bool addObjectsCb(std_srvs::SetBoolRequest& req, 
                  std_srvs::SetBoolResponse& res)
{
  if (!inb->readObjectFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return false;
  }
  ROS_INFO("load objects complete");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_pick_loader");
  ros::NodeHandle nh("inbound_pick_server");

  inb = std::make_shared<manipulation::InboundPickFromParam>(nh);

  if (!inb->readBoxesFromParam())
  {
    ROS_ERROR("Unable to load boxes");
    return 0;
  }

  if (!inb->readObjectFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return 0;
  }

  ROS_INFO("Inbound boxed loaded");

  ros::ServiceServer src = nh.advertiseService("inbound/add_objects",&addObjectsCb);
  ros::spin();
  return 0;
}
