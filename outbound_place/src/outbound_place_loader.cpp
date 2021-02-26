#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <manipulation_utils/manipulation_load_params_utils.h>



std::shared_ptr<manipulation::OutboundPlaceFromParam> oub;

bool addObjectsCb(std_srvs::SetBoolRequest& req, 
                  std_srvs::SetBoolResponse& res)
{
  if (!oub->readSlotsFromParam())
  {
    ROS_ERROR("Unable to load objects in the boxes");
    return false;
  }
  ROS_INFO("load objects complete");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "outbound_place_loader");
  ros::NodeHandle nh("outbound_place_server");

  oub = std::make_shared<manipulation::OutboundPlaceFromParam>(nh);

  if (!oub->readSlotsFromParam())
  {
    ROS_ERROR("Unable to load slots");
    return 0;
  }

  ROS_INFO("Outbound slot loaded");

  ros::ServiceServer src = nh.advertiseService("outbound/add_slots",&addObjectsCb);
  ros::spin();
  return 0;
}
