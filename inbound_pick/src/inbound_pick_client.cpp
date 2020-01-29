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

  ros::ServiceClient add_objs_client=nh.serviceClient<manipulation_msgs::AddObjects>("add_objects");
  ros::ServiceClient add_box_client=nh.serviceClient<manipulation_msgs::AddBox>("add_box");


  actionlib::SimpleActionClient<manipulation_msgs::PickObjectsAction> pick_ac("inbound_pick");
  ROS_INFO("Waiting for pick server");
  pick_ac.waitForServer();
  ROS_INFO("Connection ok");

  pickplace::InboundFromParam inb(nh);
  if (!inb.readBoxesFromParam())
  {
    ROS_ERROR("Unable to read");
    return 0;
  }
  if (!inb.readObjectFromParam())
  {
    ROS_ERROR("Unable to read");
    return 0;
  }

//  Eigen::Affine3d T_w_box1;
//  T_w_box1.setIdentity();
//  Eigen::AngleAxisd rot(M_PI,Eigen::Vector3d::UnitX());
//  T_w_box1=rot;
//  T_w_box1.translation()(0)=0.9;
//  T_w_box1.translation()(1)=0.35;
//  T_w_box1.translation()(2)=1.05;

//  manipulation_msgs::AddBox box_srv;
//  box_srv.request.inbound_box_name="box1";
//  tf::poseEigenToMsg(T_w_box1,box_srv.request.box_pose);
//  box_srv.request.height=0.15;
//  add_box_client.call(box_srv);

//  Eigen::Affine3d T_w_box2=T_w_box1;
//  T_w_box2.translation()(0)=0.4;
//  box_srv.request.inbound_box_name="box2";
//  tf::poseEigenToMsg(T_w_box2,box_srv.request.box_pose);
//  box_srv.request.height=0.15;
//  add_box_client.call(box_srv);

//  manipulation_msgs::AddObjects add_objs_srv;
//  add_objs_srv.request.inbound_box_name="box1";
//  manipulation_msgs::Object obj1;
//  obj1.type="type1";
//  std::string tool_name="tip";

//  Eigen::Affine3d T_w_obj1=T_w_box1;
//  T_w_obj1.translation()(2)+=0.02;
//  manipulation_msgs::Grasp grasp1_obj1;
//  tf::poseEigenToMsg(T_w_obj1,grasp1_obj1.pose);
//  grasp1_obj1.tool_name=tool_name;
//  obj1.grasping_poses.push_back(grasp1_obj1);

//  add_objs_srv.request.add_objects.push_back(obj1);
//  add_objs_client.call(add_objs_srv);

//  manipulation_msgs::Object obj2;
//  obj2.type="type1";
//  Eigen::Affine3d T_w_obj2=T_w_box2;
//  T_w_obj1.translation()(2)+=0.02;
//  manipulation_msgs::Grasp grasp1_obj2;
//  tf::poseEigenToMsg(T_w_obj2,grasp1_obj2.pose);
//  grasp1_obj2.tool_name=tool_name;
//  obj2.grasping_poses.push_back(grasp1_obj2);
//  add_objs_srv.request.inbound_box_name="box2";
//  add_objs_srv.request.add_objects.at(0)=obj2;
//  add_objs_client.call(add_objs_srv);



  manipulation_msgs::PickObjectsGoal pick_goal;
  pick_goal.object_types.push_back("type1");

  pick_ac.sendGoalAndWait(pick_goal);

  pick_ac.sendGoalAndWait(pick_goal);

  ROS_INFO("pick client stopped");
  return 0;
}
