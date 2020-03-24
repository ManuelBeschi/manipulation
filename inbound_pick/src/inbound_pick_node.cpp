#include <ros/ros.h>
#include <inbound_pick/inbound_pick.h>
#include <inbound_pick/pick_objects.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inbound_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(5);
  spinner.start();

  ros::Publisher target_pub=nh.advertise<geometry_msgs::PoseStamped>("target",1);
  geometry_msgs::PoseStamped target;
  target.header.frame_id="world";

  pickplace::PickObjects pick("ur5_on_guide");

  double height=0.15;
  Eigen::Affine3d T_w_box1;
  T_w_box1.setIdentity();
  Eigen::AngleAxisd rot(M_PI,Eigen::Vector3d::UnitX());
  T_w_box1=rot;
  T_w_box1.translation()(0)=0.9;
  T_w_box1.translation()(1)=0.35;
  T_w_box1.translation()(2)=1.05;
  pick.createInboundBox("box1",T_w_box1,height);

  Eigen::Affine3d T_w_box2=T_w_box1;
  T_w_box2.translation()(0)=0.4;
  pick.createInboundBox("box2",T_w_box2,height);



  std::string tool_name="tool";
  Eigen::Affine3d T_w_obj1=T_w_box1;
  T_w_obj1.translation()(2)+=0.05;

  pickplace::PosesMap poses;
  poses.insert(pickplace::PosesPair(tool_name,T_w_obj1));

  pick.createObject("type1","id01","box1",poses);


  Eigen::Affine3d T_w_obj2=T_w_box2;
  T_w_obj2.translation()(2)+=0.1;

  pickplace::PosesMap poses_obj2;
  poses_obj2.insert(pickplace::PosesPair(tool_name,T_w_obj2));

  pick.createObject("type1","id02","box2",poses_obj2);


  for (unsigned int it=0;it<2;it++)
  {
    std::map<std::string,pickplace::InboundBoxPtr> possible_boxes=pick.searchBoxWithType("type1");

    ROS_FATAL("find %zu boxes",possible_boxes.size());

    moveit::planning_interface::MoveItErrorCode result;
    pickplace::InboundBoxPtr selected_box;
    Eigen::VectorXd approach_jconf;
    moveit::planning_interface::MoveGroupInterface::Plan plan=pick.planToBestBox(possible_boxes,
                                                                                 result,
                                                                                 selected_box,
                                                                                 approach_jconf);


    if (!result)
    {
      ROS_ERROR("error in plan to best box, code = %d",result.val);
      return 0;
    }
    ROS_FATAL("plan to approach in %f second",plan.planning_time_);

    tf::poseEigenToMsg(selected_box->getApproachPose(),target.pose);
    target_pub.publish(target);
    pick.execute(plan);

    std::vector<std::string> type_names;
    pickplace::ObjectPtr selected_object;
    pickplace::GraspPosePtr selected_grasp_pose;

    type_names.push_back("type1");
    moveit::planning_interface::MoveGroupInterface::Plan pick_plan=pick.planToObject(type_names,
                                                                                     selected_box,
                                                                                     approach_jconf,
                                                                                     tool_name,
                                                                                     result,
                                                                                     selected_object,
                                                                                     selected_grasp_pose
                                                                                     );

    if (!result)
    {
      ROS_ERROR("error in plan to best object in the box, code = %d",result.val);
      return 0;
    }

    pick.wait();
    tf::poseEigenToMsg(selected_grasp_pose->getPose(),target.pose);
    target_pub.publish(target);
    pick.execute(pick_plan);

    moveit::planning_interface::MoveGroupInterface::Plan return_plan=pick.planToReturnToApproach(type_names,
                                                                                                 selected_box,
                                                                                                 approach_jconf,
                                                                                                 selected_grasp_pose,
                                                                                                 result
                                                                                                 );


    if (!result)
    {
      ROS_ERROR("error in plan black to box, code = %d",result.val);
      return 0;
    }

    if (!selected_box->removeObject(selected_object->getId()))
      ROS_WARN("unable to remove object");

    pick.wait();
    tf::poseEigenToMsg(selected_box->getApproachPose(),target.pose);
    target_pub.publish(target);
    pick.execute(return_plan);
  }
  return 0;
}
