#include <kin_base.h>
#include <moveit_base.h>

#include <ros/ros.h>

#include <control_msgs/FollowJointTrajectoryResult.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>


namespace manipulation
{  
  class SkillBase: public MoveItBase
  {
    protected:
      ros::NodeHandle m_nh;
      ros::NodeHandle m_pnh; 

      ros::ServiceClient m_grasp_srv;
      ros::ServiceClient m_attach_detach_object_srv;
      ros::ServiceServer m_reset_srv;

      ros::Publisher m_target_pub;
      ros::Publisher m_display_publisher;

      virtual bool execute( const std::string& group_name,
                            const moveit::planning_interface::MoveGroupInterface::Plan& plan) = 0;

      virtual bool wait(const std::string& group_name) = 0;

      virtual void doneCb(const actionlib::SimpleClientGoalState& state,
                          const control_msgs::FollowJointTrajectoryResultConstPtr& result,
                          const std::string& group_name) = 0;

    public:
      SkillBase(const ros::NodeHandle& nh,
                const ros::NodeHandle& pnh,
                const std::string& name);

      virtual bool init() = 0;

  };

}