#include <skill_base.h>

#include <manipulation_utils.h>

#include <std_srvs/SetBool.h>
#include <manipulation_msgs/PlaceObjectsAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace manipulation
{

  class PlaceObjects: public SkillBase
  {
    protected:
      
      ros::NodeHandle m_nh;
      ros::NodeHandle m_pnh;

      std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>> m_as;
      std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;

      bool m_init=false; // può essere spostato in SkillBase
      std::map<std::string,bool> m_slot_busy; // può essere spostato in SkillBase

      std::map<std::string, Eigen::Affine3d, std::less<std::string>, 
              Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > > m_slot_map; // verificare se può servire in SkillBase
      std::map<std::string, Eigen::Affine3d, std::less<std::string>, 
              Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > > m_approach_slot_map; // verificare se può servire in SkillBase

      std::map<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>> m_slot_configurations; // verificare se può servire in SkillBase
      std::map<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>> m_approach_slot_configurations; // verificare se può servire in SkillBase

      bool execute(const std::string& group_name,
                          const moveit::planning_interface::MoveGroupInterface::Plan& plan);

      bool wait(const std::string& group_name);

      void doneCb(const actionlib::SimpleClientGoalState& state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result,
                  const std::string& group_name);

    public:
      PlaceObjects(ros::NodeHandle m_nh,
                  ros::NodeHandle m_pnh);
  
      bool init();

      void placeObjectGoalCb(const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                              const std::string& group_name);


      moveit::planning_interface::MoveGroupInterface::Plan planToSlot(const std::string& group_name,
                                                                      const std::string& place_id,
                                                                      const Eigen::VectorXd& starting_jconf,
                                                                      moveit::planning_interface::MoveItErrorCode& result,
                                                                      Eigen::VectorXd& slot_jconf);

      moveit::planning_interface::MoveGroupInterface::Plan planToApproachSlot(const std::string& group_name,
                                                                              const std::string& place_id,
                                                                              const Eigen::VectorXd& starting_jconf,
                                                                              moveit::planning_interface::MoveItErrorCode& result,
                                                                              Eigen::VectorXd& slot_jconf);



      bool resetCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res); // verificare se può servire in SkillBase

      friend std::ostream& operator<<  (std::ostream& os, const PlaceObjects& pick_objs);

  };


}