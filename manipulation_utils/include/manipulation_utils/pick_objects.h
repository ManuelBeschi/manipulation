#include <skill_base.h>

#include <manipulation_utils.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <std_srvs/SetBool.h> 
#include <manipulation_msgs/AddBox.h>
#include <manipulation_msgs/AddObjects.h>
#include <manipulation_msgs/ListOfObjects.h>
#include <manipulation_msgs/PickObjectsAction.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

namespace manipulation
{

  typedef std::map<std::string, Eigen::Affine3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d>>>  PosesMap;
  typedef std::pair<std::string,Eigen::Affine3d>  PosesPair;


  class PickObjects: public SkillBase
  {
    protected:
      
      ros::NodeHandle m_nh;
      ros::NodeHandle m_pnh; 

      std::mutex m_mtx;

      std::map<std::string,InboundBoxPtr> m_boxes;
      std::vector<std::string> m_request_adapters;

      tf::TransformBroadcaster m_broadcaster;
      std::map<std::string,tf::Transform> m_tf;

      ros::ServiceServer m_add_obj_srv;
      ros::ServiceServer m_add_box_srv;
      ros::ServiceServer m_list_objects_srv;

      Eigen::Affine3d m_T_w_as; // world <- approach to slot
      Eigen::Affine3d m_T_w_s;  // world <- slot
      std::map<std::string,Eigen::VectorXd> m_preferred_configuration;
      std::map<std::string,Eigen::VectorXd> m_preferred_configuration_weight;

      std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>>> m_pick_servers;
      std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;

      std::map<std::string,InboundBoxPtr> searchBoxWithType(const std::string& type_name);
      std::map<std::string,InboundBoxPtr> searchBoxWithTypes(const std::vector<std::string>& type_names);

      moveit::planning_interface::MoveGroupInterface::Plan planToApproachSlot(const std::string& group_name,
                                                                              const Eigen::VectorXd& starting_jconf,
                                                                              const Eigen::Affine3d& approach_pose,
                                                                              const ObjectPtr& selected_object,
                                                                              const GraspPosePtr& selected_grasp_pose,
                                                                              moveit::planning_interface::MoveItErrorCode& result,
                                                                              Eigen::VectorXd& slot_jconf);

      moveit::planning_interface::MoveGroupInterface::Plan planToBestBox(const std::string& group_name,
                                                                        const std::map<std::string,InboundBoxPtr>& possible_boxes,
                                                                        moveit::planning_interface::MoveItErrorCode& result,
                                                                        InboundBoxPtr& selected_box,
                                                                        Eigen::VectorXd& jconf);


      moveit::planning_interface::MoveGroupInterface::Plan planToObject(const std::string& group_name,
                                                                        const std::vector<std::string>& type_name,
                                                                        const InboundBoxPtr& selected_box,
                                                                        const Eigen::VectorXd& starting_jconf,
                                                                        const std::string& tool_name,
                                                                        moveit::planning_interface::MoveItErrorCode& result,
                                                                        ObjectPtr& selected_object,
                                                                        GraspPosePtr& selected_grasp_pose);

      /* 
      return false if already present
      */
      bool createInboundBox(const std::string& box_name,
                            const Eigen::Affine3d& T_w_box,
                            const double heigth);

      std::map<std::string,InboundBoxPtr>::iterator findBox(const std::string& box_name);

      /* return false if box does not exist
      */
      bool removeInboundBox(const std::string& box_name);

      ObjectPtr createObject(const std::string& type, const std::string& id,
                            const std::string& box_name,
                            const PosesMap& poses);

      bool removeObject(const std::string& type,
                        const std::string& box_name);

      bool execute(const std::string& group_name,
                          const moveit::planning_interface::MoveGroupInterface::Plan& plan);

      bool wait(const std::string& group_name);

      void doneCb(const actionlib::SimpleClientGoalState& state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result,
                  const std::string& group_name);


    public:
      PickObjects(  ros::NodeHandle m_nh,
                    ros::NodeHandle m_pnh);

      bool init();

      bool addObjectCb( manipulation_msgs::AddObjects::Request& req,
                        manipulation_msgs::AddObjects::Response& res);

      bool addBoxCb(manipulation_msgs::AddBox::Request& req,
                    manipulation_msgs::AddBox::Response& res);

      bool listObjects( manipulation_msgs::ListOfObjects::Request& req,
                        manipulation_msgs::ListOfObjects::Response& res);

      void pickObjectGoalCb(const manipulation_msgs::PickObjectsGoalConstPtr& goal,
                            const std::string& group_name);

      bool resetBoxesCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

      friend std::ostream& operator<<  (std::ostream& os, const PickObjects& pick_objs);

      void publishTF();

  };

}