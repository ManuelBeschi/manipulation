#include <skill_base.h>

#include <manipulation_utils.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <std_srvs/SetBool.h> 
#include <manipulation_msgs/AddBox.h>
#include <manipulation_msgs/AddObjects.h>
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
      ros::ServiceClient m_attach_object_srv;
      ros::ServiceServer m_reset_srv;


      //Eigen::Affine3d m_T_w_as; // world <- approach to slot
      //Eigen::Affine3d m_T_w_s;  // world <- slot

      std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PickObjectsAction>>> m_pick_servers;
      std::map<std::string,std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>> m_fjt_clients;

    

    public:
      PickObjects(const ros::NodeHandle& m_nh,
                  const ros::NodeHandle& m_pnh);

      bool init();

      void pickObjectGoalCb(const manipulation_msgs::PickObjectsGoalConstPtr& goal,
                            const std::string& group_name);

      friend std::ostream& operator<<  (std::ostream& os, const PickObjects& pick_objs);

      //void publishTF();

  };

}