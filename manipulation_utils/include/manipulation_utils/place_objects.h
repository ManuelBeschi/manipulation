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
      
      ros::ServiceClient m_detach_object_srv;
      ros::ServiceServer m_reset_srv;

      std::map<std::string,std::shared_ptr<actionlib::SimpleActionServer<manipulation_msgs::PlaceObjectsAction>>> m_as;

      std::map<std::string,bool> m_slot_busy; // può essere spostato in SkillBase

      std::map<std::string, Eigen::Affine3d, std::less<std::string>, 
              Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > > m_slot_map; // verificare se può servire in SkillBase
      std::map<std::string, Eigen::Affine3d, std::less<std::string>, 
              Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d> > > m_approach_slot_map; // verificare se può servire in SkillBase

      std::map<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>> m_slot_configurations; // verificare se può servire in SkillBase
      std::map<std::string,std::map<std::string,std::vector<Eigen::VectorXd>>> m_approach_slot_configurations; // verificare se può servire in SkillBase

      
      bool createOutboundBox(const std::string& box_name,
                            const Eigen::Affine3d& T_w_box,
                            const double heigth);

      std::map<std::string,InboundBoxPtr>::iterator findBox(const std::string& box_name);

      bool removeOutboundBox(const std::string& box_name);

      ObjectPtr createObject(const std::string& type, const std::string& id,
                            const std::string& box_name,
                            const PosesMap& poses);

      bool removeObject(const std::string& type,
                        const std::string& box_name);



    public:
      PlaceObjects( const ros::NodeHandle& m_nh,
                    const ros::NodeHandle& m_pnh);
  
      bool init();

      void placeObjectGoalCb(const manipulation_msgs::PlaceObjectsGoalConstPtr& goal,
                              const std::string& group_name);

      friend std::ostream& operator<<  (std::ostream& os, const PlaceObjects& pick_objs);

  };


}