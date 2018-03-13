#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "platform.pb.h"
#include "platformLanding.pb.h"
#include "gazebo/msgs/msgs.hh"
#include <regex>

namespace gazebo {


typedef const boost::shared_ptr<const platform_msgs::msgs::platform> PlatformPtr;
typedef const boost::shared_ptr<const platform_msgs::msgs::PlatformLanding> PlatformLandingPtr;
typedef const boost::shared_ptr<const msgs::Contacts> ContactsPtr;

static const std::string kDefaultPlatformSubTopic = "platform";
static const std::string kDefaultContactSubTopic = "platform/box/contact";

class GazeboPlatformPlugin : public ModelPlugin {
 public:
  GazeboPlatformPlugin() :
	ModelPlugin(),
	platform_sub_topic_(kDefaultPlatformSubTopic),
	contact_sub_topic_(kDefaultContactSubTopic)
  {}
  ~GazeboPlatformPlugin();
  typedef ignition::math::Vector3d Vector;

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void PlatformCallback(PlatformPtr& platform_message);
  void ContactsCallback(ContactsPtr& contact_message);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
 physics::ModelPtr model_;
 physics::LinkPtr box_;
 physics::CollisionPtr collision_;
 math::Pose initialPose_;
 event::ConnectionPtr updateConnection_;  // Pointer to the update event connection

 Vector scale;
 transport::NodePtr node_;

 /** Subtopic name for the platform */
 std::string platform_sub_topic_;
 /** Subtopic name for the contact */
 std::string contact_sub_topic_;

 /** Subscription to platform topic */
 transport::SubscriberPtr platform_sub_;
 /** Subscription to contact topic */
 transport::SubscriberPtr contact_sub_;

 transport::PublisherPtr platform_landing_pub_;

 /** Height */
 float _h;
 /** Position X coordinate */
 float _x = 100;
 /** Position Y coordinate */
 float _y = 0;

 math::Vector3 _landing_vel;
 platform_msgs::msgs::PlatformLanding _landing_msg;
};
}
