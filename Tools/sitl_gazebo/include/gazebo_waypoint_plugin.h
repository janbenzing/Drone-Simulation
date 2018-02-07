#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/msgs/msgs.hh"
#include "waypoint.pb.h"

namespace gazebo {

typedef const boost::shared_ptr<const waypoint_msgs::msgs::Waypoint> WaypointPtr;

class GazeboWaypointPlugin : public ModelPlugin {
 public:
  GazeboWaypointPlugin() :
    ModelPlugin()
    {}
  ~GazeboWaypointPlugin();
  typedef ignition::math::Vector3d Vector;

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  /** Pointer to the update event connection */
  event::ConnectionPtr _updateConnection;
  /** Pointer to the visual topic publisher */
  transport::PublisherPtr _visual_pub;
  /** Pointer to gazebo transport node */
  transport::NodePtr _node;
  /** Pointer to current model */
  physics::ModelPtr _model;
  /** Flag is set when the drone pass through the waypoint */
  bool _valid = false;
  /** Waypoint number set by <no> element */
  int32_t _no;
  /** Waypoint radius set by <radius> element. Default to 2. */
  double _radius;
  /** Unique name of the visual based on #_no */
  std::string _visual_name;

  /** Pointer to the waypoint topic publisher */
  transport::PublisherPtr _waypoint_pub;
  /** Pointer to the waypoint topic subscriber */
  transport::SubscriberPtr _waypoint_sub;
  /** Flag is set if this waypoint is active. */
  bool _active = false;
  /** Color constant for inactive waypoints */
  const std::string GAZEBO_COLOR_INACTIVE = "Gazebo/Grey";
  /** Color constant for active waypoints */
  const std::string GAZEBO_COLOR_ACTIVE = "Gazebo/Yellow";
  /** Color constant for validated waypoints */
  const std::string GAZEBO_COLOR_VALIDATED = "Gazebo/Green";
  /** Current color of the waypoint */
  std::string _color = GAZEBO_COLOR_INACTIVE;

  /** Change the visual of the waypoint: set its color to the provided
   * color such as "Gazebo/Yellow" and "Gazebo/Green".
   * @param color the new color of the visual
   */
  void UpdateVisual(std::string color);
  /** Update current color
   * @param new_color[in] a string such as #GAZEBO_COLOR_ACTIVE.
   */
  void SetColor(std::string new_color);
  /** Update active state
   * @param active[in] new active state
   */
  void SetActive(bool active);
  /** Update valid state
   * @param valid[in] new active state
   */
  void SetValid(bool valid);
  /** Send a new waypoint message
   * @param valid[in] the valid state to be sent
   */
  void FireValid(bool valid);
  /** Callback for waypoint message
   * @param msg[in] the waypoint message
   */
  void OnWaypointUpdate(WaypointPtr &msg);
};
}
