#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class GazeboTruckPlugin : public ModelPlugin {
 public:
  GazeboTruckPlugin();
  ~GazeboTruckPlugin();

  void Publish();

  typedef ignition::math::Vector3d Vector;
  typedef ignition::math::Pose3d Pose;

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  enum Sens { CLOCK_WISE = -1, COUNTER_CLOCK_WISE = 1};
  void update_curve(const Pose& pose, const Vector& center, float radius, Sens sens);
  void update_straight(const Pose& pose, Vector point1, Vector point2);
  void update_control(const Pose& pose, float yaw_r);
  int choose_binary(int a, int b);
  physics::ModelPtr model_;

  float _radius;
  float _yaw_rate;
  int _track_section;
  float _speed;
  float _speed_min;
  float _speed_max;
  float _speed_bias_rate;
  float _speed_bias_rate_std;
  float _pos_bias;
  float _pos_bias_rate;
  float _pos_bias_rate_std;
  float _pos_bias_max;
  int   _rand_trajectory;
  
  event::ConnectionPtr updateConnection_;  // Pointer to the update event connection

  const static Vector UP;

  // parameter names for SDF
  const static std::string SPEED_MIN;
  const static std::string SPEED_MAX;
  const static std::string SPEED_BIAS_RATE_STD;
  const static std::string POS_BIAS_MAX;
  const static std::string POS_BIAS_RATE_STD;
  const static std::string RAND_TRAJECTORY;
};

const std::string GazeboTruckPlugin::SPEED_MIN("speed_min");
const std::string GazeboTruckPlugin::SPEED_MAX("speed_max");
const std::string GazeboTruckPlugin::SPEED_BIAS_RATE_STD("speed_bias_rate_std");
const std::string GazeboTruckPlugin::POS_BIAS_MAX("pos_bias_max");
const std::string GazeboTruckPlugin::POS_BIAS_RATE_STD("pos_bias_rate_std");
const std::string GazeboTruckPlugin::RAND_TRAJECTORY("rand_trajectory");
}
