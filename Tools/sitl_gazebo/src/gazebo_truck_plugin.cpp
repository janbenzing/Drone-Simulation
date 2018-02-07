#include "gazebo_truck_plugin.h"
#include "common.h"
using namespace std;

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboTruckPlugin);

const GazeboTruckPlugin::Vector GazeboTruckPlugin::UP(0.0f, 0.0f, 1.0f);


GazeboTruckPlugin::GazeboTruckPlugin() :
  ModelPlugin(),
  _radius(50),
  _yaw_rate(0),
  _track_section(0),
  _speed(0),
  _speed_min(0),
  _speed_max(0),
  _speed_bias_rate(0),
  _speed_bias_rate_std(0),
  _pos_bias(0),
  _pos_bias_rate(0),
  _pos_bias_rate_std(0),
  _pos_bias_max(0)
{}

GazeboTruckPlugin::~GazeboTruckPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboTruckPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTruckPlugin::OnUpdate, this, _1));

  // find trajectory parameters in sdf file
  getSdfParam(_sdf, SPEED_MIN, _speed_min, 0.0f, true);
  getSdfParam(_sdf, SPEED_MAX, _speed_max, 0.0f, true);
  getSdfParam(_sdf, SPEED_BIAS_RATE_STD, _speed_bias_rate_std, 0.0f, true);
  getSdfParam(_sdf, POS_BIAS_MAX, _pos_bias_max, 0.0f, true);
  getSdfParam(_sdf, POS_BIAS_RATE_STD, _pos_bias_rate_std, 0.0f, true);
  getSdfParam(_sdf, RAND_TRAJECTORY, _rand_trajectory, 0, true);
}

// This gets called by the world update start event.
void GazeboTruckPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {

  Pose pose = model_->GetWorldPose().Ign();
  Vector pos = pose.Pos();

  Vector P1(0, -_radius, 0);
  Vector P2(2*_radius, -_radius,0);
  Vector P3(2*_radius,  _radius,0);
  Vector P4(0,  _radius, 0);
  Vector C0(2*_radius,0,0);
  Vector C1(0,0,0);

  switch(_track_section)
  {
    case 0: // straight segment bottom towards right
      update_straight(pose, P1, P2);
      if(pos.X() > P2.X())
      {
        _track_section = 4;
      }
      break;

    case 1: // straight segment bottom towards left
      update_straight(pose, P2, P1);
      if(pos.X() < P1.X())
      {
        _track_section = 10;
      }
      break;

    case 2: // straight segment top towards right
      update_straight(pose, P4, P3);
      if(pos.X() > P3.X())
      {
        _track_section = 6;
      }
      break;

    case 3: // straight segment top towards left
      update_straight(pose, P3, P4);
      if(pos.X() < P4.X())
      {
        _track_section = 8;
      }
      break;

    case 4: // circle C0 counter-clock-wise
      update_curve(pose, C0, _radius, COUNTER_CLOCK_WISE);
      if(pos.X() < C0.X() && pos.Y() > _radius/2.0f)
      {
        _track_section = choose_binary(3,5);
      }
      break;

    case 5: // circle C0 counter-clock
      update_curve(pose, C0, _radius, COUNTER_CLOCK_WISE);
      if(pos.X() < C0.X() && pos.Y() < C0.Y())
      {
        _track_section = choose_binary(4,10);
      }
      break;

    case 6: // circle C0 clock-wise
      update_curve(pose, C0, _radius, CLOCK_WISE);
      if(pos.X() < C0.X() && pos.Y() < -_radius/2.0f)
      {
        _track_section = choose_binary(1,7);
      }
      break;

    case 7: // circle C0 clock-wise
      update_curve(pose, C0, _radius, CLOCK_WISE);
      if(pos.X() < C0.X() && pos.Y() > C0.Y())
      {
        _track_section = choose_binary(6,8);
      }
      break;

    case 8: // circle C1 counter-clock-wise
      update_curve(pose, C1, _radius, COUNTER_CLOCK_WISE);
      if(pos.X() > C1.X() && pos.Y() < -_radius/2.0f)
      {
        _track_section = choose_binary(0,9);
      }
      break;

    case 9: // circle C1 counter-clock-wise
      update_curve(pose, C1, _radius, COUNTER_CLOCK_WISE);
      if(pos.X() > C1.X() && pos.Y() > C1.Y())
      {
        _track_section = choose_binary(8,6);
      }
      break;

    case 10: // circle C1 clock-wise
      update_curve(pose, C1, _radius, CLOCK_WISE);
      if(pos.X() > C1.X() && pos.Y() > _radius/2.0f)
      {
        _track_section = choose_binary(2,11);
      }
      break;

    case 11: // circle C1 clock-wise
      update_curve(pose, C1, _radius, CLOCK_WISE);
      if(pos.X() > C1.X() && pos.Y() < C1.Y())
      {
        _track_section = choose_binary(10,4);
      }
      break;
  }
}


void GazeboTruckPlugin::update_curve(const Pose& pose, const Vector& center, float radius, Sens sens)
{
  Vector radial = pose.Pos() - center; // radial vector (Center to current position)
  float pos_err = radius - radial.Length();
  radial /= radial.Length();                 // normalize radial vector

  Vector tan = -sens * radial.Cross(UP);    // tangential unit vector

  Vector bias = tan.Cross(UP)*_pos_bias;

  Vector dir_r = 0.1*(pos_err*radial + bias) + tan;

  float yaw_r = atan2(dir_r.Y(), dir_r.X());  // reference heading

  // add radius correction
  update_control(pose, yaw_r);

  // model_->SetWorldPose(pose);
}


void GazeboTruckPlugin::update_straight(const Pose& pose, Vector point1, Vector point2)
{
  Vector u = point2 - point1;         // direction vector
  u /= u.Length();                    // normalize to unit vector
  // get position error
  Vector pp1 = point1 - pose.Pos();   // vector from current position to point1
  Vector d = pp1 - (pp1.Dot(u) * u);  // position_error (shortest vector form current pos to line)

  // get position bias vector
  Vector bias = u.Cross(UP)*_pos_bias;



  Vector dir_r = 0.05*(d + bias) + u;           // reference direction
  float yaw_r = atan2(dir_r.Y(), dir_r.X());  // reference heading

  update_control(pose, yaw_r);
}


void GazeboTruckPlugin::update_control(const Pose& pose, float yaw_r)
{
  // --------------
  // adjust biases
  // --------------
  _speed_bias_rate += math::Rand::GetDblNormal(0.0, _speed_bias_rate_std);
  _speed += _speed_bias_rate;
  if(_speed > _speed_max)
  {
    _speed = _speed_max;
    _speed_bias_rate = 0;
  }
  if(_speed < _speed_min)
  {
    _speed_bias_rate = 0.01;
  }

  _pos_bias_rate += math::Rand::GetDblNormal(0.0, _pos_bias_rate_std);
  _pos_bias += _pos_bias_rate * _speed/10;
  if(_pos_bias > _pos_bias_max)
  {
    _pos_bias = _pos_bias_max;
    _pos_bias_rate = 0;
  } else if(_pos_bias < -_pos_bias_max)
  {
    _pos_bias = -_pos_bias_max;
    _pos_bias_rate = 0;
  }

  // -----------------
  // compute yaw_rate
  // -----------------
  float yaw_err = yaw_r - pose.Rot().Yaw();
  while(yaw_err > M_PI)
    yaw_err -= 2*M_PI;
  while(yaw_err < -M_PI)
    yaw_err += 2*M_PI;
  _yaw_rate = yaw_err * _speed + 0.8*_yaw_rate;

  model_->SetAngularVel(Vector(0.0f,0.0f,_yaw_rate));
  Vector lin_vel = pose.Rot().RotateVector(Vector(_speed,0.0f,0.0f));
  lin_vel.Z() = 0.0f; // make sure there is no vertical velocity
  model_->SetLinearVel(lin_vel);
}

int GazeboTruckPlugin::choose_binary(int a, int b)
{
  if(_rand_trajectory > 0){
    return math::Rand::GetIntUniform(0,1) ? a : b;
  }else {
    return a;
  }
}

}
