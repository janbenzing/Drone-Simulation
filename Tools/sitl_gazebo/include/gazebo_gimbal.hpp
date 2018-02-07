#ifndef _GAZEBO_GIMBAL_HH_
#define _GAZEBO_GIMBAL_HH_


#include <gazebo/gazebo.hh>
#include "GimbalCommand.pb.h"


namespace gazebo
{

  static const std::string gimbal_command_sub_topic_ = "/gazebo/command/gimbal_command";
  typedef const boost::shared_ptr<const target_camera::msgs::GimbalCommand> GimbalCommandPtr;
  
  class Gimbal
  {
  public:
    const static std::string GIMBAL_JOINT;

    Gimbal();

    void Load(const sdf::ElementPtr _sdf, const physics::ModelPtr _model);

    void SetCmd(float pitch, float yaw);

    float GetPitch() const {return pitch_;};
    float GetYaw() const {return yaw_;};

    void Update(double current_time);

  private:
    transport::SubscriberPtr gimbal_command_sub_;
    transport::NodePtr node_handle_;
    physics::LinkPtr child_;
    double cmd_pitch_;
    double cmd_yaw_;
    double pitch_;
    double yaw_;

    void GimbalCommandCallback(GimbalCommandPtr& msg);
    void FindJoint(const sdf::ElementPtr _sdf, const physics::ModelPtr model);
  };
}

#endif /* _GAZEBO_GIMBAL_HH_ */
