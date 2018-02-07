#include "gazebo_gimbal.hpp"
#include <gazebo/physics/physics.hh>


namespace gazebo {

const std::string Gimbal::GIMBAL_JOINT = "gimbal_joint";

Gimbal::Gimbal() :
  child_(NULL),
  pitch_(0),
  yaw_(0)
{
  SetCmd(-M_PI/2.0f,0.0f);
}


void Gimbal::Load(const sdf::ElementPtr _sdf, const physics::ModelPtr model)
{
  // connection to publish pose over google proto buf
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();
  gimbal_command_sub_ = node_handle_->Subscribe("~/" + model->GetName() + gimbal_command_sub_topic_,  &Gimbal::GimbalCommandCallback, this);
  FindJoint(_sdf, model);
}

void Gimbal::FindJoint(const sdf::ElementPtr _sdf, const physics::ModelPtr model)
{
    if(_sdf->HasElement(GIMBAL_JOINT))
    {
      std::string joint_name = _sdf->GetElement(GIMBAL_JOINT)->Get<std::string>();
      physics::JointPtr joint = model->GetJoint(joint_name);
      
      if(joint != NULL)
      {
        child_ = joint->GetChild();
      }
    }
}

void Gimbal::GimbalCommandCallback(GimbalCommandPtr& msg)
{
  float pitch = msg->pitch();
  float yaw = msg->yaw();
  SetCmd(pitch, yaw);
}


void Gimbal::SetCmd(float pitch, float yaw)
{
  cmd_pitch_ = pitch;
  cmd_yaw_ = yaw;
}

void Gimbal::Update(double current_time)
{
  if(child_ == NULL)
  {
    return;
  }

  const math::Pose pose(0,0,0,0,-cmd_pitch_,-cmd_yaw_);
  child_->SetRelativePose(pose);

  pitch_ = cmd_pitch_;
  yaw_ = cmd_yaw_;
}

}