#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo_platform_plugin.h"
#include "common.h"

using namespace std;

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboPlatformPlugin);

GazeboPlatformPlugin::~GazeboPlatformPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboPlatformPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  box_ = _model->GetChildLink("box");
  initialPose_ = model_->GetWorldPose();
  _h = 1;

  node_ = transport::NodePtr(new transport::Node());
  node_->Init();
  getSdfParam<std::string>(_sdf, "platformSubTopic", platform_sub_topic_, platform_sub_topic_);
  getSdfParam<std::string>(_sdf, "contactSubTopic", contact_sub_topic_, contact_sub_topic_);

  platform_sub_ = node_->Subscribe("~/" + platform_sub_topic_, &GazeboPlatformPlugin::PlatformCallback, this);
  contact_sub_ = node_->Subscribe("~/" + contact_sub_topic_, &GazeboPlatformPlugin::ContactsCallback, this);

  string topicName = "~/platform/landing";
  platform_landing_pub_ = node_->Advertise<platform_msgs::msgs::PlatformLanding>(topicName, 1);

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPlatformPlugin::OnUpdate, this, _1));
}

void GazeboPlatformPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {
	if(_h > 0) {
		model_->SetScale(Vector(1, 1, _h), true);
		model_->SetWorldPose(math::Pose(_x,_y,_h/2,0,0,0), true);
	} else {
		model_->SetScale(Vector(1, 1, -_h), true);
		model_->SetWorldPose(math::Pose(_x,_y,_h/2-0.001,0,0,0), true);
	}
}

void GazeboPlatformPlugin::PlatformCallback(PlatformPtr& platform_message) {
	if(platform_message->has_h()){
		_h = platform_message->h();
	}
	if(platform_message->has_x()){
		_x = platform_message->x();
	}
	if(platform_message->has_y()){
		_y = platform_message->y();
	}
}

void GazeboPlatformPlugin::ContactsCallback(ContactsPtr& msg) {
	_landing_vel = model_->GetWorld()->GetModel("iris_dronecourse")->GetRelativeLinearVel();
	_landing_msg.set_time_msec(0);
	_landing_msg.set_vx(_landing_vel.x);
	_landing_msg.set_vy(_landing_vel.y);
	_landing_msg.set_vz(_landing_vel.z);
	_landing_msg.set_contact(false);
	if(msg->contact_size() > 0)  {
		if(model_->GetWorld()->GetModel("iris_dronecourse")->GetLink("iris::base_link")->GetBoundingBox().Ign().Intersects(model_->GetBoundingBox().Ign())){
			_landing_msg.set_contact(true);
		}
	}
	platform_landing_pub_->Publish(_landing_msg);
}
}
