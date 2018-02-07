#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/lexical_cast.hpp>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo_waypoint_plugin.h"
#include <gazebo/msgs/msgs.hh>
#include "waypoint.pb.h"

using namespace std;

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboWaypointPlugin);

GazeboWaypointPlugin::~GazeboWaypointPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->_updateConnection);
}

void GazeboWaypointPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  auto _name = sdf->GetAttribute("name")->GetAsString();
  boost::to_upper(_name);
  if (boost::starts_with(_name, "WAYPOINT")) {
    this->_no = atoi(_name.substr(strlen("WAYPOINT")).c_str());
  } else {
    gzerr << "[waypoint] Waypoint N must be named 'waypointN'\n";
    this->_no = -1;
    return;
  }

  if(sdf->HasElement("radius")) {
    this->_radius = sdf->GetElement("radius")->Get<double>();
  } else {
    this->_radius = 2;
  }

  this->_visual_name = "__WAYPOINT__VISUAL__NO__" + this->_no;
  this->_model = model;
  this->_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->_node->Init(_model->GetWorld()->GetName());
  this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWaypointPlugin::OnUpdate, this, _1));
  this->_visual_pub = this->_node->Advertise<msgs::Visual>("~/visual", 10);
  this->_waypoint_pub = this->_node
          ->Advertise<waypoint_msgs::msgs::Waypoint>("~/waypoint", 10);
  this->_waypoint_sub = this->_node
          ->Subscribe("~/waypoint", &GazeboWaypointPlugin::OnWaypointUpdate, this);

  /* Waypoint no 0 is active by default */
  this->SetActive(this->_no == 0);
  this->SetValid(false);
  this->UpdateVisual(this->_color);
}

void GazeboWaypointPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {
  if(this->_active && !this->_valid) {
    /* Retrieve position of drone and waypoint */
    ignition::math::Vector3d pose = _model->GetWorld()
          ->GetModel("iris_dronecourse")
          ->GetWorldPose().Ign().Pos();
    ignition::math::Vector3d wp_pose = _model
          ->GetWorldPose().Ign().Pos();
    /* Compute distance from center waypoint */
    double distance = (pose - wp_pose).Length();
    /* Validate waypoint if drone is inside the waypoint radius */
    this->SetValid(distance < this->_radius);
  }
  this->UpdateVisual(this->_color);
}

void GazeboWaypointPlugin::UpdateVisual(std::string color) {
  msgs::Visual visualMsg;

  visualMsg.set_name(this->_visual_name);

  visualMsg.set_parent_name(_model->GetScopedName());
  msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::SPHERE);
  geomMsg->mutable_sphere()->set_radius(this->_radius);
  visualMsg.mutable_material()->mutable_script()
        ->set_name(color);
  msgs::Set(visualMsg.mutable_pose(),
    _model->GetWorldPose().Ign());
  visualMsg.set_cast_shadows(false);
  visualMsg.set_transparency(0.5);

  this->_visual_pub->Publish(visualMsg);
}

void GazeboWaypointPlugin::SetValid(bool valid) {
  bool old_valid = this->_valid;
  this->_valid = valid;
  if(old_valid != valid) {
    this->FireValid(valid);
    if(this->_valid) {
      this->SetColor(GAZEBO_COLOR_VALIDATED);
    } else {
      if(this->_active) {
        this->SetColor(GAZEBO_COLOR_ACTIVE);
      } else {
        this->SetColor(GAZEBO_COLOR_INACTIVE);
      }
    }
  }
}

void GazeboWaypointPlugin::SetActive(bool active) {
  bool old_active = this->_active;
  this->_active = active;
  if(old_active != active) {
    if(this->_valid) {
      this->SetColor(GAZEBO_COLOR_VALIDATED);
    } else {
      if(this->_active) {
        this->SetColor(GAZEBO_COLOR_ACTIVE);
      } else {
        this->SetColor(GAZEBO_COLOR_INACTIVE);
      }
    }
  }
}

void GazeboWaypointPlugin::SetColor(std::string color) {
  this->_color = color;
}

void GazeboWaypointPlugin::FireValid(bool valid) {
  /* Only notify validation. Do not notify de-validation. */
  if(!valid) {
    return;
  }
  waypoint_msgs::msgs::Waypoint msg;
  msg.set_waypoint_no(this->_no);
  msg.set_validated(valid);
  _waypoint_pub->Publish(msg);
}

void GazeboWaypointPlugin::OnWaypointUpdate(WaypointPtr &msg) {
  if(msg->has_waypoint_no() && msg->waypoint_no() + 1 == this->_no) {
    this->SetActive(true);
  }
}
}
