#include "gazebo_target_camera_plugin.h"
#include <gazebo/sensors/sensors.hh>
#include "gazebo/rendering/Camera.hh"
#include <string>

namespace gazebo {


// Zurich Irchel Park
static const double lat_zurich = 47.397742 * M_PI / 180;  // rad
static const double lon_zurich = 8.545594 * M_PI / 180;  // rad
static const double alt_zurich = 488.0; // meters
// Seattle downtown (15 deg declination): 47.592182, -122.316031
// static const double lat_zurich = 47.592182 * M_PI / 180;  // rad
// static const double lon_zurich = -122.316031 * M_PI / 180;  // rad
// static const double alt_zurich = 86.0; // meters
static const float earth_radius = 6353000;  // m

GZ_REGISTER_MODEL_PLUGIN(TargetCameraPlugin);

// debug only
void printPose(const math::Pose& pose, const std::string& name)
{
  const math::Vector3 euler = pose.rot.GetAsEuler();
  std::cout << "pose " << name << " pos ( " << pose.pos.x << " | " << pose.pos.y << " | " << pose.pos.z << ")   rot ( " << euler.x << " | " << euler.y << " | " << euler.z << " ) " << std::endl;
}

TargetCameraPlugin::TargetCameraPlugin() :
  // GazeboMavlinkPlugin(),
  // ModelPlugin(),
  image_width2_(IMAGE_WIDTH_DEFAULT_ / 2.0f),
  image_height2_(IMAGE_HEIGHT_DEFAULT_ / 2.0f),
  hfov2_(HFOV_DEFAULT_ / 2.0f),
  noise_xy_std_(NOISE_XY_STD_DEFAULT_),
  noise_z_std_(NOISE_Z_STD_DEFAULT_)
{
  vfov2_ = hfov2_*image_height2_/((float)image_width2_);
  focal_length_ = image_width2_ / tan(hfov2_);
  period_s_ = 1.0f / UPDATE_RATE_DEFAULT_;
}


void TargetCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GazeboMavlinkPlugin::Load(_model, _sdf);
 
  world_ = _model->GetWorld();

  // -----------------------
  // find camera parameters
  // -----------------------
  const sensors::CameraSensorPtr camera_sensor = FindCameraSensor(_model);
  if(camera_sensor != NULL)
  {
    const rendering::CameraPtr camera = camera_sensor->Camera();
    hfov2_ = float(camera->HFOV().Radian())/2.0f;
    vfov2_ = float(camera->VFOV().Radian())/2.0f;
    image_width2_ = camera->ImageWidth()/2.0f;
    image_height2_ = camera->ImageHeight()/2.0f;
    focal_length_ = image_width2_ / tan(hfov2_);
    period_s_ = 1.0f / camera_sensor->UpdateRate();
    // add callback to camera
    // newFrameConnection_ = camera->ConnectNewImageFrame(boost::bind(&TargetCameraPlugin::OnNewFrame, this));
  }
  else
  {
    camera_link_ = _model;
    std::cout << "TargetCameraPlugin::Load(..) could not find camera sensor! Using default values;" << std::endl;
    std::cout << "   using model (" << _model->GetName() << ") for camera pose" << std::endl;
    // add callback for world update
  }

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TargetCameraPlugin::OnUpdate, this, _1));

  // Find targets listed in sdf tag <target_link>
  FindTargets(_sdf);

  // Load detection parameters
  FindDetectionParameters(_sdf);

  gimbal_.Load(_sdf, _model);

  // connection to publish pose over google proto buf
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  std::string topicName = "~/" + _model->GetName() + "/TargetPos";
  targetPos_pub_ = node_handle_->Advertise<target_camera::msgs::TargetPositionImage>(topicName, 10);
}


void TargetCameraPlugin::OnUpdate(const common::UpdateInfo& info)
{
  // Get Delta T
  common::Time current_time = info.simTime;
  double dt = (current_time - last_time_).Double();

  gimbal_.Update(current_time.Double());

  if(dt > period_s_)
  {
    OnNewFrame();
    last_time_ = current_time;
  }
}



void TargetCameraPlugin::OnNewFrame()
{
  const Pose& camera_pose = camera_link_->GetWorldPose().Ign();
  uint64_t timestamp_us = (uint64_t)(world_->GetSimTime().Double()*1e6);

  // iterate over all targets
  std::map<physics::EntityPtr, TargetMsg>::iterator msg_it; 
  for(msg_it = message_map_.begin(); msg_it != message_map_.end(); msg_it++)
  {
      // Retrieve entity and message for this target
      physics::EntityPtr entity = msg_it->first;
      TargetMsg& msg = msg_it->second;

      // Get name of entity
      std::string name = entity->GetName();

      if (name == "target_platform") {
          // This is the truck
        
          // Get bearing of target in camera frame
          const Pose& target_pose = msg_it->first->GetWorldPose().Ign();
          const Vector rel_pos = (target_pose - camera_pose).Pos();

          float pixel_x = round(-(focal_length_ * rel_pos.Y()/rel_pos.X()) + image_width2_); // column
          float pixel_y = round(-(focal_length_ * rel_pos.Z()/rel_pos.X()) + image_height2_); // row
          float z = abs(rel_pos.Length());

          float noise_z_std = noise_z_std_ * z;

          // add noise
          pixel_x += math::Rand::GetDblNormal(0.0, noise_xy_std_);
          pixel_y += math::Rand::GetDblNormal(0.0, noise_xy_std_);
          z +=       math::Rand::GetDblNormal(0.0, noise_z_std);

          if(pixel_x >= 0 && pixel_x < 2*image_width2_ && pixel_y >= 0 && pixel_y < 2*image_height2_)
          {
            if(rel_pos.X() > 0){
              msg.set_time_usec(timestamp_us);
              msg.set_u(pixel_x);
              msg.set_v(pixel_y);
              msg.set_dist(z);
              msg.set_pitch(gimbal_.GetPitch());
              msg.set_yaw(gimbal_.GetYaw());
              msg.set_var_u(noise_xy_std_*noise_xy_std_);
              msg.set_var_v(noise_xy_std_*noise_xy_std_);
              msg.set_var_dist(noise_z_std*noise_z_std);
              targetPos_pub_->Publish(msg);
            }
          }

          Vector target_vel = msg_it->first->GetWorldLinearVel().Ign();
          
          // Send ground truth position of Truck
          SendTruckPositionMsg(msg.target_num(), target_pose.Pos(), target_vel, 0.0f, timestamp_us);

      } else if (name == "box") {
          // This is the platform

          // Get bearing of target in camera frame
          const Pose& target_pose = msg_it->first->GetWorldPose().Ign();
          const Vector rel_pos = (target_pose - camera_pose).Pos();

          // Send ground truth position of Platform
          SendPlatformPositionMsg(msg.target_num(), target_pose.Pos(), timestamp_us);
      }
  }

  SendPositionMsg(-1, camera_pose.Pos(),  camera_link_->GetWorldLinearVel().Ign(), timestamp_us);
  SendGlobalPositionMsg(-1, camera_pose, timestamp_us/1000);
}


const sensors::CameraSensorPtr TargetCameraPlugin::FindCameraSensor(physics::ModelPtr model)
{
  int model_sensor_count = model->GetSensorCount();
  if(model_sensor_count > 0)
  {
    sensors::SensorManager* sensor_manager = sensors::SensorManager::Instance();
    const physics::Link_V& links = model->GetLinks();
    physics::Link_V::const_iterator it;
    for(it = links.begin(); it != links.end(); it++)
    {
      physics::LinkPtr link = *it;
      int link_sensor_count = link->GetSensorCount();
      for(int i = 0; i < link_sensor_count; i++)
      {
        const sensors::SensorPtr sensor = sensor_manager->GetSensor(link->GetSensorName(i));
        sensors::CameraSensorPtr camera_sensor;
        #if GAZEBO_MAJOR_VERSION >= 7
          camera_sensor = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
        #else
          camera_sensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
        #endif
        if(camera_sensor != NULL)
        {
          camera_link_ = link;
          return camera_sensor;
        }
      }
    }
  }
  return NULL;
}


int TargetCameraPlugin::FindTargets(const sdf::ElementPtr _sdf)
{
  int target_num = 0;

  if(_sdf->HasElement(TARGET_LINK))
  {
    for(sdf::ElementPtr target = _sdf->GetElement(TARGET_LINK); target != NULL; target = target->GetNextElement(TARGET_LINK))
    {
      // Get Target Link
      std::string target_name = target->Get<std::string>();
      physics::EntityPtr target_link = world_->GetEntity(target_name);

      if(target_link != NULL)
      {
        std::cout << "target link found: " << target_name << std::endl; 

        // create new entry in message_map
        message_map_[target_link].set_target_num(target_num++);
      }
      else
      {
        std::cout << "target link NOT found: " << target_name << std::endl;
      }
    }
  }
  
  return target_num;
}


bool TargetCameraPlugin::FindDetectionParameters(const sdf::ElementPtr _sdf)
{
  bool success = true;
  if(_sdf->HasElement(NOISE_XY_STD_NAME))
  {
    sdf::ElementPtr element = _sdf->GetElement(NOISE_XY_STD_NAME);
    noise_xy_std_ = element->Get<float>();
    std::cout << "setting xy noise to " << noise_xy_std_ << std::endl;
  } else {
    success = false;
  }

  if(_sdf->HasElement(NOISE_Z_STD_NAME))
  {
    sdf::ElementPtr element = _sdf->GetElement(NOISE_Z_STD_NAME);
    noise_z_std_ = element->Get<float>();
    std::cout << "setting z noise to " << noise_z_std_ << std::endl;
  } else {
    success = false;
  }

  return success;
}

bool TargetCameraPlugin::SendTruckPositionMsg(uint16_t target_id, const Vector& pos, const Vector& vel, float heading, uint64_t timestamp_us)
{
  mavlink_truck_position_t msg;
  msg.time_usec = timestamp_us;

  msg.x = pos.Y();
  msg.y = pos.X();
  msg.z = -pos.Z();
  msg.vx = vel.Y();
  msg.vy = vel.X();
  msg.vz = -vel.Z();
  msg.heading = heading;

  send_mavlink_message(MAVLINK_MSG_ID_TRUCK_POSITION, &msg, target_id + 100, 200);

  return true;
}

bool TargetCameraPlugin::SendPlatformPositionMsg(uint16_t target_id, const Vector& pos, uint64_t timestamp_us)
{
  mavlink_platform_position_t msg;
  msg.time_usec = timestamp_us;
  msg.x = pos.Y();
  msg.y = pos.X();
  msg.z = -pos.Z();

  send_mavlink_message(MAVLINK_MSG_ID_PLATFORM_POSITION, &msg, target_id + 100, 200);

  return true;
}


bool TargetCameraPlugin::SendPositionMsg(uint16_t target_id, const Vector& target_pos, const Vector& speed, uint32_t timestamp_ms)
{
  mavlink_local_position_ned_t msg;
  msg.time_boot_ms = timestamp_ms;
  msg.x = target_pos.Y();
  msg.y = target_pos.X();
  msg.z = -target_pos.Z();
  msg.vx = speed.Y();
  msg.vy = speed.X();
  msg.vz = -speed.Z();

  send_mavlink_message(MAVLINK_MSG_ID_LOCAL_POSITION_NED, &msg, target_id + 100, 200);
  return true;
}


bool TargetCameraPlugin::SendGlobalPositionMsg(uint16_t target_id, const Pose& pose, uint32_t timestamp_ms)
{
  Vector pos_lf = pose.Pos(); // position in local frame

  // TODO: Remove GPS message from IMU plugin. Added gazebo GPS plugin. This is temp here.
  // reproject local position to gps coordinates
  double x_rad = pos_lf.Y() / earth_radius; // north
  double y_rad = pos_lf.X() / earth_radius; // east
  double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);
  float lat_rad = lat_zurich;
  float lon_rad = lon_zurich;

  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_zurich) + (x_rad * sin_c * cos(lat_zurich)) / c);
    lon_rad = (lon_zurich + atan2(y_rad * sin_c, c * cos(lat_zurich) * cos_c - x_rad * sin(lat_zurich) * sin_c));
  }

  float heading = M_PI/2.0-pose.Rot().Yaw();
  while(heading > 2*M_PI)
    heading -= 2*M_PI;
  while(heading < 0)
    heading += 2*M_PI;

  mavlink_global_position_int_t msg;
  msg.time_boot_ms = timestamp_ms;
  msg.lat = lat_rad * 180 / M_PI * 1e7;
  msg.lon = lon_rad * 180 / M_PI * 1e7;
  msg.alt = (pos_lf.Z() + alt_zurich) * 1000;
  msg.relative_alt = pos_lf.Z() * 1000 ;
  msg.vx = 0;
  msg.vy = 0;
  msg.vz = 0;
  msg.hdg = heading * 18000/M_PI;

  send_mavlink_message(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &msg, target_id + 100, 200);

}
}