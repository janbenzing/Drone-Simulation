#ifndef _GAZEBO_TARGET_CAMERA_PLUGIN_HH_
#define _GAZEBO_TARGET_CAMERA_PLUGIN_HH_

#include "gazebo_mavlink_plugin.h"
#include <gazebo/physics/physics.hh>
#include "gazebo/gazebo.hh"
#include "TargetPositionImage.pb.h"
#include "gazebo_gimbal.hpp"


namespace gazebo
{
  class TargetCameraPlugin : public GazeboMavlinkPlugin
  {
    public:
      // Default camera parameters (values defined after class definition)
      const static float HFOV_DEFAULT_;                // default horizontal field of view [rad]
      const static unsigned int IMAGE_WIDTH_DEFAULT_;  // default image width [pixel]
      const static unsigned int IMAGE_HEIGHT_DEFAULT_; // default image height [pixel]
      const static float UPDATE_RATE_DEFAULT_;         // default update rate of the camera [Hz]
      const static float NOISE_XY_STD_DEFAULT_;        // default standard deviation of noise in xy [pix^2]
      const static float NOISE_Z_STD_DEFAULT_;        // default standard deviation of noise in z [m^2]

      // SDF parameter names (values defined after class definition)
      const static std::string TARGET_LINK;
      const static std::string NOISE_XY_STD_NAME;
      const static std::string NOISE_Z_STD_NAME;

      typedef ignition::math::Vector3d Vector;
      typedef ignition::math::Pose3d Pose;

    
      typedef target_camera::msgs::TargetPositionImage TargetMsg;


    	TargetCameraPlugin();
    	
    	virtual ~TargetCameraPlugin(){};

    	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      
      virtual void OnNewFrame();

      /**
       * \brief   Callback for Model update, calls OnNewFrame when required
       * \details This func is only used if no camera sensor is used, i.e.,
                  we cannot update on OnNewFrame
       */
      virtual void OnUpdate(const common::UpdateInfo& /*_info*/);

    private:
      const sensors::CameraSensorPtr FindCameraSensor(physics::ModelPtr model);

      int FindTargets(const sdf::ElementPtr _sdf);

      bool FindDetectionParameters(const sdf::ElementPtr _sdf);

      bool SendTruckPositionMsg(uint16_t target_id, const Vector& pos, const Vector& vel, float heading, uint64_t timestamp_us);
      bool SendPlatformPositionMsg(uint16_t target_id, const Vector& pos, uint64_t timestamp_us);

      bool SendPositionMsg(uint16_t target_id, const Vector& target_pos, const Vector& speed, uint32_t timestamp_ms);
      bool SendGlobalPositionMsg(uint16_t target_id, const Pose& pose, uint32_t timestamp_ms);


      common::Time last_time_;          // Time of previous update (simulated time)

      physics::WorldPtr world_;            // World the model lives in
      physics::EntityPtr camera_link_;  // Link/Model which Pose is used as reference

      // camera parameters
      float hfov2_;                 // half horizontal field of view [rad]
      float vfov2_;                 // half vertical field of view [rad]
      unsigned int image_width2_;   // half the image width [pixel]
      unsigned int image_height2_;  // half the image height [pixel]
      float focal_length_;          // focal length of the camera as image_width2_ / tan(hfov2_) [pix]
      float period_s_;              // period of the camera (1/frequency) [s]

      std::map<physics::EntityPtr, TargetMsg> message_map_;  // Map containing target models and messages for each model
      event::ConnectionPtr updateConnection_;               // 
      event::ConnectionPtr newFrameConnection_;      // connection between OnNewFrame and camera, triggers OnNewFrame
      transport::PublisherPtr targetPos_pub_;
      transport::NodePtr node_handle_;

      float noise_xy_std_;           // Standard deviation of noise in xy (pixels^2)
      float noise_z_std_;            // Standard deviation of noise in z (meters^2)

      Gimbal gimbal_;
  };

  // Default camera parameters
  const float TargetCameraPlugin::HFOV_DEFAULT_ = 1.0f;               // default horizontal field of view [rad]
  const unsigned int TargetCameraPlugin::IMAGE_WIDTH_DEFAULT_  = 640; // default image width [pixel]
  const unsigned int TargetCameraPlugin::IMAGE_HEIGHT_DEFAULT_ = 340; // default image height [pixel]
  const float TargetCameraPlugin::UPDATE_RATE_DEFAULT_ = 30;          // default update rate of the camera [Hz]
  // Default detection parameters
  const float TargetCameraPlugin::NOISE_XY_STD_DEFAULT_ = 0;//5;        // default standard deviation of noise in xy [pix^2]
  const float TargetCameraPlugin::NOISE_Z_STD_DEFAULT_ = 0;//0.01;      // default standard deviation of noise in z [m^2]

  // SDF parameter names
  const std::string TargetCameraPlugin::TARGET_LINK = "target_link";
  const std::string TargetCameraPlugin::NOISE_XY_STD_NAME = "noise_xy";
  const std::string TargetCameraPlugin::NOISE_Z_STD_NAME = "noise_z";


}
#endif