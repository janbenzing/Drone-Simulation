#include "gazebo/common/Plugin.hh"
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <sys/socket.h>
#include <netinet/in.h>

#include "mavlink/v1.0/dronecourse/mavlink.h"

#ifndef _GAZEBO_MAVLINK_PLUGIN_HH_
#define _GAZEBO_MAVLINK_PLUGIN_HH_

namespace gazebo {


static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

int helloWorld();

class GazeboMavlinkPlugin : public ModelPlugin {
public:
	GazeboMavlinkPlugin() : ModelPlugin(){};
	~GazeboMavlinkPlugin(){};

 	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
	//virtual void OnUpdate(const common::UpdateInfo& /*_info*/) = 0;
	void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t system_ID, uint8_t component_ID);

	int _fd;
	struct sockaddr_in _myaddr;  ///< The locally bound address
	struct sockaddr_in _srcaddr;  ///< SITL instance
	socklen_t _addrlen;
	unsigned char _buf[65535];
	in_addr_t mavlink_addr_;
  	int mavlink_udp_port_;
};
}

#endif /* _GAZEBO_MAVLINK_PLUGIN_HH_ */