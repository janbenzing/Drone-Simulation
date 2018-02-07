#include "gazebo_mavlink_plugin.h"

namespace gazebo {


int helloWorld()
{
	return 4+4;
}

void GazeboMavlinkPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "GazeboMavlinkPlugin::Load starting" << std::endl;

  //Create socket
  // udp socket data
  mavlink_addr_ = htonl(INADDR_ANY);
  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr = _sdf->GetElement("mavlink_addr")->Get<std::string>();
    if (mavlink_addr != "INADDR_ANY") {
      mavlink_addr_ = inet_addr(mavlink_addr.c_str());
      if (mavlink_addr_ == INADDR_NONE) {
        fprintf(stderr, "invalid mavlink_addr \"%s\"\n", mavlink_addr.c_str());
        return;
      }
    }
  }
  if (_sdf->HasElement("mavlink_udp_port")) {
    mavlink_udp_port_ = _sdf->GetElement("mavlink_udp_port")->Get<int>();
  }else {
  	return;
  }

  // try to setup udp socket for communcation with simulator
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("create socket failed\n");
    return;
  }

  memset((char *)&_myaddr, 0, sizeof(_myaddr));
  _myaddr.sin_family = AF_INET;
  _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  // Let the OS pick the port
  _myaddr.sin_port = htons(0);

  if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
    printf("bind failed\n");
    return;
  }

  _srcaddr.sin_family = AF_INET;
  _srcaddr.sin_addr.s_addr = mavlink_addr_;
  _srcaddr.sin_port = htons(mavlink_udp_port_);
  _addrlen = sizeof(_srcaddr);

  std::cout << "GazeboMavlinkPlugin::Load finished" << std::endl;
}



void GazeboMavlinkPlugin::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t system_ID, uint8_t component_ID) {
  component_ID = 0;	//???
  uint8_t payload_len = mavlink_message_lengths[msgid];
  unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  /* header */
  buf[0] = MAVLINK_STX;
  buf[1] = payload_len;
  /* no idea which numbers should be here*/
  buf[2] = 100;
  buf[3] = system_ID;
  buf[4] = component_ID;
  buf[5] = msgid;

  /* payload */
  memcpy(&buf[MAVLINK_NUM_HEADER_BYTES],msg, payload_len);

  /* checksum */
  uint16_t checksum;
  crc_init(&checksum);
  crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
  crc_accumulate(mavlink_message_crcs[msgid], &checksum);

  buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
  buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

  ssize_t len;

  len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

  if (len <= 0) {
    printf("Failed sending mavlink message\n");
  }
}
}