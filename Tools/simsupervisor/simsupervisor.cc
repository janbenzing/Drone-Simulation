/*
 * Copyright (c) 2017 Arthur Gay
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include "waypoint.pb.h"

#ifdef __linux__
#define COLOR_RESET "\033[0m"
#define COLOR_ERROR "\033[31m"
#define COLOR_SUCCESS "\033[32m"
#else
#define COLOR_RESET ""
#define COLOR_ERROR ""
#define COLOR_SUCCESS ""
#endif

#define NWAYPOINTS 3

typedef const boost::shared_ptr<const waypoint_msgs::msgs::Waypoint>
    WaypointPtr;

bool run = true;

void _cb_waypoint(WaypointPtr &msg) {
    std::cout
        << "wp "
        << msg->waypoint_no()
        << " "
        << msg->validated()
        << "\n";
    if(msg->waypoint_no() + 1 >= NWAYPOINTS) {
        run = false;
    }
}

int main(int _argc, char **_argv)
{
  waypoint_msgs::msgs::Waypoint msg;

  // Load gazebo
  std::cout<<"Loading gazebo...\n";
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  std::cout<<"Initializing node...\n";
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Subscribe to Gazebo topics
  std::cout<<"Listening to topics...\n";
  gazebo::transport::SubscriberPtr pub = node->Subscribe("~/waypoint", _cb_waypoint);

  while (run)
        gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
  return 0;
}
