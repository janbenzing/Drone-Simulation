#! /bin/bash

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../


while [ "`netcat 127.0.0.1 11345 | head -c 1`" == "" ] ; do
	sleep 1
done;

# setup environment variables
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC_DIR}/Tools/sitl_gazebo/models

# start client
gzclient --verbose
