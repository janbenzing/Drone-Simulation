#!/bin/sh -e

USAGE="Usage: $0 [OPTIONS]...
Install the requirements for the dronecourse.

Options:
    -f          ignore recommended system warning
    -h          show this message and exit

"

O_FORCE=0
while getopts ":hf" opt; do
    case $opt in
        f)  O_FORCE=1;;
        \?) echo "Invalid option: -$OPTARG" >&2; printf "$USAGE"
            exit 1;;
        h)  printf "$USAGE"; exit 0;;
    esac
done

# check if running as root
if [ "`id -u`" -ne "0" ]
then
    echo "This script needs to be run as root."
    echo "Try again with 'sudo $0'."
    exit 1;
fi

# check system
RECOMM_SYSTEM="Ubuntu 16.04.3 LTS"
CURREN_SYSTEM=`lsb_release --description | cut -f2`
if [ "$RECOMM_SYSTEM" != "$CURREN_SYSTEM" ] && [ $O_FORCE -eq "0" ]
then
    echo "You are using '$CURREN_SYSTEM'."
    echo "Use '$RECOMM_SYSTEM' to ensure compatibility."
    # echo "Aborting. Use -f to ignore this message and force install."
    # exit 0
fi

# add osrf repository
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list

# setup key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# install required packages
apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    gazebo7 \
    git \
    libgazebo7-dev \
    libopencv-dev \
    libeigen3-dev \
    ninja-build \
    protobuf-compiler \
    python-empy \
    python-jinja2 \
    xsltproc

