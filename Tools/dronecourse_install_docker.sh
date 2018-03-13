set +e

# Update gazebo repository
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-get update

# install required packages
sudo apt-get install git docker.io gazebo7

# add user to group docker
sudo groupadd docker
sudo usermod -a -G docker $USER
sudo service docker restart

set -e