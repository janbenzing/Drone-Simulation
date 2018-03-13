#! /bin/bash

if [ -z "$PX4_DOCKER_REPO" ]; then
	PX4_DOCKER_REPO=dronecourseepfl/dronecourse:spring_2018
fi

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

CCACHE_DIR=${HOME}/.ccache
mkdir -p ${CCACHE_DIR}

X11_TMP=/tmp/.X11-unix

COMMAND=$@

# Update docker image
docker image pull ${PX4_DOCKER_REPO}

# Start gzclient if $COMMAND contains substring "gazebo"
if [[ $COMMAND = *"gazebo"* ]]; then
  xterm -title "Gzclient" -e "${SRC_DIR}/Tools/dronecourse_gzclient_run.sh" &
  GZCLIENT_PID=`echo $!`
fi

# Start PX4 in docker container
docker run -it --rm -w ${SRC_DIR} \
	--name "dronecourse" \
	-e AWS_ACCESS_KEY_ID=${AWS_ACCESS_KEY_ID} \
	-e AWS_SECRET_ACCESS_KEY=${AWS_SECRET_ACCESS_KEY} \
	-e BRANCH_NAME=${BRANCH_NAME} \
	-e CCACHE_DIR=${CCACHE_DIR} \
	-e CI=${CI} \
	-e DISPLAY=$DISPLAY \
	-e GIT_SUBMODULES_ARE_EVIL=1 \
	-e LOCAL_USER_ID=`id -u` \
	-e TRAVIS_BRANCH=${TRAVIS_BRANCH} \
	-e TRAVIS_BUILD_ID=${TRAVIS_BUILD_ID} \
	-v ${CCACHE_DIR}:${CCACHE_DIR}:rw \
	-v ${SRC_DIR}:${SRC_DIR}:rw \
	-v ${X11_TMP}:${X11_TMP}:ro \
	--device=/dev/dri \
	-p 11345:11345 \
  -p 8888:8888 \
	${PX4_DOCKER_REPO} /bin/bash -c "export HEADLESS=true \
					&& $COMMAND"

if [[ $COMMAND = *"gazebo"* ]]; then
	kill ${GZCLIENT_PID}
fi
