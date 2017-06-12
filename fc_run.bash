#!/usr/bin/env bash

# example run: ./run_fc_team_docker.bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
codedir="/home/ubuntu/code"
current="$(pwd)"
TC=false
if [ -f $codedir/enable_traffic_shaper.cfg ] ; then
  TC=true
fi

# launch script to monitor SRC tasks
kill -9 `pgrep -f src_monitor`
if [ "$TC" = true ] ; then
 echo "starting src monitor"
 $DIR/src_monitor.bash |& tee -a $codedir/cloudsim-src-monitor.log &
fi

echo "running team's docker container"
echo $current


# XAUTH=/tmp/.docker.xauth
# xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
if [ ! -f /tmp/.docker.xauth ]
then
  export XAUTH=/tmp/.docker.xauth
  xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# Use lspci to check for the presence of an nvidia graphics card
has_nvidia=`lspci | grep -i nvidia | wc -l`

# Set docker gpu parameters
if [ ${has_nvidia} -gt 0 ]
then
  # check if nvidia-modprobe is installed
  if ! which nvidia-modprobe > /dev/null
  then
    echo nvidia-docker-plugin requires nvidia-modprobe
    echo please install nvidia-modprobe
    exit -1
  fi
  # check if nvidia-docker-plugin is installed
  if curl -s http://localhost:3476/docker/cli > /dev/null
  then
    DOCKER_GPU_PARAMS=" $(curl -s http://localhost:3476/docker/cli)"
  else
    echo nvidia-docker-plugin not responding on http://localhost:3476/docker/cli
    echo please install nvidia-docker-plugin
    echo https://github.com/NVIDIA/nvidia-docker/wiki/Installation
    exit -1
  fi
else
  DOCKER_GPU_PARAMS=""
fi

DISPLAY="${DISPLAY:-:0}"

docker run -it --name team_container \
  -e DISPLAY=unix$DISPLAY \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e ROS_MASTER_URI=http://192.168.2.1:11311 \
  -e ROS_IP=192.168.2.10 \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "/tmp/.docker.xauth:/tmp/.docker.xauth" \
  -v /dev/log:/dev/log \
  -v /home/ubuntu/fc-dryrun:/home/docker/fc-dryrun \
  --ulimit rtprio=99 \
  --net=vpn-br0 \
  --ip=192.168.2.10 \
  ${DOCKER_GPU_PARAMS} \
  fcomputer /bin/bash
