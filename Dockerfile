FROM ros:indigo-perception

# add ihmc messages
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    python-pip \
    python-catkin-tools \
    ros-indigo-catkin \
    ros-indigo-ihmc-msgs \
    ros-indigo-rosbag \
    ros-indigo-tf \
    ros-indigo-tf2 \
    ros-indigo-rosbash \
 && rm -rf /var/lib/apt/lists/*

# clone srcsim
ENV WS /home/docker/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim

# build srcsim messages
RUN . /opt/ros/indigo/setup.sh \
 && catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
 && catkin config --install \
 && catkin build

# include bag file with footsteps preprogrammed
ADD footsteps_2017-05-02-14-40-59.bag ${WS}/
RUN echo "while rosbag play ${WS}/footsteps_2017-05-02-14-40-59.bag && python ${WS}/src/srcsim/scripts/rossleep.py 8; do date; done" \
  > do_footsteps.bash

ADD posix_ipc-1.0.0.tar.gz . 
WORKDIR posix_ipc-1.0.0
RUN python setup.py install 
WORKDIR ${WS}
RUN pip install trollius autobahn 
RUN apt-get update \
 && apt-get install -y \
    ros-indigo-gazebo-ros

EXPOSE 8000 8081 8082 8083 9000

# start a roscore
CMD ["python", "-m", "SimpleHTTPServer", "8000"]
