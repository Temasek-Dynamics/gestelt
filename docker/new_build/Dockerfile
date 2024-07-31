FROM ros:noetic-ros-core-focal
LABEL maintainer="johntgz@nus.edu.sg"

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    iproute2 \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git tmux python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Install mavROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-pcl-ros libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# Make directory and clone repository
WORKDIR /gestelt_ws/src/
RUN git clone https://github.com/JohnTGZ/gestelt.git -b master 
WORKDIR /gestelt_ws/
RUN source /opt/ros/noetic/setup.bash \
    && catkin config --skiplist decomp_ros_utils decomp_test_node radxa_utils gestelt_test trajectory_inspector \
    && catkin build 
RUN echo "source /gestelt_ws/devel/setup.bash" >> /root/.bashrc 

# Copy .bashrc file
COPY .bashrc /root/

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
