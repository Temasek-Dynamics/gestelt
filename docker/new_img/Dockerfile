FROM ros:noetic-robot-focal
LABEL maintainer="johntgz@nus.edu.sg"

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim \
    git \
    tmux \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    cpufrequtils \
    screen \
    ros-noetic-tf2-geometry-msgs  \
    && rm -rf /var/lib/apt/lists/*

# Install mavROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-mavlink ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# Install eigen dependency
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev\
    && rm -rf /var/lib/apt/lists/*

# Copy .bashrc file
# COPY .bashrc /root/ # need further verify

# Install ACADOS prerequisites
COPY requirements.txt /root/
RUN pip install -r /root/requirements.txt

# Install ACADOS
WORKDIR /root/
RUN git clone --recursive https://github.com/acados/acados.git \
    && cd acados \
    && mkdir -p build \
    && cd build \
    && cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON -DACADOS_WITH_OPENMP=ON -DACADOS_NUM_THREADS=3 .. \
    && make install -j4 \
    && pip install -e /root/acados/interfaces/acados_template

# Install tera_renderer (dependency of ACADOS) 
WORKDIR /root/
RUN apt-get update && apt-get install --no-install-recommends -y \
    cargo \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/acados/tera_renderer.git \
    && cd tera_renderer \
    && cargo build --verbose --release \
    && mv target/release/t_renderer /root/acados/bin/t_renderer \
    && chmod +x /root/acados/bin/t_renderer 

WORKDIR /gestelt_ws/src/
RUN git clone https://github.com/ethz-asl/vicon_bridge.git

# Make directory and clone repository
WORKDIR /gestelt_ws/src/
RUN git clone https://github.com/JohnTGZ/gestelt.git -b tianchensun_learning_agile
WORKDIR /gestelt_ws/src/gestelt
RUN git pull
WORKDIR /gestelt_ws/
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/acados/lib \
    && export ACADOS_SOURCE_DIR=/root/acados \
    && source /opt/ros/noetic/setup.bash \
    && catkin config --skiplist trajectory_planner se3_controller\
    && catkin build 

# DRONE_ID environment variable can be overriden on docker image startup
ENV MASTER_IP=localhost
ENV SELF_IP=localhost

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]