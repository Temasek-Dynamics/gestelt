FROM tianchensun787/auto-multi-lift:latest
LABEL maintainer="johntgz@nus.edu.sg"

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# Copy .bashrc file
# COPY .bashrc /root/
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     libstdc++6 \
#     && rm -rf /var/lib/apt/lists/*

# WORKDIR /gestelt_ws/src/
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     ros-noetic-vrpn-client-ros\
#     ros-noetic-roslint\ 
#     && rm -rf /var/lib/apt/lists/*
# RUN pip install --no-cache-dir "networkx<3.2"

# RUN pip install --no-cache-dir \
#         onnxruntime

#         # torchvision==0.19.1 \
#         # torchaudio==2.4.1 \
#         --index-url https://download.pytorch.org/whl/cpu

# RUN pip install --no-cache-dir \
#     ray\
#     && rm -rf /var/lib/apt/lists/*
# RUN git clone https://github.com/ros-drivers/vrpn_client_ros.git

# Make directory and clone repository
WORKDIR /gestelt_ws/src/gestelt
# RUN git fetch origin
RUN git pull
RUN git checkout TEST/RPT_control_tracking
RUN vcs import < thirdparty.repos --recursive

# compile the workspace
WORKDIR /gestelt_ws/
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/acados/lib \
    && export ACADOS_SOURCE_DIR=/root/acados \
    && source /opt/ros/noetic/setup.bash \
    # && catkin clean -y \
    # && catkin build vicon_bridge\ 
    && rm -rf /var/lib/apt/lists/* 


# DRONE_ID environment variable can be overriden on docker image startup
ENV MASTER_IP=localhost
ENV SELF_IP=localhost
ENV TZ=Asia/Singapore
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]