FROM gestelt/learning-agile:latest
LABEL maintainer="johntgz@nus.edu.sg"

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# Copy .bashrc file
# COPY .bashrc /root/
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     jax \
#     && rm -rf /var/lib/apt/lists/*

# WORKDIR /gestelt_ws/src/
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     ros-noetic-vrpn-client-ros\
#     ros-noetic-roslint\ 
#     && rm -rf /var/lib/apt/lists/*
# RUN pip install --no-cache-dir "networkx<3.2"
# RUN pip install --no-cache-dir \
#         torch==2.4.1\
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
RUN git checkout tianchensun_learning_agile


## if the c_generated_code is git ignored, than this script is required to generate the c code
# WORKDIR /gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/
# RUN  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/acados/lib \
#     && export ACADOS_SOURCE_DIR=/root/acados \
#     && source ~/.bashrc \
#     && python3 learning_agile_sim.py --MANUAL_SET_POSE_TEST True --VISUALIZE False \
#     && rm -rf /var/lib/apt/lists/*

# copy the deployed model in the image
WORKDIR /gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/
# COPY deployed_model /gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/deployed_model
# COPY libtorch /opt/libtorch
# compile the workspace
WORKDIR /gestelt_ws/
# RUN python3 ./src/gestelt/gestelt_bringup/src/Learning_Agile/learning_agile_sim.py
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/acados/lib \
    && export ACADOS_SOURCE_DIR=/root/acados \
    && source /opt/ros/noetic/setup.bash \
    && catkin config --skiplist trajectory_planner se3_controller\
    # && catkin config --skiplist mppi_py_interface\
    && catkin config --skiplist model_inference\
    # && catkin config --skiplist vicon_bridge\
    && catkin build \
    && rm -rf /var/lib/apt/lists/* 


# DRONE_ID environment variable can be overriden on docker image startup
ENV MASTER_IP=localhost
ENV SELF_IP=localhost
ENV TZ=Asia/Singapore
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]