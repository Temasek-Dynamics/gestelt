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
# RUN git clone https://github.com/ros-drivers/vrpn_client_ros.git

# Make directory and clone repository
WORKDIR /gestelt_ws/src/gestelt
RUN git fetch origin
RUN git checkout origin/with_gate_model_16_Jan


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
# compile the workspace
WORKDIR /gestelt_ws/
# RUN python3 ./src/gestelt/gestelt_bringup/src/Learning_Agile/learning_agile_sim.py
RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/acados/lib \
    && export ACADOS_SOURCE_DIR=/root/acados \
    && source /opt/ros/noetic/setup.bash \
    && catkin config --skiplist trajectory_planner se3_controller\
    && catkin config --skiplist mppi_py_interface\
    && catkin build \
    && rm -rf /var/lib/apt/lists/* 


# DRONE_ID environment variable can be overriden on docker image startup
ENV MASTER_IP=localhost
ENV SELF_IP=localhost

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]