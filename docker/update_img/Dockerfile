FROM gestelt/learning-agile:base
LABEL maintainer="johntgz@nus.edu.sg"

# Enable the use of `source` keyword
SHELL ["/bin/bash", "-c"] 

# Copy .bashrc file
COPY .bashrc /root/

# Make directory and clone repository
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