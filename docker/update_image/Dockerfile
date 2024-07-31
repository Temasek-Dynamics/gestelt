FROM gestelt/sfc:base
LABEL maintainer="johntgz@nus.edu.sg"

SHELL ["/bin/bash", "-c"] 

# Make directory and clone repository
WORKDIR /gestelt_ws/src/gestelt
RUN git checkout master
RUN git pull
WORKDIR /gestelt_ws/
RUN source /opt/ros/noetic/setup.bash \
    && catkin config --skiplist decomp_ros_utils decomp_test_node radxa_utils gestelt_test trajectory_inspector \
    && catkin build 

# Copy .bashrc file
COPY .bashrc /root/

# DRONE_ID environment variable can be overriden on docker image startup
ENV DRONE_ID=0

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]