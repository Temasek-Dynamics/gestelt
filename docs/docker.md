# Docker

# Official ROS docker images
They can be found on the [docker hub registry](https://registry.hub.docker.com/_/ros/). We are currently using `noetic-ros-core-focal`.

# Building images
```bash
docker build -t gestelt/ros:noetic .
# --rm flag tells tdocker to clean up the container after exiting
docker run -it --rm gestelt/ros:noetic .
```

# Pushing images to Docker Hub repository
```bash
docker tag gestelt/ros:noetic johntgz95/gestelt
docker push johntgz95/gestelt
```

# Resources
[Uploading an image to a Docker Hub repo](https://docs.docker.com/guides/workshop/04_sharing_app/).