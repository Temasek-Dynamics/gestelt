# Docker

# Official ROS docker images
They can be found on the [docker hub registry](https://registry.hub.docker.com/_/ros/). We are currently using `noetic-ros-core-focal`.

# Host

## Building images
```bash
docker build -t gestelt/ros:noetic .
# --rm flag tells tdocker to clean up the container after exiting
docker run -it --rm gestelt/ros:noetic .
```

## Pushing images to Docker Hub repository
```bash
docker tag gestelt/ros:noetic johntgz95/gestelt
docker push johntgz95/gestelt

# All-in-one build and push
docker build --platform linux/arm64 -t johntgz95/radxa-base:latest --push .

docker build --platform linux/arm64 -t johntgz95/radxa-gestelt:latest --push .
```

## Running containers
```bash
docker run -it gestelt/noetic:latest

# Find name of new machine 
docker ps -l

# Start additional bash sessions in same container
docker exec -it <name_of_container> bash

#Set up environment
source ros_entrypoint.sh
```

# Radxa

## Installing docker
```bash
curl -fsSL test.docker.com -o get-docker.sh && sh get-docker.sh
sudo usermod -aG docker $USER 

# Test the installation
docker run hello-world 

# [TROUBLESHOOTING] If there is an error with the repository not being signed, add this:
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 9B98116C9AA302C7
```

## Pulling containers 
```bash
docker pull johntgz95/radxa-base
```

# Repositories
[Radxa-base](https://hub.docker.com/repository/docker/johntgz95/radxa-base/general)


# Resources
[Uploading an image to a Docker Hub repo](https://docs.docker.com/guides/workshop/04_sharing_app/).

[A Guide to Docker and ROS](https://roboticseabass.com/2021/04/21/docker-and-ros/)