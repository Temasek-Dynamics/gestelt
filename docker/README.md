# Docker

# Official ROS docker images
They can be found on the [docker hub registry](https://registry.hub.docker.com/_/ros/). We are currently using `noetic-ros-core-focal`.

# Installing docker (Host)
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 9B98116C9AA302C7
sudo apt-get update
sudo apt-get install curl

curl -fsSL test.docker.com -o get-docker.sh && sh get-docker.sh

# non-root user setup
sudo usermod -aG docker $USER 
sudo reboot now

# Test the installation
docker run hello-world 

# [for host only] To enable cross compilation for arm64 architecture
docker run --privileged --rm tonistiigi/binfmt --install all
```


# Host

## Building and Pushing images to Docker Hub repository
```bash
# Build image
docker build --platform linux/arm64 -t gestelt/learning-agile:latest .
# Push image to docker hub repository
docker push gestelt/learning-agile:latest

# All-in-one build and push
docker build --platform linux/arm64 -t gestelt/learning-agile:latest --push .

# [Optional] Remove dockerfile build cache
docker buildx prune
```

## Running containers
```bash
# To use host USB devices, add "--privileged" flag or "--device=/dev/ttyAML1"
docker run -it --rm --network host --privileged -e "MASTER_IP=$MASTER_IP" -e "SELF_IP=$SELF_IP" gestelt/learning-agile:latest

# Find name of new machine 
docker ps -l

# List all docker containers
docker ps -a
# Start stopped container
docker start <container_id>
# Start additional bash sessions in same container
docker exec -it <container_id> bash
# Stop all containersa
docker stop $(docker ps -a -q)
# Remove all containers
docker rm $(docker ps -a -q)
```

## Make changes to containers and save them as new images
```bash
# List all docker containers
docker ps -a
# Commit changes
docker commit CONTAINER_NAME gestelt/learning-agile:latest
# push 
docker push gestelt/learning-agile:latest
# Inspect the container
docker container inspect CONTAINER_NAME
```

# Radxa

## Commands
```bash
# Pull Images
docker pull gestelt/learning-agile:latest
docker run -it --rm --network host --privileged -e "MASTER_IP=$MASTER_IP" -e "SELF_IP=$SELF_IP" gestelt/learning-agile:latest
```

# Repositories
[Radxa-base](https://hub.docker.com/repository/docker/gestelt/radxa-base/general)

# Resources
[Uploading an image to a Docker Hub repo](https://docs.docker.com/guides/workshop/04_sharing_app/).

[A Guide to Docker and ROS](https://roboticseabass.com/2021/04/21/docker-and-ros/)