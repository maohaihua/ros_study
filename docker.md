sudo service docker start 

xhost +
sudo docker ps -l -q
sudo docker start 3b62
sudo docker exec -ti 3b62 /bin/bash
sudo docker exec -ti feeee /bin/bash


切换当前会话到新 group 或者重启 X 会话
# Add the docker group if it doesn't already exist.
	sudo groupadd docker

	# Add the connected user "${USER}" to the docker group.
	# Change the user name to match your preferred user.
	# You may have to logout and log back in again for
	# this to take effect.
	sudo gpasswd -a ${USER} docker

	# Restart the docker daemon.
	sudo service docker restart
    
newgrp - docker


sudo docker exec -ti 3b62 roslaunch husky_gazebo husky_empty_world.launch

/data/nucsim/docker_share/catkin_ws
export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

ROS_PACKAGE_PATH=~/catkin_ws/src/mbot_description:$ROS_PACKAGE_PATH

export ROS_PACKAGE_PATH=$HUSKY_GAZEBO_DESCRIPTION:$ROS_PACKAGE_PATH

sudo docker exec -ti feeee78bba8e /bin/bash

sudo docker run -it -v /home:/data \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:indigo-desktop-full  \

sudo docker run -it -v /home/nucsim/Downloads/kinetic:/catkin_ws/src \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:kinetic-desktop-full  \

sudo docker exec -ti 456b567ab7b4 /bin/bash


sudo docker run -it -v /home:/data \
    --env="DISPLAY" --network=host \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    mhh/indigo_desktop_full roslaunch husky_gazebo husky_empty_world.launch  \

docker run --network=host rosindustrial/fetch:indigo roslaunch fetch_gazebo_demo demo.launch

sudo docker run -it -v /home:/data \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    osrf/ros:indigo-desktop-full  \


sudo docker pull osrf/ros:kinetic-desktop-full
sudo docker pull osrf/ros:melodic-desktop-full

sudo docker commit -m='mhh kinetic desktop full image' --author='mhh' 456b567ab7b4 mhh/ros:kinetic-desktop-full

sudo docker run --network=host mhh/ros:kinetic-desktop-full roslaunch husky_gazebo husky_empty_world.launch

sudo docker run -it -v /home/nucsim/Downloads/kinetic:/catkin_ws/src \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    mhh/ros:kinetic-desktop-full roslaunch husky_gazebo husky_empty_world.launch  \

catkin_make -DCMAKE_INSTALL_PREFIX="/opt/ros/kinetic"

docker exec -ti <Container's ID> /bin/bash
sudo docker commit -m='A new image' --author='Aomine' 614122c0aabb aoct/apache2
