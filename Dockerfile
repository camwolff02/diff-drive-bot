FROM ros:humble-ros-base-jammy
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-humble-perception=0.10.0-1* \
	ros-humble-foxglove-bridge \
	&& rm -rf /var/lib/apt/lists/*

# set domain id
RUN export ROS_DOMAIN_ID=74 && echo "export ROS_DOMAIN_ID=74" >> ~/.bashrc
