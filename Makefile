
CONTAINER_NAME := 

build:
sudo docker build -t ros-gobilda .
docker run -it --name ros-gobilda \
	--net=host \
	-v ~/diff-drive-bot/:/diff-drive-bot/:Z \
	ros-gobilda
