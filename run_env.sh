docker run -it --name ros-gobilda \
	--net=host \
	-v ~/diff-drive-bot/:/diff-drive-bot/:Z \
	ros-gobilda
