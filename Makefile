build:
	sudo docker build -t ros-gobilda .

run: 
	docker run -it --rm --name ros-gobilda \
		--net=host \
		-v ~/Workspaces/diff-drive-bot/ros2_ws:/ros2_ws/:Z \
		ros-gobilda
