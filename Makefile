build:
	sudo docker build -t ros-gobilda .

run: 
	docker run -it --rm --name ros-gobilda \
		--net=host \
		-v ~/Workspaces:/Workspaces/:Z \
		ros-gobilda
