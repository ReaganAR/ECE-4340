if [ "$(docker ps -a | grep rosdocker)" ]
then
	docker exec -w /home/catkin_ws -it rosdocker bash --init-file ./docker/setup.bash
else

	xhost local:root

	XAUTH=/tmp/.docker.xauth

	docker run -it --rm \
	    -w /home/catkin_ws \
	    --name=rosdocker \
	    --env="DISPLAY=$DISPLAY" \
	    --env="QT_X11_NO_MITSHM=1" \
	    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	    --volume="$PWD":/home/catkin_ws \
	    --env="XAUTHORITY=$XAUTH" \
	    --volume="$XAUTH:$XAUTH" \
	    --net=host \
	    --privileged \
	    osrf/ros:kinetic-desktop-full \
	    bash --init-file /home/catkin_ws/devel/setup.bash

	echo "Done."
fi
