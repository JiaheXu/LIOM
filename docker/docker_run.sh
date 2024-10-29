#!/bin/bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
	touch $XAUTH
	xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
	if [ -n "$xauth_list" ]; then
		echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
	else
		touch "$XAUTH"
	fi
	chmod a+r "$XAUTH"
fi

DATASET_PATH=/media/mmpug/T77

docker run --name  multi_robot_slam:latest  -itd \
	--privileged \
	--gpus all \
	--ulimit core=-1 \
	-v /var/lib/systemd/coredump/:/cores \
	-v "$(readlink -f ../../../dcl_slam_ws)":/multi_robot_slam \
	-v "$DATASET_PATH":/media/shibo/T7 \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-e XAUTHORITY="$XAUTH" \
	-v "$XAUTH":"$XAUTH" \
	--runtime=nvidia \
	--name  multi_robot_slam  \
	--rm \
	multi_robot_slam:latest \
	bash	     