#!/usr/bin/env bash

. "$(dirname "$0")"/variables.sh

xhost +local:*

if [ ! -f "$XAUTH" ]; then
	touch "$XAUTH"
	xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
	if [ -n "$xauth_list" ]; then
		echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
	else
		touch "$XAUTH"
	fi
	chmod a+r "$XAUTH"
fi

if [ "$(docker ps -a -q -f name="$CONTAINER_NAME")" ]; then
	echo "A container with name $CONTAINER_NAME is running, force removing it"
	docker rm -f "$CONTAINER_NAME"
	echo "Done"
fi

docker run \
	--name "$CONTAINER_NAME" \
	--hostname "$(hostname)" \
	--privileged \
	--platform=linux/amd64 \
	--gpus all \
	--runtime nvidia \
	--network host \
	--ipc host \
	--ulimit core=-1 \
	-e DISPLAY="$DISPLAY" \
	-e QT_X11_NO_MITSHM=1 \
	-e XAUTHORITY="$XAUTH" \
	-v /var/lib/systemd/coredump/:/cores \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v "$XAUTH":"$XAUTH" \
	-v "$(dirname "$0")"/../../:"$CONTAINER_HOME_FOLDER"/slam_project/ \
	-v "$DATASET_PATH":"$CONTAINER_HOME_FOLDER"/data \
	-w "$CONTAINER_HOME_FOLDER"/slam_project/ \
	--rm \
	-itd "$DOCKER_USER"/"$IMAGE_NAME":"$IMAGE_TAG"