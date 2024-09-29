#!/usr/bin/bash

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker_count=$(docker ps -a | grep saws | wc -l)
((docker_count=docker_count+1))

# Create a string with all --device options for each device in /dev/
device_options=""
for device in /dev/*; do
    if [ -e "$device" ]; then
        device_options+="--device=$device "
    fi
done

docker run -it --rm\
    --name saws-$docker_count \
    --volume="${PWD%/*}:/home/shared" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --env="XDG_RUNTIME_DIR=/tmp" \
    --env="XAUTHORITY=$XAUTH" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="HOME=/home/shared" \
    $device_options \
    --net=host \
    --privileged \
    saws:latest \
    bash

echo "Done."
