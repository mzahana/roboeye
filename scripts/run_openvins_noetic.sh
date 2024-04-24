#!/bin/bash
#
# Copyright (c) 2023, Mohamed Abdelkader.  All rights reserved.

USERNAME=vio

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/../utils/print_color.sh

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi

PLATFORM="$(uname -m)"

BASE_NAME="mzahana/openvins_rpi:noetic"
CONTAINER_NAME="ros_noetic"


CMD="/bin/bash"

if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=${CONTAINER_NAME})" ]; then
        # cleanup
        echo "Restarting the container..."
        docker start ${CONTAINER_NAME}
    fi
    docker exec -it --workdir /home/${USERNAME}/shared_volume ${CONTAINER_NAME} env TERM=xterm-256color bash -c "${CMD}"
    return 0
fi



# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/${USERNAME}/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-v /dev/*:/dev/*")
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro")


# Custom command to run after logging into the container
CMD="/bin/bash"

HOST_DEV_DIR=$HOME/${CONTAINER_NAME}_shared_volume
if [ ! -d "$HOST_DEV_DIR" ]; then
    mkdir -p $HOST_DEV_DIR
fi

# Run container from image
print_info "Running $CONTAINER_NAME"
docker run -it \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v $HOST_DEV_DIR:/home/${USERNAME}/shared_volume \
     \
    --name "$CONTAINER_NAME" \
    --user="${USERNAME}" \
    --workdir /home/${USERNAME}/shared_volume \
    $@ \
    $BASE_NAME \
    bash -c "${CMD}"