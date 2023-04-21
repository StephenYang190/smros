#!/usr/bin/env bash
PROJECT_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P )"
# Common setting
source "$PROJECT_ROOT_DIR/scripts/docker_base.sh"
WORK_PATH="$WORKDIR/catkin_ws/src"
echo $WORK_PATH
# Cpu setting
MAX_CPU_FOR_DOCKER=$(( $(nproc) -1 ))
CPUSETS="0-$MAX_CPU_FOR_DOCKER"
# Nvidia setting
NVIDIA_OPTION=""
NVIDIA_DRIVER_CAPABILITIES=""
if command -v nvidia-container-runtime > /dev/null; then
      NVIDIA_OPTION="--runtime=nvidia"
      NVIDIA_DRIVER_CAPABILITIES="all"
else
      warning "nvidia-container-runtime not found! No Nvidia GPU Support!"
      info "If Nvidia GPU available, please install 'nvidia-container-runtime'"
      info "See $INSTALATION_DOC"
fi
# general setting
HOST_NAME=$(hostname)
USER_ID=$(id -u)
GRP=$(id -g -n)
GRP_ID=$(id -g)
DOCKER_HOME="/home/$USER"
# Local volumes
function local_volumes() {
    volumes="-v $PROJECT_ROOT_DIR/src:$WORK_PATH \
             -v $PROJECT_ROOT_DIR/docker:$WORKDIR/docker \
             -v $PROJECT_ROOT_DIR/scripts:$WORKDIR/scripts \
             -v $PROJECT_ROOT_DIR/shared_directory:$WORKDIR/shared_directory \
             -v $HOME/.ssh:$DOCKER_HOME/.ssh \
             -v /media:/media \
             -v /dev:/dev \
             -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
             -v /etc/localtime:/etc/localtime:ro \
             -v /usr/src:/usr/src \
             -v /lib/modules:/lib/modules \
             -v /sys/kernel/debug:/sys/kernel/debug \
             -v /dev/null:/dev/raw1394 \
             -v /etc/machine-id:/etc/machine-id "
    echo "$volumes"
}
# Display setting
display="$DISPLAY"
if [ -z "$DISPLAY" ];then
# The env variable DISPLAY is empty when logging in with SSH, so we must find the correct value
# on the remote host. otherwise,the cyber_visualizer will fail to open.
# look up remote host env DISPLAY
      HOST_DISPLAY=$(for file in /proc/[0-9]*; do grep -ao 'DISPLAY=[^[:cntrl:]]*' \
      "$file"/environ 2>/dev/null;done | head -1 | cut -d '=' -f2)
      display="$HOST_DISPLAY"
      [ -z "$HOST_DISPLAY" ] && display=":0"
fi
# Make git work
    if [ -f "$HOME/.gitconfig" ]; then
      GITCONFIG="-v $HOME/.gitconfig:/etc/.gitconfig"
    fi
# Clion setting
if [ "$CLION_ROOT_PATH" != "" ] && [ "$CLION_CONFIG_PATH" != "" ]; then
      echo "Using CLION"
      CLION_COMMAND=" \
      --mount type=bind,source=$CLION_ROOT_PATH,target=/$HOME/clion
      --mount type=bind,source=$CLION_CONFIG_PATH,target=$CLION_CONFIG_PATH
      --mount type=bind,source=/$HOME/.java,target=/$HOME/.java"
fi
# Stop container with same name
docker container stop "$MAIN_CONTAINER_NAME" > /dev/null 2>&1

docker run -itd \
        --rm \
        --cpuset-cpus=$CPUSETS \
        --name "$MAIN_CONTAINER_NAME" \
        --privileged \
        $NVIDIA_OPTION \
        -e NVIDIA_DRIVER_CAPABILITIES="$NVIDIA_DRIVER_CAPABILITIES" \
        -e DOCKER_USER="$USER" \
        -e USER="$USER" \
        -e DOCKER_USER_ID="$USER_ID" \
        -e DOCKER_GRP="$GRP" \
        -e DOCKER_GRP_ID="$GRP_ID" \
        -e LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib" \
        -e OPENBLAS_NUM_THREADS=1 \
        -e CC=/usr/bin/gcc \
        -e CXX=/usr/bin/g++ \
        -e DISPLAY="$display" \
        $(local_volumes) \
        $CLION_COMMAND \
        -w $WORKDIR \
        --ipc private \
        --net host \
        --hostname $HOST_NAME \
        --ulimit rtprio=99 \
        --cap-add=sys_nice \
        --shm-size 2G \
        --pid=host \
        $GITCONFIG \
        $MAIN_IMAGE \
        /bin/bash > /dev/null

if [ $? -ne 0 ]; then
      error "Failed to start docker container \"$MAIN_CONTAINER_NAME\"" \
            "based on image: $MAIN_IMAGE_FULL_TAG"
      exit 1
else
      info "$MAIN_CONTAINER_NAME:" \
            "$(docker ps -a | grep "$MAIN_CONTAINER_NAME" | head -c 12)"
fi

docker cp /etc/group "$MAIN_CONTAINER_NAME":/etc/group
docker exec "$MAIN_CONTAINER_NAME" \
        bash -c "$WORKDIR/scripts/docker_adduser.sh"

docker exec -u "$USER" "$MAIN_CONTAINER_NAME" \
      bash -c "ln -s /etc/.gitconfig /home/$USER/.gitconfig"
# Remove sudo warning
docker exec -u "$USER" "$MAIN_CONTAINER_NAME" \
bash -c 'touch ~/.sudo_as_admin_successful'
# Download jdk and zsh
if [ "$CLION_ROOT_PATH" != "" ] && [ "$CLION_CONFIG_PATH" != "" ]; then
      echo "Download jdk and zsh"
      docker exec -u "$USER" "$MAIN_CONTAINER_NAME" \
            bash -c "$WORKDIR/scripts/setup_environment.sh"
fi

ok "Now you can enter into container using: bash scripts/into_docker.sh"
