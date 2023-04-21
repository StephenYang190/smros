#!/usr/bin/env bash

# Common setting
PROJECT_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P )"
source "$PROJECT_ROOT_DIR/scripts/docker_base.sh"

xhost +local:root 1>/dev/null 2>&1

case $1 in
  clion)
PROGRAM="/home/$USER/clion/bin/clion.sh"
ARG1=""
  ;;  
  *)
PROGRAM="/bin/bash"
ARG1=""
  ;;
esac

info "use dev name [\033[32m$MAIN_IMAGE\033[0m]"

docker exec \
  -u "$USER" \
  -w "$WORKDIR" \
  -e HISTFILE=/$WORKDIR/.dev_bash_hist \
  -it "$MAIN_CONTAINER_NAME" \
  ${PROGRAM} ${ARG1}
