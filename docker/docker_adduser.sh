#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

PROJECT_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P )"
source "$PROJECT_ROOT_DIR/docker/docker_base.sh"

getent group | grep "${DOCKER_USER}:x" > /dev/null 2>&1 || addgroup  --quiet --gid "$DOCKER_GRP_ID" "$DOCKER_GRP"
adduser  --quiet --disabled-password --force-badname --gecos '' "$DOCKER_USER" \
    --uid "$DOCKER_USER_ID" --gid "$DOCKER_GRP_ID" 2>/dev/null
usermod -aG sudo "$DOCKER_USER"
usermod -aG input "$DOCKER_USER"
echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
cp -r /etc/skel/. "/home/${DOCKER_USER}"

echo "
genhtml_branch_coverage = 1
lcov_branch_coverage = 1
" >> "/home/${DOCKER_USER}/.lcovrc"
echo "
source /opt/ros/noetic/setup.bash
" >> "/home/${DOCKER_USER}/.bashrc"

# Set user files ownership to current user, such as .bashrc, .profile, etc.
chown "${DOCKER_USER}:${DOCKER_GRP}" "/home/${DOCKER_USER}"
chown -R "${DOCKER_USER}:${DOCKER_GRP}" "$WORKDIR"
find "/home/${DOCKER_USER}" -maxdepth 1 -name '.??*' -exec chown -R "${DOCKER_USER}:${DOCKER_GRP}" {} +
