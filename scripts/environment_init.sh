cd /smros_ws/catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "
source /smros_ws/catkin_ws/devel/setup.bash
" >> "/home/${DOCKER_USER}/.bashrc"

source ~/.bashrc
