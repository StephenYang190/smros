PROJECT_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P )"
cd $PROJECT_ROOT_DIR/catkin_ws
catkin init
catkin_make
echo "
source ${PROJECT_ROOT_DIR}/catkin_ws/devel/setup.bash
" >> "/home/${DOCKER_USER}/.bashrc"
