PROJECT_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P )"
cd $PROJECT_ROOT_DIR/docker
docker build -t smros:v3.0 .
