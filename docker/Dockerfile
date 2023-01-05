FROM nvcr.io/nvidia/tensorrt:22.04-py3

COPY install_gtsam.sh /tmp/

RUN DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends ca-certificates && \
    sed -i "s@http://.*archive.ubuntu.com@https://mirrors.bfsu.edu.cn@g" /etc/apt/sources.list && \
    sed -i "s@http://.*security.ubuntu.com@https://mirrors.bfsu.edu.cn@g" /etc/apt/sources.list && \
    DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
                lsb-release \
                && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
                ros-noetic-desktop-full \
                cmake \
                git \
                build-essential \
                sudo \
                apt-utils \
                libboost-all-dev \
                libyaml-cpp-dev \
                libopencv-dev \
                libgtest-dev \
                libeigen3-dev \
                libboost-all-dev \
                ca-certificates \
                && \
    DEBIAN_FRONTEND=noninteractive pip3 install \
    		catkin_tools \
    		trollius \
    		numpy \
    		catkin_tools_fetch \
    		empy \
    		&& \
    bash /tmp/install_gtsam.sh && \
    rm -rf /var/lib/apt/lists/*
