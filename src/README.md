# Semantic Mapping based on ROS

This repository contains the completion of semantic mapping based on ros.

Developed by Tongda Yang.

![image](picture/example_gif.gif)

## Now statue:

- [x] complete pre-processing
- [x] complete map representation
- [x] complete  odometry(odt-omp)
- [x] complete map updating
- [x] complete loop sure detection(scan context)
- [x] complete factor graph optimization
- [x] key frame selection method
- [ ] map update method
- [ ] semantic information usage
- [ ] submap
- [x] problem occur in normal estimation and update
- [x] run in docker


## Following target:
- [ ] Optimise the performance of nonlinear pose estimation
- [ ] Replace ndt with nonlinear pose estimation in loop sure detection
- [ ] Merge semantic information in pose estimation
- [ ] More accurate normal computing and pose estimation
- [ ] Map store and updating method
- [ ] Separate localization and mapping modules
- [ ] Redefine point type with pcl::XYZRGB
## Docker
You can now run this project by docker. The following will conduct you to run this project in docker.

First, you need to clone this project and build docker image by Dockerfile.
```bash
git clone https://github.com/StephenYang190/smros.git
cd smros/docker
docker build -t smros:v2.0 .
```
After the docker image has been build successfully, you can start the docker container by script.
```bash
cd ..
bash docker/start_docker.sh
```
Now, you can go into docker by script and build this project
```bash
bash docker/into_docker.sh
# run this command in docker container
bash script/initiation.sh
roslaunch smros lasermap.launch
```
## System dependencies:

```
catkin
libEigen = 3.1
gtsam = 4.1.1 (Please make sure you use system eigen)
Tensorrt = 8.0
CUDA drive = 11.2
CUDA running time = 11.1
cudnn = 8.2.1.32
ubuntu = 20.04
```

On ubuntu 20.04, run following command to install dependencies:

```bash
sudo apt-get update 
sudo apt-get install -yqq  python3-dev python3-pip apt-utils git cmake libyaml-cpp-dev libopencv-dev python-empy build-essential libgtest-dev libeigen3-dev libboost-all-dev libglew-dev catkin
pip3 install catkin_tools trollius numpy catkin_tools_fetch empy
```

You need to clone official [gtsam](https://github.com/borglab/gtsam) code on github and compile it.

## How to use

First, you need to create a catkin workspace and clone the repository:

```bash
mkdir -p ~/semantic-mapping/catkin_wc/src
cd ~/semantic-mapping/catkin_wc/src
git clone --recurse-submodules git@gitlab.isus.tech:tongda.yang/semantic-mapping.git
```

### Build rangenet_lib

To use smros, you need to first build the rangenet_lib with the TensorRT and C++ interface. 
For more details about building and using rangenet_lib you could find in [rangenet_lib](https://github.com/StephenYang190/rangenet_lib/).

#### Build smros
For the first setup of your workspace containing this project, you need:
  ```bash
catkin_make
  ```

## Code structure

The full implementation of semantic mapping is in **SumaSLAM** Class.

We use **VertexMap** Class to transform **PointXYZI** to **Surfel** based point cloud type.

The input point cloud will pass to pre-processing to filter and transform to **Surfel** based point cloud type. The filtering includes several step:

1. compute coordination of each point on the vertex map
2. if the position do not have point, store the point
3. if the position have a point , we compare the distance from point to origin and save the nearest point
4. compute the normal of point
5. transform and set the properties of point on the vertex map

Then, we use **Rangenet** API to predict the label of each point.

We use **SurfelMap** class to store the representation of our map. we store the map frame by frame so that we can modify the pose of each frame easily and we can generate the active map(several frames before this timestamp) for odometry. We use **Eigen::Matrix4f** to represent our poses which are the pose to **world coordinate**.

We first render the active map as the target of ndt. Then we use voxel filter to filter the input point cloud. The leaf size is 0.1, 0.1, 0.1. We set the initial pose as last timestamp pose to compute the pose of this timestamp. 

Next, we create a new point cloud to store the points from this frame. We also generate the scan context and store in **SCManager** which is scan context class. For each new pose, we compute the pose transfer from last pose as: p<sub>m</sub>  =  p<sub>t - 1</sub><sup>-1</sup>  * p<sub>t</sub>. Then we add edge to factor graph using **addEdge()** method in **BackEndOpt** class which is the class of storing factor graph and computing optimising based on **gtsam**.

We find loopsure from pass frame. If successful, we compute the transfer from current frame to matching frame and add loopsure edge in factor graph. If the loopsure appears in the follow n frames, we optimise our all pose using **optimize(int num_iters)** method.

## How to run

**Important Notice**

- Before running smros, you need to first build the [rangenet_lib](https://github.com/StephenYang190/rangenet_lib/) and download the pretrained [model](https://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz).
- You need to specify the model path in the configuration file in the `config/` folder.
- For the first time using, rangenet_lib will take several minutes to build a `.trt` model for smros.

You have two ways to run it:

1. Online running. You can input the scan point cloud using **step(const pcl::PointCloud\<pcl::PointXYZI\> &)** method as **pcl::PointCloud\<pcl::PointXYZI\>**.
2. Offline running. You can input the path while stores the sequences of velodyne data using **readFromFile(std::string)** method.

You can get the map using **generateMap(pcl::PointCloud\<Surfel\>)** to get the map from begin to current frame in world coordinate.
