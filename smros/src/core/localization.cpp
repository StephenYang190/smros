//
// Created by tongdayang on 1/6/23.
//

#include "core/localization.h"
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>

Localization::Localization()
    : odometry_result_(), scManager_(new SCManager()) {}

Localization::~Localization() {}

pose_type Localization::computePose(pcl::PointCloud<Surfel>::Ptr source,
                                    pcl::PointCloud<Surfel>::Ptr target,
                                    pose_type init_pose) {
  pclomp::NormalDistributionsTransform<Surfel, Surfel> ndt;
  // Setting point cloud to be aligned.
  ndt.setInputSource(source);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(target);

  // Calculating required rigid transform to align the input cloud to the target cloud.
  ndt.align(odometry_result_, init_pose);

  return ndt.getFinalTransformation();
}
bool Localization::loopdetection(pcl::PointCloud<Surfel>::Ptr crt_pointcloud) {

  scManager_->makeAndSaveScancontextAndKeys(*crt_pointcloud);
  auto pair_result = scManager_->detectLoopClosureID();
  int pre_index = pair_result.first;
  if (pre_index == -1 || pre_index > crt_id_) {
    if (!od_loopsure) {
      scManager_.popBack();
    }
    return false;
  }
  // get previous point cloud
  pcl::PointCloud<Surfel>::Ptr pre_point_clouds =
      map_->getPointCloudsInLocal(pre_index);
  // set initial_pose
  pose_type init_pose = pose_type::Identity();
  std::cout << "start compute loopsure pose" << std::endl;
  pose_type res_pose = odometry_->computePose(
      current_frame_->getPointCloudsPtr(), pre_point_clouds, init_pose);
  std::cout << "end compute loopsure pose" << std::endl;
  map_->setLoopsureEdge(crt_id_, pre_index, res_pose);
  std::cout << "end add loopsure edge" << std::endl;
  return true;
}
bool Localization::odometry(pcl::PointCloud<Surfel>::Ptr source,
                            pcl::PointCloud<Surfel>::Ptr target,
                            pose_type init_pose) {
  crt_pose_ = computePose(source, target, init_pose);
  // key frame selection
  // moving distance
  float distance = 0;
  for (int i = 0; i < 3; i++) {
    float dis = crt_pose_(i, 3);
    distance += dis * dis;
  }
  distance = sqrt(distance);
  // rotation angle
  Eigen::Matrix3f rotation = crt_pose_.block<3, 3>(0, 0);
  //    Eigen::Vector3f eulerangle = rotation.eulerAngles(0, 1, 2);
  //    for(int i = 0; i < 3; i++)
  //    {
  //        std::cout << "axis angle : " << eulerangle[i] / M_PI * 180 << std::endl;
  //    }
  //    float angle_change = eulerangle[2] + M_PI;
  //    angle_change = angle_change / M_PI * 180;
  //    if(angle_change > 260 || angle_change < 90)
  //    {
  //        return false;
  //    }
  //    Eigen::Quaternionf q(rotation);
  //    std::cout << "x angle : " << q.x() << std::endl;
  //    std::cout << "y angle : " << q.y() << std::endl;
  //    std::cout << "z angle : " << q.z() << std::endl;
  float angle_change = 1;

  std::cout << "distance between two frames is : " << distance;
  std::cout << ", and angle is : " << angle_change << std::endl;
  if (distance < max_distance_ && angle_change < max_angle_) {
    return false;
  }

  return true;
}
pose_type Localization::getLocalPose() {
  return crt_pose_;
}
bool Localization::haveloop() {
  return have_loop_;
}
std::pair<int, pose_type> Localization::getLoopResult() {
  return {last_timestamp_, loop_local_pose_};
}
void Localization::step(pcl::PointCloud<Surfel>::Ptr crt_pointcloud,
                        pcl::PointCloud<Surfel>::Ptr last_pointcloud,
                        pose_type init_pose) {
  if (last_pointcloud) {
    odometry(crt_pointcloud, last_pointcloud, init_pose);
  }
}
