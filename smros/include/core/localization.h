//
// Created by tongdayang on 1/6/23.
//

#ifndef SRC_LOCALIZATION_H
#define SRC_LOCALIZATION_H

#include "surfel.h"
#include "surfelmap.h"

#include <Scancontext/Scancontext.h>

class Localization {
 public:
  Localization();

  ~Localization();

  void step(pcl::PointCloud<Surfel>::Ptr crt_pointcloud,
            pcl::PointCloud<Surfel>::Ptr last_pointcloud, pose_type init_pose);

  pose_type getLocalPose();
  bool haveloop();
  std::pair<int, pose_type> getLoopResult();

 protected:
  bool odometry(pcl::PointCloud<Surfel>::Ptr source,
                pcl::PointCloud<Surfel>::Ptr target, pose_type init_pose);
  bool loopdetection(pcl::PointCloud<Surfel>::Ptr crt_pointcloud);
  pose_type computePose(pcl::PointCloud<Surfel>::Ptr source,
                        pcl::PointCloud<Surfel>::Ptr target,
                        pose_type init_pose);

 private:
  pcl::PointCloud<Surfel> odometry_result_;
  // scan context manager
  std::shared_ptr<SCManager> scManager_;
  // current pose
  pose_type crt_pose_;
  // loop detection result
  bool have_loop_{false};
  int last_timestamp_;
  pose_type loop_local_pose_;
  // distance threshold
  float max_distance_;
  // angle thresholdmap
  float max_angle_;
};

#endif  //SRC_LOCALIZATION_H
