//
// Created by tongdayang on 4/18/23.
//
#include "utils.h"
#include "pose_estimate.h"
#include <ros/ros.h>

#define PCL_NO_PRECOMPILE

#include "pclomp/ndt_omp.h"
#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"
#include <pcl/filters/approximate_voxel_grid.h>

void FilterPointCloud(pcl::PointCloud<SemanticSurfel>::Ptr in,
                      pcl::PointCloud<SemanticSurfel>::Ptr out,
                      float resolution) {
  pcl::ApproximateVoxelGrid<SemanticSurfel> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(resolution, resolution, resolution);
  approximate_voxel_filter.setInputCloud(in);
  approximate_voxel_filter.filter(*out);
}

Eigen::Matrix4d
ComputePoseWithNdt(pcl::PointCloud<SemanticSurfel>::Ptr input_points,
                   pcl::PointCloud<SemanticSurfel>::Ptr target_points,
                   Eigen::Matrix4d &initial_pose) {
  pcl::PointCloud<SemanticSurfel> odometry_result;
  pclomp::NormalDistributionsTransform<SemanticSurfel, SemanticSurfel> ndt;
  ndt.setNumThreads(4);
  // Setting point cloud to be aligned.
  ndt.setInputSource(input_points);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(target_points);

  // Calculating required rigid transform to align the input cloud to the target
  // cloud.
  ndt.align(odometry_result, initial_pose.cast<float>());

  return ndt.getFinalTransformation().cast<double>();
}

Eigen::Matrix4d
ComputePoseWithNonlinear(pcl::PointCloud<SemanticSurfel>::Ptr input_points,
                         pcl::PointCloud<SemanticSurfel>::Ptr target_points,
                         Eigen::Matrix4d &initial_pose) {
  auto start_time = ros::Time::now();
  NonlinearEstimate odometry(900, 64, 27.0, 2.0);
  odometry.SetMaxIterations(10);
  odometry.SetResolution(0.05);
  odometry.SetSourcePointCloud(input_points);
  odometry.SetTargetPointCloud(target_points);
  odometry.Align();
  auto end_time = ros::Time::now();
  std::cout << "odometry use " << (end_time - start_time).toSec() << "s"
            << std::endl;
  return odometry.GetFinalResult();
}