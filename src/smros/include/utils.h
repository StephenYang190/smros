//
// Created by tongdayang on 4/18/23.
//

#ifndef SMROS_UTILS_H
#define SMROS_UTILS_H

#include "surfel.h"

void FilterPointCloud(pcl::PointCloud<Surfel>::Ptr in,
                      pcl::PointCloud<Surfel>::Ptr out,
                      float resolution);

Eigen::Matrix4f ComputePoseWithNdt(pcl::PointCloud<Surfel>::Ptr input_points,
                                   pcl::PointCloud<Surfel>::Ptr target_points,
                                   Eigen::Matrix4f &initial_pose);

Eigen::Matrix4f ComputePoseWithNonlinear(pcl::PointCloud<Surfel>::Ptr input_points,
                                         pcl::PointCloud<Surfel>::Ptr target_points,
                                         Eigen::Matrix4f &initial_pose);

#endif //SMROS_UTILS_H
