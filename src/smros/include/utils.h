//
// Created by tongdayang on 4/18/23.
//

#ifndef SMROS_UTILS_H
#define SMROS_UTILS_H

#include "semantic_surfel.h"

void FilterPointCloud(pcl::PointCloud<SemanticSurfel>::Ptr in,
                      pcl::PointCloud<SemanticSurfel>::Ptr out,
                      float resolution);

Eigen::Matrix4f ComputePoseWithNdt(pcl::PointCloud<SemanticSurfel>::Ptr input_points,
                                   pcl::PointCloud<SemanticSurfel>::Ptr target_points,
                                   Eigen::Matrix4f &initial_pose);

Eigen::Matrix4f ComputePoseWithNonlinear(pcl::PointCloud<SemanticSurfel>::Ptr input_points,
                                         pcl::PointCloud<SemanticSurfel>::Ptr target_points,
                                         Eigen::Matrix4f &initial_pose);

#endif //SMROS_UTILS_H
