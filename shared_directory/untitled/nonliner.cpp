/******************************************************************************
 * Copyright 2023 AutoX. All Rights Reserved.
 *****************************************************************************/
//
// Created by tongdayang on 4/12/23.
//

#include "nonliner.h"
#include <pcl/common/transforms.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "lidar_cost_function.h"
#include <pcl/filters/voxel_grid.h>

NonlinearEstimate::NonlinearEstimate() :
        q_last_crt_(q_array_),
        t_last_crt_(t_array_),
        target_cloud_(new pcl::PointCloud<pointT>),
        source_cloud_(new pcl::PointCloud<pointT>) {
}

void NonlinearEstimate::SetSourcePointCloud(pcl::PointCloud<pointT>::Ptr cloud) {
    pcl::VoxelGrid<pointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(resolution_, resolution_, resolution_);
    voxelGrid.filter(*source_cloud_);
}

void NonlinearEstimate::SetTargetPointCloud(pcl::PointCloud<pointT>::Ptr cloud) {
    pcl::VoxelGrid<pointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(resolution_, resolution_, resolution_);
    voxelGrid.filter(*target_cloud_);
    // compute target point cloud normal
    target_point_kdtree_.setInputCloud(target_cloud_);
    ComputeNormal(target_cloud_, target_normal_, target_planarity_);
}

bool NonlinearEstimate::Align() {
    for (int i = 0; i < max_iterations_; i++) {
        // match source and target
        Match();
        // reject pair
        Reject();
        std::cout << "before reject point: " << match_result_.size() << std::endl;
        std::cout << "end reject point: " << reject_result_.size() << std::endl;
        // estimate
        if (!Estimate()) {
            return false;
        }
    }
    return true;
}

void NonlinearEstimate::SetMaxIterations(int n) {
    max_iterations_ = n;
}

void NonlinearEstimate::Match() {
    int point_number = source_cloud_->points.size();
    match_result_.clear();
    distance_.resize(target_cloud_->points.size(), 1000.0);
    int K = 1;
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    for (int i = 0; i < point_number; i++) {
        pointT t_point = TransformPoint(source_cloud_->points[i]);
        if (target_point_kdtree_.nearestKSearch(t_point, K, pointIdxKNNSearch,
                                                pointKNNSquaredDistance) > 0) {
            if (pointKNNSquaredDistance[0] > 1) {
                continue;
            }
            int target_index = pointIdxKNNSearch[0];
            auto iter = match_result_.find(target_index);
            if (iter != match_result_.end() && distance_[target_index] < pointKNNSquaredDistance[0]) {
                continue;
            }
            match_result_[target_index] = i;
            distance_[target_index] = pointKNNSquaredDistance[0];
        }
    }
}

bool NonlinearEstimate::Estimate() {
    int point_number = reject_result_.size();
    std::cout << "match point number: " << point_number << std::endl;
    if (point_number < 5) {
        return false;
    }
//    ceres::LossFunction *loss_function = nullptr;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem problem;
    ceres::EigenQuaternionManifold *quaternionManifold = new ceres::EigenQuaternionManifold();
    problem.AddParameterBlock(q_array_, 4, quaternionManifold);
    problem.AddParameterBlock(t_array_, 3);
    problem.SetParameterLowerBound(t_array_, 0, -7);
    problem.SetParameterLowerBound(t_array_, 1, -7);
    problem.SetParameterLowerBound(t_array_, 2, -7);
    problem.SetParameterUpperBound(t_array_, 0, 7);
    problem.SetParameterUpperBound(t_array_, 1, 7);
    problem.SetParameterUpperBound(t_array_, 2, 7);

    for (auto iter: reject_result_) {
        int target_index = iter.first;
        int source_index = iter.second;
        Eigen::Vector3d source_point(source_cloud_->points[source_index].x, source_cloud_->points[source_index].y,
                                     source_cloud_->points[source_index].z);
        pointT target_point = target_cloud_->points[target_index];
        Eigen::Vector3d target_p(target_point.x, target_point.y, target_point.z);
        ceres::CostFunction *cost_function = LidarCostfunction::Create(source_point, target_p,
                                                                       target_normal_[target_index]);
        problem.AddResidualBlock(cost_function,
                                 loss_function,
                                 q_array_,
                                 t_array_);
    }
    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_QR;
    option.minimizer_progress_to_stdout = true;
    option.max_num_iterations = 2;
    option.num_threads = 4;
    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    std::cout << "-----------optional after---------------" << std::endl;
    converged_ = summary.termination_type == ceres::TerminationType::CONVERGENCE;

    return true;
}

pointT NonlinearEstimate::TransformPoint(pointT in_point) {
    Eigen::Vector3d point(in_point.x, in_point.y, in_point.z);
    Eigen::Vector3d t_point = q_last_crt_ * point + t_last_crt_;
    pointT out_point;
    out_point.x = t_point.x();
    out_point.y = t_point.y();
    out_point.z = t_point.z();
    return out_point;
}

Eigen::Matrix4f NonlinearEstimate::GetFinalResult() {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = q_last_crt_.matrix();
    Eigen::Matrix4f final_pose = Eigen::Matrix4f::Identity();
    final_pose.block<3, 3>(0, 0) = rotation_matrix.cast<float>();
    final_pose.block<3, 1>(0, 3) = Eigen::Vector3f(t_last_crt_[0], t_last_crt_[1], t_last_crt_[2]);
    return final_pose;
}

void NonlinearEstimate::ComputeNormal(pcl::PointCloud<pointT>::Ptr cloud, std::vector<Eigen::Vector3d> &normal,
                                      std::vector<double> &planarity) {
    int point_number = cloud->points.size();
    normal.resize(point_number);
    planarity.resize(point_number);
    int K = 11;
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    for (int i = 0; i < point_number; i++) {
        if (target_point_kdtree_.nearestKSearch(cloud->points[i], K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
            Eigen::MatrixXd neighbors(K - 1, 3);
            for (int j = 1; j < K; j++) {
                neighbors(j - 1, 0) = cloud->points[pointIdxKNNSearch[j]].x;
                neighbors(j - 1, 1) = cloud->points[pointIdxKNNSearch[j]].y;
                neighbors(j - 1, 2) = cloud->points[pointIdxKNNSearch[j]].z;
            }

            // Covariance matrix
            Eigen::MatrixXd centered = neighbors.rowwise() - neighbors.colwise().mean();
            Eigen::MatrixXd C = (centered.adjoint() * centered) / double(neighbors.rows() - 1);

            // Normal vector as eigenvector corresponding to smallest eigenvalue
            // Note that SelfAdjointEigenSolver is faster than EigenSolver for symmetric matrices. Moreover,
            // the eigenvalues are sorted in increasing order.
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(C);
            auto eigenvectors{es.eigenvectors()};
            auto eigenvalues{es.eigenvalues()};

            normal[i](0) = eigenvectors(0, 0);
            normal[i](1) = eigenvectors(1, 0);
            normal[i](2) = eigenvectors(2, 0);
            normal[i].normalize();
            planarity[i] = (eigenvalues[1] - eigenvalues[0]) / eigenvalues[2];
        }
    }
}

void NonlinearEstimate::Reject() {
    reject_result_.clear();
    for (auto iter: match_result_) {
        if (target_planarity_[iter.first] >= min_planarity_) {
            reject_result_[iter.first] = iter.second;
        }
    }
}

void NonlinearEstimate::SetResolution(float r) {
    resolution_ = r;
}

bool NonlinearEstimate::IsConverged() {
    return converged_;
}

void NonlinearEstimate::SetMinPlanarity(double mp) {
    min_planarity_ = mp;
}
