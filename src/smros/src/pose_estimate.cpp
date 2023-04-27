//
// Created by tongdayang on 4/14/23.
//

#include "pose_estimate.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "point_cost_factor.h"
#include "utils.h"
#include <fstream>
#include <ros/ros.h>

const float PI = acos(-1);

NonlinearEstimate::NonlinearEstimate(int width, int height, float fov,
                                     float fov_up)
    : q_last_crt_(q_array_), t_last_crt_(t_array_),
      target_cloud_(new PointCloudT), source_cloud_(new PointCloudT),
      width_(width), height_(height), fov_(fov), fov_up_(fov_up) {}

void NonlinearEstimate::SetSourcePointCloud(PointCloudT::Ptr cloud) {
  //    FilterPointCloud(cloud, source_cloud_, resolution_);
  source_cloud_ = cloud;
}

void NonlinearEstimate::SetTargetPointCloud(PointCloudT::Ptr cloud) {
  //    FilterPointCloud(cloud, target_cloud_, resolution_);
  target_cloud_ = cloud;
  // compute target point cloud normal
  target_point_kdtree_.setInputCloud(target_cloud_);
  ComputeNormal(target_cloud_, target_normal_, target_planarity_);
  GenerateVertexMap();
}

bool NonlinearEstimate::Align() {
  for (int i = 0; i < max_iterations_; i++) {
    auto start_time = ros::Time::now();
    // match source and target
    Match();
    auto match_time = ros::Time::now();
    // reject pair
    Reject();
    auto reject_time = ros::Time::now();
    //        std::cout << "source: " << source_cloud_->points.size() << ",
    //        target: " << target_cloud_->points.size()
    //                  << std::endl;
    //        std::cout << "match: " << match_result_.size() << ", reject: " <<
    //        reject_result_.size() << std::endl;
    // estimate
    if (!Estimate()) {
      return false;
    }
    auto estimate_time = ros::Time::now();
    std::cout << "match time: " << (match_time - start_time).toSec() << "s"
              << std::endl;
    std::cout << "reject time: " << (reject_time - match_time).toSec() << "s"
              << std::endl;
    std::cout << "estimate time : " << (estimate_time - reject_time).toSec()
              << "s" << std::endl;
    if (converged_) {
      return true;
    }
  }
  return true;
}

void NonlinearEstimate::SetMaxIterations(int n) { max_iterations_ = n; }

void NonlinearEstimate::Match() {
  int point_number = source_cloud_->points.size();
  match_result_.clear();
  distance_.resize(target_cloud_->points.size(), 1000.0);
  auto &vertex_maps_ = vertex_maps_ptr_->GetMap();
  for (int i = 0; i < point_number; i++) {
    PointT t_point = TransformPoint(source_cloud_->points[i]);
    // compute u v index
    int u, v;
    float r;
    if (ComputeUVIndex(t_point, u, v, r)) {
      int target_index = vertex_maps_[u][v].index;
      if (target_index > 0) {
        match_result_[target_index] = i;
      }
    }
  }
}

bool NonlinearEstimate::Estimate() {
  int point_number = reject_result_.size();
  if (point_number < 5) {
    return false;
  }
  //    ceres::LossFunction *loss_function = nullptr;
  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::Problem problem;
  ceres::EigenQuaternionManifold *quaternionManifold =
      new ceres::EigenQuaternionManifold();
  problem.AddParameterBlock(q_array_, 4, quaternionManifold);
  problem.AddParameterBlock(t_array_, 3);
  problem.SetParameterLowerBound(t_array_, 0, -7);
  problem.SetParameterLowerBound(t_array_, 1, -7);
  problem.SetParameterLowerBound(t_array_, 2, -7);
  problem.SetParameterUpperBound(t_array_, 0, 7);
  problem.SetParameterUpperBound(t_array_, 1, 7);
  problem.SetParameterUpperBound(t_array_, 2, 7);

  for (auto iter : reject_result_) {
    int target_index = iter.first;
    int source_index = iter.second;
    Eigen::Vector3d source_point(source_cloud_->points[source_index].x,
                                 source_cloud_->points[source_index].y,
                                 source_cloud_->points[source_index].z);
    PointT target_point = target_cloud_->points[target_index];
    Eigen::Vector3d target_p(target_point.x, target_point.y, target_point.z);
    ceres::CostFunction *cost_function = LidarCostfunction::Create(
        source_point, target_p, target_normal_[target_index]);
    problem.AddResidualBlock(cost_function, loss_function, q_array_, t_array_);
  }
  ceres::Solver::Options option;
  option.linear_solver_type = ceres::DENSE_QR;
  option.minimizer_progress_to_stdout = false;
  option.max_num_iterations = 2;
  option.num_threads = 4;
  ceres::Solver::Summary summary;
  ceres::Solve(option, &problem, &summary);
  converged_ = summary.termination_type == ceres::TerminationType::CONVERGENCE;

  return true;
}

PointT NonlinearEstimate::TransformPoint(PointT in_point) {
  Eigen::Vector3d point(in_point.x, in_point.y, in_point.z);
  Eigen::Vector3d t_point = q_last_crt_ * point + t_last_crt_;
  PointT out_point;
  out_point.x = t_point.x();
  out_point.y = t_point.y();
  out_point.z = t_point.z();
  return out_point;
}

Eigen::Matrix4d NonlinearEstimate::GetFinalResult() {
  Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
  final_pose.block<3, 3>(0, 0) = q_last_crt_.matrix();
  final_pose.block<3, 1>(0, 3) =
      Eigen::Vector3d(t_last_crt_[0], t_last_crt_[1], t_last_crt_[2]);
  return final_pose;
}

void NonlinearEstimate::ComputeNormal(PointCloudT::Ptr cloud,
                                      std::vector<Eigen::Vector3d> &normal,
                                      std::vector<double> &planarity) {
  int point_number = cloud->points.size();
  normal.resize(point_number);
  planarity.resize(point_number);
  int K = 11;
  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);
  for (int i = 0; i < point_number; i++) {
    if (target_point_kdtree_.nearestKSearch(cloud->points[i], K,
                                            pointIdxKNNSearch,
                                            pointKNNSquaredDistance) > 0) {
      Eigen::MatrixXd neighbors(K - 1, 3);
      for (int j = 1; j < K; j++) {
        neighbors(j - 1, 0) = cloud->points[pointIdxKNNSearch[j]].x;
        neighbors(j - 1, 1) = cloud->points[pointIdxKNNSearch[j]].y;
        neighbors(j - 1, 2) = cloud->points[pointIdxKNNSearch[j]].z;
      }

      // Covariance matrix
      Eigen::MatrixXd centered =
          neighbors.rowwise() - neighbors.colwise().mean();
      Eigen::MatrixXd C =
          (centered.adjoint() * centered) / double(neighbors.rows() - 1);

      // Normal vector as eigenvector corresponding to smallest eigenvalue
      // Note that SelfAdjointEigenSolver is faster than EigenSolver for
      // symmetric matrices. Moreover, the eigenvalues are sorted in increasing
      // order.
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
  for (auto iter : match_result_) {
    if (target_cloud_->points[iter.first].label !=
        source_cloud_->points[iter.second].label) {
      continue;
    }
    if (target_planarity_[iter.first] >= min_planarity_) {
      reject_result_[iter.first] = iter.second;
    }
  }
}

void NonlinearEstimate::SetResolution(float r) { resolution_ = r; }

bool NonlinearEstimate::IsConverged() { return converged_; }

void NonlinearEstimate::SetMinPlanarity(double mp) { min_planarity_ = mp; }

void NonlinearEstimate::GenerateVertexMap() {
  vertex_maps_ptr_.reset(new VertexMap(width_, height_));
  auto &vertex_maps_ = vertex_maps_ptr_->GetMap();
  for (int i = 0; i < target_cloud_->points.size(); i++) {
    auto point = target_cloud_->points[i];
    vertex_maps_[point.u][point.v].index = i;
    vertex_maps_[point.u][point.v].point = point;
  }
}

bool NonlinearEstimate::ComputeUVIndex(PointT point, int &u, int &v,
                                       float &r_xyz) {
  float x, y, z;
  float yaw_angle, pitch_angle;
  x = point.x;
  y = point.y;
  z = point.z;
  // compute the distance to zero point
  r_xyz = sqrt(x * x + y * y + z * z);
  // compute the yaw angle (max:360)
  yaw_angle = atan2(y, x);
  // compute u coordination
  u = (int)(width_ * 0.5 * (yaw_angle / PI + 1));
  // compute the pitch angle (max:360)
  pitch_angle = asin(z / r_xyz) * 180.0 / PI;
  // compute v coordination
  v = (int)(height_ * ((abs(fov_up_) - pitch_angle) / fov_));
  // coordination can not out of range
  if (u > width_ - 1 || u < 0 || v > height_ - 1 || v < 0) {
    return false;
  }
  return true;
}

void NonlinearEstimate::SetQuaternion(double *q_init) {
  for (int i = 0; i < 4; i++) {
    q_array_[i] = q_init[i];
  }
}

void NonlinearEstimate::SetTranslation(double *t_init) {
  for (int i = 0; i < 3; i++) {
    t_array_[i] = t_init[i];
  }
}

const double *NonlinearEstimate::GetQuaternion() { return q_array_; }

const double *NonlinearEstimate::GetTranslation() { return t_array_; }
