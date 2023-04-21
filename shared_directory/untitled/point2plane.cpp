//
// Created by tongdayang on 1/17/23.
//

#include "point2plane.h"
#include <pcl/common/transforms.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "lidar_cost_function.h"


void Point2Plane::setSourcePointCloud(pcl::PointCloud<pointT>::Ptr cloud) {
    source_cloud_ = cloud;
}

void Point2Plane::setTargetPointCloud(pcl::PointCloud<pointT>::Ptr cloud) {
    target_cloud_ = cloud;
    // compute target point cloud normal
    target_point_kdtree_.setInputCloud(target_cloud_);
    computeNormal(target_cloud_, normal_);
}

Eigen::Matrix4f Point2Plane::align() {
//    Eigen::Matrix4f pose_new = Eigen::Matrix4f::Identity();
//    Eigen::Matrix4f delta_pose = Eigen::Matrix4f::Identity();
//    Eigen::VectorXf residual_dists;
//    std::vector<float> residual_dists_mean;
//    std::vector<float> residual_dists_var;

    for (int i = 0; i < max_iterations_; i++) {
        // compute last pose
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = q_last_crt.matrix();
        Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
        last_pose.block<3, 3>(0, 0) = rotation_matrix.cast<float>();
        last_pose.block<3, 1>(0, 3) = Eigen::Vector3f(t_array[0], t_array[1], t_array[2]);
        // transform point
        pcl::transformPointCloud(*source_cloud_, transform_cloud_, last_pose);
        // match source and target
        matchPointCloud();
//        // TODO: reject point
//        // estimate
//        estimateTransform(delta_pose, residual_dists);
//        // update pose
//        pose_new = delta_pose * pose_old;
//        pose_old = pose_new;
//        // transform point
//        pcl::transformPointCloud(transform_cloud_, transform_cloud_, delta_pose);
//        // save mean and var
//        residual_dists_mean.push_back(residual_dists.mean());
//        residual_dists_var.push_back(computVar(residual_dists));
//        if (checkConvergenceCriteria(residual_dists_mean, residual_dists_var)) {
//            break;
//        }
        // estimate
        nonlinearestimate();
    }
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = q_last_crt.matrix();
    Eigen::Matrix4f final_pose = Eigen::Matrix4f::Identity();
    final_pose.block<3, 3>(0, 0) = rotation_matrix.cast<float>();
    final_pose.block<3, 1>(0, 3) = Eigen::Vector3f(t_last_crt[0], t_last_crt[1], t_last_crt[2]);
    return final_pose;
}

void Point2Plane::setMaxIterations(int n) {
    max_iterations_ = n;
}

void Point2Plane::computeNormal(pcl::PointCloud<pointT>::Ptr cloud, std::vector<Eigen::Vector3f> &normal) {
    int point_number = cloud->points.size();
    normal.resize(point_number);
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

            normal[i](0) = eigenvectors(0, 0);
            normal[i](1) = eigenvectors(1, 0);
            normal[i](2) = eigenvectors(2, 0);
        }
    }
}

void Point2Plane::matchPointCloud() {
    int point_number = transform_cloud_.points.size();
    match_result_.clear();
    int K = 1;
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    for (int i = 0; i < point_number; i++) {
        if (target_point_kdtree_.nearestKSearch(transform_cloud_.points[i], K, pointIdxKNNSearch,
                                                pointKNNSquaredDistance) > 0) {
            if (pointKNNSquaredDistance[0] > 1) {
                continue;
            }
            match_result_[i] = pointIdxKNNSearch[0];
        }
    }
}

void Point2Plane::estimateTransform(Eigen::Matrix4f &delta_pose, Eigen::VectorXf &residuals) {
    int point_number = match_result_.size();
    Eigen::MatrixXf A_matrix(point_number, 6);
    Eigen::VectorXf b_vector(point_number);
    for (int i = 0; i < point_number; i++) {
        int target_index = match_result_[i];
        float source_x = transform_cloud_.points[i].x;
        float source_y = transform_cloud_.points[i].y;
        float source_z = transform_cloud_.points[i].z;
        Eigen::Vector3f target_normal = normal_[target_index];
        pointT target_point = target_cloud_->points[target_index];

        A_matrix(i, 0) = source_y * target_normal.z() - source_z * target_normal.y();
        A_matrix(i, 1) = source_z * target_normal.x() - source_x * target_normal.z();
        A_matrix(i, 2) = source_x * target_normal.y() - source_y * target_normal.x();
        A_matrix.block<1, 3>(i, 3) = target_normal;

        Eigen::Vector3f point_dis(target_point.x - source_x,
                                  target_point.y - source_y,
                                  target_point.z - source_z);
        b_vector(i) = target_normal.transpose() * point_dis;
    }
    Eigen::Matrix<float, 6, 1> pose_result{A_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_vector)};
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisd(pose_result(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisd(pose_result(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisd(pose_result(2), Eigen::Vector3d::UnitZ()));

    Eigen::Matrix3f rotation_matrix(yawAngle * pitchAngle * rollAngle);
    delta_pose.block<3, 3>(0, 0) = rotation_matrix;
    delta_pose.block<3, 1>(0, 3) = pose_result.block<3, 1>(3, 0);

    residuals = A_matrix * pose_result - b_vector;
}


float Point2Plane::computeChange(float &new_val, float &old_val) {
    return abs((new_val - old_val) / old_val * 100);
}

bool
Point2Plane::checkConvergenceCriteria(std::vector<float> &residual_dists_mean, std::vector<float> &residual_dists_var) {
    if (computeChange(residual_dists_mean.end()[-1], residual_dists_mean.end()[-2]) < min_change_) {
        if (computeChange(residual_dists_var.end()[-1], residual_dists_var.end()[-2]) < min_change_) {
            return true;
        }
    }
    return false;
}

float Point2Plane::computVar(Eigen::VectorXf &res) {
    return sqrt((res.array() - res.mean()).square().sum() / (res.size() - 1));
}

void Point2Plane::reject() {
}

void Point2Plane::computeDists() {

}

Point2Plane::Point2Plane() :
        q_last_crt(q_array),
        t_last_crt(t_array) {
}

float Point2Plane::findMedianValue(std::vector<float> &array) {
    std::sort(array.begin(), array.end());
    if (array.size() % 2 == 0) {
        return (array[int(array.size() / 2)] + array[int(array.size() / 2) - 1]) / 2;
    }
    return array[int(array.size() / 2)];
}

void Point2Plane::nonlinearestimate() {
    int point_number = match_result_.size();
//    ceres::LossFunction *loss_function = nullptr;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem problem;
    problem.AddParameterBlock(q_array, 4);
    problem.AddParameterBlock(t_array, 3);

    for (int i = 0; i < point_number; i++) {
        int target_index = match_result_[i];
        Eigen::Vector3d source_point(source_cloud_->points[i].x, source_cloud_->points[i].y,
                                     source_cloud_->points[i].z);
        Eigen::Vector3d target_normal = normal_[target_index].cast<double>();
        pointT target_point = target_cloud_->points[target_index];
        Eigen::Vector3d target_p(target_point.x, target_point.y, target_point.z);
        ceres::CostFunction *cost_function = LidarCostfunction::Create(source_point, target_p, target_normal);
        problem.AddResidualBlock(cost_function,
                                 loss_function,
                                 q_array,
                                 t_array);
    }
    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_SCHUR;
    option.minimizer_progress_to_stdout = false;
    option.max_num_iterations = 4;
    option.num_threads = 4;
    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);
    std::cout << "match point number: " << point_number << std::endl;
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "-----------optional after---------------" << std::endl;
//    Eigen::Quaterniond q(angle);
//    Eigen::Matrix3d rotation_matrix;
//    rotation_matrix = q.matrix();
//    Eigen::Matrix4f final_pose = Eigen::Matrix4f::Identity();
//    final_pose.block<3, 3>(0, 0) = rotation_matrix.cast<float>();
//    final_pose.block<3, 1>(0, 3) = Eigen::Vector3f(translation[0], translation[1], translation[2]);
//    return final_pose;
}
