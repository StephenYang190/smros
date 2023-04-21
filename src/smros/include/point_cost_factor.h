//
// Created by tongdayang on 4/14/23.
//

#ifndef SMROS_POINT_COST_FACTOR_H
#define SMROS_POINT_COST_FACTOR_H
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

struct LidarCostfunction {
    LidarCostfunction(Eigen::Vector3d source_point,
                      Eigen::Vector3d target_point,
                      Eigen::Vector3d normal) :
            source_point_(source_point),
            target_point_(target_point),
            target_normal_(normal) {}

    template<typename T>
    bool operator()(const T *q, const T *t, T *residual) const {
        Eigen::Matrix<T, 3, 1> sp{T(source_point_.x()), T(source_point_.y()), T(source_point_.z())};
        Eigen::Matrix<T, 3, 1> tp{T(target_point_.x()), T(target_point_.y()), T(target_point_.z())};
        Eigen::Matrix<T, 3, 1> tn{T(target_normal_.x()), T(target_normal_.y()), T(target_normal_.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * sp + t_last_curr;

        Eigen::Matrix<T, 3, 1> diff;
        diff = tp - lp;
        residual[0] = diff.dot(tn);
        return true;
    }

    static ceres::CostFunction *Create(Eigen::Vector3d &source_point,
                                       Eigen::Vector3d &target_point,
                                       Eigen::Vector3d &normal) {
        return new ceres::AutoDiffCostFunction<LidarCostfunction, 1, 4, 3>(
                new LidarCostfunction(source_point, target_point, normal));
    }

    Eigen::Vector3d source_point_;
    Eigen::Vector3d target_point_;
    Eigen::Vector3d target_normal_;
};
#endif //SMROS_POINT_COST_FACTOR_H
