/******************************************************************************
 * Copyright 2023 AutoX. All Rights Reserved.
 *****************************************************************************/
//
// Created by tongdayang on 4/12/23.
//

#ifndef UNTITLED_NONLINER_H
#define UNTITLED_NONLINER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <map>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>

using pointT = pcl::PointXYZRGB;

class NonlinearEstimate {
public:
    NonlinearEstimate();

    void SetSourcePointCloud(pcl::PointCloud<pointT>::Ptr cloud);

    void SetTargetPointCloud(pcl::PointCloud<pointT>::Ptr cloud);

    void SetMaxIterations(int n);

    bool Align();

    Eigen::Matrix4f GetFinalResult();

    void SetResolution(float r);

    bool IsConverged();

    void SetMinPlanarity(double mp);

protected:

    void Match();

    bool Estimate();

    pointT TransformPoint(pointT in_point);

    void ComputeNormal(pcl::PointCloud<pointT>::Ptr cloud, std::vector<Eigen::Vector3d> &normal,
                       std::vector<double> &planarity);

    void Reject();

private:
    pcl::PointCloud<pointT>::Ptr source_cloud_;
    pcl::PointCloud<pointT>::Ptr target_cloud_;
    std::map<int, int> match_result_;
    std::map<int, int> reject_result_;
    int max_iterations_{1};
    pcl::KdTreeFLANN<pointT> target_point_kdtree_;
    std::vector<Eigen::Vector3d> target_normal_;
    std::vector<double> target_planarity_;
    double q_array_[4]{0, 0, 0, 1};
    double t_array_[3]{0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_last_crt_;
    Eigen::Map<Eigen::Vector3d> t_last_crt_;
    std::vector<double> distance_;
    float resolution_{0.01};
    bool converged_{false};
    double min_planarity_{0.3};
};


#endif //UNTITLED_NONLINER_H
