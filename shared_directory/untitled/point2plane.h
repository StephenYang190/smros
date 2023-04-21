//
// Created by tongdayang on 1/17/23.
//

#ifndef UNTITLED_POINT2PLANE_H
#define UNTITLED_POINT2PLANE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <map>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>

using pointT = pcl::PointXYZRGB;

class Point2Plane {
public:
    Point2Plane();

    void setSourcePointCloud(pcl::PointCloud<pointT>::Ptr cloud);

    void setTargetPointCloud(pcl::PointCloud<pointT>::Ptr cloud);

    void setMaxIterations(int n);

    Eigen::Matrix4f align();

protected:
    void computeNormal(pcl::PointCloud<pointT>::Ptr cloud, std::vector<Eigen::Vector3f> &normal);

    void matchPointCloud();

    void estimateTransform(Eigen::Matrix4f &delta_pose, Eigen::VectorXf &residuals);

    bool checkConvergenceCriteria(std::vector<float> &residual_dists_mean,
                                  std::vector<float> &residual_dists_var);

    float computeChange(float &new_val, float &old_val);

    float computVar(Eigen::VectorXf &res);

    void reject();

    void computeDists();

    float findMedianValue(std::vector<float> &array);

    void nonlinearestimate();

private:
    pcl::PointCloud<pointT>::Ptr source_cloud_;
    pcl::PointCloud<pointT>::Ptr target_cloud_;
    pcl::PointCloud<pointT> transform_cloud_;
    std::map<int, int> match_result_;
    int max_iterations_{1};
    pcl::KdTreeFLANN<pointT> target_point_kdtree_;
    std::vector<Eigen::Vector3f> normal_;
    float min_change_{1.0};
    double q_array[4]{0, 0, 0, 1};
    double t_array[3]{0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_last_crt;
    Eigen::Map<Eigen::Vector3d> t_last_crt;
};


#endif//UNTITLED_POINT2PLANE_H
