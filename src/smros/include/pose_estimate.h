//
// Created by tongdayang on 4/14/23.
//

#ifndef SRC_POSE_ESTIMATE_H
#define SRC_POSE_ESTIMATE_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <map>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>
#include "semantic_surfel.h"

using pointT = SemanticSurfel;

struct Vertex {
    float radius = 0.0;
    int index = -1;
    SemanticSurfel point;
};

class VertexMap {
public:
    VertexMap(int width, int height) : width_(width), height_(height) {
        // init Vertex map
        vertex_maps_.resize(width_);
        for (int i = 0; i < width_; i++) {
            Vertex vertex;
            vertex_maps_[i].resize(height_, vertex);
        }
    }

    std::vector<std::vector<Vertex>> &GetMap() {
        return vertex_maps_;
    }

    std::vector<std::vector<Vertex>> vertex_maps_;
    int height_, width_;
};

class NonlinearEstimate {
public:
    NonlinearEstimate();

    void SetSourcePointCloud(pcl::PointCloud<pointT>::Ptr cloud);

    void SetTargetPointCloud(pcl::PointCloud<pointT>::Ptr cloud);

    void SetMaxIterations(int n);

    bool Align();

    Eigen::Matrix4d GetFinalResult();

    void SetResolution(float r);

    bool IsConverged();

    void SetMinPlanarity(double mp);

    void SetQuaternion(double *q_init);

    void SetTranslation(double *t_init);

    const double *GetQuaternion();

    const double *GetTranslation();

protected:

    void Match();

    bool Estimate();

    pointT TransformPoint(pointT in_point);

    void ComputeNormal(pcl::PointCloud<pointT>::Ptr cloud, std::vector<Eigen::Vector3d> &normal,
                       std::vector<double> &planarity);

    void Reject();

    void GenerateVertexMap();

    bool ComputeUVIndex(pointT point, int &u, int &v, float &r_xyz);

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

    // Vertex map, which store the index of point in point clouds
    std::shared_ptr<VertexMap> vertex_maps_ptr_;
    int width_{900};
    int height_{64};
    float fov_{27.0};
    float fov_up_{2.0};
};


#endif //SRC_POSE_ESTIMATE_H
