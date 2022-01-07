//
// Created by tongda on 2021/12/19.
//

#ifndef SRC_SURFELMAP_H
#define SRC_SURFELMAP_H

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <ctime>
#include <pcl/cloud_iterator.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/filter.hpp>

#include "rv/ParameterList.h"
#include "VertexMap.h"
#include "Map_2_Point.h"
#include "BackEndOpt.h"
#include <Scancontext/Scancontext.h>

using pose_type = Eigen::Matrix4f;

class SurfelMap {
private:
    std::vector<pose_type, Eigen::aligned_allocator<pose_type>> poses_;
    std::vector<std::shared_ptr<pcl::PointCloud<Surfel>>> surfels_;
    std::shared_ptr<VertexMap> active_map_;
    int width_, height_;
    float p_stable_, p_prior_, odds_p_prior_;
    float sigma_angle_2_, sigma_distance_2_;
    float initial_confidence_;
    float distance_thred_, angle_thred_;
    float max_distance_;
    float gamma_;
    float confidence_thred_;
    int time_gap_;
    pose_type init_pose_;
    std::vector<int> active_map_index_in_map_;
    rv::ParameterList param_;
    BackEndOpt pose_graph_;
    Eigen::DiagonalMatrix<double, 6> info_;
    SCManager scManager_;
    int loop_thred_, loop_times_;


public:
    SurfelMap(rv::ParameterList parameter_list);
    ~SurfelMap();
    bool pushBackPose(pose_type pose);
    pose_type getPose(int timestamp);
    bool mapInitial(std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud, pose_type init_pose = pose_type::Identity());
    float updateConfidence(float confidence, float angle_2, float distance_2);
    bool generateMap(pcl::PointCloud<Surfel> & global_map);
    const std::shared_ptr<VertexMap> getActiveMapPtr() {return active_map_;}
    float getInitConfidence() {return initial_confidence_;}
    bool generateActiveMap(int timestamp);
    bool updateMap(int timestamp, std::shared_ptr<VertexMap> current_frame);
    bool removeUnstableSurfel();
    std::pair<int, int> loopDectection();
    std::shared_ptr<pcl::PointCloud<Surfel>> getPointCloudsInLocal(int timestamp);
    std::shared_ptr<pcl::PointCloud<Surfel>> getPointCloudsInGlobal(int timestamp);
    bool setLoopPose(int from, int to, pose_type pose);

};


#endif //SRC_SURFELMAP_H
