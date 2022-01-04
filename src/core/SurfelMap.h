//
// Created by tongda on 2021/12/19.
//

#ifndef SRC_SURFELMAP_H
#define SRC_SURFELMAP_H

#include <Eigen/Dense>
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
#include "Point_2_Map.h"
#include "Map_2_Point.h"

class SurfelMap {
private:
    std::vector<Eigen::Matrix4f> poses_;
    std::vector<std::shared_ptr<pcl::PointCloud<Surfel>>> surfels_;
    std::shared_ptr<Point_2_Map> active_map_;
    int width_, height_;
    float p_stable_, p_prior_, odds_p_prior_;
    float sigma_angle_2_, sigma_distance_2_;
    float initial_confidence_;
    float distance_thred_, angle_thred_;
    float max_distance_;
    float gamma_;
    float confidence_thred_;
    int time_gap_;
    Eigen::Matrix4f init_pose_;
    std::vector<int> active_map_index_in_map_;
    rv::ParameterList param_;

public:
    SurfelMap(rv::ParameterList parameter_list);
    ~SurfelMap();
    bool pushBackPose(Eigen::Matrix4f pose);
    Eigen::Matrix4f getPose(int timestamp);
    bool updateMap(const Point_2_Map & current_frame, int timestamp);
    bool updateMap(const pcl::PointCloud<Surfel> & align_point,
                   const pcl::CorrespondencesPtr mapping, int timestamp, std::shared_ptr<Point_2_Map> current_frame);
    bool mapInitial(std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud, Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity());
    float updateConfidence(float confidence, float angle_2, float distance_2);
    bool generateMap(pcl::PointCloud<Surfel> & global_map);
    const std::shared_ptr<Point_2_Map> getActiveMapPtr() {return active_map_;}
    float getInitConfidence() {return initial_confidence_;}
    bool generateActiveMap(int timestamp);
    bool updateMap(int timestamp, std::shared_ptr<Point_2_Map> current_frame);
    bool removeUnstableSurfel();
};


#endif //SRC_SURFELMAP_H
