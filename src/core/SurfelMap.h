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

#include "rv/ParameterList.h"
#include "Frame.h"

class SurfelMap {
private:
    std::vector<Eigen::Matrix4f> poses_;
    std::vector<pcl::PointCloud<Surfel>> surfels_;
    pcl::PointCloud<Surfel> active_map_;
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
    pcl::PointCloud<Surfel> my_global_map_;

public:
    SurfelMap(rv::ParameterList parameter_list);
    ~SurfelMap();
    bool pushBackPose(Eigen::Matrix4f pose);
    Eigen::Matrix4f getPose(int timestamp);
    bool updateMap(const Frame & current_frame, int timestamp);
    bool updateMap(const pcl::PointCloud<Surfel> & align_point,
                   const pcl::CorrespondencesPtr mapping, int timestamp, std::shared_ptr<Frame> current_frame);
    bool mapInitial(const pcl::PointCloud<Surfel> & pointcloud);
    float updateConfidence(float confidence, float angle_2, float distance_2);
    bool initialSurfel(frm::map new_surfel, int timestamp, Surfel & surfel);
    bool initialSurfel(int timestamp, Surfel & surfel);
    bool updateActiveMap(int timestamp);
    bool generateMap(pcl::PointCloud<Surfel> & global_map);
    const pcl::PointCloud<Surfel>::Ptr getActiveMapPtr() {return active_map_.makeShared();}
    float getInitConfidence() {return initial_confidence_;}
    int getPointNum() {return active_map_.size();}
    bool generateActiveMap(int timestamp);
    bool updateMap(const pcl::CorrespondencesPtr mapping,
                              int timestamp, std::shared_ptr<Frame> current_frame);
    pcl::PointCloud<Surfel> & getMyGlobalMap() {return my_global_map_;}
};


#endif //SRC_SURFELMAP_H
