//
// Created by tongda on 2021/12/16.
//

#ifndef SMROS_SUMASLAM_H
#define SMROS_SUMASLAM_H

#define PCL_NO_PRECOMPILE

#include "RangenetAPI.hpp"
#include "rv/ParameterList.h"
#include "SurfelMap.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>


class SumaSLAM {
private:
    std::shared_ptr<RangenetAPI> net_;
    std::shared_ptr<Point_2_Map> current_frame_;
    rv::ParameterList params_;
    uint32_t timestamp_;
    pcl::PointCloud<Surfel> odometry_result_;
    std::shared_ptr<SurfelMap> map_;
    pcl::CorrespondencesPtr mapping_;
    float initial_confidence_;
    ros::Publisher pub1, pub2, pub3;
    ros::Publisher pub;
    ifstream inposes;

public:
    SumaSLAM(const std::string& parameter_path = "");
    void init();
    ~SumaSLAM() {}
    bool step(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    bool render();
    bool preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    bool odometry();
    bool initialSystem(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    bool mapUpdate();
    bool generateMap(pcl::PointCloud<Surfel> & point_cloud);
    bool readFromFile(std::string dir_path);
    bool readPose(int timestamp);
};


#endif //SMROS_SUMASLAM_H
