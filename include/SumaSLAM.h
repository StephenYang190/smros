/* SumaSLAM class
 * Create by Tongda Yang
 * This class is used to implement whole slam function
 * */

#ifndef SMROS_SUMASLAM_H
#define SMROS_SUMASLAM_H

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <iostream>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <Scancontext/Scancontext.h>

#include "RangenetAPI.hpp"
#include "SurfelMap.h"


class SumaSLAM {
private:
    // rangenet
    std::shared_ptr<RangenetAPI> net_;
    // frame to store current point clouds and tansform to surfel based point clouds
    std::shared_ptr<VertexMap> current_frame_;
    // custom parameters list
    rv::ParameterList params_;
    // timestamp
    std::shared_ptr<Timestamp> timestamp_;
    // store odometry transform result
    pcl::PointCloud<Surfel> odometry_result_;
    // map
    std::shared_ptr<SurfelMap> map_;
    // scan context manager
    SCManager scManager_;
    // debug parameters
    ros::Publisher pub1, pub2, pub3;
    ros::Publisher pub;
    std::ifstream inposes;

protected:
    // update map
    bool mapUpdate();
    // loopsure detection
    bool loopsureDetection();
    // pre-process point clouds(transform to surfel based point clouds)
    bool preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    // odometry(ndt omp)
    bool odometry();
    // function used to compute pose
    pose_type computePose(std::shared_ptr<pcl::PointCloud<Surfel>> input_points,
                          std::shared_ptr<pcl::PointCloud<Surfel>> target_points,
                          pose_type initial_pose);
public:
    SumaSLAM(const std::string& parameter_path = "");
    void init();
    ~SumaSLAM() {}
    // operate a step
    bool step(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    // generate global map
    bool generateMap(pcl::PointCloud<Surfel> & point_cloud);
    bool readFromFile(std::string dir_path);
    bool readPose(int timestamp);
};


#endif //SMROS_SUMASLAM_H
