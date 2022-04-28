/* SumaSLAM class
 * Create by Tongda Yang
 * This class is used to implement whole slam function
 * */

#ifndef SMROS_SUMASLAM_H
#define SMROS_SUMASLAM_H

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <iostream>
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pcl/io/pcd_io.h>
#include <Scancontext/Scancontext.h>
#include <chrono>

#include "RangenetAPI.hpp"
#include "surfelmap.h"


class SumaSLAM {
private:
    // rangenet
    std::shared_ptr<RangenetAPI> net_;
    // frame to store current point clouds and tansform to surfel based point clouds
    std::shared_ptr<VertexMap> current_frame_;
    // timestamp
    std::shared_ptr<Timestamp> timestamp_;
    int crt_id_;
    // store odometry transform result
    pcl::PointCloud<Surfel> odometry_result_;
    // map
    std::shared_ptr<SurfelMap> map_;
    // scan context manager
    SCManager scManager_;
    // distance threshold
    float max_distance_;
    // angle thresholdmap
    float max_angle_;
    // current pose
    pose_type crt_pose_;
    // ros node handle
    ros::NodeHandle nh_;
    // debug parameters
    ros::Publisher pub1, pub2, pub3;
    ros::Publisher pub;
    std::ifstream inposes;
    // parameters to control method
    bool isloop_, iskey_, isremove_, isdown_;
    // parameters to load and save result
    std::string pcd_in_path_, pcd_out_path_;
    std::string sequence_;
    std::string suffix_;

protected:
    // update map
    bool mapUpdate();
    // loopsure detection
    bool loopsureDetection();
    bool loopsureDetection(bool od_loopsure);
    // pre-process point clouds(transform to surfel based point clouds)
    bool preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    // odometry(ndt omp)
    bool odometry();
    // function used to compute pose
    pose_type computePose(pcl::PointCloud<Surfel>::Ptr input_points,
                          pcl::PointCloud<Surfel>::Ptr target_points,
                          pose_type& initial_pose);
    // function to pose global map to ros node
    void publicMap();
    // function to demonstrate map construction
    void brocastMap(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    // function of voxel filter
    void filterPointCloud(const pcl::PointCloud<Surfel>::Ptr input,
                          pcl::PointCloud<Surfel>::Ptr output,
                          const double leafsize = 0.1);
public:
    SumaSLAM();
    void init();
    ~SumaSLAM() {
        inposes.close();
    }
    // operate a step
    bool step(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi);
    // generate global map
    bool generateMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
    bool readFromFile();
    bool readPose();
    // debug
    void testLoopsure();
};


#endif //SMROS_SUMASLAM_H
