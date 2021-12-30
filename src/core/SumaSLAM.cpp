
#include "SumaSLAM.h"

#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

SumaSLAM::SumaSLAM(const std::string& parameter_path) :
    odometry_result_()
{
    parseXmlFile(parameter_path, params_);

//    std::cout << "Init rangenet." << std::endl;
    net_ = std::make_shared<RangenetAPI>(params_);
    map_ = std::make_shared<SurfelMap>(params_);

    current_frame_ = std::make_shared<Frame>(params_, map_->getInitConfidence());

    timestamp_ = 0;
}

void SumaSLAM::init()
{
    ros::NodeHandle nh;
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("inputpoint", 1000);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("target", 1000);
    pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output", 1000);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("SemanticMap", 1000);
}

bool SumaSLAM::preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi)
{
    std::clock_t start_time = std::clock();
    int num_points = point_clouds_xyzi.points.size();
    // store pointcloud in vector type
    std::vector<float> points_xyzi_list(num_points * 4);

    // frame parameter reference
    auto& point_clouds_surfel = current_frame_->setPointCloud();
    std::vector<int>& labels = current_frame_->setLabels();
    std::vector<float>& labels_prob = current_frame_->setLabelProbability();
    // resize
    pcl::copyPointCloud(point_clouds_xyzi, point_clouds_surfel);
    labels.resize(num_points);
    labels_prob.resize(num_points);
    // transform pointcloud to vector
    for(int i = 0; i < num_points; i++)
    {
        int iter = i * 4;
        points_xyzi_list[iter] = point_clouds_xyzi.points[i].x;
        points_xyzi_list[iter + 1] = point_clouds_xyzi.points[i].y;
        points_xyzi_list[iter + 2] = point_clouds_xyzi.points[i].z;
        points_xyzi_list[iter + 3] = point_clouds_xyzi.points[i].data[3];
    }
    // get semantic result
    std::vector<std::vector<float>> semantic_result = net_->infer(points_xyzi_list, num_points);
    // match the label from label_map and color the point cloud
    for(int i = 0; i < num_points; i++)
    {
        float prob = 0;
        int index = 0;
        for(int k = 0; k < 20; k++)
        {
            if(prob <= semantic_result[i][k])
            {
                prob = semantic_result[i][k];
                index = k;
            }
        }
        labels[i] = net_->getLabel(index);
        labels_prob[i] = prob;
        // get semantic point cloud
        net_->setColorMap(labels[i]);
        point_clouds_surfel.points[i].point_type = labels[i];
        point_clouds_surfel.points[i].r = net_->getColorR() / 256.0;
        point_clouds_surfel.points[i].g = net_->getColorG() / 256.0;
        point_clouds_surfel.points[i].b = net_->getColorB() / 256.0;
    }

    current_frame_->generateMap(timestamp_);
    std::clock_t end_time = std::clock();
//    std::cout << "preprocess points in suma slam. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl;

    return true;
}

bool SumaSLAM::step(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    std::clock_t start_time = std::clock();
    std::cout << "suma slam time " << timestamp_ << ":" << std::endl;
    if(timestamp_ == 0)
    {
        return initialSystem(point_clouds_xyzi);
    }
    preprocess(point_clouds_xyzi);
    std::cout << "Preprocess using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
    odometry();
    std::cout << "Odometry using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
    map_->updateMap(mapping_, timestamp_, current_frame_);
    std::cout << "Update Map using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    timestamp_++;
    std::clock_t end_time = std::clock();
    std::cout << "A step finish. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl << std::endl;
    return true;
}

bool SumaSLAM::initialSystem(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    preprocess(point_clouds_xyzi);
//    current_frame_->generateMap();
    map_->mapInitial(current_frame_->getPointClouds());
    timestamp_++;
    return true;
}

bool SumaSLAM::odometry() {
//    std::cout << "odometry in suma slam." << std::endl;
    pcl::IterativeClosestPoint<Surfel, Surfel> icp;

//    std::cout << "points number for icp: " << std::endl;
//    std::cout << "current_frame: " << current_frame_->getPointClouds().size() << std::endl;
//    std::cout << "active map: " << map_->getPointNum() << std::endl;

    std::clock_t t1 = std::clock();
    map_->generateActiveMap(timestamp_);
    std::clock_t t2 = std::clock();
    icp.setInputSource(current_frame_->getPointCloudsPtr());
    icp.setInputTarget(map_->getActiveMapPtr());
    icp.setMaxCorrespondenceDistance(0.05);
    icp.setMaximumIterations(30);

    icp.align(odometry_result_);
    std::clock_t t3 = std::clock();

    mapping_ = icp.correspondences_;

    std::cout << "render active map using: " << (float)(t2 - t1) / CLOCKS_PER_SEC <<
    " have: " << current_frame_->getPointCloudsPtr()->size() << " as input and " <<
    map_->getActiveMapPtr()->size() << std::endl;
    std::cout << "mapping: " << mapping_->size() << std::endl;
    std::cout << "icp using: " << (float)(t3 - t2) / CLOCKS_PER_SEC << std::endl;
//    std::cout << icp.getFinalTransformation().block<3, 1>(0, 3) << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    map_->pushBackPose(icp.getFinalTransformation());

//    pcl::PointCloud<pcl::PointXYZRGB> cloud;
//    pcl::copyPointCloud(odometry_result_, cloud);
//    pcl::PointCloud<pcl::PointXYZRGB> cloud1;
//    pcl::copyPointCloud(*map_->getActiveMapPtr(), cloud1);
//    for (int i = 0; i < cloud.size(); ++i) {
//        cloud.points[i].r = 100;
//    }
//    for (int i = 0; i < cloud1.size(); ++i) {
//        cloud1.points[i].b = 100;
//    }
//    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//    viewer.showCloud ((cloud + cloud1).makeShared());
//    while (!viewer.wasStopped ())
//    {
//
//    }
//    sensor_msgs::PointCloud2 msg1, msg2, msg3;
//    pcl::toROSMsg(*current_frame_->getPointCloudsPtr(), msg1);
//    pcl::toROSMsg(*map_->getActiveMapPtr(), msg2);
//    pcl::toROSMsg(odometry_result_, msg3);
//    msg1.header.frame_id = "velodyne";
//    msg2.header.frame_id = "velodyne";
//    msg3.header.frame_id = "velodyne";
//    pub1.publish(msg1);
//    pub2.publish(msg2);
//    pub3.publish(msg3);

    return true;
}

bool SumaSLAM::generateMap(pcl::PointCloud<Surfel> & point_cloud)
{
    if(map_->generateMap(point_cloud))
        return true;
    return false;
}

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

bool SumaSLAM::readFromFile(std::string dir_path) {
    std::vector<std::string> out_filelsits;
    read_filelists(dir_path, out_filelsits, "pcd");
    sort_filelists(out_filelsits, "pcd");


    for(auto file : out_filelsits)
    {
        auto pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile<pcl::PointXYZI>(dir_path + file, *pointcloud);
        step(*pointcloud);

        auto pointcloud1(new pcl::PointCloud<Surfel>);
        generateMap(*pointcloud1);
        std::cout << "render map. have: " << pointcloud1->size() << std::endl;
//        auto pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
//        pcl::copyPointCloud(*pointcloud1, *pointcloud2);
//        for(int i = 0; i < pointcloud2->size(); i++)
//        {
//            pointcloud2->points[i].r = pointcloud1->points[i].r * 256;
//            pointcloud2->points[i].g = pointcloud1->points[i].g * 256;
//            pointcloud2->points[i].b = pointcloud1->points[i].b * 256;
//        }

            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*pointcloud1, msg);
            msg.header.frame_id = "velodyne";
            pub.publish(msg);

            sensor_msgs::PointCloud2 msg1;
            pcl::toROSMsg(map_->getMyGlobalMap(), msg1);
            msg1.header.frame_id = "velodyne";
            pub1.publish(msg1);
    }


    return false;
}

