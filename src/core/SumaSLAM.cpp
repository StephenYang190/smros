
#include "SumaSLAM.h"



SumaSLAM::SumaSLAM(const std::string& parameter_path) :
    odometry_result_()
{
    parseXmlFile(parameter_path, params_);

//    std::cout << "Init rangenet." << std::endl;
    net_ = std::make_shared<RangenetAPI>(params_);
    map_ = std::make_shared<SurfelMap>(params_);

    current_frame_ = std::make_shared<Point_2_Map>(params_, map_->getInitConfidence());

    timestamp_ = 0;
}

void SumaSLAM::init()
{
    ros::NodeHandle nh;
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("inputpoint", 1000);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("target", 1000);
    pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output", 1000);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("SemanticMap", 1000);
    inposes.open("/home/tongda/workspace/kitti_00_pose_velodyne_local.txt");
    if(inposes.is_open())
    {

    }
}

bool SumaSLAM::preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi)
{
    std::clock_t start_time = std::clock();
    // frame parameter reference
    std::shared_ptr<pcl::PointCloud<Surfel>> point_clouds_surfel = current_frame_->setPointCloud();
    // set point
    pcl::copyPointCloud(point_clouds_xyzi, *point_clouds_surfel);
    // generate surfel
    current_frame_->generateSurfel(timestamp_);
    int num_points = current_frame_->getPointNum();
    // store pointcloud in vector type
    std::vector<float> points_xyzi_list(num_points * 4);
    // transform pointcloud to vector
    for(int i = 0; i < num_points; i++)
    {
        int iter = i * 4;
        points_xyzi_list[iter] = point_clouds_surfel->points[i].x;
        points_xyzi_list[iter + 1] = point_clouds_surfel->points[i].y;
        points_xyzi_list[iter + 2] = point_clouds_surfel->points[i].z;
        points_xyzi_list[iter + 3] = point_clouds_surfel->points[i].data[3];
    }
    // get semantic result
    std::vector<std::vector<float>> semantic_result = net_->infer(points_xyzi_list, num_points);
    std::cout << "finish predict" << endl;
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
        int label = net_->getLabel(index);
        // get semantic point cloud
        net_->setColorMap(label);
        point_clouds_surfel->points[i].point_type = label;
        point_clouds_surfel->points[i].r = net_->getColorR() / 256.0f;
        point_clouds_surfel->points[i].g = net_->getColorG() / 256.0f;
        point_clouds_surfel->points[i].b = net_->getColorB() / 256.0f;
    }
//
//    std::clock_t end_time = std::clock();
//    std::cout << "preprocess points in suma slam. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl;

    return true;
}

bool SumaSLAM::readPose(int timestamp)
{
    float poes[12];
    for(int i = 0; i < 12; i++)
    {
        inposes >> poes[i];
    }
    Eigen::Matrix4f pose;
    pose << poes[0], poes[1], poes[2], poes[3],
            poes[4], poes[5], poes[6], poes[7],
            poes[8], poes[9], poes[10], poes[11],
            0, 0, 0, 1;

    map_->pushBackPose(pose);

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
//    readPose(timestamp_);
    std::cout << "Odometry using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
    map_->updateMap(timestamp_, current_frame_);
    std::cout << "Update Map using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;

    timestamp_++;
    std::clock_t end_time = std::clock();
    std::cout << "A step finish. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl << std::endl;
    return true;
}

bool SumaSLAM::initialSystem(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    preprocess(point_clouds_xyzi);
//    current_frame_->generateMap();
    map_->mapInitial(current_frame_->getPointCloudsPtr());
    timestamp_++;
    return true;
}

bool SumaSLAM::odometry() {
    map_->generateActiveMap(timestamp_);

    pcl::PointCloud<Surfel> filtered_cloud;
    pcl::ApproximateVoxelGrid<Surfel> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
    approximate_voxel_filter.setInputCloud (current_frame_->getPointCloudsPtr()->makeShared());
    approximate_voxel_filter.filter (filtered_cloud);

    pclomp::NormalDistributionsTransform<Surfel, Surfel> ndt;

    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1.0);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);

    // Setting point cloud to be aligned.
    ndt.setInputSource (filtered_cloud.makeShared());
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (map_->getActiveMapPtr()->getPointCloudsPtr()->makeShared());

    // Calculating required rigid transform to align the input cloud to the target cloud.
    ndt.align (odometry_result_, map_->getPose(timestamp_ - 1));

//    std::cout << ndt.getFinalTransformation() << std::endl;

    map_->pushBackPose(ndt.getFinalTransformation());

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
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        pcl::io::loadPCDFile<pcl::PointXYZI>(dir_path + file, pointcloud);
        step(pointcloud);

        if(timestamp_ % 10 == 9)
        {
            pcl::PointCloud<Surfel> global_points;
            generateMap(global_points);

            pcl::ApproximateVoxelGrid<Surfel> approximate_voxel_filter;
            approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
            approximate_voxel_filter.setInputCloud(global_points.makeShared());
            approximate_voxel_filter.filter(global_points);
            for(int i = 0; i < 10; i++)
            {
                std::cout << "*\t";
            }
            std::cout << std::endl;
            std::cout << "render map. Map have: " << global_points.size() << std::endl;
            for(int i = 0; i < 10; i++)
            {
                std::cout << "*\t";
            }
            std::cout << std::endl;
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(global_points, msg);
            msg.header.frame_id = "velodyne";
            pub.publish(msg);

            std::shared_ptr<pcl::PointCloud<Surfel>> active_points;
            map_->generateActiveMap(timestamp_);
            active_points = map_->getActiveMapPtr()->getPointCloudsPtr();
            sensor_msgs::PointCloud2 msg1;
            pcl::toROSMsg(*active_points, msg);
            msg.header.frame_id = "velodyne";
            pub1.publish(msg);
        }
    }


    return false;
}

