#include "sumaslam.h"

SumaSLAM::SumaSLAM() :
        odometry_result_(),
        scManager_()
{
    timestamp_ = std::make_shared<Timestamp>();
    // init rangenet
    std::string model_path;
    if(nh_.getParam("model_path", model_path))
    {
        net_ = std::make_shared<RangenetAPI>(model_path);
    }
    // init map
    map_ = std::make_shared<SurfelMap>();
    current_frame_ = std::make_shared<VertexMap>(map_->getInitConfidence());
    // init paremeters
    crt_pose_ = pose_type::Identity();
    crt_id_ = 0;
    // get parameters from rosparam server
    nh_.getParam("max_distance_gap",max_distance_);
    nh_.getParam("max_angle_gap",max_angle_);
    nh_.getParam("is_loop_detection", isloop_);
    nh_.getParam("is_key_selection", iskey_);
    nh_.getParam("is_remove_vehicle", isremove_);
    nh_.getParam("is_down_sampling", isdown_);
    nh_.getParam("pcd_in_path", pcd_in_path_);
    nh_.getParam("pcd_out_path", pcd_out_path_);
    // get sequence
    int se = 0;
    nh_.getParam("sequence", se);
    sequence_ += int(se / 10) + '0';
    sequence_ += int(se % 10) + '0';
    // generate suffix
    suffix_ = "";
    if(isloop_)
    {
        suffix_ += "l";
    }
    if(isremove_)
    {
        suffix_ += "r";
    }
    if(isdown_)
    {
        suffix_ += "d";
    }
    if(iskey_)
    {
        suffix_ += "k";
    }
    pcd_in_path_ += sequence_ + "/";
    pcd_out_path_ += "pcd_" + sequence_ + "_" + suffix_ + ".pcd";
}

void SumaSLAM::init()
{
    pub1 = nh_.advertise<sensor_msgs::PointCloud2> ("Input", 1000, true);
    pub2 = nh_.advertise<sensor_msgs::PointCloud2> ("Semantic", 1000, true);
    pub3 = nh_.advertise<sensor_msgs::PointCloud2> ("Target", 1000, true);
    pub = nh_.advertise<sensor_msgs::PointCloud2> ("SemanticMap", 1000, true);
//    inposes.open("/home/tongda/workspace/kitti_00_pose_velodyne_local.txt");
//    if(inposes.is_open())
//    {
//
//    }
}

bool SumaSLAM::preprocess(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi)
{
    // set point clouds
    current_frame_->setPointCloud(point_clouds_xyzi.makeShared());
    // get point clouds in surfel base
    std::shared_ptr<pcl::PointCloud<Surfel>> point_clouds_surfel = current_frame_->getPointCloudsPtr();

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
        // set label
        net_->setColorMap(label);
        point_clouds_surfel->points[i].point_type = label;
        point_clouds_surfel->points[i].r = net_->getColorR() / 256.0f;
        point_clouds_surfel->points[i].g = net_->getColorG() / 256.0f;
        point_clouds_surfel->points[i].b = net_->getColorB() / 256.0f;
    }
    // generate surfel
    if(isremove_)
    {
        current_frame_->removeVehiclePoint();
    }
    current_frame_->points2Surfel(crt_id_, isdown_);
    return true;
}

bool SumaSLAM::readPose()
{
    float poes[12];
    for(int i = 0; i < 12; i++)
    {
        inposes >> poes[i];
    }
    pose_type pose;
    pose << poes[0], poes[1], poes[2], poes[3],
            poes[4], poes[5], poes[6], poes[7],
            poes[8], poes[9], poes[10], poes[11],
            0, 0, 0, 1;

    map_->pushBackPose(pose);

    return true;
}

bool SumaSLAM::step(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    crt_id_ = map_->getTimestamp();
    std::cout << "suma slam time " << timestamp_->getCurrentime() << " : " << crt_id_ << std::endl;

    preprocess(point_clouds_xyzi);
    if(crt_id_ == 0)
    {
        mapUpdate();
        loopsureDetection();
    }
    else
    {
        if(odometry() || !iskey_)
        {
            mapUpdate();
            if(isloop_)
            {
                loopsureDetection();
            }
        }
//        mapUpdate();
    }

    timestamp_->nextTime();
    return true;
}

bool SumaSLAM::odometry() {
    map_->generateActiveMap();
    pose_type init_pose = map_->getLastPose();
    crt_pose_ = computePose(current_frame_->getPointCloudsPtr(),
                            map_->getActiveMapPtr()->getPointCloudsPtr(),
                            init_pose);
    // key frame selection
    // moving distance
    float distance = 0;
    for(int i = 0; i < 3; i++)
    {
        float dis = crt_pose_(i, 3);
        distance += dis * dis;
    }
    distance = sqrt(distance);
    // rotation angle
    Eigen::Quaternionf q(crt_pose_.block<3, 3>(0 , 0));
    float angle_change = q.x() + q.y() + q.z();
    angle_change = abs(angle_change);
    std::cout << "distance between two frames is : " << distance;
    std::cout << ", and angle is : " << angle_change << std::endl;
    if(distance < max_distance_ && angle_change < max_angle_)
    {
        return false;
    }

    return true;
}

bool SumaSLAM::generateMap(std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> point_cloud)
{
    if(map_->generateMap(point_cloud))
        return true;
    return false;
}

bool SumaSLAM::loopsureDetection() {
    std::shared_ptr<pcl::PointCloud<Surfel>> pointcloud = current_frame_->getPointCloudsPtr();
    scManager_.makeAndSaveScancontextAndKeys(*pointcloud);
    auto pair_result = scManager_.detectLoopClosureID();
    int pre_index = pair_result.first;
    if( pre_index == -1 || pre_index > crt_id_ ) {
        return false;
    }

    std::shared_ptr<pcl::PointCloud<Surfel>> pre_point_clouds = map_->getPointCloudsInLocal(pre_index);
    std::shared_ptr<pcl::PointCloud<Surfel>> crt_point_clouds = map_->getPointCloudsInLocal(crt_id_);
    pose_type initial_pose;
    initial_pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    std::cout << "start compute loopsure pose" << std::endl;
    pose_type res_pose = computePose(crt_point_clouds, pre_point_clouds, initial_pose);
    std::cout << "end compute loopsure pose" << std::endl;
    bool result = map_->setLoopsureEdge(crt_id_, pre_index, res_pose);
    std::cout << "end add loopsure edge" << std::endl;
    return true;
}

bool SumaSLAM::mapUpdate() {
    map_->updateMap(current_frame_, crt_pose_);
    return false;
}

pose_type SumaSLAM::computePose(std::shared_ptr<pcl::PointCloud<Surfel>> input_points,
                                std::shared_ptr<pcl::PointCloud<Surfel>> target_points, pose_type& initial_pose) {
    pcl::PointCloud<Surfel> filtered_cloud;
    pcl::ApproximateVoxelGrid<Surfel> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
    approximate_voxel_filter.setInputCloud (input_points->makeShared());
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
    ndt.setInputTarget (target_points->makeShared());

    // Calculating required rigid transform to align the input cloud to the target cloud.
    ndt.align (odometry_result_, initial_pose);

    return ndt.getFinalTransformation();
}

void SumaSLAM::publicMap()
{
    // generate global map
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> global_map = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    generateMap(global_map);
    // broadcast point numbers
    for(int i = 0; i < 10; i++)
    {
        std::cout << "*\t";
    }
    std::cout << std::endl;
    std::cout << "render map. Map have: " << global_map->size() << std::endl;
    for(int i = 0; i < 10; i++)
    {
        std::cout << "*\t";
    }
    std::cout << std::endl;
    // voxel filter
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    approximate_voxel_filter.setInputCloud(global_map->makeShared());
    approximate_voxel_filter.filter(*global_map);
    // save point cloud
//    pcl::io::savePCDFile(pcd_out_path_, *global_map);
//    // visualization
//    pcl::visualization::CloudViewer viewer("default");
//    viewer.showCloud(global_map->makeShared());
//    while(!viewer.wasStopped())
//    {
//    }
    // public to rviz
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*global_map, msg);
    msg.header.frame_id = "velodyne";
    pub.publish(msg);

//    std::shared_ptr<pcl::PointCloud<Surfel>> active_points;
//    map_->generateActiveMap();
//    active_points = map_->getActiveMapPtr()->getPointCloudsPtr();
//    sensor_msgs::PointCloud2 msg1;
//    pcl::toROSMsg(*active_points, msg);
//    msg.header.frame_id = "velodyne";
//    pub1.publish(msg);
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

bool SumaSLAM::readFromFile() {
    int render_gap, skip;
    nh_.getParam("render_gap",render_gap);
    nh_.getParam("skip_gap",skip);

    std::vector<std::string> out_filelsits;
    read_filelists(pcd_in_path_, out_filelsits, "pcd");
    sort_filelists(out_filelsits, "pcd");

    int k = 0;

    auto start = std::chrono::steady_clock::now();
    for(auto file : out_filelsits)
    {
        if(k < skip)
        {
            k++;
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_in_path_ + file, pointcloud);
        step(pointcloud);
        brocastMap(pointcloud);

//        if(timestamp_->getCurrentime() % render_gap == render_gap - 1)
//        {
//
//        }
    }
    auto end = std::chrono::steady_clock::now();
    std::cout << "finish." << std::endl;
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "It took " << elapsed_seconds.count() << " seconds." << std::endl;
    //save pose
    std::string suffix = sequence_ + "_" + suffix_ + ".txt";
    if(map_->savePose2File(suffix))
    {
        std::cout << "Success to save pose." << std::endl;
    }
    else
    {
        std::cout << "Fail to save pose." << std::endl;
    }
    //post odometry
    map_->rospath();
    //post global map
    publicMap();
    return true;
}

void SumaSLAM::testLoopsure() {
    std::string inpath, outpath;
    if(nh_.getParam("pcdpath", inpath))
    {

    }
    if(nh_.getParam("loop_result", outpath))
    {

    }
    std::vector<std::string> out_filelsits;
    read_filelists(inpath, out_filelsits, "pcd");
    sort_filelists(out_filelsits, "pcd");

    std::ofstream filehandle;
    filehandle.open(outpath);

    int k = 0;

    for(auto file : out_filelsits)
    {
        k++;
        // read point cloud
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        pcl::io::loadPCDFile<pcl::PointXYZI>(inpath + file, pointcloud);
        // preprocess
        preprocess(pointcloud);
        // loopsure detection
        scManager_.makeAndSaveScancontextAndKeys(*current_frame_->getPointCloudsPtr());
        auto pair_result = scManager_.detectLoopClosureID();
        int SCclosestHistoryFrameID = pair_result.first;
        if( SCclosestHistoryFrameID == -1 ) {
            continue;
        }
        filehandle << k << " : " << SCclosestHistoryFrameID << std::endl;
    }
    filehandle.close();
    std::cout << "finish." << std::endl;

}

void SumaSLAM::brocastMap(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    // public input point cloud
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(point_clouds_xyzi, msg);
    msg.header.frame_id = "velodyne";
    pub1.publish(msg);
    // public semantic segmentation result
    sensor_msgs::PointCloud2 msg1;
    pcl::toROSMsg(*current_frame_->getPointCloudsPtr(),msg1);
    msg1.header.frame_id = "velodyne";
    pub2.publish(msg1);
    // public target
    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(*map_->getActiveMapPtr()->getPointCloudsPtr(),msg2);
    msg2.header.frame_id = "velodyne";
    pub3.publish(msg2);
    // public global map
    publicMap();
}