#include "sumaslam.h"

SumaSLAM::SumaSLAM(const std::string& parameter_path) :
        odometry_result_(),
        scManager_()
{
    parseXmlFile(parameter_path, params_);
    timestamp_ = std::make_shared<Timestamp>();

//    std::cout << "Init rangenet." << std::endl;
    net_ = std::make_shared<RangenetAPI>(params_);
    map_ = std::make_shared<SurfelMap>(params_, timestamp_);

    current_frame_ = std::make_shared<VertexMap>(params_, map_->getInitConfidence());
    max_distance_ = params_["max_distance_gap"];

    max_angle_ = params_["max_angle_gap"];
    update_mode_ = true;
    crt_pose_ = pose_type::Identity();
//    init();
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

//    std::unordered_map<int, int> label_nums;

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
        // set label
        net_->setColorMap(label);
//        label_nums[label]++;
        point_clouds_surfel->points[i].point_type = label;
        point_clouds_surfel->points[i].r = net_->getColorR() / 256.0f;
        point_clouds_surfel->points[i].g = net_->getColorG() / 256.0f;
        point_clouds_surfel->points[i].b = net_->getColorB() / 256.0f;
    }
//    for(auto iter : label_nums)
//    {
//        std::cout << "type: " << iter.first << " number: " << iter.second << std::endl;
//    }
    // generate surfel
    current_frame_->removeVehiclePoint();
    current_frame_->points2Surfel(timestamp_->getCurrentime());
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
    std::clock_t start_time = std::clock();
    std::cout << "suma slam time " << timestamp_->getCurrentime() << ":" << std::endl;
    preprocess(point_clouds_xyzi);
    if(timestamp_->getCurrentime() == 0)
    {
        mapUpdate();
    }
    else
    {
        std::cout << "Preprocess using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
        odometry();
//    readPose();
        std::cout << "Odometry using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
        mapUpdate();
        std::cout << "Update Map using: " << (float)(std::clock() - start_time) / CLOCKS_PER_SEC << std::endl;
        if(update_mode_)
        {
            loopsureDetection(point_clouds_xyzi);
        }
        else
        {
            map_->resetLoopTimes();
        }
    }

    timestamp_->nextTime();
    std::clock_t end_time = std::clock();
    std::cout << "A step finish. " << "using " << (float)(end_time - start_time) / CLOCKS_PER_SEC << std::endl << std::endl;
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
        float dis = crt_pose_(i, 3) - init_pose(i, 3);
        distance += dis * dis;
    }
    distance = sqrt(distance);
    // rotation angle
    pose_type localpose = init_pose.inverse() * crt_pose_;
    Eigen::Quaternionf q(localpose.block<3, 3>(0 , 0));
    float angle_change = q.x() + q.y() + q.z();
    angle_change = abs(angle_change);

    if(distance > max_distance_ || angle_change > max_angle_)
    {
        update_mode_ = true;
    } else{
        update_mode_ = false;
    }
    std::cout << "angle: " << angle_change << std::endl;
    std::cout << "distance: " << distance << std::endl;
    std::cout << "mode: " << update_mode_ << std::endl;

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
    std::string path = params_["pcdpath"];
    std::vector<std::string> out_filelsits;
    read_filelists(path, out_filelsits, "pcd");
    sort_filelists(out_filelsits, "pcd");
    int render_gap = params_["render_gap"];

    int k = 0;
    int skip = params_["skip_gap"];
    for(auto file : out_filelsits)
    {
        if(k < skip)
        {
            k++;
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI> pointcloud;
        pcl::io::loadPCDFile<pcl::PointXYZI>(path + file, pointcloud);
        step(pointcloud);

        if(timestamp_->getCurrentime() % render_gap == render_gap - 1)
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
            map_->generateActiveMap();
            active_points = map_->getActiveMapPtr()->getPointCloudsPtr();
            sensor_msgs::PointCloud2 msg1;
            pcl::toROSMsg(*active_points, msg);
            msg.header.frame_id = "velodyne";
            pub1.publish(msg);
        }
    }

    std::cout << "finish." << std::endl;


    return false;
}

bool SumaSLAM::loopsureDetection(const pcl::PointCloud<pcl::PointXYZI> & point_clouds_xyzi) {
    scManager_.makeAndSaveScancontextAndKeys(*current_frame_->getPointCloudsPtr());
    auto pair_result = scManager_.detectLoopClosureID();
    int SCclosestHistoryFrameID = pair_result.first;
    if( SCclosestHistoryFrameID == -1 ) {
        return false;
    }
    int pre_index = SCclosestHistoryFrameID;
    int crt_index = map_->getCurrentIndex();

    std::shared_ptr<pcl::PointCloud<Surfel>> pre_point_clouds = map_->getPointCloudsInLocal(pre_index);
    std::shared_ptr<pcl::PointCloud<Surfel>> crt_point_clouds = map_->getPointCloudsInLocal(crt_index);
    pose_type initial_pose;
    initial_pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    pose_type res_pose = computePose(crt_point_clouds, pre_point_clouds, initial_pose);
    map_->setLoopsureEdge(crt_index, pre_index, res_pose);
    return true;
}

bool SumaSLAM::mapUpdate() {
//    if(timestamp_->getCurrentime() == 0)
//    {
//        map_->mapInitial();
//    }
    map_->updateMap(current_frame_, update_mode_, crt_pose_);
    return false;
}

pose_type SumaSLAM::computePose(std::shared_ptr<pcl::PointCloud<Surfel>> input_points,
                                std::shared_ptr<pcl::PointCloud<Surfel>> target_points, pose_type initial_pose) {
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

