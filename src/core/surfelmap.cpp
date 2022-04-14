//
// Created by tongda on 2021/12/19.
//

#include "surfelmap.h"
const float PI = acos(-1);


SurfelMap::SurfelMap()
//        global_poses_(),
//        local_poses_(),
//        surfel_map_(0),
//        pose_graph_()
{
    // read parameters from parameters server
    nh_.getParam("p_stable", p_stable_);
    nh_.getParam("p_prior", p_prior_);
    nh_.getParam("sigma_angle", sigma_angle_2_);
    nh_.getParam("sigma_distance", sigma_distance_2_);
    nh_.getParam("distance_thred", distance_thred_);
    nh_.getParam("angle_thred", angle_thred_);
    nh_.getParam("gamma", gamma_);
    nh_.getParam("confidence_threshold", confidence_thred_);
    nh_.getParam("time_gap", time_gap_);
    nh_.getParam("loop_thred", loop_thred_);
    nh_.getParam("pose_path",pose_out_path_);

    pub_ = nh_.advertise<nav_msgs::Path> ("odometry", 1000);

    odds_p_prior_ = std::log(p_prior_ / (1 - p_prior_));
    initial_confidence_ = std::log(p_stable_ / (1 - p_stable_)) - odds_p_prior_;
    sigma_angle_2_ = sigma_angle_2_ * sigma_angle_2_;
    sigma_distance_2_ = sigma_distance_2_ * sigma_distance_2_;
    angle_thred_ = std::sin(angle_thred_);

    active_map_ = std::make_shared<VertexMap>(initial_confidence_);
    have_loop_ = false;
    
//    loop_edges_ = std::make_shared<sfm::loopsure_edge>();
    loopsure_poses_.resize(1);
    timestamp_ = 0;

    auto& diag = info_.diagonal();
    // translational noise is smaller than rotational noise.
    float transNoise = 1.0;
    float rotNoise = 1.0;
    diag[0] = (transNoise * transNoise);
    diag[1] = (transNoise * transNoise);
    diag[2] = (transNoise * transNoise);

    diag[3] = (rotNoise * rotNoise);
    diag[4] = (rotNoise * rotNoise);
    diag[5] = (rotNoise * rotNoise);
}

bool SurfelMap::pushBackPose(pose_type& pose) {
    // save local speed
    local_poses_.push_back(pose);
    // compute and save global pose
    pose_type global_pose;
    if(timestamp_ == 0)
    {
        global_pose = pose;
    }
    else
    {
        global_pose = global_poses_[timestamp_ - 1] * pose;
    }
    // svae global pose
    global_poses_.push_back(global_pose);
    // set optional graph node and edge
    pose_graph_.setInitialValues(timestamp_, global_pose.cast<double>());
    if(timestamp_ > 0)
    {
        pose_graph_.addEdge(timestamp_ - 1, timestamp_, pose.cast<double>(), info_);
    }
    return true;
}

pose_type SurfelMap::getLastPose() {
    return local_poses_.back();
}

bool SurfelMap::mapInitial(pose_type& init_pose)
{
    // push init pose
    pushBackPose(init_pose);
    return true;
}

float SurfelMap::updateConfidence(float confidence, float angle_2, float distance_2) {
    float angle_error = std::exp(-1.0 * angle_2 / sigma_angle_2_);
    float distance_error = std::exp(-1.0 * distance_2 / sigma_angle_2_);
    float odds_error = p_stable_ * angle_error * distance_error;
    float odds_p_stable = std::log(odds_error / (1 - odds_error));
    return confidence + odds_p_stable - odds_p_prior_;
}

bool SurfelMap::generateMap(pcl::PointCloud<Surfel> & global_map)
{
    if (surfel_map_.empty())
    {
        return false;
    }
    removeUnstableSurfel();
    pcl::PointCloud<Surfel> transform_result;
    for(int i = 0; i < surfel_map_.size(); i++)
    {
        transform_result.clear();
        pcl::transformPointCloud(*surfel_map_[i], transform_result, global_poses_[i]);
        global_map += transform_result;
    }
    return true;
}

bool SurfelMap::generateMap(std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> global_map)
{
    if (surfel_map_.empty())
    {
        return false;
    }
    // potential process
    // compute point number
    int result = 0;
    for(int i = 0; i < surfel_map_.size(); i++)
    {
        result += surfel_map_[i]->size();
    }
    std::cout << "Have " << result << " points" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> tmp_points;
    pcl::PointCloud<pcl::PointXYZRGB> transform_result;
    for(int i = 0; i < surfel_map_.size(); i++)
    {
        transform_result.clear();
        pcl::copyPointCloud(*surfel_map_[i], transform_result);
        for(int j = 0; j < transform_result.size(); j++)
        {
            transform_result.points[j].r = surfel_map_[i]->points[j].r * 256;
            transform_result.points[j].g = surfel_map_[i]->points[j].g * 256;
            transform_result.points[j].b = surfel_map_[i]->points[j].b * 256;
        }
        pcl::transformPointCloud(transform_result, tmp_points, global_poses_[i]);
//        surfel_map_[i]->clear();
        *global_map += tmp_points;
    }
    return true;
}

bool SurfelMap::updateMap(std::shared_ptr<VertexMap> current_frame, pose_type crt_pose)
{
    // save pose
    pushBackPose(crt_pose);
    // create a new timestamp surfel
    std::shared_ptr<pcl::PointCloud<Surfel>> new_time_clouds = std::make_shared<pcl::PointCloud<Surfel>>();
    surfel_map_.push_back(new_time_clouds);
    std::shared_ptr<pcl::PointCloud<Surfel>> origin_point = current_frame->getPointCloudsPtr();
    int num_point = origin_point->size();
    for(int i = 0; i < num_point; i++)
    {
        new_time_clouds->push_back(origin_point->points[i]);
    }
    timestamp_++;

    return true;
}
bool SurfelMap::generateActiveMap()
{
    int total_frame = surfel_map_.size();
    std::shared_ptr<pcl::PointCloud<Surfel>> point_in_active_map = active_map_->getPointCloudsPtr();
    point_in_active_map->clear();

    if (surfel_map_.empty())
    {
        return false;
    }
    // generate map
    pose_type last_pose = pose_type::Identity();
    for(int i = total_frame - 1, k = 0; i > total_frame - time_gap_ && i > -1; i--, k++)
    {
        pcl::PointCloud<Surfel> transform_result;
        pcl::transformPointCloud(*surfel_map_[i], transform_result, last_pose);
        *point_in_active_map += transform_result;
        last_pose = last_pose * local_poses_[i].inverse();
    }
    return true;
}

bool SurfelMap::removeUnstableSurfel() {
    for(int i = 0; i < surfel_map_.size(); i++)
    {
        int num_points = surfel_map_[i]->size();
        std::shared_ptr<pcl::PointCloud<Surfel>> point_clouds = surfel_map_[i];
        for(int i_2 = 0; i_2 < num_points; i_2++)
        {
            if(point_clouds->points[i_2].confidence < confidence_thred_)
            {
                point_clouds->points[i_2].x = NAN;
            }
        }
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*point_clouds, *point_clouds, mapping);
    }
    return false;
}

std::shared_ptr<pcl::PointCloud<Surfel>> SurfelMap::getPointCloudsInLocal(int id)
{
    return surfel_map_[id];
}

std::shared_ptr<pcl::PointCloud<Surfel>> SurfelMap::getPointCloudsInGlobal(int id)
{
    std::shared_ptr<pcl::PointCloud<Surfel>> transform_result = std::make_shared<pcl::PointCloud<Surfel>>();
    pcl::transformPointCloud(*surfel_map_[id], *transform_result, global_poses_[id]);
    return transform_result;
}

bool SurfelMap::setLoopsureEdge(int from, int to, pose_type& pose) {
    // valid last loopsure
    if(have_loop_ && from - from_ < loop_thred_)
    {
        // optimise pose
        pose_graph_.addEdge(from_, to_, loopsure_poses_[0].cast<double>(), info_);
        pose_graph_.optimize(30);
        pose_graph_.updatePoses(global_poses_);
    }
    from_ = from;
    to_ = to;
    loopsure_poses_[0] = pose;
    have_loop_ = true;

    return true;
}

bool SurfelMap::resetLoopTimes()
{
    have_loop_ = false;
    return true;
}

bool SurfelMap::savePose2File(std::string suffix)
{
    std::ofstream filehandle;
    pose_out_path_ += suffix;
    filehandle.open(pose_out_path_);
    if(filehandle.is_open())
    {
        for(int i = 0; i < global_poses_.size(); i++)
        {
            // transfer pose to camera coordinate
            pose_type campose = global_poses_[i];
            // save pose as kitti style
            for(int j = 0; j < 11; j++)
            {
                int u = j / 4;
                int v = j % 4;
                filehandle << campose(u, v) << " ";
            }
            filehandle << campose(3, 3) << std::endl;
        }
        filehandle.close();
    }
    else
    {
        filehandle.close();
        return false;
    }
    return true;
}

int SurfelMap::getCurrentIndex()
{
    return surfel_map_.size() - 1;
}

std::shared_ptr<VertexMap> SurfelMap::getActiveMapPtr()
{
    return active_map_;
}

float SurfelMap::getInitConfidence() {return initial_confidence_;}

int SurfelMap::getTimestamp()
{
    return timestamp_;
}

bool SurfelMap::rospath()
{

    nav_msgs::Path path_s;
    path_s.header.frame_id = "velodyne";
    path_s.header.stamp = ros::Time::now();
    for(int i = 0; i < global_poses_.size(); i++)
    {
        // transfer pose to camera coordinate
        pose_type campose = global_poses_[i];
        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = "velodyne";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = campose(0, 3);
        pose.pose.position.y = campose(1, 3);
        pose.pose.position.z = campose(2, 3);

        path_s.poses.push_back(pose);
    }

    pub_.publish(path_s);

    return true;
}