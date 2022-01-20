/* SurfelMap class
 * Create by Tongda Yang
 * This class is used to store the point clouds and poses in each timestampe
 * We store the poses and point clouds frame by frame
 * so that we do not need to regenerate the global map
 * after pose graph optimization
 * */

#ifndef SRC_SURFELMAP_H
#define SRC_SURFELMAP_H

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <ctime>
#include <pcl/cloud_iterator.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "vertexmap.h"
#include "backendopt.h"
#include "timestamp.h"

// define the pose type as matrix4f
using pose_type = Eigen::Matrix4f;
namespace sfm{
    struct loopsure_edge{
        int from;
        int to;
        pose_type pose;
    };
}

class SurfelMap {
private:
    // store pose frame by frame
    std::vector<pose_type, Eigen::aligned_allocator<pose_type>> pose_list_;
    // store point clouds frame by frame
    std::vector<std::shared_ptr<pcl::PointCloud<Surfel>>> surfel_map_;
    // active map
    std::shared_ptr<VertexMap> active_map_;
    // parameters used to compute confidence
    float p_stable_, p_prior_, odds_p_prior_;
    float sigma_angle_2_, sigma_distance_2_;
    float initial_confidence_;
    float distance_thred_, angle_thred_;
    float gamma_;
    float confidence_thred_;
    // parameters used to define the length of active map
    int time_gap_;
    // store the point clouds number of each frame in active map
    std::vector<int> active_map_index_in_map_;
    // custom parameters list
    rv::ParameterList param_;
    // pose graph
    BackEndOpt pose_graph_;
    // information metrix
    Eigen::DiagonalMatrix<double, 6> info_;
    // parameters used to control the time to optimization
    int loop_thred_, loop_times_;
    // timestamp
    std::shared_ptr<Timestamp> timestamp_;
    // store the loopsure limitation
    std::vector<sfm::loopsure_edge> loop_edges_;

protected:
    // update confidence
    float updateConfidence(float confidence, float angle_2, float distance_2);
    // remove unstable surfels from map
    bool removeUnstableSurfel();

public:
    SurfelMap(rv::ParameterList parameter_list, std::shared_ptr<Timestamp> time);
    ~SurfelMap();
    // add pose to pose list
    bool pushBackPose(pose_type pose);
    // get pose at timestamp
    pose_type getLastPose();
    // intial map and pose
    bool mapInitial(pose_type init_pose = pose_type::Identity());
    // generate global map
    bool generateMap(pcl::PointCloud<Surfel> & global_map);
    // get active map ptr
    const std::shared_ptr<VertexMap> getActiveMapPtr() {return active_map_;}
    // get initial confidence
    float getInitConfidence() {return initial_confidence_;}
    // generate active map at now timestamp
    bool generateActiveMap();
    // update map
    bool updateMap(std::shared_ptr<VertexMap> current_frame);
    bool updateMap(std::shared_ptr<VertexMap> current_frame, bool mode = false,
                   pose_type crt_pose = pose_type::Identity());
    // get point clouds at timestamp in local coordination
    std::shared_ptr<pcl::PointCloud<Surfel>> getPointCloudsInLocal(int timestamp);
    // get point clouds at timestamp in global coordination
    std::shared_ptr<pcl::PointCloud<Surfel>> getPointCloudsInGlobal(int timestamp);
    // set loop edge in factor graph
    bool setLoopsureEdge(int from, int to, pose_type pose);
    // get final point cloud index
    int getCurrentIndex() {return surfel_map_.size() - 1;}
    // reset loop times
    bool resetLoopTimes();

};


#endif //SRC_SURFELMAP_H
