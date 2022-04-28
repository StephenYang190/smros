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
#include <fstream>
#include <nav_msgs/Path.h>

#include "vertexmap.h"
#include "backendopt.h"
#include "timestamp.h"

// define the pose type as matrix4f
using pose_type = Eigen::Matrix4f;
//namespace sfm{
//    class loopsure_edge{
//    public:
//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//        int from = -1;
//        int to = -1;
//        pose_type pose;
//
////        loopsure_edge() = default;
//
//        loopsure_edge() : from(-1), to(-1), pose(Eigen::Matrix4f::Identity()){};
////            pose.setIdentity();
////            pose << 1,0,0,0,
////            0,1,0,0,
////            0,0,1,0,
////            0,0,0,1;
////        }
//    };
//}

class SurfelMap {
private:
    // store pose frame by frame
    std::vector<pose_type, Eigen::aligned_allocator<pose_type>> global_poses_;
    std::vector<pose_type, Eigen::aligned_allocator<pose_type>> local_poses_;
    std::vector<pose_type, Eigen::aligned_allocator<pose_type>> loopsure_poses_;
    int from_;
    int to_;
    // store point clouds frame by frame
    std::vector<pcl::PointCloud<Surfel>::Ptr> surfel_map_;
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
    // ros nodehandle
    ros::NodeHandle nh_;
    // ros publisher
    ros::Publisher pub_;
    // pose graph
    BackEndOpt pose_graph_;
    // information metrix
    Eigen::DiagonalMatrix<double, 6> info_;
    // parameter used to control the time to optimization
    int loop_thred_;
    // parameter used to mark loopsure
    bool have_loop_;
    // timestamp
    int timestamp_;
    // store the loopsure edge
//    std::shared_ptr<sfm::loopsure_edge> loop_edges_;
    // path to save pose
    std::string pose_out_path_;
    // navigation path message
    nav_msgs::Path path_s;

protected:
    // update confidence
    float updateConfidence(float confidence, float angle_2, float distance_2);
    // remove unstable surfels from map
    bool removeUnstableSurfel();

public:
    SurfelMap();
    // add pose to pose list
    bool pushBackPose(pose_type& pose);
    // get pose at timestamp
    pose_type getLastPose();
    // intial map and pose
    bool mapInitial(pose_type& init_pose);
    // generate global map
    bool generateMap(pcl::PointCloud<Surfel> & global_map);
    bool generateMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map);
    // get active map ptr
    std::shared_ptr<VertexMap> getActiveMapPtr();
    // get initial confidence
    float getInitConfidence();
    // generate active map at now timestamp
    bool generateActiveMap();
    // update map
    bool updateMap(std::shared_ptr<VertexMap> current_frame,
                   pose_type crt_pose = pose_type::Identity());
    // get point clouds at id in local coordination
    pcl::PointCloud<Surfel>::Ptr getPointCloudsInLocal(int id);
    // get point clouds at id in global coordination
    pcl::PointCloud<Surfel>::Ptr getPointCloudsInGlobal(int id);
    // set loop edge in factor graph
    bool setLoopsureEdge(int from, int to, pose_type& pose);
    // get final point cloud index
    int getCurrentIndex();
    // reset loop times
    bool resetLoopTimes();
    // sae pose to file
    bool savePose2File(std::string suffix);
    // get timestamp
    int getTimestamp();
    // ros path
    bool rospath();
};


#endif //SRC_SURFELMAP_H
