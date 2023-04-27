/* SemanticSurfel
 * The definition of SemanticSurfel structure
 * We define a custom pcl::point which called surfel
 * */

#ifndef SMROS_SEMANTIC_SURFEL_H
#define SMROS_SEMANTIC_SURFEL_H

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct SemanticSurfel {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  float radius;
  float confidence;
  float curvature;
  int label;
  int u;
  int v;

  PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
}; // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    SemanticSurfel, // here we assume a XYZ + "test" (as fields)
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(float, rgb, rgb)(
        float, radius, radius)(float, confidence, confidence)(
        float, curvature, curvature)(int, label, point_type)(int, u, u)(int, v,
                                                                        v))

using PointT = SemanticSurfel;
using PointCloudT = pcl::PointCloud<PointT>;

struct Vertex {
  float radius = 0.0;
  int index = -1;
  PointT point;
};

class VertexMap {
public:
  VertexMap(int width, int height) : width_(width), height_(height) {
    // init Vertex map
    vertex_maps_.resize(width_);
    for (int i = 0; i < width_; i++) {
      Vertex vertex;
      vertex_maps_[i].resize(height_, vertex);
    }
  }

  std::vector<std::vector<Vertex>> &GetMap() { return vertex_maps_; }

  std::vector<std::vector<Vertex>> vertex_maps_;
  int height_, width_;
};

#endif // SMROS_SEMANTIC_SURFEL_H
