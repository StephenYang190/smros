/* SemanticSurfel
 * The definition of SemanticSurfel structure
 * We define a custom pcl::point which called surfel
 * */

#ifndef SMROS_SEMANTIC_SURFEL_H
#define SMROS_SEMANTIC_SURFEL_H

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct SemanticSurfel {
    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D;
    PCL_ADD_RGB;
    float radius;
    float confidence;
    float curvature;
    int label;

    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (SemanticSurfel,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                           (float, y, y)
                                           (float, z, z)
                                           (float, normal_x, normal_x)
                                           (float, normal_y, normal_y)
                                           (float, normal_z, normal_z)
                                           (float, rgb, rgb)
                                           (float, radius, radius)
                                           (float, confidence, confidence)
                                           (float, curvature, curvature)
                                           (int, label, point_type)
)

#endif //SMROS_SEMANTIC_SURFEL_H
