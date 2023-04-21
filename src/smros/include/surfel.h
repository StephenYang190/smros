/* Surfel
 * The definition of Surfel structure
 * We define a custom pcl::point which called surfel
 * */

#ifndef SMROS_SURFEL_H
#define SMROS_SURFEL_H

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct Surfel
{
    PCL_ADD_POINT4D;
    float nx, ny, nz;
    float r, g, b;
    float radius; // radius
    float confidence;
    int point_type;

    int create_timestamp;
    int update_timestamp;
    PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (Surfel,           // here we assume a XYZ + "test" (as fields)
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, r, r)
        (float, g, g)
        (float, b, b)
        (float, nx, nx)
        (float, ny, ny)
        (float, nz, nz)
        (float, radius, radius)
        (float, confidence, confidence)
        (int, point_type, point_type)
        (int, create_timestamp, create_timestamp)
        (int, update_timestamp, update_timestamp)
)

#endif //SMROS_SURFEL_H
