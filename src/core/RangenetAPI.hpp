#ifndef RANGENETAPI_H_
#define RANGENETAPI_H_

#endif /* RANGENETAPI_H_ */

// c++ stuff
#include <iostream>
#include <string>

#include "rv/ParameterList.h"

// net stuff
#include <selector.hpp>
namespace cl = rangenet::segmentation;

/** \brief A class of rangenet apis.
 *
 * \author Xieyuanli Chen
 */

class RangenetAPI
{
public:
    typedef std::tuple< u_char, u_char, u_char> semantic_color;

    RangenetAPI(){};
    RangenetAPI(const rv::ParameterList& params);

    /** @brief      Infer logits from LiDAR scan **/
    std::vector<std::vector<float>> infer(const std::vector<float>& scan, const uint32_t num_points);

    /** @brief      Get the label map from rangenet_lib **/
    std::vector<int> getLabelMap(){return net->getLabelMap();}

    /** @brief      Get the color map from rangenet_lib **/
    std::map<uint32_t, semantic_color> getColorMap(){return net->getColorMap();}

    float getColorR() {return float(r);}
    float getColorB() {return float(b);}
    float getColorG() {return float(g);}

    bool setColorMap(int pointcloud_type);

    int getLabel(int index);

protected:
    std::unique_ptr<cl::Net> net;
    std::vector<int> label_map_;
    std::map<uint32_t, semantic_color> color_map_;
    std::uint8_t r, g, b;
};