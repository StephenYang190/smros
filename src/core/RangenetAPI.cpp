#include "RangenetAPI.hpp"


RangenetAPI::RangenetAPI(const rv::ParameterList& params){
    std::string model_path;
    if (params.hasParam("model_path")) {
        model_path = std::string(params["model_path"]);
    }
    else{
        std::cerr << "No model could be found!" << std::endl;
    }

    std::string backend = "tensorrt";

    // initialize a network
    net = cl::make_net(model_path, backend);
    label_map_ = getLabelMap();
    color_map_ = getColorMap();
}


std::vector<std::vector<float>> RangenetAPI::infer(const std::vector<float>& scan,
                                                          const uint32_t num_points){
    return net->infer(scan, num_points);
}

bool RangenetAPI::setColorMap(int pointcloud_type)
{
    semantic_color color = color_map_[pointcloud_type];
    r = std::uint8_t (std::get<2>(color));
    b = std::uint8_t (std::get<0>(color));
    g = std::uint8_t (std::get<1>(color));
}

int RangenetAPI::getLabel(int index)
{
    return label_map_[index];
}
