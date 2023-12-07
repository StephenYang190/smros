#include "RangenetAPI.hpp"

RangenetAPI::RangenetAPI(const std::string &model_path) {
  if (model_path.empty()) {
    std::cerr << "No model could be found!" << std::endl;
  }

  std::string backend = "tensorrt";

  // initialize a network
  net = cl::make_net(model_path, backend);
  label_map_ = net->getLabelMap();
  color_map_ = net->getColorMap();
}

std::vector<int> RangenetAPI::infer(const std::vector<float> &scan,
                                    const uint32_t num_points) {
  crt_labels_.resize(num_points);
  crt_color_mask_.resize(num_points);
  auto semantic_result = net->infer(scan, num_points);
  for (int i = 0; i < num_points; i++) {
    float prob = 0;
    int label_index = 0;
    for (int k = 0; k < 20; k++) {
      if (prob <= semantic_result[i][k]) {
        prob = semantic_result[i][k];
        label_index = k;
      }
    }
    crt_labels_[i] = label_map_[label_index];
    crt_color_mask_[i] = color_map_[crt_labels_[i]];
  }

  return crt_labels_;
}

int RangenetAPI::getLabel(int index) { return crt_labels_[index]; }
