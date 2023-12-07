#ifndef RANGENETAPI_H_
#define RANGENETAPI_H_

#endif /* RANGENETAPI_H_ */

// c++ stuff
#include <iostream>
#include <string>

// net stuff
#include "selector.hpp"

namespace cl = rangenet::segmentation;

/** \brief A class of rangenet apis.
 *
 * \author Tongda Yang
 */

class RangenetAPI {
public:
  typedef std::tuple<u_char, u_char, u_char> color;

  RangenetAPI(){};
  /**
   * @brief      Constructs the object.
   *
   * @param[in]  model_path  The model path for the inference model directory
   */
  RangenetAPI(const std::string &model_path);

  /**
   * @brief      Infer the point cloud.
   *
   * @param[in]  scan  The scan in list in order of x, y, z, intensity
   * @param[in]  num_points  number of points
   *
   * @returns labels for each point
   */
  std::vector<int> infer(const std::vector<float> &scan,
                         const uint32_t num_points);
  /**
   * @brief      Get color mask result.
   *
   * @returns color for each point
   */
  std::vector<color> getColorMask() { return crt_color_mask_; }

  /**
   * @brief      Get label for particular point.
   *
   * @param[in]  index  The index of point
   *
   * @returns label
   */
  int getLabel(int index);

protected:
  std::unique_ptr<cl::Net> net;
  /** @brief      the label map from rangenet_lib **/
  std::vector<int> label_map_;
  /** @brief      the color map from rangenet_lib **/
  std::map<uint32_t, color> color_map_;
  std::vector<int> crt_labels_;
  std::vector<color> crt_color_mask_;
};