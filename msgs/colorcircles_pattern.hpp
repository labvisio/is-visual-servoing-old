#ifndef __IS_MSG_COLORCIRCLESPATTERN_HPP__
#define __IS_MSG_COLORCIRCLESPATTERN_HPP__

#include <is/packer.hpp>
#include <is/msgs/camera.hpp>

namespace is {
namespace msg {
namespace pattern {

struct ImageAndThresholds {
  is::msg::camera::CompressedImage image;
  std::vector<int> thresholds_color_1;
  std::vector<int> thresholds_color_2;
  std::string reference;
  IS_DEFINE_MSG(image, thresholds_color_1,thresholds_color_2, reference);
};

}  // ::pattern
}  // ::msg
}  // ::is

#endif  // __IS_MSG_COLORCIRCLESPATTERN_HPP__
