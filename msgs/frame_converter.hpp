#ifndef __IS_MSG_FRAME_CONVERTER_HPP__
#define __IS_MSG_FRAME_CONVERTER_HPP__

#include <is/msgs/geometry.hpp>
#include <is/packer.hpp>

namespace is {
namespace msg {

struct FrameConverterRequest {
  std::vector<geometry::PointsWithReference> patterns;
  double z;
  IS_DEFINE_MSG(patterns, z);
};

} // ::msg
} // ::is

#endif // __IS_MSG_FRAME_CONVERTER_HPP__