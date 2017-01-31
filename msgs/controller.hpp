#ifndef __IS_MSG_CONTROLLER_HPP__
#define __IS_MSG_CONTROLLER_HPP__

#include <is/msgs/robot.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/common.hpp>
#include <is/packer.hpp>

namespace is {
namespace msg {
namespace controller {

using namespace is::msg::robot;
using namespace is::msg::camera;
using namespace is::msg::geometry;
using namespace is::msg::common;
using namespace boost;

struct FinalPositionRequest {
	Pose current_pose;
  Pose desired_pose;

  double gain_x;
  double gain_y;
  double max_vel_x;
  double max_vel_y;
  double center_offset;

  IS_DEFINE_MSG(current_pose, desired_pose, gain_x, gain_y, max_vel_x, max_vel_y, center_offset);
};

struct VisualServoingRequest {
  Point point;
  std::string reference;
  IS_DEFINE_MSG(point, reference);
};

struct VisualServoingConfigure {
  std::vector<std::string> cameras;
  std::string robot;
  // Cameras parameters
  optional<Resolution> resolution = none;
  optional<SamplingRate> sample_rate = none;
  optional<ImageType> image_type = none;
  // Final position controller parameters 
  optional<double> max_vel_x = none;
  optional<double> max_vel_y = none;
  optional<double> gain_x = none;
  optional<double> gain_y = none;
  optional<double> center_offset = none;
  optional<double> final_error = none;
  
  IS_DEFINE_MSG(cameras, resolution, sample_rate, image_type, robot, max_vel_x, max_vel_y, gain_x, gain_y, center_offset);
};

}  // ::controller
}  // ::msg
}  // ::is

#endif  // __IS_MSG_CONTROLLER_HPP__