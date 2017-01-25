#ifndef __IS_MSG_CONTROLLER_HPP__
#define __IS_MSG_CONTROLLER_HPP__

#include <is/msgs/robot.hpp>
#include <is/packer.hpp>

namespace is {
namespace msg {
namespace controller {

using namespace is::msg::robot;

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

}  // ::controller
}  // ::msg
}  // ::is

#endif  // __IS_MSG_CONTROLLER_HPP__