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

struct FormationRequest {
  Pose current_pose_robot_1;
  Pose current_pose_robot_2;
  
  Pose desired_pose_formation;
  double desired_distance_formation;

  double gain;
  double max_vel;
  double center_offset;

  IS_DEFINE_MSG(current_pose_robot_1, current_pose_robot_2, desired_pose_formation, desired_distance_formation, 
                gain, max_vel, center_offset);
};

}  // ::controller
}  // ::msg
}  // ::is

#endif  // __IS_MSG_CONTROLLER_HPP__