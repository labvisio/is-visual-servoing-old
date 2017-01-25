#include <armadillo>
#include <boost/program_options.hpp>
#include <cmath>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/robot.hpp>
#include "../../msgs/controller.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace arma;
using namespace is::msg::robot;
using namespace is::msg::controller;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string name{"controller"};

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 1;
  }

  auto is = is::connect(uri);

  // clang-format off
  auto thread = is::advertise(uri, name, {
      {
        "final_position", [&](is::Request request) -> is::Reply {
          FinalPositionRequest req = is::msgpack<FinalPositionRequest>(request);
          double x_til = req.desired_pose.position.x - req.current_pose.position.x;
          double y_til = req.desired_pose.position.y - req.current_pose.position.y;

          mat invA(2, 2);
          invA.at(0, 0) = cos(req.current_pose.heading);
          invA.at(0, 1) = sin(req.current_pose.heading);
          invA.at(1, 0) = -(1.0 / req.center_offset) * sin(req.current_pose.heading);
          invA.at(1, 1) = (1.0 / req.center_offset) * cos(req.current_pose.heading);

          mat C(2, 1);
          C.at(0, 0) = req.max_vel_x * tanh((req.gain_x / req.max_vel_x) * x_til);
          C.at(1, 0) = req.max_vel_y * tanh((req.gain_y / req.max_vel_y) * y_til);

          mat vels_vec = invA * C;

          Speed speed{vels_vec.at(0), vels_vec.at(1)};
          return is::msgpack(speed);
        }
      }
   });
  // clang-format on

  thread.join();
  return 0;
}