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

mat f(mat const& x) {
  mat q(4, 1);
  q(0, 0) = (x(0, 0) + x(2, 0)) / 2.0;                                        // x_f = (x1+x2)/2
  q(1, 0) = (x(1, 0) + x(3, 0)) / 2.0;                                        // y_f = (y1+y2)/2
  q(2, 0) = sqrt(pow(x(2, 0) - x(0, 0), 2.0) + pow(x(1, 0) - x(3, 0), 2.0));  // rho_f = sqrt((x2-x1)^2 + (y2-y1)^2)
  q(3, 0) = atan2(x(3, 0) - x(1, 0), x(2, 0) - x(0, 0));                      // alpha_f = atan((y2-y1)/(x2-x1))

  return q;
}

mat invJ(mat const& q) {
  mat invJ = zeros<mat>(4, 4);
  invJ(0,0) = 1.0;
  invJ(1,1) = 1.0;
  invJ(2,0) = 1.0;
  invJ(3,1) = 1.0;

  invJ(0, 2) = -cos(q(3, 0)) / 2.0;            // J^-1(1,3)= -cos(alpha_f)/2
  invJ(0, 3) = q(2, 0) * sin(q(3, 0)) / 2.0;   // J^-1(1,4)= rho_f*sin(alpha_f)/2
  invJ(1, 2) = -sin(q(3, 0)) / 2.0;            // J^-1(2,3)= -sin(alpha_f)/2
  invJ(1, 3) = -q(2, 0) * cos(q(3, 0)) / 2.0;  // J^-1(2,4)= -rho_f*cos(alpha_f)/2

  invJ(2, 2) = -invJ(0, 2);  // J^-1(3,3)= cos(alpha_f)/2
  invJ(2, 3) = -invJ(0, 3);  // J^-1(3,4)= -rho_f*sin(alpha_f)/2
  invJ(3, 2) = -invJ(1, 2);  // J^-1(4,3)= sin(alpha_f)/2
  invJ(3, 3) = -invJ(1, 3);  // J^-1(4,4)= rho_f*cos(alpha_f)/2

  return invJ;
}

mat invK(double const& psi1, double const& psi2, double const& a) {
  mat invK = zeros<mat>(4, 4);
  invK(0, 0) = cos(psi1);
  invK(0, 1) = sin(psi1);
  invK(1, 0) = -(1.0 / a) * sin(psi1);
  invK(1, 1) = (1.0 / a) * cos(psi1);

  invK(2, 2) = cos(psi2);
  invK(2, 3) = sin(psi2);
  invK(3, 2) = -(1.0 / a) * sin(psi2);
  invK(3, 3) = (1.0 / a) * cos(psi2);

  return invK;
}

int main(int argc, char* argv[]) {
  std::string uri;
  std::string name{"formation_controller"};

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
        "compute", [&](is::Request request) -> is::Reply {
          FormationRequest req = is::msgpack<FormationRequest>(request);

          mat L = req.max_vel*eye<mat>(4,4);
          mat InvL = (1.0/req.max_vel)*eye<mat>(4,4);
          mat kp = req.gain*eye<mat>(4,4);

          double psi1 = req.current_pose_robot_0.heading;
          double psi2 = req.current_pose_robot_1.heading;
          double a  = req.center_offset;

          mat x(4,1); // x = [x1 y1 x2 y2]'
          x(0,0) = req.current_pose_robot_0.position.x;
          x(1,0) = req.current_pose_robot_0.position.y;
          x(2,0) = req.current_pose_robot_1.position.x;
          x(3,0) = req.current_pose_robot_1.position.y;
                    
          mat q_des(4,1); // q_des = [x_f y_f rho_f alpha_f];
          q_des(0,0) = req.desired_pose_formation.position.x;
          q_des(1,0) = req.desired_pose_formation.position.y;
          q_des(2,0) = req.desired_distance_formation;
          q_des(3,0) = req.desired_pose_formation.heading;

          mat q = f(x);
          mat q_til = q_des - q;
          mat q_diff1_ref = L*tanh(InvL*kp*q_til);
          mat x_diff1_ref = invJ(q)*q_diff1_ref;
          mat v_ref = invK(psi1, psi2, a)*x_diff1_ref;

          std::pair<Speed,Speed> speeds({v_ref(0,0), v_ref(1,0)}, 
                                        {v_ref(2,0), v_ref(3,0)});
          return is::msgpack(speeds);
        }
      }
   });
  // clang-format on

  thread.join();
  return 0;
}