#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/program_options.hpp>
#include <cmath>
#include <chrono>
#include <iostream>
#include <tuple>
#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/robot.hpp>
#include "../../msgs/controller.hpp"

using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::controller;
using namespace std::chrono_literals;
namespace po = boost::program_options;

constexpr double pi() {
  return std::atan(1) * 4;
}
auto deg2rad = [](double deg) { return deg * (pi() / 180.0); };
auto rad2deg = [](double rad) { return rad * (180.0 / pi()); };

int main(int argc, char* argv[]) {
  std::string uri;
  std::string robot0, robot1;
  int64_t period;
  std::string desired_pose_str;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");
  options("robot0,0", po::value<std::string>(&robot0)->default_value("robot.0"), "robot 0");
  options("robot1,1", po::value<std::string>(&robot1)->default_value("robot.1"), "robot 1");
  options("period,t", po::value<int64_t>(&period)->default_value(100), "sampling period in ms");
  options("desired_pose,p", po::value<std::string>(&desired_pose_str)->default_value("2000.0,2000.0,45.0,1000.0"),
          "desired pose (x[mm],y[mm],th[deg],p[mm]) e.g.: 0.0,1000.0,30.0,1000.0");
      
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 1;
  }

  FormationRequest formation_req;
 
  std::vector<std::string> fields;
  boost::split(fields, desired_pose_str, boost::is_any_of(","), boost::token_compress_on);
  if (fields.size() == 4) {
    formation_req.desired_pose_formation = { .position = {stod(fields[0]), stod(fields[1])}, .heading = deg2rad(stod(fields[2]))};
    formation_req.desired_distance_formation = stod(fields[3]);
  }

  is::logger()->info("[desired pose] {},{},{},{}", formation_req.desired_pose_formation.position.x,
                                                   formation_req.desired_pose_formation.position.y, 
                                                   formation_req.desired_pose_formation.heading,
                                                   formation_req.desired_distance_formation );

  formation_req.gain = 0.48;
  formation_req.max_vel = 120; // [mm/s]
  formation_req.center_offset = 50.0; // [mm]

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  if (vm.count("period")) {
    SamplingRate sample_rate;
    sample_rate.period = period;
    client.request(robot0 + ".set_sample_rate", is::msgpack(sample_rate));
    client.request(robot1 + ".set_sample_rate", is::msgpack(sample_rate));
  }

  client.request(robot0 + ".set_pose", is::msgpack<Pose>({{0.0, 0.0}, 0.0}));
  client.request(robot1 + ".set_pose", is::msgpack<Pose>({{1000.0, 1000.0}, 0.0}));

  while (client.receive_for(1s) != nullptr) { }

  auto pose0_tag = is.subscribe({robot0 + ".pose"});
  auto pose1_tag = is.subscribe({robot1 + ".pose"});

  while (1) {
    auto poses_msg = is.consume_sync({pose0_tag, pose1_tag}, period);
    auto pose0 = is::msgpack<Pose>(poses_msg[0]);
    auto pose1 = is::msgpack<Pose>(poses_msg[1]);

    is::logger()->info("[{}] {}, {}, {} | [{}] {}, {}, {}", robot0, pose0.position.x, pose0.position.y, pose0.heading, 
                                                            robot1, pose1.position.x, pose1.position.y, pose1.heading );

    formation_req.current_pose_robot_0 = pose0;
    formation_req.current_pose_robot_1 = pose1;

    auto id = client.request("formation_controller.compute", is::msgpack(formation_req));
    auto control_msg = client.receive_for(50ms, id, is::policy::discard_others);
    auto velocities = is::msgpack<std::pair<Speed,Speed>>(control_msg);
    
    is::logger()->info("{} {} {} {}", velocities.first.linear, velocities.first.angular, velocities.second.linear, velocities.second.angular);

    client.request(robot0 + ".set_speed", is::msgpack(velocities.first));
    client.request(robot1 + ".set_speed", is::msgpack(velocities.second));

    while (client.receive_for(50ms) != nullptr) { }
  }
  return 0;
}