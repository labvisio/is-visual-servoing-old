#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/program_options.hpp>
#include <cmath>
#include <chrono>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/robot.hpp>

using namespace is::msg::robot;
using namespace is::msg::common;
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
  options("desired_pose,p", po::value<std::string>(&desired_pose_str)->default_value("1000.0,1000.0,45.0,1000.0"),
          "desired pose (x[mm],y[mm],th[deg],p[mm]) e.g.: 0.0,1000.0,30.0,1000.0");
  /*
  options("entity,e", po::value<std::string>(&entity)->default_value("pioneer.0"), "gateway name");
  options("linvel,v", po::value<double>(&v)->default_value(100.0), "linear velocity in mm/s");
  
  
  options("linvel,v", po::value<double>(&v)->default_value(100.0), "linear velocity in mm/s");
  options("rotvel,w", po::value<double>(&w)->default_value(5.0), "rotational velocity in deg/s");
  options("initial_pose,p", po::value<std::string>(&pose_str)->default_value("0.0,0.0,0.0"),
          "initial pose (x[mm],y[mm],th[deg]) e.g.: 0.0,100.0,30");

  */
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("entity")) {
    std::cout << description << std::endl;
    return 1;
  }

/*
  auto is = is::connect(uri);
  auto client = is::make_client(is);

  if (vm.count("initial_pose")) {
    std::vector<std::string> fields;
    boost::split(fields, pose_str, boost::is_any_of(","), boost::token_compress_on);
    if (fields.size() == 3) {
      Pose pose{.position = {stod(fields[0]), stod(fields[1])}, .heading = deg2rad(stod(fields[2]))};
      client.request(entity + ".set_pose", is::msgpack(pose));
    }
  }

  if (vm.count("linvel") && vm.count("rotvel")) {
    Speed speed{v, deg2rad(w)};
    client.request(entity + ".set_speed", is::msgpack(speed));
  }

  if (vm.count("period")) {
    SamplingRate sample_rate;
    sample_rate.period = period;
    client.request(entity + ".set_sample_rate", is::msgpack(sample_rate));
  }

  while (client.receive_for(1s) != nullptr) {
  }

  auto poses = is.subscribe({entity + ".pose"});
  auto timestamps = is.subscribe({entity + ".timestamp"});

  int i = 10;
  while (1) {
    if (i++ % 10 == 0) {
      is::logger()->info("[ns] [mm], [mm], [deg]");
    }

    auto pose_msg = is.consume(poses);
    auto timestamp_msg = is.consume(timestamps);

    auto pose = is::msgpack<Pose>(pose_msg);
    auto timestamp = is::msgpack<Timestamp>(timestamp_msg);
    
    is::logger()->info("[{}] {}, {}, {}", timestamp.nanoseconds, pose.position.x, pose.position.y,
                       rad2deg(pose.heading));
  }

*/
  return 0;
}