#include "circles_pattern.hpp"
#include <boost/program_options.hpp>
#include <is/is.hpp>

namespace po = boost::program_options;
using namespace is::msg::common;
using namespace is::msg::camera;
using namespace is::msg::robot;
using namespace is::msg::geometry;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string name{"circles_pattern"};

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

  is::Connection is(is::connect(uri));

  // clang-format off
  auto thread = is::advertise(uri, name, {
    {
      "find", [&](is::Request request) -> is::Reply {
        auto image = is::msgpack<CompressedImage>(request);
        return is::msgpack<PointsWithReference>(find_pattern(image));
      }
    },
    {
      "get_pose", [&](is::Request request) -> is::Reply {
        auto points = is::msgpack<PointsWithReference>(request);
        return is::msgpack(get_pose(points));
      }
    }
  });
  // clang-format on

  thread.join();
  return 0;
}