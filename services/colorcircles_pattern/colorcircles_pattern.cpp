#include "../../msgs/colorcircles_pattern.hpp"
#include <boost/program_options.hpp>
#include <iostream>
#include <is/is.hpp>
#include "colorcircles_pattern.hpp"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
  std::string uri;
  std::string name{"colorcircles_pattern"};

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("guest:guest@localhost"), "broker uri");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("uri")) {
    std::cout << description << std::endl;
    return 1;
  }

  // clang-format off
	auto thread = is::advertise(uri, name, {
    {
      "find", [&](is::Request request) -> is::Reply {
				auto find_request =	is::msgpack<is::msg::pattern::ImageAndThresholds>(request);
  			auto compressed_image = find_request.image;
  			auto thresholds_color_1 = find_request.thresholds_color_1;
  			auto thresholds_color_2 = find_request.thresholds_color_2;
  			cv::Mat image = cv::imdecode(compressed_image.data, CV_LOAD_IMAGE_COLOR);

  			auto pattern = find_pattern(image, thresholds_color_1, thresholds_color_2);
				pattern.reference = find_request.reference;
				auto msg = is::msgpack<PointsWithReference>(pattern);
  			return msg;
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
