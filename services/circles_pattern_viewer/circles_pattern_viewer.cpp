#include <armadillo>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "../../msgs/frame_converter.hpp"

using namespace arma;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace is::msg;
using namespace is::msg::camera;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
namespace po = boost::program_options;

constexpr double pi() {
  return std::atan(1) * 4;
}
auto deg2rad = [](double deg) { return deg * (pi() / 180.0); };
auto rad2deg = [](double rad) { return rad * (180.0 / pi()); };

int main(int argc, char* argv[]) {
  std::string uri;
  std::string entity;
  is::msg::camera::Resolution resolution;
  is::msg::common::SamplingRate sample_rate;
  double fps;
  std::string img_type;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");
  options("entity,e", po::value<std::string>(&entity), "entity name");
  options("width,w", po::value<unsigned int>(&resolution.width)->default_value(1288), "image width");
  options("height,h", po::value<unsigned int>(&resolution.height)->default_value(728), "image height");
  options("fps,f", po::value<double>(&fps)->default_value(10.0), "frames per second");
  options("type,t", po::value<std::string>(&img_type)->default_value("gray"), "image type");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("entity")) {
    std::cout << description << std::endl;
    return 1;
  }

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  sample_rate.rate = fps;
  client.request(entity + ".set_sample_rate", is::msgpack(sample_rate));
  client.request(entity + ".set_resolution", is::msgpack(resolution));
  client.request(entity + ".set_image_type", is::msgpack(ImageType{img_type}));

  while (client.receive_for(1s) != nullptr) {
  }

  auto frames = is.subscribe({entity + ".frame"});

  while (1) {
    cv::Mat frame;
    Pose pose;

    auto image_message = is.consume(frames);
    auto image = is::msgpack<is::msg::camera::CompressedImage>(image_message);
    frame = cv::imdecode(image.data, CV_LOAD_IMAGE_COLOR);

    auto deadline = system_clock::now() + milliseconds(static_cast<int>(1000.0 / fps));

    auto req_id = client.request("circles_pattern.find", image_message->Message());
    auto reply = client.receive_until(deadline, req_id, is::policy::discard_others);

    if (reply != nullptr) {
      PointsWithReference pattern = is::msgpack<PointsWithReference>(reply);
      pattern.reference = entity;

      if (!pattern.points.empty()) {
        FrameConverterRequest fc_request{{pattern}, 650.0};
        req_id = client.request("frame_converter.camera_to_world;circles_pattern.get_pose", is::msgpack(fc_request));
        reply = client.receive_until(deadline, req_id, is::policy::discard_others);
        if (reply != nullptr) {
          pose = is::msgpack<Pose>(reply);
          cv::putText(frame, std::to_string(pose.position.x) + ";" + std::to_string(pose.position.y) + ";" +
                                 std::to_string(rad2deg(pose.heading)),
                      cv::Point2f(50, 100), 1, 4, cv::Scalar(0, 255, 0), 6);
        }
      }

      for (auto& p : pattern.points) {
        cv::circle(frame, cv::Point2d(p.x, p.y), 3, cv::Scalar(0, 255, 0), 3);
      }
    }
    cv::imshow("Pattern", frame);
    cv::waitKey(1);
  }
  return 0;
}