#include <boost/program_options.hpp>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "msgs/controller.hpp"
#include "msgs/frame_converter.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace is::msg;
using namespace is::msg::camera;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
using namespace is::msg::controller;
namespace po = boost::program_options;

struct callback_handle {
  Point point;
  bool request;
  std::string reference;
  Resolution resolution;
};

auto mouse_callback = [](int event, int x, int y, int, void* userdata) {
  callback_handle* handle = (callback_handle*)userdata;
  if (event == cv::EVENT_LBUTTONUP) {
    handle->point.x = x;
    handle->point.y = y;
    int x_div = handle->point.x / (handle->resolution.width / 2);
    int y_div = handle->point.y / (handle->resolution.height / 2);
    if (x_div == 0) {
      if (y_div == 0) {
        handle->reference = "ptgrey.0";
      } else {
        handle->reference = "ptgrey.2";
      }
    } else {
      if (y_div == 0) {
        handle->reference = "ptgrey.1";
      } else {
        handle->reference = "ptgrey.3";
      }
    }
    handle->point.x = 2 * (static_cast<int>(handle->point.x) % (handle->resolution.width / 2));
    handle->point.y = 2 * (static_cast<int>(handle->point.y) % (handle->resolution.height / 2));
    handle->request = true;
  }
};

int main(int argc, char* argv[]) {
  std::string uri;
  std::string robot;
  std::vector<std::string> cameras;
  is::msg::camera::Resolution resolution;
  double fps;
  std::string img_type;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");
  options("cameras,c", po::value<std::vector<std::string>>(&cameras)->multitoken(), "cameras");
  options("robot,r", po::value<std::string>(&robot), "robot");
	options("height,h", po::value<unsigned int>(&resolution.height)->default_value(728), "image height");
  options("width,w", po::value<unsigned int>(&resolution.width)->default_value(1288), "image width");
  options("fps,f", po::value<double>(&fps), "frames per second");
  options("type,t", po::value<std::string>(&img_type), "image type");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("cameras") || !vm.count("robot")) {
    std::cout << description << std::endl;
    return 1;
  }

  VisualServoingConfigure configure;
  configure.cameras = cameras;
  configure.robot = robot;
  if (vm.count("height") && vm.count("width"))
    configure.resolution = resolution;
  if (vm.count("fps")) {
		SamplingRate sample_rate;
		sample_rate.rate = fps;
    configure.sample_rate = sample_rate;
	}
  if (vm.count("type"))
    configure.image_type = ImageType{img_type};

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  client.request("visual_servoing.configure", is::msgpack(configure));
  client.receive_for(10ms);

  std::vector<std::string> frames_tags;
  for (auto& camera : cameras) {
    frames_tags.push_back(is.subscribe({camera + ".frame"}));
  }

  int n_cameras = cameras.size();
  std::vector<AmqpClient::Envelope::ptr_t> images_message(n_cameras);

  callback_handle handle;
  handle.resolution = resolution;
  handle.request = false;

  cv::namedWindow("Visual Servoring");
  cv::setMouseCallback("Visual Servoring", mouse_callback, &handle);

  while (1) {
    for (int i = 0; i < n_cameras; ++i) {
      images_message[i] = is.consume(frames_tags[i]);
    }

    std::vector<cv::Mat> up_frames, down_frames;
    int n_frame = 0;
    for (auto& msg : images_message) {
      auto image = is::msgpack<CompressedImage>(msg);
      cv::Mat current_frame = cv::imdecode(image.data, CV_LOAD_IMAGE_COLOR);
      cv::resize(current_frame, current_frame, cv::Size(current_frame.cols / 2, current_frame.rows / 2));
      if (n_frame < 2) {
        up_frames.push_back(current_frame);
      } else {
        down_frames.push_back(current_frame);
      }
      n_frame++;
    }

    cv::Mat output_image;
    cv::Mat up_row, down_row;
    std::vector<cv::Mat> rows_frames;
    cv::hconcat(up_frames, up_row);
    rows_frames.push_back(up_row);
    cv::hconcat(down_frames, down_row);
    rows_frames.push_back(down_row);
    cv::vconcat(rows_frames, output_image);

    cv::imshow("Visual Servoring", output_image);
    cv::waitKey(1);

    if (handle.request) {
      is::logger()->info("Mouse clicked [{}][{},{}]", handle.reference, handle.point.x, handle.point.y);

      VisualServoingRequest request;
      request.point = handle.point;
      request.reference = handle.reference;
      auto req_id = client.request("visual_servoing.go_to", is::msgpack(request));
      client.receive_for(10ms, req_id, is::policy::discard_others);
      handle.request = false;
    }
  }
  return 0;
}