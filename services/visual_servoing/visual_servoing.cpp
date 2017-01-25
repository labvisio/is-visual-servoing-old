#include <armadillo>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../../msgs/controller.hpp"
#include "../../msgs/frame_converter.hpp"

using namespace arma;
using namespace std::chrono_literals;
using namespace std::chrono;
using namespace is::msg;
using namespace is::msg::camera;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
using namespace is::msg::controller;
namespace po = boost::program_options;

enum class ControllerState {
  CONSUME_IMAGE,
  REQUEST_PATTERN,
  REQUEST_3D_POINTS,
  REQUEST_POSE,
  REQUEST_CONTROL_ACTION,
  DEADLINE_EXCEEDED
};

int main(int argc, char* argv[]) {
  std::string uri;
  std::string robot;
  std::vector<std::string> cameras;
  is::msg::camera::Resolution resolution;
  is::msg::common::SamplingRate sample_rate;
  double fps;
  std::string img_type;
  double x, y;

  po::options_description description("Allowed options");
  auto&& options = description.add_options();
  options("help,", "show available options");
  options("uri,u", po::value<std::string>(&uri)->default_value("amqp://localhost"), "broker uri");
  options("cameras,c", po::value<std::vector<std::string>>(&cameras)->multitoken(), "cameras");
  options("robot,r", po::value<std::string>(&robot), "robot");
  options("height,h", po::value<unsigned int>(&resolution.height)->default_value(1288), "image height");
  options("width,w", po::value<unsigned int>(&resolution.width)->default_value(728), "image width");
  options("fps,f", po::value<double>(&fps)->default_value(5.0), "frames per second");
  options("type,t", po::value<std::string>(&img_type)->default_value("GRAY"), "image type");
  options("pose_x,x", po::value<double>(&x)->default_value(0.0), "desired pose x");
  options("pose_y,y", po::value<double>(&y)->default_value(0.0), "desired pose y");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("cameras") || !vm.count("robot")) {
    std::cout << description << std::endl;
    return 1;
  }

  auto is = is::connect(uri);
  auto client = is::make_client(is);

  sample_rate.rate = fps;
  for (auto& camera : cameras) {
    client.request(camera + ".set_sample_rate", is::msgpack(sample_rate));
    client.request(camera + ".set_resolution", is::msgpack(resolution));
    client.request(camera + ".set_image_type", is::msgpack(ImageType{img_type}));
  }

  while (client.receive_for(1s) != nullptr) {
  }

  std::vector<std::string> frames_tags;
  for (auto& camera : cameras) {
    frames_tags.push_back(is.subscribe({camera + ".frame"}));
  }
  int n_cameras = static_cast<int>(cameras.size());

  ControllerState controller_state = ControllerState::CONSUME_IMAGE;
  time_point<system_clock> deadline;
  std::vector<AmqpClient::Envelope::ptr_t> images_message(n_cameras);
  FrameConverterRequest frame_converter_request;
  frame_converter_request.z = 650.0;
  PointsWithReference points3d;
  FinalPositionRequest action;
  action.desired_pose = {{x, y}, 0.0};
  action.max_vel_x = 250.0;
  action.max_vel_y = 250.0;
  action.gain_x = 250.0 / 300.0;
  action.gain_y = 250.0 / 300.0;
  action.center_offset = 200.0;
  int n_deadlines = 0;

  while (1) {
    switch (controller_state) {
      case ControllerState::CONSUME_IMAGE: {
        for (int i = 0; i < n_cameras; ++i) {
          images_message[i] = is.consume(frames_tags[i]);
        }
        deadline = system_clock::now() + milliseconds(static_cast<int>(1000.0 / fps));
        controller_state = ControllerState::REQUEST_PATTERN;
        break;
      }

      case ControllerState::REQUEST_PATTERN: {
        std::vector<std::string> ids;
        for (int i = 0; i < n_cameras; ++i) {
          ids.push_back(client.request("circles_pattern.find", images_message[i]->Message()));
        }

        auto replies = client.receive_until(deadline, ids, is::policy::discard_others);
        auto n_replies = replies.size();

        if (n_replies == 0) {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
          break;
        }

        frame_converter_request.patterns.clear();
        for (int i = 0; i < n_cameras && n_replies; ++i) {
          auto reply = replies.find(ids[i]);

          if (reply != std::end(replies)) {
            --n_replies;
            PointsWithReference pattern = is::msgpack<PointsWithReference>(reply->second);

            if (!pattern.points.empty()) {
              pattern.reference = cameras[i];
              frame_converter_request.patterns.push_back(pattern);
            }
          }
        }

        is::logger()->info("Found {}/{} patterns.", frame_converter_request.patterns.size(), replies.size());

        if (frame_converter_request.patterns.empty()) {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        } else {
          controller_state = ControllerState::REQUEST_3D_POINTS;
        }
        break;
      }

      case ControllerState::REQUEST_3D_POINTS: {
        auto id = client.request("frame_converter.camera_to_world", is::msgpack(frame_converter_request));
        auto reply = client.receive_until(deadline, id, is::policy::discard_others);

        if (reply != nullptr) {
          points3d = is::msgpack<PointsWithReference>(reply);
          controller_state = ControllerState::REQUEST_POSE;
        } else {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        }
        break;
      }

      case ControllerState::REQUEST_POSE: {
        auto id = client.request("circles_pattern.get_pose", is::msgpack(points3d));
        auto reply = client.receive_until(deadline, id, is::policy::discard_others);

        if (reply != nullptr) {
          auto pose = is::msgpack<Pose>(reply);
          action.current_pose = pose;
          is::logger()->info("{};{};{}", pose.position.x, pose.position.y, pose.heading);
          controller_state = ControllerState::REQUEST_CONTROL_ACTION;
        } else {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        }
        break;
      }

      case ControllerState::REQUEST_CONTROL_ACTION: {
        double error = std::sqrt(std::pow(action.desired_pose.position.x - action.current_pose.position.x, 2.0) +
                                 std::pow(action.desired_pose.position.y - action.current_pose.position.y, 2.0));

        std::string id;
        if (error < 100.0) {
          id = client.request(robot + ".set_speed", is::msgpack<Speed>({0.0, 0.0}));
          return 0;
        } else {
          id = client.request("controller.final_position;" + robot + ".set_speed",
                              is::msgpack<FinalPositionRequest>(action));
          auto reply = client.receive_until(deadline, id, is::policy::discard_others);
          if (reply == nullptr) {
            controller_state = ControllerState::DEADLINE_EXCEEDED;
          } else {
            controller_state = ControllerState::CONSUME_IMAGE;
            n_deadlines = 0;
          }
        }
        break;
      }

      case ControllerState::DEADLINE_EXCEEDED: {
        is::logger()->warn("Deadline exceeded");
        controller_state = ControllerState::CONSUME_IMAGE;
        if (++n_deadlines == 5) {
          is::logger()->critical("Deadline exceeded too many times, stopping robot...");
          client.request(robot + ".set_speed", is::msgpack<Speed>({0.0, 0.0}));
          n_deadlines = 0;
        }
        break;
      }
    }
  }
  return 0;
}