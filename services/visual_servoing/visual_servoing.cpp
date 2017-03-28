#include <armadillo>
#include <atomic>
#include <boost/program_options.hpp>
#include <chrono>
#include <condition_variable>
#include <condition_variable>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include <mutex>
#include <thread>
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
  CONTROL_ACTION,
  DEADLINE_EXCEEDED,
  NEW_CONFIGURE,
  FIRST_CONFIGURE,
  SET_DESIRED_POSE
};

int main(int argc, char* argv[]) {
  std::string uri;
  std::string name{"visual_servoing"};

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

  time_point<system_clock> deadline;
  std::vector<std::string> cameras;
  std::vector<is::QueueInfo> frames_tags;
  std::vector<AmqpClient::Envelope::ptr_t> images_message;
  int n_cameras;
  std::string robot;

  Resolution resolution = {1288, 728};
  ImageType image_type = {"gray"};
  SamplingRate sample_rate;
  sample_rate.rate = 5.0;

  VisualServoingConfigure new_configure;
  std::mutex mtx;
  std::condition_variable is_configured;
  bool reset_configure = false;

  Pose desired_pose;
  Pose current_pose;
  bool first_pose = true;
  std::atomic<bool> set_desired_pose;
  set_desired_pose.store(false);
  VisualServoingRequest goto_request;

  double max_vel_x = 250.0;
  double max_vel_y = 250.0;
  double gain_x = 250.0 / 300.0;
  double gain_y = 250.0 / 300.0;
  double center_offset = 200.0;
  double final_error = 100.0;

  FrameConverterRequest frame_converter_request;
  frame_converter_request.z = 650.0;
  PointsWithReference points3d;
  Pose image_pose;

  int n_deadlines = 0;
  bool arrived = false;

  auto is = is::connect(uri);
  auto client = is::make_client(is);
  // clang-format off
  auto thread = is::advertise(uri, name, {
    {
      "configure", [&](is::Request request) -> is::Reply {
        is::logger()->info("Configuration received.");
        mtx.lock();
        new_configure = is::msgpack<VisualServoingConfigure>(request);
        reset_configure = true;
        mtx.unlock();
        is_configured.notify_one();
        return is::msgpack(status::ok);
      }
    },
    {
      "go_to", [&](is::Request request) -> is::Reply {
        mtx.lock();
        goto_request = is::msgpack<VisualServoingRequest>(request);
        set_desired_pose.store(true);
        mtx.unlock();
        return is::msgpack(status::ok);
      }
    }
  });
  // clang-format on

  ControllerState controller_state = ControllerState::FIRST_CONFIGURE;

  while (1) {
    switch (controller_state) {
      case ControllerState::CONSUME_IMAGE: {
        for (int i = 0; i < n_cameras; ++i) {
          images_message[i] = is.consume(frames_tags[i]);
        }
        deadline = system_clock::now() + milliseconds(static_cast<int>(1000.0 / (sample_rate.rate).get()));

        if (set_desired_pose.load() && !first_pose) {
          controller_state = ControllerState::SET_DESIRED_POSE;
        } else {
          controller_state = ControllerState::REQUEST_PATTERN;
        }
        break;
      }

      case ControllerState::SET_DESIRED_POSE: {
        FrameConverterRequest request;
        PointsWithReference points;
        request.z = 0.0;
        mtx.lock();
        points.points.push_back(goto_request.point);
        points.reference = goto_request.reference;
        mtx.unlock();
        request.patterns.push_back(points);

        auto id = client.request("frame_converter.camera_to_world", is::msgpack(request));
        auto reply = client.receive_until(deadline, id, is::policy::discard_others);

        if (reply != nullptr) {
          points3d = is::msgpack<PointsWithReference>(reply);
          if (points3d.points.size() == 1) {
            desired_pose = {{points3d.points[0].x, points3d.points[0].y}, 0.0};
            is::logger()->info("Setting new desired pose: {},{}", desired_pose.position.x, desired_pose.position.y);
            controller_state = ControllerState::REQUEST_PATTERN;
          } else {
            controller_state = ControllerState::DEADLINE_EXCEEDED;
          }
        } else {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        }

        set_desired_pose.store(false);
        arrived = false;
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
          current_pose = pose;
          is::logger()->info("Robot pose: {};{};{}", pose.position.x, pose.position.y, pose.heading);

          if (first_pose) {
            is::logger()->info("Setting first pose");
            desired_pose = pose;
            arrived = true;
            first_pose = false;
          }
          controller_state = ControllerState::CONTROL_ACTION;

        } else {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        }
        break;
      }

      case ControllerState::CONTROL_ACTION: {
        if (arrived) {
          controller_state = ControllerState::NEW_CONFIGURE;
          break;
        }

        double error = std::sqrt(std::pow(desired_pose.position.x - current_pose.position.x, 2.0) +
                                 std::pow(desired_pose.position.y - current_pose.position.y, 2.0));

        Speed speed;
        if (error < final_error) {
          speed = {0.0, 0.0};
          arrived = true;
        } else {
          // Final position controller
          double x_til = desired_pose.position.x - current_pose.position.x;
          double y_til = desired_pose.position.y - current_pose.position.y;

          mat invA(2, 2);
          invA.at(0, 0) = cos(current_pose.heading);
          invA.at(0, 1) = sin(current_pose.heading);
          invA.at(1, 0) = -(1.0 / center_offset) * sin(current_pose.heading);
          invA.at(1, 1) = +(1.0 / center_offset) * cos(current_pose.heading);

          mat C(2, 1);
          C.at(0, 0) = max_vel_x * tanh((gain_x / max_vel_x) * x_til);
          C.at(1, 0) = max_vel_y * tanh((gain_y / max_vel_y) * y_til);

          mat vels_vec = invA * C;

          speed = {vels_vec.at(0), vels_vec.at(1)};
        }

        auto id = client.request(robot + ".set_speed", is::msgpack(speed));
        auto reply = client.receive_until(deadline, id, is::policy::discard_others);

        if (reply == nullptr) {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        } else {
          controller_state = ControllerState::NEW_CONFIGURE;
          n_deadlines = 0;
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

      case ControllerState::NEW_CONFIGURE: {
        std::unique_lock<std::mutex> lk(mtx);
        if (reset_configure) {
          is::logger()->info("Setting new configuration.");
          // clang-format off
          if (new_configure.resolution) { resolution = (new_configure.resolution).get(); }
          if (new_configure.sample_rate) { sample_rate = (new_configure.sample_rate).get(); }
          if (new_configure.image_type) { image_type = (new_configure.image_type).get(); }
          if (new_configure.max_vel_x) { max_vel_x = (new_configure.max_vel_x).get(); }
          if (new_configure.max_vel_y) { max_vel_y = (new_configure.max_vel_y).get(); }
          if (new_configure.gain_x) { gain_x = (new_configure.gain_x).get(); }
          if (new_configure.gain_y) { gain_y = (new_configure.gain_y).get(); }
          if (new_configure.center_offset) { center_offset = (new_configure.center_offset).get(); }
          if (new_configure.final_error) { final_error = (new_configure.final_error).get(); }
          cameras = new_configure.cameras;
          robot = new_configure.robot;
          // clang-format on

          n_cameras = cameras.size();

          for (auto& camera : cameras) {
            client.request(camera + ".set_sample_rate", is::msgpack(sample_rate));
            client.request(camera + ".set_resolution", is::msgpack(resolution));
            client.request(camera + ".set_image_type", is::msgpack(image_type));
          }
          client.receive_for(1s);

          frames_tags.clear();
          for (auto& camera : cameras) {
            frames_tags.push_back(is.subscribe({camera + ".frame"}));
          }
          images_message.resize(n_cameras);

          reset_configure = false;
        }

        controller_state = ControllerState::CONSUME_IMAGE;
        break;
      }

      case ControllerState::FIRST_CONFIGURE: {
        std::unique_lock<std::mutex> lk(mtx);
        is_configured.wait(lk, [&] { return reset_configure; });
        is::logger()->info("Setting first configuration.");
        controller_state = ControllerState::NEW_CONFIGURE;
        break;
      }
    }
  }

  thread.join();
  return 0;
}