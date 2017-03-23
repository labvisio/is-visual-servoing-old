// https://youtu.be/4z_spuaPWSo
#include <armadillo>
#include <atomic>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <is/is.hpp>
#include <is/msgs/camera.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include "../../msgs/controller.hpp"
#include "../../msgs/frame_converter.hpp"
#include "../../msgs/colorcircles_pattern.hpp"

using namespace arma;
using namespace std::chrono_literals;
using namespace std::chrono;
using namespace is::msg;
using namespace is::msg::camera;
using namespace is::msg::robot;
using namespace is::msg::common;
using namespace is::msg::geometry;
using namespace is::msg::controller;
using namespace is::msg::pattern;
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
  options("height,h", po::value<unsigned int>(&resolution.height)->default_value(728), "image height");
  options("width,w", po::value<unsigned int>(&resolution.width)->default_value(1288), "image width");
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

  std::vector<is::QueueInfo> frames_tags;
  for (auto& camera : cameras) {
    frames_tags.push_back(is.subscribe({camera + ".frame"}));
  }
  int n_cameras = static_cast<int>(cameras.size());

  ControllerState controller_state = ControllerState::CONSUME_IMAGE;
  time_point<system_clock> deadline;
  std::vector<AmqpClient::Envelope::ptr_t> images_message(n_cameras);

  std::vector<AmqpClient::Envelope::ptr_t> images_message_display(n_cameras);
  std::mutex mtx;
  std::condition_variable cv;
  std::atomic<bool> running;

  FrameConverterRequest frame_converter_request;
  frame_converter_request.z = 208.0;
  PointsWithReference points3d;
  Pose image_pose;
  std::atomic<bool> new_point;
  new_point.store(false);

  FinalPositionRequest action;
  FinalPositionRequest image_action;
  action.desired_pose = {{x, y}, 0.0};
  action.max_vel_x = 120.0;
  action.max_vel_y = 120.0;
  action.gain_x = 120.0 / 300.0;
  action.gain_y = 120.0 / 300.0;
  action.center_offset = 100.0;
  int n_deadlines = 0;

  bool arrived = false;

  std::vector<int> thresholds_yellow = {0, 255, 93, 140, 170, 255};
  std::vector<int> thresholds_red = {0, 255, 140, 255, 120, 180};
  //std::vector<int> thresholds_green = {0, 252, 60, 93, 150, 220};
  
  ImageAndThresholds pattern_request;
  pattern_request.thresholds_color_1 = thresholds_yellow;
  pattern_request.thresholds_color_2 = thresholds_red;


  // std::thread visual_layer([&images_message_display, &mtx, &cv, &running, &uri, &resolution, &cameras]() {
  std::thread visual_layer([&]() {

    struct callback_handle {
      Point point;
      std::atomic<bool> request_3d;
      std::string reference;
      Resolution resolution;
    };

    auto mouse_callback = [](int event, int x, int y, int, void* userdata) {
      callback_handle* handle = (callback_handle*)userdata;
      /*
      if (event == cv::EVENT_MOUSEMOVE) {
        time_point<system_clock>* time = (time_point<system_clock>*)userdata;
        if (duration_cast<milliseconds>(system_clock::now() - *time).count() > 100) {
          *time = system_clock::now();
          is::logger()->info("Mouse position [{},{}]", x, y);
        }
      } else*/ if (event == cv::EVENT_LBUTTONUP) {
        handle->point.x = x;
        handle->point.y = y;
        int x_div = handle->point.x / (handle->resolution.width / 2);
        int y_div = handle->point.y / (handle->resolution.height / 2);
        if (x_div == 0) {
          if (y_div == 0) {  // 0,0
            handle->reference = "ptgrey.0";
          } else {  // 0,1
            handle->reference = "ptgrey.2";
          }
        } else {
          if (y_div == 0) {  // 1,0
            handle->reference = "ptgrey.1";
          } else {  // 1,1
            handle->reference = "ptgrey.3";
          }
        }
        handle->point.x = 2 * (static_cast<int>(handle->point.x) % (handle->resolution.width / 2));
        handle->point.y = 2 * (static_cast<int>(handle->point.y) % (handle->resolution.height / 2));
        handle->request_3d.store(true);
      }
    };

    // time_point<system_clock> time_point = system_clock::now();
    callback_handle handle;
    handle.resolution = resolution;
    cv::namedWindow("Visual Servoing");
    cv::setMouseCallback("Visual Servoing", mouse_callback, &handle);

    int gain_x_window;
    int max_vel_x_window;
    int offset_window;
    cv::createTrackbar("gain_x", "Visual Servoing", &gain_x_window, 1000);
    cv::createTrackbar("max_vel_x", "Visual Servoing", &max_vel_x_window, 1000);
    cv::createTrackbar("offset", "Visual Servoing", &offset_window, 1000);

    auto is = is::connect(uri);
    auto client = is::make_client(is);
    handle.request_3d.store(false);

    while (running.load()) {
      std::unique_lock<std::mutex> lk(mtx);
      auto status = cv.wait_for(lk, 1s);
      if (status == std::cv_status::no_timeout) {
        std::vector<cv::Mat> up_frames;
        std::vector<cv::Mat> down_frames;
        int n_frame = 0;
        for (auto& msg : images_message_display) {
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
        mtx.unlock();

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

        if (handle.request_3d.load()) {
          FrameConverterRequest request;
          PointsWithReference point_reference;
          point_reference.points.push_back(handle.point);
          point_reference.reference = handle.reference;
          request.patterns.push_back(point_reference);
          request.z = 0.0;

          auto id = client.request("frame_converter.camera_to_world", is::msgpack(request));
          auto reply = client.receive_for(100ms, id, is::policy::discard_others);
          if (reply != nullptr) {
            auto points3d = is::msgpack<PointsWithReference>(reply);
            // Set desired pose
            mtx.lock();
            image_pose.position.x = points3d.points.front().x;
            image_pose.position.y = points3d.points.front().y;
             
            image_action.max_vel_x = 250.0*(max_vel_x_window/1000.0);
            image_action.max_vel_y = 250.0*(max_vel_x_window/1000.0);
            image_action.gain_x = 0.5*(gain_x_window/1000.0);
            image_action.gain_y = 0.5*(gain_x_window/1000.0);
            image_action.center_offset = 150.0*(offset_window/1000.0);

            mtx.unlock();
            new_point.store(true);
          }

          handle.request_3d.store(false);
        }
      }
    }
  });

  running.store(true);
  while (1) {
    switch (controller_state) {
      case ControllerState::CONSUME_IMAGE: {
        for (int i = 0; i < n_cameras; ++i) {
          images_message[i] = is.consume(frames_tags[i]);
        }
        deadline = system_clock::now() + milliseconds(static_cast<int>(1000.0 / fps));
        if (mtx.try_lock()) {
          images_message_display = images_message;
          if (new_point.load()) {
            action.desired_pose = image_pose;

            action.max_vel_x = image_action.max_vel_x;
            action.max_vel_y = image_action.max_vel_y;
            action.gain_x = image_action.gain_x;
            action.gain_y = image_action.gain_y;
            action.center_offset = image_action.center_offset;

            arrived = false;
            is::logger()->info("New action desired pose [{},{}]", action.desired_pose.position.x,
                               action.desired_pose.position.y);
            new_point.store(false);
          }
          mtx.unlock();
          cv.notify_one();
        }
        controller_state = ControllerState::REQUEST_PATTERN;
        break;
      }

      case ControllerState::REQUEST_PATTERN: {
        std::vector<std::string> ids;
        for (int i = 0; i < n_cameras; ++i) {
          pattern_request.reference = cameras[i];
          pattern_request.image = is::msgpack<CompressedImage>(images_message[i]);
          ids.push_back(client.request("colorcircles_pattern.find", is::msgpack(pattern_request)));
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
              //pattern.reference = cameras[i];
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
        auto id = client.request("colorcircles_pattern.get_pose", is::msgpack(points3d));
        auto reply = client.receive_until(deadline, id, is::policy::discard_others);

        if (reply != nullptr) {
          auto pose = is::msgpack<Pose>(reply);
          action.current_pose = pose;
          is::logger()->info("Robot pose: {};{};{}", pose.position.x, pose.position.y, pose.heading);
          controller_state = ControllerState::REQUEST_CONTROL_ACTION;
        } else {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        }
        break;
      }

      case ControllerState::REQUEST_CONTROL_ACTION: {
        if (arrived) {
          controller_state = ControllerState::CONSUME_IMAGE;
          break;
        }

        double error = std::sqrt(std::pow(action.desired_pose.position.x - action.current_pose.position.x, 2.0) +
                                 std::pow(action.desired_pose.position.y - action.current_pose.position.y, 2.0));

        std::string id;
        if (error < 100.0) {
          arrived = true;
          id = client.request(robot + ".set_speed", is::msgpack<Speed>({0.0, 0.0}));
        } else {
          id = client.request("controller.final_position;" + robot + ".set_speed",
                              is::msgpack<FinalPositionRequest>(action));
        }

        auto reply = client.receive_until(deadline, id, is::policy::discard_others);
        if (reply == nullptr) {
          controller_state = ControllerState::DEADLINE_EXCEEDED;
        } else {
          controller_state = ControllerState::CONSUME_IMAGE;
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
    }
  }
  return 0;
}