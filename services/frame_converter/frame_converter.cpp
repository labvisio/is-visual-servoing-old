#include <algorithm>
#include <armadillo>
#include <boost/program_options.hpp>
#include <chrono>
#include <is/is.hpp>
#include <is/msgs/common.hpp>
#include <is/msgs/geometry.hpp>
#include <map>
#include <string>
#include <vector>

#include "../../msgs/frame_converter.hpp"

namespace po = boost::program_options;
using namespace arma;
using namespace is::msg;
using namespace is::msg::common;
using namespace is::msg::geometry;

struct CameraParameters {
  mat K;  // Intrinsic Parameters
  mat R;  // Rotation matrix (extrinsic parameters)
  mat T;  // Translation vector (extrinsic parameters)
};

int main(int argc, char* argv[]) {
  std::string uri;
  std::string name{"frame_converter"};

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

  // Load camera parameters
  std::map<std::string, CameraParameters> parameters;
  std::vector<std::string> cameras{"ptgrey.0", "ptgrey.1", "ptgrey.2", "ptgrey.3"};

  for (auto& camera : cameras) {
    mat E, I;
    E.load(camera + "/extrinsic.dat");
    I.load(camera + "/intrinsic.dat");
    parameters.insert({camera, {I, E(span(0, 2), span(0, 2)), E(span(0, 2), 3)}});
    is::logger()->info("Loading {} parameters", camera);
  }

  is::Connection is(is::connect(uri));

  // clang-format off
  auto thread = is::advertise(uri, name, {
    {
      "camera_to_world", [&](is::Request request) -> is::Reply {
          // clang-format on
          auto req = is::msgpack<FrameConverterRequest>(request);
          req.patterns.erase(std::remove_if(std::begin(req.patterns), std::end(req.patterns),
                                            [](auto& pattern) { return pattern.points.empty(); }),
                             std::end(req.patterns));

          PointsWithReference world_points;
          auto n_patterns = req.patterns.size();

          if (n_patterns != 0) {
            int n_points = static_cast<int>(req.patterns.front().points.size());

            if (n_patterns > 1) {
              for (int n = 0; n < n_points; ++n) {
                mat A, b;
                int i = 0;
                for (auto& pattern : req.patterns) {
                  mat pt = {pattern.points.at(n).x, pattern.points.at(n).y, 1.0};
                  mat B = inv(parameters[pattern.reference].R) * inv(parameters[pattern.reference].K) * pt.t();
                  A = join_vert(
                      A, join_horiz(-eye(3, 3), join_horiz(zeros(3, i), join_horiz(B, zeros(3, n_patterns - i - 1)))));
                  b = join_vert(b, inv(parameters[pattern.reference].R) * parameters[pattern.reference].T);
                  i++;
                }
                mat P = pinv(A) * b;
                world_points.points.push_back({P.at(0), P.at(1), P.at(2)});
              }
            } else if (n_patterns == 1) {
              for (int n = 0; n < n_points; ++n) {
                auto pattern = req.patterns.front();
                mat pt = {pattern.points.at(n).x, pattern.points.at(n).y, 1.0};
                mat E = join_vert(join_horiz(parameters[pattern.reference].R, parameters[pattern.reference].T),
                                  mat({0.0, 0.0, 0.0, 1.0}));
                mat B = parameters[pattern.reference].K * eye(3, 4) * E;
                mat A = join_horiz(B.cols(0, 1), -pt.t());
                mat b = -B.col(3) - req.z * B.col(2);
                mat P = inv(A) * b;
                world_points.points.push_back({P.at(0), P.at(1), req.z});
              }
            }
          }
          return is::msgpack(world_points);
          // clang-format off
      }
    }
  });
  // clang-format on

  thread.join();
  return 0;
}