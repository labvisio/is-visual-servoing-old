#ifndef __COLORCIRCLES_PATTERN_HPP__
#define __COLORCIRCLES_PATTERN_HPP__

#include <math.h>
#include <string.h>
#include <iostream>
#include <is/msgs/camera.hpp>
#include <is/msgs/geometry.hpp>
#include <is/msgs/robot.hpp>
#include <limits>
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace is::msg::geometry;
using namespace is::msg::robot;
using namespace std;

struct Blob {
  cv::Point2d center;
  double area;
  double size;
  Blob(cv::Point2d center, double size, double area) {
    this->center = center;
    this->size = size;
    this->area = area;
  }
};

float get_distance(Point2f p1, Point2f p2) {
  float x = (p1.x - p2.x);
  float y = (p1.y - p2.y);
  return sqrt((x * x) + (y * y));
}

std::vector<Blob> find_blobs(cv::Mat image) {
  const double AREA_MIN = 150;
  const double AREA_MAX = 1200;
  const double AREA_RATIO_THR = 0.90;
  const double F_THR = 0.7;
  std::vector<std::vector<cv::Point>> initial_contours;
  std::vector<Blob> blobs;
  cv::Point2d centroid, reference;
  std::vector<cv::Point2d> points;
  cv::findContours(image, initial_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

  double a, b, f, area_p, area_d, area_ratio;
  cv::RotatedRect fit_e;
  double contour_area = 0;
  for (auto& contour : initial_contours) {
    contour_area = contourArea(contour);
    if (!((contour_area < AREA_MIN) || (contour_area > AREA_MAX) || (contour.size() < 5) ||
          (isContourConvex(contour)))) {
      fit_e = fitEllipse(contour);
      if (fit_e.size.height > fit_e.size.width) {
        a = fit_e.size.height / 2;
        b = fit_e.size.width / 2;
      } else {
        a = fit_e.size.width / 2;
        b = fit_e.size.height / 2;
      }
      f = (a - b) / a;
      area_p = contour_area;
      area_d = (CV_PI * a * b);
      area_ratio = std::min(area_p, area_d) / std::max(area_p, area_d);

      if ((area_ratio > AREA_RATIO_THR) && (f < F_THR)) {
        blobs.push_back({fit_e.center, a, area_d});
      }
    }
  }

  return blobs;
}

PointsWithReference find_pattern(Mat img, std::vector<int> thresholdsColor1, std::vector<int> thresholdsColor2) {
  cv::Mat img_luv, luv[3];

  cv::blur(img, img, Size(3, 3));

  cv::cvtColor(img, img_luv, CV_BGR2Luv);  // BGR, not RGB

  cv::split(img_luv, luv);

  cv::Mat circle_color1 = ((luv[0] >= thresholdsColor1.at(0)) & (luv[0] < thresholdsColor1.at(1))) &
                          ((luv[1] > thresholdsColor1.at(2)) & (luv[1] < thresholdsColor1.at(3))) &
                          ((luv[2] > thresholdsColor1.at(4)) & (luv[2] < thresholdsColor1.at(5)));
  cv::Mat circle_color2 = ((luv[0] >= thresholdsColor2.at(0)) & (luv[0] < thresholdsColor2.at(1))) &
                          ((luv[1] > thresholdsColor2.at(2)) & (luv[1] < thresholdsColor2.at(3))) &
                          ((luv[2] > thresholdsColor2.at(4)) & (luv[2] < thresholdsColor2.at(5)));

  cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(2, 2));
  cv::erode(circle_color1, circle_color1, element);
  cv::erode(circle_color2, circle_color2, element);

  element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(3, 3));
  cv::dilate(circle_color1, circle_color1, element);
  cv::dilate(circle_color2, circle_color2, element);

  vector<Blob> blobs_color1 = find_blobs(circle_color1);
  vector<Blob> blobs_color2 = find_blobs(circle_color2);

  Point2f point_color1, point_color2;
  float diffSize = 0.0f;
  float percent = 0.0f;
  float less_distance = std::numeric_limits<float>::max();
  float distance = 0.0f;
  bool hasCircles = false;

  for (auto blob_color1 : blobs_color1) {
    float circle = blob_color1.size * 2;
    for (auto blob_color2 : blobs_color2) {
      diffSize = abs(blob_color1.size - blob_color2.size);
      percent = ((blob_color1.size + blob_color2.size) / 2.0) * 0.2;

      distance = get_distance(blob_color1.center, blob_color2.center);
      if (distance < less_distance && distance < circle && distance > (circle / 2.0) && (diffSize < percent)) {
        less_distance = distance;
        point_color1 = blob_color1.center;
        point_color2 = blob_color2.center;
        hasCircles = true;
      }
    }
  }

  PointsWithReference pattern;
  if (hasCircles) {
    is::msg::geometry::Point p1 = {point_color1.x, point_color1.y};
    is::msg::geometry::Point p2 = {point_color2.x, point_color2.y};
    pattern.points = {p1, p2};
  }
  return pattern;
}

Pose get_pose(PointsWithReference P) {
  Pose pose;
  if (P.points.size() == 2) {
    pose.position.x = P.points[0].x;
    pose.position.y = P.points[0].y;
    pose.heading = std::atan2(P.points[1].y - P.points[0].y, P.points[1].x - P.points[0].x);
  } else {
    // TODO: return error code
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.heading = 0.0;
  }
  return pose;
}

#endif  // __COLORCIRCLES_PATTERN_HPP__