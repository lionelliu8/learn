#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace monocular_6d_pose
{

struct PoseResult
{
  cv::Vec3d rotation;
  cv::Vec3d translation;
  double reprojection_error;
};

inline PoseResult solve_square_pose(
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion,
  double marker_length)
{
  if (image_points.size() != 4 || !std::isfinite(marker_length) || marker_length <= 0.0) {
    throw std::invalid_argument("four corners and a positive marker length are required");
  }
  if (camera_matrix.rows != 3 || camera_matrix.cols != 3 ||
      camera_matrix.type() != CV_64F || !cv::checkRange(camera_matrix) ||
      camera_matrix.at<double>(0,0) <= 0 || camera_matrix.at<double>(1,1) <= 0 ||
      (!distortion.empty() && !cv::checkRange(distortion))) {
    throw std::invalid_argument("invalid camera calibration");
  }
  for (const auto & point : image_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("non-finite image corner");
    }
  }
  if (std::abs(cv::contourArea(image_points)) < 1.0) {
    throw std::invalid_argument("degenerate marker corners");
  }

  const float h = static_cast<float>(marker_length / 2.0);
  const std::vector<cv::Point3f> object_points = {
    {-h, h, 0.0F}, {h, h, 0.0F}, {h, -h, 0.0F}, {-h, -h, 0.0F}};

  PoseResult result{};
  if (!cv::solvePnP(
      object_points, image_points, camera_matrix, distortion,
      result.rotation, result.translation, false, cv::SOLVEPNP_IPPE_SQUARE))
  {
    throw std::runtime_error("solvePnP failed");
  }
  if (!cv::checkRange(cv::Mat(result.rotation)) ||
      !cv::checkRange(cv::Mat(result.translation)) || result.translation[2] <= 0) {
    throw std::runtime_error("invalid pose or object behind camera");
  }

  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, result.rotation, result.translation,
    camera_matrix, distortion, projected);
  double error = 0.0;
  for (std::size_t i = 0; i < projected.size(); ++i) {
    error += cv::norm(projected[i] - image_points[i]);
  }
  result.reprojection_error = error / projected.size();
  return result;
}

}  // namespace monocular_6d_pose
