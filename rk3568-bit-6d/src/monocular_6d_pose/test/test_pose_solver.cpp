#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include "monocular_6d_pose/pose_solver.hpp"
#include "monocular_6d_pose/resize.hpp"

TEST(Resize, PreservesPoseUnderNonUniformScaling)
{
  const cv::Mat camera = (cv::Mat_<double>(3, 3) <<
    600, 0, 320, 0, 600, 240, 0, 0, 1);
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  std::vector<cv::Point3f> object = {{-.04F,.04F,0},{.04F,.04F,0},
    {.04F,-.04F,0},{-.04F,-.04F,0}};
  const cv::Vec3d rotation(.2,-.1,.3), translation(.03,-.02,.7);
  std::vector<cv::Point2f> pixels;
  cv::projectPoints(object, rotation, translation, camera, distortion, pixels);
  for (auto & p : pixels) {p.x=(p.x+.5F)*.5F-.5F; p.y=(p.y+.5F)*.75F-.5F;}
  const auto resized = monocular_6d_pose::scaled_intrinsics(camera, {640,480}, {320,360});
  const auto pose = monocular_6d_pose::solve_square_pose(pixels, resized, distortion, .08);
  EXPECT_LT(cv::norm(pose.translation-translation), 1e-4);
  cv::Mat result_rotation, expected_rotation;
  cv::Rodrigues(pose.rotation, result_rotation);
  cv::Rodrigues(rotation, expected_rotation);
  EXPECT_LT(cv::norm(result_rotation-expected_rotation), 1e-3);
}

TEST(Resize, RejectsUnavailableRgaAndInvalidInput)
{
  cv::Mat src(480,640,CV_8UC3,cv::Scalar(100,50,20)), dst;
  monocular_6d_pose::resize_image(src,dst,{320,240},false);
  EXPECT_EQ(dst.size(), cv::Size(320,240));
  EXPECT_THROW(monocular_6d_pose::resize_image(src,dst,{0,240},false),std::invalid_argument);
#ifndef USE_RGA
  EXPECT_THROW(monocular_6d_pose::resize_image(src,dst,{320,240},true),std::runtime_error);
#endif
}

TEST(PoseSolver, RecoversSyntheticPose)
{
  const double length = 0.08;
  const float h = static_cast<float>(length / 2.0);
  const std::vector<cv::Point3f> object_points = {
    {-h, h, 0.0F}, {h, h, 0.0F}, {h, -h, 0.0F}, {-h, -h, 0.0F}};
  const cv::Mat camera = (cv::Mat_<double>(3, 3) <<
    600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0);
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  const cv::Vec3d expected_rotation(0.1, -0.15, 0.2);
  const cv::Vec3d expected_translation(0.03, -0.02, 0.7);
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(
    object_points, expected_rotation, expected_translation,
    camera, distortion, image_points);

  const auto result = monocular_6d_pose::solve_square_pose(
    image_points, camera, distortion, length);
  EXPECT_LT(cv::norm(result.translation - expected_translation), 1e-4);
  EXPECT_LT(result.reprojection_error, 1e-3);
}
