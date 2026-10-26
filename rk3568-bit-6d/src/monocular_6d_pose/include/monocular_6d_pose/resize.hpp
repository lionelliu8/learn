#pragma once
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#ifdef USE_RGA
#include <im2d.h>
#include <RgaUtils.h>
#endif

namespace monocular_6d_pose {
inline void resize_image(const cv::Mat & input, cv::Mat & output, cv::Size size, bool rga)
{
  if (input.type() != CV_8UC3 || size.width <= 0 || size.height <= 0) {
    throw std::invalid_argument("resize requires BGR8 and positive dimensions");
  }
  output.create(size, CV_8UC3);
  if (!rga) {
    cv::resize(input, output, size, 0, 0, cv::INTER_LINEAR);
    return;
  }
#ifdef USE_RGA
  // Virtual-address path: ROS Image owns CPU memory. This is not DMA-BUF zero-copy.
  if (!input.isContinuous() || !output.isContinuous()) {
    throw std::runtime_error("RGA requires packed contiguous BGR buffers");
  }
  auto src = wrapbuffer_virtualaddr(input.data, input.cols, input.rows, RK_FORMAT_BGR_888);
  auto dst = wrapbuffer_virtualaddr(output.data, output.cols, output.rows, RK_FORMAT_BGR_888);
  im_rect src_rect = {}, dst_rect = {};
  auto status = imcheck(src, dst, src_rect, dst_rect);
  if (status != IM_STATUS_NOERROR) {
    throw std::runtime_error(std::string("RGA validation: ") + imStrError(status));
  }
  status = imresize(src, dst);
  if (status != IM_STATUS_SUCCESS) {
    throw std::runtime_error(std::string("RGA resize: ") + imStrError(status));
  }
#else
  throw std::runtime_error("RGA requested but binary was built without -DENABLE_RGA=ON");
#endif
}

inline cv::Mat scaled_intrinsics(const cv::Mat & camera, cv::Size original, cv::Size resized)
{
  if (original.width <= 0 || original.height <= 0 || resized.width <= 0 || resized.height <= 0) {
    throw std::invalid_argument("invalid image dimensions");
  }
  cv::Mat result = camera.clone();
  const double sx = double(resized.width) / original.width;
  const double sy = double(resized.height) / original.height;
  result.at<double>(0, 0) *= sx;
  result.at<double>(1, 1) *= sy;
  // OpenCV resize maps destination pixel centers to source pixel centers.
  result.at<double>(0, 2) = (camera.at<double>(0, 2) + 0.5) * sx - 0.5;
  result.at<double>(1, 2) = (camera.at<double>(1, 2) + 0.5) * sy - 0.5;
  return result;
}
}
