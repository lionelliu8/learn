#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "monocular_6d_pose/pose_solver.hpp"
#include "monocular_6d_pose/resize.hpp"

class PoseNode : public rclcpp::Node
{
public:
  PoseNode() : Node("monocular_6d_pose_node")
  {
    marker_length_ = declare_parameter("marker_length", 0.08);
    marker_id_ = declare_parameter("marker_id", 0);
    child_frame_ = declare_parameter("child_frame", "detected_object");
    use_rga_ = declare_parameter("use_rga", false);
    resize_width_ = declare_parameter("resize_width", 320);
    resize_height_ = declare_parameter("resize_height", 240);
    debug_enabled_ = declare_parameter("publish_debug", false);
    const int threads = declare_parameter("opencv_threads", 1);
    if (resize_width_ <= 0 || resize_height_ <= 0 || threads < 1 || marker_length_ <= 0) {
      throw std::invalid_argument("invalid resize, thread count or marker length");
    }
#ifndef USE_RGA
    if (use_rga_) {throw std::runtime_error("rebuild with ENABLE_RGA=ON to use RGA");}
#endif
    cv::setNumThreads(threads);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PoseNode::on_image, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {camera_info_ = msg;});
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/object_pose", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/pose_debug_image", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!camera_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for camera_info");
      return;
    }

    const auto start = std::chrono::steady_clock::now();
    cv::Mat image;
    cv_bridge::CvImageConstPtr input;
    const bool debug = debug_enabled_ && debug_pub_->get_subscription_count() > 0;
    try {
      input = cv_bridge::toCvShare(msg, "bgr8");
      if (camera_info_->width != msg->width || camera_info_->height != msg->height ||
          camera_info_->k[0] <= 0 || camera_info_->k[4] <= 0 ||
          camera_info_->header.frame_id != msg->header.frame_id ||
          camera_info_->distortion_model != "plumb_bob") {
        throw std::runtime_error("CameraInfo must match image dimensions/frame and use plumb_bob calibration");
      }
      monocular_6d_pose::resize_image(
        input->image, resized_, cv::Size(resize_width_, resize_height_), use_rga_);
      image = resized_;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", error.what());
      return;
    }
    const auto resized_at = std::chrono::steady_clock::now();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids);
    if (debug) {cv::aruco::drawDetectedMarkers(image, corners, ids);}
    const auto detected_at = std::chrono::steady_clock::now();

    const auto found = std::find(ids.begin(), ids.end(), marker_id_);
    if (found == ids.end()) {
      if (debug) {publish_debug(image, msg->header);}
      log_timing(start, resized_at, detected_at);
      return;
    }

    const auto index = static_cast<std::size_t>(std::distance(ids.begin(), found));
    const cv::Mat original_camera(3, 3, CV_64F, const_cast<double *>(camera_info_->k.data()));
    const auto camera_matrix = monocular_6d_pose::scaled_intrinsics(
      original_camera, input->image.size(), image.size());
    cv::Mat distortion(camera_info_->d, true);
    if (distortion.empty()) {
      distortion = cv::Mat::zeros(1, 5, CV_64F);
    }

    try {
      const auto result = monocular_6d_pose::solve_square_pose(
        corners[index], camera_matrix, distortion, marker_length_);
      publish_pose(result, msg->header);
      if (debug) {cv::drawFrameAxes(
        image, camera_matrix, distortion, result.rotation, result.translation,
        static_cast<float>(marker_length_ * 0.6));
      cv::putText(
        image, cv::format("x %.3f y %.3f z %.3f m", result.translation[0],
        result.translation[1], result.translation[2]), {15, 30},
        cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);}
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "marker %d: x=%.3f y=%.3f z=%.3f m, reprojection=%.3f px",
        marker_id_, result.translation[0], result.translation[1],
        result.translation[2], result.reprojection_error);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "%s", error.what());
    }
    if (debug) {publish_debug(image, msg->header);}
    log_timing(start, resized_at, detected_at);
  }

  using Clock = std::chrono::steady_clock;
  void log_timing(Clock::time_point start, Clock::time_point resized, Clock::time_point detected)
  {
    const auto end = Clock::now();
    const auto ms = [](Clock::duration d) {return std::chrono::duration<double, std::milli>(d).count();};
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "backend=%s preprocess=%.3fms detect=%.3fms pose_publish=%.3fms total=%.3fms",
      use_rga_ ? "rga" : "opencv", ms(resized-start), ms(detected-resized),
      ms(end-detected), ms(end-start));
  }

  void publish_pose(
    const monocular_6d_pose::PoseResult & result,
    const std_msgs::msg::Header & header)
  {
    cv::Mat rotation_matrix;
    cv::Rodrigues(result.rotation, rotation_matrix);
    tf2::Matrix3x3 basis(
      rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
      rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
      rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
    tf2::Quaternion quaternion;
    basis.getRotation(quaternion);
    quaternion.normalize();

    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = result.translation[0];
    pose.pose.position.y = result.translation[1];
    pose.pose.position.z = result.translation[2];
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    pose_pub_->publish(pose);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = header;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_debug(const cv::Mat & image, const std_msgs::msg::Header & header)
  {
    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
  }

  double marker_length_;
  bool use_rga_, debug_enabled_;
  int resize_width_, resize_height_;
  cv::Mat resized_;
  int marker_id_;
  std::string child_frame_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseNode>());
  rclcpp::shutdown();
  return 0;
}
