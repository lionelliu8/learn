#include <chrono>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class SyntheticCameraNode : public rclcpp::Node
{
public:
  SyntheticCameraNode() : Node("synthetic_camera_node")
  {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);
    timer_ = create_wall_timer(100ms, std::bind(&SyntheticCameraNode::publish_frame, this));
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }

private:
  void publish_frame()
  {
    constexpr int width = 640;
    constexpr int height = 480;
    constexpr int marker_pixels = 200;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat marker;
    cv::aruco::drawMarker(dictionary_, 0, marker_pixels, marker, 1);
    cv::Mat marker_bgr;
    cv::cvtColor(marker, marker_bgr, cv::COLOR_GRAY2BGR);
    marker_bgr.copyTo(image(cv::Rect((width - marker_pixels) / 2, (height - marker_pixels) / 2,
      marker_pixels, marker_pixels)));

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "camera_optical_frame";
    image_pub_->publish(*cv_bridge::CvImage(header, "bgr8", image).toImageMsg());

    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    info.k = {600.0, 0.0, width / 2.0, 0.0, 600.0, height / 2.0, 0.0, 0.0, 1.0};
    info.p = {600.0, 0.0, width / 2.0, 0.0, 0.0, 600.0, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    info_pub_->publish(info);
  }

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyntheticCameraNode>());
  rclcpp::shutdown();
  return 0;
}
