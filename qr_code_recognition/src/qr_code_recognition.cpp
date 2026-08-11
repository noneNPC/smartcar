#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/wechat_qrcode.hpp>
#include "origincar_msg/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class QRCodeDetectorNode : public rclcpp::Node {
 public:
  QRCodeDetectorNode() : Node("qr_code_recognition_node") {

    auto pkg_path =
        ament_index_cpp::get_package_share_directory("qr_code_recognition");
    auto model_dir = std::filesystem::path(pkg_path) / "model";

    detector_ = cv::wechat_qrcode::WeChatQRCode(
        (model_dir / "detect.prototxt").string(),
        (model_dir / "detect.caffemodel").string(),
        (model_dir / "sr.prototxt").string(),
        (model_dir / "sr.caffemodel").string());

    // 订阅 image_cut 发布的原生裁切图像，避免格式转换消耗
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/yolo_cropped_image", rclcpp::SensorDataQoS(),
        std::bind(&QRCodeDetectorNode::image_callback, this, _1));

    qr_pub_ = this->create_publisher<std_msgs::msg::String>("/display_info", 10);
    sign_pub_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);

    RCLCPP_INFO(this->get_logger(), "二维码识别节点（优化版）已启动");
  }

 private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto now = this->now();

    // 【优化 5】：冷却期提前打断机制，如果在冷却期直接返回，不调用耗算力的 WeChatQRCode，CPU 占用降为 0%
    if (has_detected_ && (now - last_pub_time_).seconds() < cooldown_sec_) {
      return;
    }

    // 使用 toCvShare 共享内存，避免深拷贝
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    const cv::Mat& image = cv_ptr->image;
    if (image.empty()) return;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<std::string> results = detector_.detectAndDecode(gray);

    for (const auto &result : results) {
      if (result.empty()) continue;

      try {
        int number = std::stoi(result);

        origincar_msg::msg::Sign sign_msg;
        std_msgs::msg::String qrinfo_msg;

        if (number % 2 == 1) {
          sign_msg.sign_data = 3;  // 奇数 -> 顺时针
          qrinfo_msg.data = std::to_string(number) + " 顺时针";
          RCLCPP_INFO(this->get_logger(), "识别到奇数 %d -> 顺时针", number);
        } else {
          sign_msg.sign_data = 4;  // 偶数 -> 逆时针
          qrinfo_msg.data = std::to_string(number) + " 逆时针";
          RCLCPP_INFO(this->get_logger(), "识别到偶数 %d -> 逆时针", number);
        }

        sign_pub_->publish(sign_msg);
        qr_pub_->publish(qrinfo_msg);

        last_result_ = result;
        last_pub_time_ = now;
        has_detected_ = true;

      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "非数字二维码: %s", result.c_str());
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;
  rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_pub_;

  cv::wechat_qrcode::WeChatQRCode detector_;

  std::string last_result_;
  rclcpp::Time last_pub_time_;
  bool has_detected_ = false;
  double cooldown_sec_ = 10.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRCodeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}