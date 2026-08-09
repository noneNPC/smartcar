#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/wechat_qrcode.hpp>
#include "origincar_msg/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class QRCodeDetectorNode : public rclcpp::Node {
 public:
  QRCodeDetectorNode() : Node("qr_code_recognition_node") {

    // ===== 模型路径 =====
    auto pkg_path =
        ament_index_cpp::get_package_share_directory("qr_code_recognition");
    auto model_dir = std::filesystem::path(pkg_path) / "model";

    detector_ = cv::wechat_qrcode::WeChatQRCode(
        (model_dir / "detect.prototxt").string(),
        (model_dir / "detect.caffemodel").string(),
        (model_dir / "sr.prototxt").string(),
        (model_dir / "sr.caffemodel").string());

    // ===== 订阅图像 =====
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/image", rclcpp::SensorDataQoS(),
        std::bind(&QRCodeDetectorNode::image_callback, this, _1));

    // ===== 发布 TTS =====
    qr_pub_ = this->create_publisher<std_msgs::msg::String>("/display_info", 10);

    // ===== 发布控制指令 =====
    sign_pub_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);

    RCLCPP_INFO(this->get_logger(), "二维码识别节点已启动");
  }

 private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // ===== 格式检查 =====
    if (msg->format.find("jpeg") == std::string::npos &&
        msg->format.find("jpg") == std::string::npos) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "跳过非JPEG压缩图像: %s",
                           msg->format.c_str());
      return;
    }

    // ===== 解码图像 =====
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "JPEG解码失败");
      return;
    }

    // ===== 转灰度 =====
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // ===== 识别二维码 =====
    std::vector<std::string> results = detector_.detectAndDecode(gray);

    for (const auto &result : results) {
      if (result.empty()) continue;

      auto now = this->now();

      // ===== 核心：去重 + 冷却 =====
      if (result == last_result_) {
        double dt = (now - last_pub_time_).seconds();
        if (dt < cooldown_sec_) {
          continue;  // 同一个结果，且在冷却时间内 → 不发布
        }
      }

      // 更新记录
      last_result_ = result;
      last_pub_time_ = now;

      // ===== 解析二维码 =====
      try {
        int number = std::stoi(result);

        origincar_msg::msg::Sign sign_msg;
        std_msgs::msg::String qrinfo_msg;

        // ===== 逻辑控制 =====
        if (number % 2 == 1) {
          // 奇数 → 顺时针
          sign_msg.sign_data = 3;
          qrinfo_msg.data = std::to_string(number) + " 顺时针";

          RCLCPP_INFO(this->get_logger(),
                      "识别到奇数 %d -> 顺时针", number);

        } else {
          // 偶数 → 逆时针
          sign_msg.sign_data = 4;
          qrinfo_msg.data = std::to_string(number) + " 逆时针";

          RCLCPP_INFO(this->get_logger(),
                      "识别到偶数 %d -> 逆时针", number);
        }

        // ===== 发布 =====
        sign_pub_->publish(sign_msg);
        qr_pub_->publish(qrinfo_msg);

      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "非数字二维码，忽略: %s", result.c_str());
      }

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "识别结果: %s", result.c_str());
    }
  }

  // ===== ROS接口 =====
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;
  rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_pub_;

  // ===== QR检测器 =====
  cv::wechat_qrcode::WeChatQRCode detector_;

  // ===== 防重复触发 =====
  std::string last_result_;
  rclcpp::Time last_pub_time_;
  double cooldown_sec_ = 10.0;  // 冷却时间（秒）
};

// ===== main =====
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRCodeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}