#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "origincar_msg/msg/sign.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

// 发送模式枚举
enum class SendMode {
    DISTANCE,
    INDEX
};

class FilePathPublisher : public rclcpp::Node
{
public:
    FilePathPublisher()
        : Node("file_path_planner")
    {
        // ==============================
        // 一、声明与读取 YAML 参数
        // ==============================
        this->declare_parameter<int>("search_window_size", 120);
        this->declare_parameter<int>("search_backtrack_step", 8);
        this->declare_parameter<double>("out_of_window_thresh", 0.6);
        this->declare_parameter<double>("recovery_cooldown", 0.5);

        this->declare_parameter<std::string>("send_mode", "index");
        this->declare_parameter<double>("min_move_dist", 0.05);
        this->declare_parameter<int>("min_index_step", 8);
        this->declare_parameter<double>("force_send_interval", 1.0);

        this->declare_parameter<bool>("trim_sent_path", true);
        this->declare_parameter<double>("trim_sent_distance", 0.15);
        this->declare_parameter<double>("max_trim_ratio", 0.4);
        this->declare_parameter<int>("backtrack_tolerance", 5);

        this->declare_parameter<double>("path_config.path_2.forward_length", 3.0);
        this->declare_parameter<double>("path_config.path_3.forward_length", 2.4);
        this->declare_parameter<double>("path_config.path_4.forward_length", 2.4);

        this->get_parameter("search_window_size", search_window_size_);
        this->get_parameter("search_backtrack_step", search_backtrack_step_);
        this->get_parameter("out_of_window_thresh", out_of_window_thresh_);
        this->get_parameter("recovery_cooldown", recovery_cooldown_);

        std::string send_mode_str;
        this->get_parameter("send_mode", send_mode_str);
        if (send_mode_str == "distance") {
            send_mode_ = SendMode::DISTANCE;
        } else {
            send_mode_ = SendMode::INDEX;
        }

        this->get_parameter("min_move_dist", min_move_dist_);
        this->get_parameter("min_index_step", min_index_step_);
        this->get_parameter("force_send_interval", force_send_interval_);

        this->get_parameter("trim_sent_path", trim_sent_path_);
        this->get_parameter("trim_sent_distance", trim_sent_distance_);
        this->get_parameter("max_trim_ratio", max_trim_ratio_);
        this->get_parameter("backtrack_tolerance", backtrack_tolerance_);

        double len_2, len_3, len_4;
        this->get_parameter("path_config.path_2.forward_length", len_2);
        this->get_parameter("path_config.path_3.forward_length", len_3);
        this->get_parameter("path_config.path_4.forward_length", len_4);
        forward_lengths_[2] = len_2;
        forward_lengths_[3] = len_3;
        forward_lengths_[4] = len_4;

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), " FilePathPublisher Initialized Params:");
        RCLCPP_INFO(this->get_logger(), "   - search_window_size   : %d", search_window_size_);
        RCLCPP_INFO(this->get_logger(), "   - search_backtrack_step: %d", search_backtrack_step_);
        RCLCPP_INFO(this->get_logger(), "   - out_of_window_thresh : %.2f m", out_of_window_thresh_);
        RCLCPP_INFO(this->get_logger(), "   - recovery_cooldown    : %.2f s", recovery_cooldown_);
        RCLCPP_INFO(this->get_logger(), "   - send_mode            : %s", send_mode_ == SendMode::DISTANCE ? "distance" : "index");
        RCLCPP_INFO(this->get_logger(), "   - min_move_dist        : %.2f m", min_move_dist_);
        RCLCPP_INFO(this->get_logger(), "   - min_index_step       : %d", min_index_step_);
        RCLCPP_INFO(this->get_logger(), "   - force_send_interval  : %.2f s", force_send_interval_);
        RCLCPP_INFO(this->get_logger(), "   - trim_sent_path       : %s", trim_sent_path_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "   - trim_sent_distance   : %.2f m", trim_sent_distance_);
        RCLCPP_INFO(this->get_logger(), "   - max_trim_ratio       : %.2f", max_trim_ratio_);
        RCLCPP_INFO(this->get_logger(), "   - backtrack_tolerance  : %d points", backtrack_tolerance_);
        RCLCPP_INFO(this->get_logger(), "========================================");

        // ==============================
        // Action 与 TF 初始化
        // ==============================
        action_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "FollowPath server currently unavailable, will retry automatically...");
        }

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

        std::string pkg = ament_index_cpp::get_package_share_directory("pub_plan");
        std::string dir = pkg + "/path/";

        path_map_[2] = load_path(dir + "1.json");
        path_map_[3] = load_path(dir + "2.json");
        path_map_[4] = load_path(dir + "3.json");

        path_index_[2] = 0;
        path_index_[3] = 0;
        path_index_[4] = 0;

        sign_sub_ = this->create_subscription<origincar_msg::msg::Sign>(
            "/sign_switch", 10,
            std::bind(&FilePathPublisher::sign_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&FilePathPublisher::update_path, this)
        );

        last_recovery_time_ = this->now();
        last_send_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        RCLCPP_INFO(this->get_logger(), "FilePathPublisher node started successfully.");
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
    rclcpp::Subscription<origincar_msg::msg::Sign>::SharedPtr sign_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unordered_map<int, nav_msgs::msg::Path> path_map_;
    nav_msgs::msg::Path current_path_;
    int current_id_ = 0;
    bool path_active_ = false;
    std::unordered_map<int, size_t> path_index_;

    // 机器人配置参数
    int search_window_size_;
    int search_backtrack_step_;
    double out_of_window_thresh_;
    double min_move_dist_;
    double recovery_cooldown_;
    std::unordered_map<int, double> forward_lengths_;

    SendMode send_mode_;
    int min_index_step_;
    double force_send_interval_;
    bool trim_sent_path_;
    double trim_sent_distance_;
    double max_trim_ratio_;
    int backtrack_tolerance_;

    // ==========================================
    // 精简且职责明确的索引管理变量
    // ==========================================
    size_t current_nearest_index_ = 0;   // 当前 TF 在全局路径上匹配到的最近点索引
    size_t last_crop_start_index_ = 0;   // 当前帧 crop_path 算出的局部路径起点索引
    size_t max_cropped_index_ = 0;       // 历史上裁剪推进达到的最大高水位索引（用于单调性保证）
    size_t last_sent_start_index_ = 0;   // 上一次成功发送给 Controller 的路径起点索引

    double last_sent_x_ = -999.0;
    double last_sent_y_ = -999.0;

    rclcpp::Time last_send_time_;
    rclcpp::Time last_recovery_time_;

    // 正在运行中的 Goal 句柄
    GoalHandleFollowPath::SharedPtr active_goal_handle_ = nullptr;

    nav_msgs::msg::Path load_path(const std::string &file);
    void sign_callback(const origincar_msg::msg::Sign::SharedPtr msg);
    nav_msgs::msg::Path crop_path();
    void update_path();
    void trigger_recovery();
};

nav_msgs::msg::Path FilePathPublisher::load_path(const std::string &file)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    std::ifstream f(file);
    if (!f.is_open()) return path;

    json data;
    try { f >> data; } catch (...) { return path; }

    for (auto &p : data) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = p.value("x", 0.0);
        pose.pose.position.y = p.value("y", 0.0);
        pose.pose.orientation.w = p.value("w", 1.0);
        pose.pose.orientation.x = p.value("x_ori", 0.0);
        pose.pose.orientation.y = p.value("y_ori", 0.0);
        pose.pose.orientation.z = p.value("z_ori", 0.0);
        path.poses.push_back(pose);
    }
    return path;
}

void FilePathPublisher::sign_callback(const origincar_msg::msg::Sign::SharedPtr msg)
{
    int id = msg->sign_data;
    if (path_map_.count(id) == 0) return;

    current_id_ = id;
    current_path_ = path_map_[id];
    
    current_nearest_index_ = path_index_[id];
    last_crop_start_index_ = current_nearest_index_;
    max_cropped_index_ = current_nearest_index_;
    last_sent_start_index_ = static_cast<size_t>(-1); // 重置为非法值，确保新路径第一帧必定发送

    last_sent_x_ = -999.0;
    last_sent_y_ = -999.0;
    last_send_time_ = this->now();

    path_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Switched to Path ID: %d, starting from index: %zu", id, current_nearest_index_);
}

void FilePathPublisher::trigger_recovery()
{
    rclcpp::Time now = this->now();
    double time_since_last = (now - last_recovery_time_).seconds();

    if (time_since_last < recovery_cooldown_) {
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Path execution stalled or aborted by controller. Triggering auto-recovery...");

    last_recovery_time_ = now;
    last_sent_x_ = -999.0;
    last_sent_y_ = -999.0;

    // 重置索引与时间状态，强制下一周期立即重发全新 Goal
    last_sent_start_index_ = static_cast<size_t>(-1);
    last_send_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // 释放活跃句柄
    active_goal_handle_ = nullptr;
}

nav_msgs::msg::Path FilePathPublisher::crop_path()
{
    nav_msgs::msg::Path local;
    local.header.frame_id = "map";
    if (current_path_.poses.empty()) return local;

    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &e) {
        return local;
    }

    double rx = tf.transform.translation.x;
    double ry = tf.transform.translation.y;

    // ==========================================
    // 三、Search Window 优化（允许有限度回退搜索）
    // ==========================================
    size_t window_start = 0;
    if (current_nearest_index_ > static_cast<size_t>(search_backtrack_step_)) {
        window_start = current_nearest_index_ - static_cast<size_t>(search_backtrack_step_);
    }

    size_t window_end = std::min(current_nearest_index_ + static_cast<size_t>(search_window_size_), current_path_.poses.size());

    double min_dist = 99999.0;
    size_t nearest = current_nearest_index_;
    bool found_in_window = false;

    for (size_t i = window_start; i < window_end; i++) {
        double dx = current_path_.poses[i].pose.position.x - rx;
        double dy = current_path_.poses[i].pose.position.y - ry;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < min_dist) {
            min_dist = d;
            nearest = i;
        }
    }

    if (min_dist < out_of_window_thresh_) {
        found_in_window = true;
    }

    // 全局降级搜索
    if (!found_in_window) {
        min_dist = 99999.0;
        for (size_t i = 0; i < current_path_.poses.size(); i++) {
            double dx = current_path_.poses[i].pose.position.x - rx;
            double dy = current_path_.poses[i].pose.position.y - ry;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < min_dist) {
                min_dist = d;
                nearest = i;
            }
        }
    }

    current_nearest_index_ = nearest;
    path_index_[current_id_] = current_nearest_index_;

    // 到达终点判定
    if (current_nearest_index_ >= current_path_.poses.size() - 2) {
        return local;
    }

    // 获取当前路径配置的前瞻距离
    double target_length = 3.0;
    if (forward_lengths_.count(current_id_) > 0) {
        target_length = forward_lengths_[current_id_];
    }

    // ==========================================
    // 一 & 七、Trim 逻辑单调推进与物理安全比例限制
    // ==========================================
    size_t raw_trim_start = current_nearest_index_;

    if (trim_sent_path_) {
        // 七、根据前瞻距离和比例上限保护有效裁剪距离
        double max_allowed_trim = target_length * max_trim_ratio_;
        double effective_trim_distance = std::min(trim_sent_distance_, max_allowed_trim);

        double accumulated_trim_dist = 0.0;
        for (size_t i = current_nearest_index_; i + 1 < current_path_.poses.size(); i++) {
            double dx = current_path_.poses[i + 1].pose.position.x - current_path_.poses[i].pose.position.x;
            double dy = current_path_.poses[i + 1].pose.position.y - current_path_.poses[i].pose.position.y;
            double step_d = std::sqrt(dx * dx + dy * dy);

            if (accumulated_trim_dist + step_d > effective_trim_distance) {
                break;
            }
            accumulated_trim_dist += step_d;
            raw_trim_start = i + 1;
        }
    }

    // 一、高水位单调推进算法（带有容忍阈值的防拉锯限制）
    size_t final_start_index = raw_trim_start;
    if (raw_trim_start > max_cropped_index_) {
        // 正常向前推进，更新高水位
        max_cropped_index_ = raw_trim_start;
        final_start_index = raw_trim_start;
    } else {
        // 拟回退情况：如果回退幅度在容忍范围内，强行保持高水位（消除 1-2 个点的频繁微小抖动）
        if (max_cropped_index_ - raw_trim_start <= static_cast<size_t>(backtrack_tolerance_)) {
            final_start_index = max_cropped_index_;
        } else {
            // 回退幅度过大（如发生了大的定位跳变或倒车），允许回退并重置高水位
            max_cropped_index_ = raw_trim_start;
            final_start_index = raw_trim_start;
        }
    }

    // 越界安全防护
    if (final_start_index >= current_path_.poses.size() - 1) {
        final_start_index = current_path_.poses.size() - 2;
    }

    // 保存当前计算出的实际起点
    last_crop_start_index_ = final_start_index;

    // 截取局部 Path
    double length = 0.0;
    for (size_t i = final_start_index; i < current_path_.poses.size(); i++) {
        local.poses.push_back(current_path_.poses[i]);
        if (i > final_start_index) {
            double dx = current_path_.poses[i].pose.position.x - current_path_.poses[i - 1].pose.position.x;
            double dy = current_path_.poses[i].pose.position.y - current_path_.poses[i - 1].pose.position.y;
            length += std::sqrt(dx * dx + dy * dy);
        }
        if (length >= target_length) break;
    }

    return local;
}

void FilePathPublisher::update_path()
{
    if (!path_active_) return;

    if (!action_client_->action_server_is_ready()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for /follow_path action server...");
        return;
    }

    auto local = crop_path();
    if (local.poses.size() < 2) {
        RCLCPP_INFO(this->get_logger(), "Path ID %d reached goal!", current_id_);
        path_active_ = false;
        return;
    }

    rclcpp::Time now = this->now();
    bool is_force_refresh = false;
    bool is_movement_triggered = false;

    // ==========================================
    // 二 & 六、真正实现 Force Refresh 与清晰的逻辑流
    // ==========================================
    double time_since_last_send = (now - last_send_time_).seconds();
    if (time_since_last_send >= force_send_interval_) {
        is_force_refresh = true;
    }

    // 移动触发条件判定
    if (send_mode_ == SendMode::DISTANCE) {
        double current_x = local.poses.front().pose.position.x;
        double current_y = local.poses.front().pose.position.y;
        double move_dist = std::sqrt(std::pow(current_x - last_sent_x_, 2) + std::pow(current_y - last_sent_y_, 2));
        if (move_dist >= min_move_dist_) {
            is_movement_triggered = true;
        }
    } else if (send_mode_ == SendMode::INDEX) {
        if (last_sent_start_index_ == static_cast<size_t>(-1) || 
            last_crop_start_index_ >= last_sent_start_index_ + static_cast<size_t>(min_index_step_)) {
            is_movement_triggered = true;
        }
    }

    // 如果既没达到物理移动阈值，也没达到超时强发条件，直接返回
    if (!is_movement_triggered && !is_force_refresh) {
        return;
    }

    // 六、避免重复发送完全相同 Path（除非是 Force Refresh 强发心跳包）
    if (!is_force_refresh && (last_crop_start_index_ == last_sent_start_index_)) {
        return;
    }

    // 更新上一次发送的状态标记
    double current_x = local.poses.front().pose.position.x;
    double current_y = local.poses.front().pose.position.y;
    last_sent_x_ = current_x;
    last_sent_y_ = current_y;
    last_sent_start_index_ = last_crop_start_index_;
    last_send_time_ = now;

    // 发布 Topic 用于可视化
    local.header.stamp = now;
    for (auto &p : local.poses) p.header.stamp = local.header.stamp;
    path_pub_->publish(local);

    // 构建 Goal
    auto goal = FollowPath::Goal();
    goal.path = local;
    goal.controller_id = "FollowPath";

    auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    
    options.goal_response_callback = [this](const GoalHandleFollowPath::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by controller server!");
            this->trigger_recovery();
        } else {
            this->active_goal_handle_ = goal_handle;
        }
    };

    // ==========================================
    // 五、完整生命周期管理 (正确释放 handle)
    // ==========================================
    options.result_callback = [this](const GoalHandleFollowPath::WrappedResult & result) {
        if (this->active_goal_handle_ && result.goal_id != this->active_goal_handle_->get_goal_id()) {
            return;
        }

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_DEBUG(this->get_logger(), "Active goal succeeded.");
                this->active_goal_handle_ = nullptr; // 五、成功时正常释放
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Active goal ABORTED by controller server.");
                this->active_goal_handle_ = nullptr; // 五、异常终止释放
                this->trigger_recovery();
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_DEBUG(this->get_logger(), "Active goal was canceled.");
                this->active_goal_handle_ = nullptr; // 五、被抢占或取消时释放
                break;
            default:
                this->active_goal_handle_ = nullptr;
                this->trigger_recovery();
                break;
        }
    };

    // ==========================================
    // 四、Action 抢占机制替代危险的显式 Async Cancel
    // ==========================================
    // 注意：Nav2 FollowPath Action Server 天生支持 Preemption（抢占）。
    // 直接发送 async_send_goal 会在 Action Server 端优雅挂起/抢占旧 Goal，
    // 彻底消除 Client 端 async_cancel_goal 与 async_send_goal 之间的 DDS 异步竞态条件。
    action_client_->async_send_goal(goal, options);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilePathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}