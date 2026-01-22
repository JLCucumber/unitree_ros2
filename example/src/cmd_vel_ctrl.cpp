#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"  // 引入已有的 API ID 定义!

class CmdVelToUnitree : public rclcpp::Node {
public:
  CmdVelToUnitree() : Node("cmd_vel_to_unitree") {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        handleCmdVel(msg);
      });
    
    request_pub_ = this->create_publisher<unitree_api::msg::Request>(
      "/api/sport/request", 10);
    
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() { sendControlCommand(); });
    
    RCLCPP_INFO(this->get_logger(), "cmd_vel_to_unitree started");
  }

private:
  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    latest_cmd_ = *msg;
    last_cmd_time_ = this->now();
  }
  
  void sendControlCommand() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    
    auto elapsed = (this->now() - last_cmd_time_).seconds();
    if (elapsed > 0.5) {
      latest_cmd_ = geometry_msgs::msg::Twist();
    }
    
    nlohmann::json js;
    js["x"] = latest_cmd_.linear.x;
    js["y"] = latest_cmd_.linear.y;
    js["z"] = latest_cmd_.angular.z;
    
    unitree_api::msg::Request req;
    req.parameter = js.dump();
    
    // 直接使用 ros2_sport_client.h 中定义的常量!
    req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
    
    request_pub_->publish(req);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToUnitree>());
  rclcpp::shutdown();
  return 0;
}