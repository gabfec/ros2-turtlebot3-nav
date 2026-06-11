#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>

class WallFollowerNode : public rclcpp::Node {
public:
  WallFollowerNode() : Node("wall_follower_node") {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&WallFollowerNode::scan_callback, this,
                  std::placeholders::_1));
  }

  void stop_rover() {
    auto stop_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(stop_msg);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty())
      return;

    // Read the required target rays safely
    float front_ray = msg->ranges[0];
    float right_ray = msg->ranges[270]; // 90 degrees to the right

    // Fallbacks if sensors return inf/nan
    if (!std::isfinite(front_ray) || front_ray <= 0.0)
      front_ray = msg->range_max;
    if (!std::isfinite(right_ray) || right_ray <= 0.0)
      right_ray = msg->range_max;

    auto twist = geometry_msgs::msg::Twist();

    // CORNERING RULE: If an upcoming wall crosses our way (front < 0.5m)
    if (front_ray < 0.5) {
      RCLCPP_INFO(this->get_logger(), "Corner detected! Fast turn left.");
      twist.linear.x = 0.05; // Keep moving forward slightly
      twist.angular.z = 0.5; // Turn fast to the left
    }
    // WALL-FOLLOWING RULE: Track the right wall distance
    else {
      if (right_ray > 0.3) {
        // Too far away from the wall -> move toward it
        RCLCPP_INFO(this->get_logger(), "Distance > 0.3m: Nudging right.");
        twist.linear.x = 0.15;
        twist.angular.z = -0.25; // Turn right
      } else if (right_ray < 0.2) {
        // Too close to the wall -> move away from it
        RCLCPP_INFO(this->get_logger(), "Distance < 0.2m: Nudging left.");
        twist.linear.x = 0.15;
        twist.angular.z = 0.25; // Turn left
      } else {
        // Perfect zone (between 0.2m and 0.3m) -> cruise straight
        RCLCPP_INFO(this->get_logger(), "In the zone: Driving straight.");
        twist.linear.x = 0.2;
        twist.angular.z = 0.0;
      }
    }

    cmd_vel_pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WallFollowerNode>();
  rclcpp::spin(node);
  node->stop_rover();
  node = nullptr;
  rclcpp::shutdown();
  return 0;
}
