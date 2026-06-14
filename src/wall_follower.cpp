#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_nav/srv/find_wall.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

class WallFollowerNode : public rclcpp::Node {
public:
  WallFollowerNode() : Node("wall_follower_node"), wall_prepared_(false) {
    cb_group_scan_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_client_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_scan_;
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&WallFollowerNode::scan_callback, this,
                  std::placeholders::_1),
        sub_options);

    client_ = this->create_client<turtlebot3_nav::srv::FindWall>(
        "find_wall", rmw_qos_profile_services_default, cb_group_client_);
  }

  void trigger_wall_alignment() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for service 'find_wall' to become available...");
    }

    auto request = std::make_shared<turtlebot3_nav::srv::FindWall::Request>();
    RCLCPP_INFO(this->get_logger(),
                "Sending service request to align with wall...");

    auto result_future = client_->async_send_request(request);

    // Since we are running in a MultiThreadedExecutor, we can safely wait on
    // the future without deadlocking the processing loops of other threads.
    if (result_future.wait_for(20s) == std::future_status::ready) {
      auto response = result_future.get();
      if (response->wallfound) {
        RCLCPP_INFO(
            this->get_logger(),
            "Service returned success! Commencing wall-following rules.");
        wall_prepared_ = true;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call timed out or failed.");
    }
  }

  void stop_rover() {
    auto stop_msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(stop_msg);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Guard checking: Do not process wall tracking maneuvers if alignment phase
    // isn't completed.
    if (!wall_prepared_ || msg->ranges.empty()) {
      return;
    }

    float front_ray = msg->ranges[0];
    float right_ray = msg->ranges[270];

    if (!std::isfinite(front_ray) || front_ray <= 0.0)
      front_ray = msg->range_max;
    if (!std::isfinite(right_ray) || right_ray <= 0.0)
      right_ray = msg->range_max;

    auto twist = geometry_msgs::msg::Twist();

    // CORNERING RULE
    if (front_ray < 0.5) {
      RCLCPP_INFO(this->get_logger(), "Corner detected! Fast turn left.");
      twist.linear.x = 0.05;
      twist.angular.z = 0.5;
    }
    // WALL-FOLLOWING RULE
    else {
      if (right_ray > 0.3) {
        RCLCPP_INFO(this->get_logger(), "Distance > 0.3m: Nudging right.");
        twist.linear.x = 0.15;
        twist.angular.z = -0.25;
      } else if (right_ray < 0.2) {
        RCLCPP_INFO(this->get_logger(), "Distance < 0.2m: Nudging left.");
        twist.linear.x = 0.15;
        twist.angular.z = 0.25;
      } else {
        RCLCPP_INFO(this->get_logger(), "In the zone: Driving straight.");
        twist.linear.x = 0.2;
        twist.angular.z = 0.0;
      }
    }
    cmd_vel_pub_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<turtlebot3_nav::srv::FindWall>::SharedPtr client_;

  std::atomic<bool> wall_prepared_;
  rclcpp::CallbackGroup::SharedPtr cb_group_scan_;
  rclcpp::CallbackGroup::SharedPtr cb_group_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WallFollowerNode>();

  // Concurrency execution via multi-threaded executor pooling
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Run service client triggering logic inside an asynchronous context thread
  // to allow executor spin to handle parallel node interfaces.
  std::thread initialization_thread([node]() {
    // Sleep briefly to ensure executor has spinning threads up to service
    // requests
    std::this_thread::sleep_for(500ms);
    node->trigger_wall_alignment();
  });

  executor.spin();

  if (initialization_thread.joinable()) {
    initialization_thread.join();
  }

  node->stop_rover();
  rclcpp::shutdown();
  return 0;
}