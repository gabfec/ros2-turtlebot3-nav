#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlebot3_nav/srv/find_wall.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>

class FindWallServerNode : public rclcpp::Node {
public:
  FindWallServerNode() : Node("find_wall_server_node") {
    // Publisher to control the robot's movement
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscriber to continuously update the laser scan data
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&FindWallServerNode::scan_callback, this,
                  std::placeholders::_1));

    // Service Server definition
    srv_ = this->create_service<turtlebot3_nav::srv::FindWall>(
        "find_wall", std::bind(&FindWallServerNode::handle_find_wall, this,
                               std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Find Wall Service Server is ready.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
  }

  void handle_find_wall(
      const std::shared_ptr<turtlebot3_nav::srv::FindWall::Request> request,
      std::shared_ptr<turtlebot3_nav::srv::FindWall::Response> response) {
    (void)request; // Unused parameter
    RCLCPP_INFO(this->get_logger(),
                "Service called! Executing wall alignment sequence...");

    auto twist = geometry_msgs::msg::Twist();
    rclcpp::Rate rate(10); // 10 Hz control loop frequency

    // Ensure we have received at least one scan message before processing
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (latest_scan_ && !latest_scan_->ranges.empty()) {
          break;
        }
      }
      RCLCPP_WARN(this->get_logger(), "Waiting for first valid /scan data...");
      rate.sleep();
    }

    // ====================================================================
    // STEP 1 & 2: Identify shortest ray and rotate front (ray 0) to face it
    // ====================================================================
    RCLCPP_INFO(this->get_logger(),
                "Step 1 & 2: Finding closest wall and rotating to face it.");
    while (rclcpp::ok()) {
      int shortest_ray_index = 0;
      float min_distance = 999.0;
      float front_ray = 999.0;

      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        // Find the index of the absolute shortest ray in the array
        auto min_it = std::min_element(latest_scan_->ranges.begin(),
                                       latest_scan_->ranges.end());
        shortest_ray_index =
            std::distance(latest_scan_->ranges.begin(), min_it);
        min_distance = *min_it;
        front_ray = latest_scan_->ranges[0];
      }

      // Fallback for infinity/nan readings
      if (!std::isfinite(front_ray))
        front_ray = 5.0;

      // Stop rotating if the shortest ray is closely aligned with the front
      // (index 0) Or if the front ray is within a fractional tolerance of the
      // minimum distance
      if (shortest_ray_index == 0 ||
          std::abs(front_ray - min_distance) < 0.05) {
        break;
      }

      // Determine rotation direction depending on which side the wall is on
      // (Assuming index 0 to 180 is left, 180 to 360 is right)
      if (shortest_ray_index < 180) {
        twist.angular.z = 0.2; // Turn left
      } else {
        twist.angular.z = -0.2; // Turn right
      }
      twist.linear.x = 0.0;
      cmd_vel_pub_->publish(twist);
      rate.sleep();
    }
    stop_robot();

    // ====================================================================
    // STEP 3: Move forward until ray 0 is shorter than 0.3m
    // ====================================================================
    RCLCPP_INFO(this->get_logger(), "Step 3: Driving forward toward the wall.");
    while (rclcpp::ok()) {
      float front_ray = 999.0;
      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        front_ray = latest_scan_->ranges[0];
      }

      if (std::isfinite(front_ray) && front_ray <= 0.3) {
        break; // Target distance achieved
      }

      twist.linear.x = 0.08; // Safe, controlled approach speed
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
      rate.sleep();
    }
    stop_robot();

    // ====================================================================
    // STEP 4: Rotate again until ray 270 (right side) points to the wall
    // ====================================================================
    RCLCPP_INFO(
        this->get_logger(),
        "Step 4: Rotating left so the right side (ray 270) faces the wall.");
    while (rclcpp::ok()) {
      float right_ray = 999.0;
      float front_ray = 999.0;
      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        right_ray = latest_scan_->ranges[270];
        front_ray = latest_scan_->ranges[0];
      }

      if (!std::isfinite(right_ray))
        right_ray = 5.0;
      if (!std::isfinite(front_ray))
        front_ray = 5.0;

      // We are turning left to put the wall on our right side.
      // The turn is complete when ray 270 becomes the localized minimum.
      if (right_ray <= 0.35 && front_ray > 0.4) {
        break;
      }

      twist.linear.x = 0.0;
      twist.angular.z = 0.25; // Rotate counter-clockwise (left)
      cmd_vel_pub_->publish(twist);
      rate.sleep();
    }
    stop_robot();

    RCLCPP_INFO(this->get_logger(),
                "Alignment complete. Robot is parallel to the right wall.");
    response->wallfound = true;
  }

  void stop_robot() {
    auto msg = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<turtlebot3_nav::srv::FindWall>::SharedPtr srv_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FindWallServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}