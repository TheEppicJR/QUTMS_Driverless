// import some generic libs
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// import some ROS messages
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "driverless_msgs/msg/point_with_covariance_stamped.hpp"
#include "driverless_msgs/msg/point_with_covariance_stamped_array.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"

// import header files for local helper files
#include "ros_msg_helpers.hpp"



using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class ConeEstimator : public rclcpp::Node
{
  public:
    ConeEstimator()
    : Node("cone_estimation")
    {
      publisher_ = this->create_publisher<driverless_msgs::msg::PointWithCovarianceStampedArray>("cone_estimation/estimations", 10);
      lidar_subscription_ = this->create_subscription<std_msgs::msg::String>("cone_estimation/lidar_cones", 10, std::bind(&ConeEstimator::topic_callback, this, _1));
      vision_subscription_ = this->create_subscription<std_msgs::msg::String>("cone_estimation/vision_cones", 10, std::bind(&ConeEstimator::topic_callback, this, _1));
      odom_subscription_ = this->create_subscription<std_msgs::msg::String>("state_estimation/odom", 10, std::bind(&ConeEstimator::topic_callback, this, _1)); // Need to find the right kind of msg for this
      timer_ = this->create_wall_timer(500ms, std::bind(&ConeEstimator::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! ";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeEstimator>());
  rclcpp::shutdown();
  return 0;
}