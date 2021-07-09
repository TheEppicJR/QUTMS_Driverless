#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "fs_msgs/msg/control_command.hpp"

using std::placeholders::_1;

class MovementControl : public rclcpp::Node
{
    public:
    MovementControl()
    : Node("control_node")
    {
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fsds/camera/cam1", 10, std::bind(&MovementControl::image_callback, this, _1));
            
        math_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "math_output", 10, std::bind(&MovementControl::move_callback, this, _1));

        control_publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>("/fsds/control_command", 10);
    }

    private:
    // Callback
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("Camera View", cv_ptr->image);
        cv::waitKey(1);
    }

    void move_callback(const std_msgs::msg::Float32::SharedPtr msg) const{
        """
        PSUDOCODE
        # Autonomous system constatns
        max_throttle = 0.2 # m/s^2
        target_speed = 4 # m/s
        max_steering = 0.3
        cones_range_cutoff = 7 # meters
            

        
        """


        auto message = fs_msgs::msg::ControlCommand();
        message.throttle = 0.2;
        message.steering = msg->data;
        message.brake = 0;

        control_publisher_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr math_subscriber_;
    rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_publisher_;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementControl>());
    rclcpp::shutdown();
    return 0;
}
