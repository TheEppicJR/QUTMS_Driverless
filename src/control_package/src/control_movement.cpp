#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <vector>

#include "rclcpp/rclcpp.hpp"
// include messages from ROS sensors
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "fs_msgs/msg/control_command.hpp"
// include messages needed for node communication
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MovementControl : public rclcpp::Node{ // create class with inheritance from ROS node
    
    double vel_x = 0; // initialise velocity
    double vel_y = 0;

    public:
        MovementControl() : Node("control_node"){ // name of our node, constructed from ROS
            // subscribe to the "/fsds/camera/cam1" node as an 'Image' sensor message
            // bind subscribed messages to function 'image_callback'
            camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/fsds/camera/cam1", 10, std::bind(&MovementControl::image_callback, this, _1));

            // subscribe to the "/fsds/gss" node as a 'TwistStamped' geometry message
            // bind subscribed messages to function 'geo_callback'
            // geo_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            //     "/fsds/gss", 10, std::bind(&MovementControl::geo_callback, this, _1));

            // subscribe to the "math_output" custom node as a 'Float32' standard message
            // bind subscribed messages to function 'move_callback'
            math_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "math_output", 10, std::bind(&MovementControl::move_callback, this, _1));

            // publish to the "/fsds/control_command" node as a 'ControlCommand' FS message
            control_publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>(
                "/fsds/control_command", 10);
        }

    private:

        // image_callback to test if camera is running
        void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) const{
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e){
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            cv::imshow("Camera View", cv_ptr->image);
            cv::waitKey(1);
        }

        // geo_callback to receive subscribed geometry messages
        // void geo_callback(const geometry_msgs::msg::TwistStamped::SharedPtr geo_msg) const{
        //     // retrieve x and y velocities
        //     double temp_vel_x = geo_msg->twist.linear.x; 
        //     double temp_vel_y = geo_msg->twist.linear.y;

        //     RCLCPP_INFO(this->get_logger(), "X Velocity: '%lf'", temp_vel_x);
        //     RCLCPP_INFO(this->get_logger(), "Y Velocity: '%lf'", temp_vel_y);

        //     MovementControl::vel_x = temp_vel_x;
        //     MovementControl::vel_y = temp_vel_y;
        // }

        // move_callback to publish movement commands
        void move_callback(const std_msgs::msg::Float32MultiArray::SharedPtr math_msg) const{
        // void move_callback(const std_msgs::msg::Float32::SharedPtr math_msg) const{

            std::vector<float> output = math_msg->data;

            double average_y = output[0];
            double calc_throttle = output[1];

            // RCLCPP_INFO(this->get_logger(), "Heard avg: '%lf'", average_y);
            // RCLCPP_INFO(this->get_logger(), "Heard throttle: '%lf'", calc_throttle);

            double steering_p = 5; // steering proportion
            double calc_steering = 0.0; // initial steering

            // double max_throttle = 0.2; // m/s^2
            // int target_vel = 4; // m/s
            // double calc_throttle = 0.0; // initial throttle

            // // calculate throttle
            // // velocity in the vehicle's frame
            // double vel = sqrt(vel_x*vel_x + vel_y*vel_y);
            // // the lower the velocity, the more throttle, up to max_throttle
            // double p_vel = (1 - (vel / target_vel));
            // if ( p_vel > 0 ){
            //     calc_throttle = max_throttle * p_vel;
            // }
            // else if ( p_vel <= 0 ){
            //     calc_throttle = 0.0;
            // }


            // determine steering
            calc_steering = -steering_p * average_y;
            if ( calc_steering > 1.0 ){
                calc_steering = 1.0;
            }
            else if ( calc_steering < -1.0 ){
                calc_steering = -1.0;
            }

            auto message = fs_msgs::msg::ControlCommand();
            message.throttle = calc_throttle;
            message.steering = calc_steering;
            message.brake = 0;
            
            control_publisher_->publish(message);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr math_subscription_;
        // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr geo_subscription_;
        rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_publisher_;
};

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementControl>());
    rclcpp::shutdown();
    return 0;
}
