#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"
#include <rmw/qos_profiles.h>
#include "sensor_msgs/msg/camera_info.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "tello_msgs/msg/tello_response.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class VideoInterface : public rclcpp::Node {
    public:
        VideoInterface() : Node("video_interface") {
            RCLCPP_INFO(this->get_logger(), "Starting video_interface node.");
            this->get_params();

            this->last_battery_data_ = 100;
            this->status = "starting";
            this->counter_ = 0;
            this->draw_frame_ = false;

            battery_sub_ = this->create_subscription<std_msgs::msg::Int16>("battery", 1, std::bind(&VideoInterface::battery_callback, this, std::placeholders::_1));
            command_sub_ = this->create_subscription<std_msgs::msg::String>("command", 1, std::bind(&VideoInterface::command_callback, this, std::placeholders::_1));
            cam_sub_ = image_transport::create_camera_subscription(this, "drone_camera/image", std::bind(&VideoInterface::image_callback, this, std::placeholders::_1, std::placeholders::_2), "raw");
            response_sub_ = this->create_subscription<tello_msgs::msg::TelloResponse>("tello_response", 1, std::bind(&VideoInterface::response_callback, this, std::placeholders::_1));
        }

        void make_interface() {
            RCLCPP_DEBUG(this->get_logger(), "making interface");
            
            // base gui
            this->write_text("Status: ",  this->status_text_origin_,  this->status_thickness_*4,  this->status_font_size_,  this->BLACK_);
            this->write_text("Status: ",  this->status_text_origin_,  this->status_thickness_,    this->status_font_size_,  this->WHITE_);
            this->write_text("Battery: ", this->battery_text_origin_, this->battery_thickness_*4, this->battery_font_size_, this->BLACK_);
            this->write_text("Battery: ", this->battery_text_origin_, this->battery_thickness_,   this->battery_font_size_, this->WHITE_);
            
            // filling status
            this->process_status(this->current_command_);
            this->write_text(this->status, this->status_origin_, this->status_thickness_*4, this->status_font_size_, this->BLACK_);
            this->write_text(this->status, this->status_origin_, this->status_thickness_, this->status_font_size_, this->STATUS_COLOR_);

            // filling battery level
            this->process_battery();

            // frame
            if(this->counter_ >= 6) {
                this->counter_ = 0;
                this->draw_frame_ = false;
                this->frame_timer_->cancel();
                this->frame_timer_.reset();
            }
            if(this->draw_frame_) {
               this->make_frame();
            }
        }

    private:
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr battery_sub_;
        image_transport::CameraSubscriber cam_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
        rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr response_sub_;
        rclcpp::TimerBase::SharedPtr frame_timer_;

        cv_bridge::CvImagePtr cv_ptr;
        int16_t last_battery_data_;
        std::string current_command_;

        std::vector<long> frame_start_;
        int frame_thickness_;

        int battery_thickness_;
        int battery_font_size_;
        std::vector<long> battery_text_origin_;
        std::vector<long> battery_prct_origin_;
        
        std::vector<long> status_text_origin_;
        std::vector<long> status_origin_;
        int status_font_size_;
        int status_thickness_;
        
        std::vector<double> success_color_;
        std::vector<double> error_color_;

        std::vector<double> WHITE_ = {255.0, 255.0, 255.0};
        std::vector<double> BLACK_ = {0.0, 0.0, 0.0};
        std::vector<double> RED_ = {0.0, 0.0, 255.0};
        std::vector<double> STATUS_COLOR_ = {255.0, 255.0, 255.0};
        std::vector<double> FRAME_COLOR_ = {0.0, 255.0, 0.0};

        std::string status = "";
        
        int counter_ = 0;
        bool draw_frame_ = false;

        bool waiting;

        void make_frame() {
            //cv::rectangle(blue, cv::Point(130, 226), cv::Point(382, 286), cv::Scalar(255, 0, 255), 15);
            cv::Scalar color = cv::Scalar(this->FRAME_COLOR_[0], this->FRAME_COLOR_[1], this->FRAME_COLOR_[2]);
            cv::Point start = cv::Point(this->frame_start_[0], this->frame_start_[1]);
            
            cv::Size size = this->cv_ptr->image.size();
            cv::Point end = cv::Point(size.width - this->frame_thickness_, size.height - this->frame_thickness_);

            cv::rectangle(this->cv_ptr->image, start, end, color, this->frame_thickness_);
        }

        void get_params() {
            this->declare_parameter("frame_start", std::vector<long>({3,3}));
            this->declare_parameter("frame_thickness", 5);
            this->declare_parameter("battery_thickness", 5);
            this->declare_parameter("battery_font_size", 1);
            this->declare_parameter("battery_text_origin", std::vector<long>({20, 40}));
            this->declare_parameter("battery_prct_origin", std::vector<long>({200, 40}));
            this->declare_parameter("status_text_origin", std::vector<long>({20, 30}));
            this->declare_parameter("status_origin", std::vector<long>({0, 0}));
            this->declare_parameter("status_font_size", 2);
            this->declare_parameter("status_thickness", 2);
            this->declare_parameter("success_color", std::vector<double>({0.0, 255.0, 0.0}));
            this->declare_parameter("error_color", std::vector<double>({0.0, 0.0, 255.0}));

            this->frame_start_ = this->get_parameter("frame_start").as_integer_array();
            this->frame_thickness_ = this->get_parameter("frame_thickness").as_int();

            this->battery_thickness_ = this->get_parameter("battery_thickness").as_int();
            this->battery_font_size_ = this->get_parameter("battery_font_size").as_int();
            this->battery_text_origin_ = this->get_parameter("battery_text_origin").as_integer_array();
            this->battery_prct_origin_ = this->get_parameter("battery_prct_origin").as_integer_array();
            
            this->status_text_origin_ = this->get_parameter("status_text_origin").as_integer_array();
            this->status_origin_ = this->get_parameter("status_origin").as_integer_array();
            this->status_font_size_ = this->get_parameter("status_font_size").as_int();
            this->status_thickness_ = this->get_parameter("status_thickness").as_int();

            this->success_color_ = this->get_parameter("success_color").as_double_array();
            this->error_color_ = this->get_parameter("error_color").as_double_array();

            RCLCPP_DEBUG(this->get_logger(), "font_size: %d, thickness: %d", this->status_font_size_, this->status_thickness_);
        }

        void battery_callback(const std_msgs::msg::Int16::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Battery: %d", msg->data);
            this->last_battery_data_ = msg->data;
        }

        void response_callback(const tello_msgs::msg::TelloResponse::SharedPtr msg) {
            if(msg->str.find("error") != std::string::npos)
                this->FRAME_COLOR_ = this->error_color_;
            else
                this->FRAME_COLOR_ = this->success_color_;

            if(this->current_command_.find("takeoff") != std::string::npos){
                this->frame_timer_ = this->create_wall_timer(200ms, std::bind(&VideoInterface::frame_callback, this));
            }

            this->waiting = false;
        }


        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info_msg) {
            try {
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                this->make_interface();
                cv::imshow("video_interface_view", cv_ptr->image);//cv_bridge::toCvShare(img_msg, "bgr8")->image);
                cv::waitKey(10);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

        }

        void frame_callback(){
            this->draw_frame_ = !this->draw_frame_;
            this->counter_++;
        }

        void command_callback(const std_msgs::msg::String::SharedPtr msg) {
            this->current_command_ = msg->data;
            this->waiting = true;
            RCLCPP_DEBUG(this->get_logger(), "%s", this->current_command_.c_str());
        }

        void process_status(std::string command) {
            RCLCPP_DEBUG(this->get_logger(), "processing status");
            if(this->last_battery_data_ <= 10) {
                this->status = "Low battery.";
                this->STATUS_COLOR_ = this->RED_;
                return;
            }

            //this->waiting = (command_str.substr(sep+1, command_str.size()) == "True");
            
            if(command.find("flip") != std::string::npos && this->last_battery_data_ < 50) {
                this->status = "can't flip, too low battery level";
                this->STATUS_COLOR_ = this->RED_;
                return;
            }

            if(waiting) {
                this->status = "Waiting " + command;
                this->STATUS_COLOR_ = this->WHITE_;
                return;
            }

            this->status = "Ready";
            this->STATUS_COLOR_ = this->WHITE_;
        }

        void process_battery() {
            RCLCPP_DEBUG(this->get_logger(), "processing_battery");
            std::vector<double> final_color = {0.0, 0.0, 0.0};
            
            float step = 1.0 - (float(this->last_battery_data_) / 100.0);
            
            double blue_start = this->success_color_[0], green_start = this->success_color_[1], red_start = this->success_color_[2];
            double blue_end = this->error_color_[0], green_end = this->error_color_[1], red_end = this->error_color_[2];

            final_color[0] = (blue_end - blue_start) * step + blue_start;
            final_color[1] = (green_end - green_start) * step + green_start;
            final_color[2] = (red_end - red_start) * step + red_start;

            this->write_text(std::to_string(this->last_battery_data_), this->battery_prct_origin_, this->battery_thickness_*4, this->battery_font_size_, this->BLACK_);
            this->write_text(std::to_string(this->last_battery_data_), this->battery_prct_origin_, this->battery_thickness_, this->battery_font_size_, final_color);
        }

        void write_text(std::string text, std::vector<long> origin, int font_thickness, int font_size, std::vector<double> color) {
            cv::Point text_origin = {(int)origin[0], (int)origin[1]};
            cv::Scalar font_color = {color[0], color[1], color[2]};
            cv::putText(this->cv_ptr->image, text, text_origin, cv::FONT_HERSHEY_COMPLEX, font_size, font_color, font_thickness, cv::LINE_8);
        }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoInterface>());
  rclcpp::shutdown();
  return 0;
}
