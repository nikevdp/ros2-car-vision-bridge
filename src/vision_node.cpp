#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "std_msgs/msg/string.hpp"

class VisionNode : public rclcpp::Node {
public:
  VisionNode() : Node("vision_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10,
      std::bind(&VisionNode::cb, this, std::placeholders::_1)
    );
    cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/car/cmd", 10);
    cont_pub_ = this->create_publisher<std_msgs::msg::String>("/car/cont", 10); //debug topic

    RCLCPP_INFO(get_logger(), "Subscribed to /image_raw");
  }

private:
  cv::RNG rng{12345};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cont_pub_;//debug topic
  
  std::shared_ptr<std_msgs::msg::String> cmd = std::make_shared<std_msgs::msg::String>();
  std::shared_ptr<std_msgs::msg::String> pre_cmd = std::make_shared<std_msgs::msg::String>();
  void cb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      const cv::Mat& frame = cv_ptr->image;
      cv::Mat no_traking_frame = frame.clone();
      cv::Mat hsv_frame;
      cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

      cv::Mat mask1, mask2, mask;

      cv::inRange(hsv_frame,
            cv::Scalar(0,   120, 80),
            cv::Scalar(10,  255, 255),
            mask1);

      cv::inRange(hsv_frame,
            cv::Scalar(170, 120, 80),
            cv::Scalar(179, 255, 255),
            mask2);

      mask = mask1 | mask2;


      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

      cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

      std::vector<std::vector<cv::Point>> contours;
    
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC3);
      /*for( size_t i = 0; i< contours.size(); i++ )
      { 
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        cv::drawContours( drawing, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 ); 
      }*/ //habilitar si queremos ver los contornos con imshow

      if (!contours.empty()) {
        auto max_it = std::max_element(
        contours.begin(),
        contours.end(),
        [](const std::vector<cv::Point>& a,
           const std::vector<cv::Point>& b) {
          return cv::contourArea(a) < cv::contourArea(b);
        }
        );
  
        double max_area = cv::contourArea(*max_it);
        std_msgs::msg::String msg;
        msg.data = std::to_string(max_area);
        if(max_area > 1000) {
          cont_pub_->publish(msg); //debug topic to see area size
        }
       
        if (max_area > 500) {
          
          cv::Mat debug_frame = frame.clone();
          cv::drawContours(debug_frame, std::vector<std::vector<cv::Point>>{*max_it},
                           -1, cv::Scalar(0,255,0), 2);
          cv::Moments m = cv::moments(*max_it);
        
          if (m.m00 != 0) {
            int error_x = 0;
            int error_y = 0;
            int cx = int(m.m10 / m.m00);
            int cy = int(m.m01 / m.m00);
            cv::circle(debug_frame, cv::Point(cx, cy), 6, cv::Scalar(0,255,0), -1);
            error_x = cx - (debug_frame.cols/2);
            error_y = cy - (debug_frame.rows/2);
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 500,
            "Error x: %d, y: %d", error_x, error_y);
            if(cmd->data == ""){
              cmd->data = "S";
            }
            else if(error_x >= -50 && error_x <= 50){
              if(max_area < 8000){
                cmd->data = "F";
              }
              else if(max_area >= 15000){
                cmd->data = "B";
              }
              else{
                cmd->data = "S";
              }
            }
            else if(error_x > 50){
              cmd->data = "L";
            }
             else if(error_x < -50){
              cmd->data = "R";
            }
            else{
              cmd->data = "S";
            }
            
            
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 500,
            "Actual direction: %s", cmd->data.c_str());
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 500,
            "Actual pre_direction: %s", pre_cmd->data.c_str());
          }
            
          cv::imshow("tracked", debug_frame);
        } else {
          cv::imshow("tracked", no_traking_frame); // se dejo de detectar
          if(pre_cmd->data != "S"){
            cmd->data = "S";
          }
        }
      } else {
        cv::imshow("tracked", no_traking_frame); // nunca se detecto nada
        if(pre_cmd->data != "S"){
          cmd->data = "S";
        }
      }
      if(cmd->data != "" && cmd->data != pre_cmd->data){
        cmd_pub_->publish(*cmd);
        pre_cmd->data = cmd->data;
      }
      cv::waitKey(1);

      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "frame: %dx%d", frame.cols, frame.rows);

    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
