#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <curl/curl.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

// curl callback: accumulate bytes into a std::string.
static size_t WriteToString(void* contents, size_t size, size_t nmemb, void* userp) {
  size_t total = size * nmemb;
  auto* s = static_cast<std::string*>(userp);
  s->append(static_cast<char*>(contents), total);
  return total;
}

// Validate command char.
bool is_valid(char c) {
  switch (c) {
    case 'F':
    case 'B':
    case 'L':
    case 'R':
    case 'S':
      return true;
    default:
      return false;
  }
}

// Helper to split strings.
static std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Simple GET. Returns true on success and fills out_body.
static bool HttpGet(const std::string& url, std::string& out_body, long& http_code, std::string& out_err) {
  out_body.clear();
  out_err.clear();
  http_code = 0;

  CURL* curl = curl_easy_init();
  if (!curl) {
    out_err = "curl_easy_init() failed";
    return false;
  }

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteToString);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out_body);

  // For LAN this is usually enough. Short timeout to avoid hanging the timer.
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 500L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 500L);

  CURLcode res = curl_easy_perform(curl);
  if (res != CURLE_OK) {
    out_err = curl_easy_strerror(res);
    curl_easy_cleanup(curl);
    return false;
  }

  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
  curl_easy_cleanup(curl);

  return (http_code >= 200 && http_code < 300);
}


class HttpStatusNode : public rclcpp::Node {

public:
  HttpStatusNode() : Node("http_status_node") {
    robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.1.34");
    robot_port_ = this->declare_parameter<int>("robot_port", 80);
    poll_rate_hz_ = this->declare_parameter<double>("poll_rate_hz", 10.0);
    RCLCPP_INFO(get_logger(), "car_bridge starting ...");
    RCLCPP_INFO(get_logger(), "robot_ip=%s robot_port=%d poll_rate_hz=%.2f", robot_ip_.c_str(), robot_port_, poll_rate_hz_);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/car/status", 10);
    health_pub_ = this->create_publisher<std_msgs::msg::String>("/car/health", 10);
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>("/car/cmd", 10, std::bind(&HttpStatusNode::onCmd, this, std::placeholders::_1));
    
    auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&HttpStatusNode::onTimer, this)
    );    
}
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr delayed_timer_;
  int consecutive_failures = 0;
  std_msgs::msg::String health_status_last;

  void sendCmd(char cmd) {
    
    std::string url =
      "http://" + robot_ip_ + ":" + std::to_string(robot_port_) + "/cmd?m=" + cmd;
    std::string body;
    std::string err;
    long code = 0;

    const bool ok = HttpGet(url, body, code, err);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "GET %s failed (http=%ld) err=%s",
                  url.c_str(), code, err.c_str());
      return;
    }
  }

  void onCmd( const std_msgs::msg::String::SharedPtr msg){
    std::vector<std::string> tokens = split(msg->data, ' ');
    char cmd = tokens[0][0];
    if(is_valid(cmd)) {
      sendCmd(cmd);
      if (delayed_timer_) {
        delayed_timer_->cancel();
      }
      if (cmd == 'S') {
        return; // No delay for stop command.
      }
        delayed_timer_ = this->create_wall_timer(
        std::chrono::seconds(20),
        [this]() {
          RCLCPP_INFO(get_logger(), "20s elapsed since last cmd");
          sendCmd('S');  // Send stop command after delay.
          delayed_timer_->cancel();
        }
        );
    } else {
      RCLCPP_WARN(this->get_logger(), "Received INVALID cmd: '%c'", cmd);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 500,
                         "Received cmd: '%s'", msg->data.c_str());
  }

  void onTimer() {
    // Build URL.
    const std::string url =
      "http://" + robot_ip_ + ":" + std::to_string(robot_port_) + "/status";

    // Perform GET.
    std::string body;
    std::string err;
    long code = 0;
    
    std_msgs::msg::String health_msg;
    const bool ok = HttpGet(url, body, code, err);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "GET %s failed (http=%ld) err=%s",
                  url.c_str(), code, err.c_str());
      consecutive_failures++;
      if(consecutive_failures > 5) {
        health_msg.data = "DISCONNECTED";
        if(health_msg.data != health_status_last.data) {
          health_status_last = health_msg;
          health_pub_->publish(health_msg);
          RCLCPP_ERROR(get_logger(), "Multiple consecutive GET %s failures. Check robot connection.", url.c_str());
        }
      }
      return;
    } else {
      consecutive_failures = 0;
    }
    
    // Parse body (format: "C AAAAA" where C is cmd and AAAAA is age_ms).
    std::vector<std::string> tokens = split(body, ' ');
    if (tokens.size() < 2) {
      health_msg.data = "BAD_FORMAT";
      if(health_msg.data != health_status_last.data) {
        health_status_last = health_msg;
        health_pub_->publish(health_msg);
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500, "Unexpected status format: '%s'", body.c_str());
      }
      return;
    }
    char cmd = tokens[0][0];
    int age_ms = std::stoi(tokens[1]);
    // Publish parsed status.
    std_msgs::msg::String msg;
    msg.data = std::string(1, cmd) + " " + std::to_string(age_ms);
    status_pub_->publish(msg);

    // Log raw status for visibility.
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                         "Status: '%s' (http=%ld)", body.c_str(), code);
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                         "Parsed - cmd: %c, age_ms: %d", cmd, age_ms);
    if (age_ms > 5000) {
      health_msg.data = "STALE";
      if(health_msg.data != health_status_last.data) {
        health_status_last = health_msg;
        health_pub_->publish(health_msg);
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Status age is high: %d ms", age_ms);
    }
    else {
      health_msg.data = "OK";
      if(health_msg.data != health_status_last.data) {
        health_status_last = health_msg;
        health_pub_->publish(health_msg);
      }
    }
  }
  std::string robot_ip_;
  int robot_port_;
  double poll_rate_hz_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HttpStatusNode>());
  rclcpp::shutdown();
  return 0;
}
