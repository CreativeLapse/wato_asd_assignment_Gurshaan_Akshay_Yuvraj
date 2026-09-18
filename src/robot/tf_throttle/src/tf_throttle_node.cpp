#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace robot
{

// Gazebo's pose publisher streams transforms at up to 1 kHz, far more than
// Foxglove or our nodes can use. This node keeps the newest transform for
// every frame it hears about and republishes the whole set at a fixed rate
// whenever something new has arrived, so the stream is thinned without any
// frame ever being dropped.
class TfThrottleNode : public rclcpp::Node {
  public:
    TfThrottleNode() : Node("tf_throttle"), dirty_(false)
    {
      const std::string input_topic = this->declare_parameter<std::string>("input_topic", "/tf_raw");
      const std::string output_topic = this->declare_parameter<std::string>("output_topic", "/tf");
      const int period_ms = this->declare_parameter<int>("publish_period_ms", 50);

      sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        input_topic, rclcpp::QoS(100),
        std::bind(&TfThrottleNode::onTransforms, this, std::placeholders::_1));
      pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>(output_topic, rclcpp::QoS(100));
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms), std::bind(&TfThrottleNode::onTimer, this));

      RCLCPP_INFO(this->get_logger(), "Throttling %s -> %s at %d ms",
                  input_topic.c_str(), output_topic.c_str(), period_ms);
    }

  private:
    void onTransforms(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
      // A child frame has exactly one parent, so its name identifies the transform.
      for (const auto& transform : msg->transforms) {
        latest_[transform.child_frame_id] = transform;
      }
      dirty_ = true;
    }

    void onTimer()
    {
      if (!dirty_) {
        return;
      }
      tf2_msgs::msg::TFMessage out;
      out.transforms.reserve(latest_.size());
      for (const auto& [child_frame, transform] : latest_) {
        out.transforms.push_back(transform);
      }
      pub_->publish(out);
      dirty_ = false;
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Newest transform seen for each child frame.
    std::map<std::string, geometry_msgs::msg::TransformStamped> latest_;
    bool dirty_;
};

}  // namespace robot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot::TfThrottleNode>());
  rclcpp::shutdown();
  return 0;
}
