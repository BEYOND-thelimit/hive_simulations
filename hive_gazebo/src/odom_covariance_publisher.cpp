#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomNoiseMaker : public rclcpp::Node
{
 public:
  OdomNoiseMaker(/* args */);
  ~OdomNoiseMaker();
  void sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

      publisher1_->publish(*msg);
    }

 private:
  /* data */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher1_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher2_;
};

OdomNoiseMaker::OdomNoiseMaker(/* args */) : Node("odom_noise_maker")
{
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "robot1/odom", 10, std::bind(&OdomNoiseMaker::sub_callback, this, std::placeholders::_1));

  publisher1_ = this->create_publisher<nav_msgs::msg::Odometry>("robot1/odom_noise", 10);
}

OdomNoiseMaker::~OdomNoiseMaker()
{
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomNoiseMaker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}