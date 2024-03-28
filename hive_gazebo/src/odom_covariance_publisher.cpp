#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>

class OdomNoiseMaker : public rclcpp::Node
{
 public:
  OdomNoiseMaker(/* args */);
  ~OdomNoiseMaker();
  void sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

 private:
  /* data */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher1_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher2_;

  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
};

OdomNoiseMaker::OdomNoiseMaker(/* args */) : Node("odom_noise_maker"), distribution_(0.0, 1.0)
{
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "robot1/odom", 10, std::bind(&OdomNoiseMaker::sub_callback, this, std::placeholders::_1));

  publisher1_ = this->create_publisher<nav_msgs::msg::Odometry>("robot1/odom_noise", 10);
}

OdomNoiseMaker::~OdomNoiseMaker()
{
}

void OdomNoiseMaker::sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // x, y 위치에 랜덤 noise 추가
  double noise_x = distribution_(generator_);
  double noise_y = distribution_(generator_);

  msg->pose.pose.position.x += noise_x;
  msg->pose.pose.position.y += noise_y;

  // 수정된 메시지를 발행
  publisher1_->publish(*msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomNoiseMaker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}