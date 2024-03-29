#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


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

  double sigma_x;
  double sigma_y;
  double sigma_yaw;

  double noise_x;
  double noise_y;
  double noise_yaw;
};

OdomNoiseMaker::OdomNoiseMaker(/* args */) : Node("odom_noise_maker")
{
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "robot1/odom", 10, std::bind(&OdomNoiseMaker::sub_callback, this, std::placeholders::_1));

  publisher1_ = this->create_publisher<nav_msgs::msg::Odometry>("robot1/odom_noise", 10);

  // 노이즈 생성 관련 변수 설정.
  std::random_device rd;
  std::mt19937 gen(rd());

  sigma_x = 0.1;
  sigma_y = 0.1;
  sigma_yaw = 0.1;

  std::normal_distribution<> dist_x(0, sigma_x);
  std::normal_distribution<> dist_y(0, sigma_y);
  std::normal_distribution<> dist_yaw(0, sigma_yaw);

  noise_x = dist_x(gen);
  noise_y = dist_y(gen);
  noise_yaw = dist_yaw(gen);
}

OdomNoiseMaker::~OdomNoiseMaker()
{
}

void OdomNoiseMaker::sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  msg->pose.pose.position.x += noise_x;
  msg->pose.pose.position.y += noise_y;

  // Yaw (자세)에 노이즈 추가하기 위한 준비
  // Quaternion에서 Yaw 추출, 노이즈 추가, 다시 Quaternion으로 변환
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Yaw에 노이즈 추가
  yaw += noise_yaw;

  // 수정된 Yaw로 Quaternion 업데이트
  q.setRPY(roll, pitch, yaw);
  msg->pose.pose.orientation.x = q.x();
  msg->pose.pose.orientation.y = q.y();
  msg->pose.pose.orientation.z = q.z();
  msg->pose.pose.orientation.w = q.w();

  // 공분산 설정
  msg->pose.covariance[0] = sigma_x * sigma_x;
  msg->pose.covariance[7] = sigma_y * sigma_y;
  msg->pose.covariance[35] = sigma_yaw * sigma_yaw;

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