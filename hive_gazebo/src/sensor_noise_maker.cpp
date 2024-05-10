#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class SensorNoiseMaker : public rclcpp::Node
{
 public:
  SensorNoiseMaker(/* args */);
  ~SensorNoiseMaker();
  void sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

 private:
  /* data */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher1_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher2_;

  double sigma_x_;
  double sigma_y_;
  double sigma_yaw_;

  double noise_x_;
  double noise_y_;
  double noise_yaw_;

  bool moving_ = false;
};

SensorNoiseMaker::SensorNoiseMaker(/* args */) : Node("sensor_noise_maker")
{
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "robot1/odom", 10, std::bind(&SensorNoiseMaker::sub_callback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "robot1/imu", 10, std::bind(&SensorNoiseMaker::imuCallback, this, std::placeholders::_1));
  vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "robot1/cmd_vel", 10, std::bind(&SensorNoiseMaker::velCallback, this, std::placeholders::_1));

  publisher1_ = this->create_publisher<nav_msgs::msg::Odometry>("robot1/odom_noise", 10);
  publisher2_ = this->create_publisher<sensor_msgs::msg::Imu>("robot1/imu_noise", 10);

  sigma_x_ = 0.1;
  sigma_y_ = 0.1;
  sigma_yaw_ = 0.1;
}

SensorNoiseMaker::~SensorNoiseMaker()
{
}

void SensorNoiseMaker::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (msg->linear.x != 0.0 || msg->angular.z != 0.0)
  {
    moving_ = true;
  }
  else
  {
    moving_ = false;
  }
}

void SensorNoiseMaker::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  imu_msg->orientation_covariance[0] = sigma_x_ * sigma_x_;
  imu_msg->orientation_covariance[4] = sigma_y_ * sigma_y_;
  imu_msg->orientation_covariance[8] = sigma_yaw_ * sigma_yaw_;

  imu_msg->angular_velocity_covariance[0] = sigma_x_ * sigma_x_;
  imu_msg->angular_velocity_covariance[4] = sigma_y_ * sigma_y_;
  imu_msg->angular_velocity_covariance[8] = sigma_yaw_ * sigma_yaw_;

  imu_msg->linear_acceleration_covariance[0] = sigma_x_ * sigma_x_;
  imu_msg->linear_acceleration_covariance[4] = sigma_y_ * sigma_y_;
  imu_msg->linear_acceleration_covariance[8] = sigma_yaw_ * sigma_yaw_;

  publisher2_->publish(*imu_msg);
}

void SensorNoiseMaker::sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (moving_)
  {
    sigma_x_ += 0.0001;
    sigma_y_ += 0.0001;
    sigma_yaw_ += 0.0001;

    // Make Noise from Gaussian distribution
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist_x(0.0, sigma_x_);
    std::normal_distribution<double> dist_y(0.0, sigma_y_);
    std::normal_distribution<double> dist_yaw(0.0, sigma_yaw_);

    // Add noise
    noise_x_ = dist_x(gen);
    noise_y_ = dist_y(gen);
    noise_yaw_ = dist_yaw(gen);
    // RCLCPP_INFO(this->get_logger(), "Adding noise to odometry: %s %s %s",
    // std::to_string(noise_x_).c_str(), std::to_string(noise_y_).c_str(), std::to_string(noise_yaw_).c_str());
  }

  msg->pose.pose.position.x += noise_x_;
  msg->pose.pose.position.y += noise_y_;

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
  yaw += noise_yaw_;

  // 수정된 Yaw로 Quaternion 업데이트
  q.setRPY(roll, pitch, yaw);
  msg->pose.pose.orientation.x = q.x();
  msg->pose.pose.orientation.y = q.y();
  msg->pose.pose.orientation.z = q.z();
  msg->pose.pose.orientation.w = q.w();

  // 공분산 설정
  msg->pose.covariance[0] = sigma_x_ * sigma_x_;
  msg->pose.covariance[7] = sigma_y_ * sigma_y_;
  msg->pose.covariance[35] = sigma_yaw_ * sigma_yaw_;

  // 수정된 메시지를 발행
  publisher1_->publish(*msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorNoiseMaker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}