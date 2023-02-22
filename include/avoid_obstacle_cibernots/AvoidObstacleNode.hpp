// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AVOID_OBSTACLE_CIBERNOTS__AVOIDOBSTACLE_HPP_
#define AVOID_OBSTACLE_CIBERNOTS__AVOIDOBSTACLE_HPP_

#include <math.h>

#include <chrono>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"

#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace avoid_obstacle_cibernots
{

using namespace std::chrono_literals;  // NOLINT

class AvoidObstacle : public rclcpp::Node
{
public:
  AvoidObstacle();

private:
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();

  //int min_pos = 45;
  int min_pos = 40;

  //int max_pos = 315;
  int max_pos = 320;

  
  int object_position_;

  static const int FORWARD = 0;
  static const int TURN = 1;
  static const int STOP = 2;
  static const int REOR = 3;
  static const int ARCH = 4;
  int state_;
  int last_state_;
  rclcpp::Time state_ts_;
  rclcpp::Time reorentation_t;

  int LED = 0;

  void go_state(int new_state);
  bool check_forward_2_turn();
  bool check_forward_2_stop();
  bool check_turn_2_arch();
  bool check_stop_2_forward();
  bool check_reor_2_forward();
  bool check_arch_2_reor();
  bool check_arch_2_turn();

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.6f;
  static constexpr float OBSTACLE_DISTANCE = 0.5f;

  
  double time_turn = (M_PI_2) * (M_PI * SPEED_LINEAR / SPEED_ANGULAR) * 3.2;
  float time_turn_90 = M_PI_2 / SPEED_ANGULAR * 1.5;

  //double time_turn = M_PI/SPEED_ANGULAR;

  rclcpp::Duration TURNING_TIME = rclcpp::Duration::from_seconds(time_turn_90);
  // rclcpp::Duration REORENTATION_TIME = rclcpp::Duration::from_seconds(M_PI_4/SPEED_ANGULAR);
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  bool pressed_ = false;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  

  int side_ = 1; // 1 o -1 para indicar el sentido de giro

  const double HALF_CIRCUMFERENCE = M_PI_2;// * 0.5;
  double linear_distance = 0.0;

  bool avoided = false;
};

}  // namespace avoid_obstacle_cibernots

#endif  // AVOID_OBSTACLE_CIBERNOTS__AvoidObstacle_HPP_
