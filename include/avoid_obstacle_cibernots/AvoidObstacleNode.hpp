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
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();

  int min_pos = 45;
  int max_pos = 315;
  
  int object_position_;

  static const int FORWARD = 0;
  static const int TURN = 1;
  static const int STOP = 2;
  static const int REOR = 3;
  int state_;
  int last_state_;
  rclcpp::Time state_ts_;
  rclcpp::Time reorentation_t;

  void go_state(int new_state);
  bool check_forward_2_turn();
  bool check_forward_2_stop();
  bool check_turn_2_arch();
  bool check_stop_2_forward();
  bool check_reor_2_forward();

  static constexpr float SPEED_LINEAR = 0.25f;
  static constexpr float SPEED_ANGULAR = 0.5f;
  static constexpr float OBSTACLE_DISTANCE = 1.0f;

  double time_turn = M_PI_2/SPEED_ANGULAR;

  rclcpp::Duration TURNING_TIME = rclcpp::Duration::from_seconds(time_turn);
  rclcpp::Duration REORENTATION_TIME = rclcpp::Duration::from_seconds(M_PI_4/SPEED_ANGULAR);
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  int side_ = 1; // 1 o -1 para indicar el sentido de giro

  const double HALF_CIRCUMFERENCE = M_PI * 0.6;
  double linear_distance = 0.0;

  bool avoided = false;
};

}  // namespace avoid_obstacle_cibernots

#endif  // AVOID_OBSTACLE_CIBERNOTS__AvoidObstacle_HPP_
