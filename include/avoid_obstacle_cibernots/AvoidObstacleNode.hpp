// Copyright 2023 avoid_obstacle_cibernots.copyright
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

#ifndef AVOID_OBSTACLE_CIBERNOTS__AVOIDOBSTACLENODE_HPP_
#define AVOID_OBSTACLE_CIBERNOTS__AVOIDOBSTACLENODE_HPP_

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
  // Callback bumper event (stop if bumper is pressed)
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);

  // Callback button event (start and stop if button is pressed)
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);

  // Callback scan event (get the last scan)
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

  // Timer callback (control the robot)
  void control_cycle();

  // Constant max and min range of the laser
  static const int MAX_POS = 320;
  static const int MIN_POS = 40;

  // Constant amplitude of values of the laser
  static const int LEN_MEDS = 80;

  // Side of rotation
  int side_ = 1;  // 1(izq) o -1(der)

  // Array of values of the laser
  int object_position_[LEN_MEDS];

  // States of the robot
  static const int FORWARD = 0;
  static const int TURN = 1;
  static const int STOP = 2;
  static const int REOR = 3;
  static const int ARCH = 4;

  // State of the robot
  int state_;

  // Last state of the robot
  int last_state_;

  // Time since the last state change
  rclcpp::Time state_ts_;

  // Go to a new state
  void go_state(int new_state);

  // Check forward to turn (if there is an obstacle)
  bool check_forward_2_turn();

  // Check forward to stop (if there is not laser data)
  bool check_forward_2_stop();

  // Check turn to arch (if the robot has rotated 90 degrees)
  bool check_turn_2_arch();

  // Check stop to forward (if the robot has laser data)
  bool check_stop_2_forward();

  // Check reorientation to forward (if the robot has rotated 90 degrees)
  bool check_reor_2_forward();

  // Check arch to reorientation (if the robot has finished the arch)
  bool check_arch_2_reor();

  // Check arch to turn (if there is an obstacle while the robot is doing the arch)
  bool check_arch_2_turn();

  // Check side to turn
  int obstacle_side();

  // Initialize the speed of the robot (later it will be changed getting the parameters)
  float SPEED_LINEAR = 0.0f;
  float SPEED_ANGULAR = 0.0f;

  // Initialize the distance of the obstacle (later it will be changed getting the parameters)
  float OBSTACLE_DISTANCE = 0.0f;

  // Initialize the multiplicative factors of the time.
  float MULTIP_ARCH = 0.0f;
  float MULTIP_TURN = 0.0f;

  // Initialize the time of the arch and the turn
  double t_arch;
  float t_turn_90d;

  // Initialize the distance recorred by the arch
  double linear_distance = 0.0;

  // Initialize the time of the arch and the turn converted to rclcpp::Duration
  rclcpp::Duration TURNING_TIME = rclcpp::Duration::from_seconds(t_turn_90d);
  rclcpp::Duration REORENTATION_TIME = rclcpp::Duration::from_seconds(t_turn_90d * 0.85);

  // Time of scan
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  // Subscription of the button, bumper and laser
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Publisher of the speed, sound and led
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;

  // Timer of the control cycle
  rclcpp::TimerBase::SharedPtr timer_;

  // Last scan
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;

  // State of the button and bumper
  bool pressed_ = false;
};

}  // namespace avoid_obstacle_cibernots

#endif  // AVOID_OBSTACLE_CIBERNOTS__AVOIDOBSTACLENODE_HPP_
