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

#include <utility>
#include <chrono>

#include "avoid_obstacle_cibernots/AvoidObstacleNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace avoid_obstacle_cibernots
{

using namespace std::chrono_literals;
using std::placeholders::_1;

AvoidObstacle::AvoidObstacle()
: Node("avoid_obstacle"),
  state_(FORWARD),
  last_state_(FORWARD),
  pressed_(false)
{
  declare_parameter("SPEED_LINEAR", 0.0f);
  declare_parameter("SPEED_ANGULAR", 0.0f);
  declare_parameter("OBSTACLE_DISTANCE", 0.0f);
  declare_parameter("MULTIP_ARCH", 0.0f);
  declare_parameter("MULTIP_TURN", 0.0f);

  get_parameter("SPEED_LINEAR", SPEED_LINEAR);
  get_parameter("SPEED_ANGULAR", SPEED_ANGULAR);
  get_parameter("OBSTACLE_DISTANCE", OBSTACLE_DISTANCE);
  get_parameter("MULTIP_ARCH", MULTIP_ARCH);
  get_parameter("MULTIP_TURN", MULTIP_TURN);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::scan_callback, this, _1));

  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::button_callback, this, _1));

  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "input_bumper", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::bumper_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);
  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("output_led", 10);

  timer_ = create_wall_timer(50ms, std::bind(&AvoidObstacle::control_cycle, this));

  t_arch = ((M_PI_2 * M_PI * SPEED_LINEAR) / SPEED_ANGULAR) * MULTIP_ARCH;
  t_turn_90d = (M_PI_2 / SPEED_ANGULAR) * MULTIP_TURN;

  TURNING_TIME = rclcpp::Duration::from_seconds(t_turn_90d);
  REORENTATION_TIME = rclcpp::Duration::from_seconds(t_turn_90d * 0.85);

  state_ts_ = now();
}

void
AvoidObstacle::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  pressed_ = false;
  kobuki_ros_interfaces::msg::Sound out_sound;
  out_sound.value = kobuki_ros_interfaces::msg::Sound::ERROR;
  sound_pub_->publish(out_sound);
}


void
AvoidObstacle::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::ButtonEvent::PRESSED)
  {
    RCLCPP_INFO(get_logger(), "BUTTON PRESSED");
    pressed_ = !pressed_;
  }
}


void
AvoidObstacle::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
AvoidObstacle::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    return;
  }

  if (pressed_) {
    geometry_msgs::msg::Twist out_vel;
    kobuki_ros_interfaces::msg::Led out_led;

    out_led.value = kobuki_ros_interfaces::msg::Led::BLACK;

    switch (state_) {
      case FORWARD:
        out_vel.linear.x = SPEED_LINEAR;
        out_vel.angular.z = 0.0f;

        if (check_forward_2_stop()) {
          out_vel.linear.x = 0.0f;
          RCLCPP_INFO(get_logger(), "FORWARD -> STOP");
          last_state_ = FORWARD;
          go_state(STOP);
        }

        if (check_forward_2_turn()) {
          RCLCPP_INFO(get_logger(), "FORWARD -> TURN");
          linear_distance = 0.0;
          last_state_ = FORWARD;
          go_state(TURN);
        }
        break;

      case TURN:
        out_vel.linear.x = 0.0f;
        out_vel.angular.z = SPEED_ANGULAR * side_;

        // Una vez gira los 90º procede a avanzar en arco
        if (check_turn_2_arch()) {
          RCLCPP_INFO(get_logger(), "TURNING -> ARCH");
          last_state_ = TURN;
          go_state(ARCH);
        }
        break;

      case STOP:
        out_led.value = kobuki_ros_interfaces::msg::Led::ORANGE;
        out_vel.linear.x = 0.0f;
        out_vel.angular.z = 0.0f;

        if (check_stop_2_forward()) {
          RCLCPP_INFO(get_logger(), "STOP -> FORWARD");
          go_state(FORWARD);
        }
        break;

      case REOR:
        out_vel.linear.x = 0.0f;
        out_vel.angular.z = SPEED_ANGULAR * side_;

        // Una vez se reorienta procede a avanzar
        if (check_reor_2_forward()) {
          RCLCPP_INFO(get_logger(), "REOR -> FORWARD");
          last_state_ = REOR;
          go_state(FORWARD);
        }
        break;

      case ARCH:
        out_led.value = kobuki_ros_interfaces::msg::Led::RED;

        linear_distance = SPEED_LINEAR * (now() - state_ts_).seconds();
        out_vel.linear.x = SPEED_LINEAR;
        out_vel.angular.z = -SPEED_ANGULAR * side_;

        if (check_arch_2_reor()) {
          last_state_ = ARCH;
          go_state(REOR);
          break;
        }

        if (check_arch_2_turn()) {
          RCLCPP_INFO(get_logger(), "ARCH -> TURN");
          last_state_ = ARCH;
          go_state(TURN);
          break;
        }
    }

    led_pub_->publish(out_led);
    vel_pub_->publish(out_vel);
  }
}

void
AvoidObstacle::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
AvoidObstacle::check_forward_2_turn()
{
  bool detected_ = false;


  for (int j = 0; j < MIN_POS; j++) {
    if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) &&
      last_scan_->ranges[j] < OBSTACLE_DISTANCE)
    {
      detected_ = true;
      object_position_ = j;
      break;
    }
  }

  if (!detected_) {
    for (int j = MAX_POS; j < last_scan_->ranges.size(); j++) {
      if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) &&
        last_scan_->ranges[j] < OBSTACLE_DISTANCE)
      {
        detected_ = true;
        object_position_ = j;
        break;
      }
    }
  }


  if (MAX_POS < object_position_) {
    /*TURNING_LEFT*/
    side_ = 1;
  } else {
    /*TURNING_RIGTH;*/
    side_ = -1;
  }

  return detected_;
}

bool
AvoidObstacle::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
AvoidObstacle::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // again
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
AvoidObstacle::check_turn_2_arch()
{
  return (now() - state_ts_) > TURNING_TIME;
}

bool
AvoidObstacle::check_reor_2_forward()
{
  return (now() - state_ts_) > REORENTATION_TIME;
}


bool
AvoidObstacle::check_arch_2_reor()
{
  return linear_distance >= (SPEED_LINEAR * t_arch);
}

bool
AvoidObstacle::check_arch_2_turn()
{
  return check_forward_2_turn();
}


}  // namespace avoid_obstacle_cibernots
