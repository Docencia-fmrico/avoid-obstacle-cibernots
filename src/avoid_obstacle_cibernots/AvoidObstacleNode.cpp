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

#include <utility>
#include "avoid_obstacle_cibernots/AvoidObstacleNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace avoid_obstacle_cibernots
{

using namespace std::chrono_literals;
using std::placeholders::_1;

AvoidObstacle::AvoidObstacle()
: Node("avoid_obstacle"),
  state_(FORWARD),
  last_state_(FORWARD)
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::scan_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&AvoidObstacle::control_cycle, this));
  
  state_ts_ = now();
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

  geometry_msgs::msg::Twist out_vel;

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

      /*Revisar si ponerlo como estado, ajustar angulo de orientacion,
      y ver si es necesario*/
      if (last_state_ == TURN && avoided) {
        RCLCPP_INFO(get_logger(), "REORENTATION");
        out_vel.linear.x = 0.0f;
        out_vel.angular.z = SPEED_ANGULAR * side_;
        if ((now() - reorentation_t) > REORENTATION_TIME) {
          avoided = false;
        }
      }
      //////////////////////////

      // Si el ultimo estado fue TURN, avanzar en arco
      if (last_state_ == TURN && linear_distance < HALF_CIRCUMFERENCE) {
        RCLCPP_INFO(get_logger(), "AVANZO EN ARCO: %f, linear_distance = %f", HALF_CIRCUMFERENCE, linear_distance);
        linear_distance = SPEED_LINEAR * (now() - state_ts_).seconds();
        out_vel.angular.z = -SPEED_ANGULAR * side_;
        if (linear_distance >= HALF_CIRCUMFERENCE) {
          reorentation_t = now();
          avoided = true;
        }
      }

      if (check_forward_2_turn()) {
        RCLCPP_INFO(get_logger(),"FORWARD -> TURNING");
        linear_distance = 0.0;
        last_state_ = FORWARD;
        go_state(TURN);
      }

      break;
    case TURN:
      out_vel.linear.x = 0.0f;
      out_vel.angular.z = SPEED_ANGULAR * side_;

      // Una vez gira los 90º procede a avanzar en arco
      if (check_turn_2_forward()) {
        RCLCPP_INFO(get_logger(),"TURNING -> FORWARD");
        last_state_ = TURN;
        go_state(FORWARD);
      }

      break;
    case STOP:
      out_vel.linear.x = 0.0f;
      out_vel.angular.z = 0.0f;

      if (check_stop_2_forward()) {
        RCLCPP_INFO(get_logger(),"STOP -> FORWARD");
        go_state(FORWARD);
      }
      break;
  }

  vel_pub_->publish(out_vel);
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

  for (int j = 0; j < min_pos; j++) {
    if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) && last_scan_->ranges[j] < DISTANCE_DETECT) {
      detected_ = true;
      object_position_ = j;
      break;
    }
  }

  if (!detected_) {
    for (int j = max_pos; j < last_scan_->ranges.size(); j++) {
      if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) && last_scan_->ranges[j] < DISTANCE_DETECT) {
        detected_ = true;
        object_position_ = j;
        break;
      }
    }
  }

  if (max_pos < object_position_) {
    /*TURNING_LEFT*/
    side_ = -1;
  }
  else {
    side_ = 1;
    /*TURNING_RIGTH;*/
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
AvoidObstacle::check_turn_2_forward()
{
  return (now() - state_ts_) > TURNING_TIME;
}

}  // namespace avoid_obstacle_cibernots
