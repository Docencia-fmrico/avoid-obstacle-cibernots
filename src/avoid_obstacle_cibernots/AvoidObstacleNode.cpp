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
#include "kobuki_ros_interfaces/msg/button_event.hpp"
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
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::scan_callback, this, _1));
  
  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacle::button_callback, this, _1));

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&AvoidObstacle::control_cycle, this));
  
  state_ts_ = now();
}

void
AvoidObstacle::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  pressed_ = true;
  RCLCPP_INFO(get_logger(), "BUTTON_PRESSED");
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

  if (pressed_) 
  {
    RCLCPP_INFO(get_logger(), "in pressed");
    geometry_msgs::msg::Twist out_vel;

    switch (state_) {
      case FORWARD:
        out_vel.linear.x = SPEED_LINEAR;
        out_vel.angular.z = 0.0f;
        RCLCPP_INFO(get_logger(), "FORWARD: %ld y %ld", now().nanoseconds(), state_ts_.nanoseconds());
        if (check_forward_2_stop()) {
          out_vel.linear.x = 0.0f;
          RCLCPP_INFO(get_logger(), "FORWARD -> STOP");
          last_state_ = FORWARD;
          go_state(STOP);
        }

        /*Revisar si ponerlo como estado, ajustar angulo de orientacion,
        y ver si es necesario*/
        /*if (last_state_ == TURN && avoided) {
          RCLCPP_INFO(get_logger(), "REORENTATION");
          out_vel.linear.x = 0.0f;
          out_vel.angular.z = SPEED_ANGULAR * side_;
          if ((now() - reorentation_t) > REORENTATION_TIME) {
            avoided = false;
          }
        }*/
        //////////////////////////

        if (check_forward_2_turn()) {
          RCLCPP_INFO(get_logger(),"FORWARD -> TURN");
          linear_distance = 0.0;
          last_state_ = FORWARD;
          go_state(TURN);
          RCLCPP_INFO(get_logger(), "CAMBIO: %ld y %ld", now().nanoseconds(), state_ts_.nanoseconds());
        }
        break;
      case TURN:
        out_vel.linear.x = 0.0f;
        out_vel.angular.z = SPEED_ANGULAR * side_;
        RCLCPP_INFO(get_logger(), "TURN: %ld y %ld", now().nanoseconds(), state_ts_.nanoseconds());
        // Una vez gira los 90º procede a avanzar en arco
        if (check_turn_2_arch()) {
          RCLCPP_INFO(get_logger(),"TURNING -> FORWARD");
          last_state_ = TURN;
          go_state(ARCH);
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
      case REOR:
        out_vel.linear.x = 0.0f;
        out_vel.angular.z = SPEED_ANGULAR * side_;
        RCLCPP_INFO(get_logger(), "REOR: %ld y %ld", now().nanoseconds(), state_ts_.nanoseconds());
        // Una vez se reorienta procede a avanzar
        if (check_reor_2_forward()) {
          RCLCPP_INFO(get_logger(),"REOR -> FORWARD");
          last_state_ = REOR;
          go_state(FORWARD);
        }
        break;

      case ARCH:
        // Si el ultimo estado fue TURN, avanzar en arco
        RCLCPP_INFO(get_logger(), "AVANZO EN ARCO: %f, linear_distance = %f", HALF_CIRCUMFERENCE, linear_distance);
        linear_distance = SPEED_LINEAR * (now() - state_ts_).seconds();
        out_vel.linear.x = SPEED_LINEAR;
        out_vel.angular.z = -SPEED_ANGULAR * side_;
        if (linear_distance >= HALF_CIRCUMFERENCE) {
          reorentation_t = now();
          last_state_ = ARCH;
          go_state(REOR);
          break;
        }
        if (check_forward_2_turn()){
          RCLCPP_INFO(get_logger(),"ARCH -> TURN");
          last_state_ = ARCH;
          go_state(TURN);
          break;
        }

    }

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


  for (int j = 0; j < min_pos; j++) {
    if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) && last_scan_->ranges[j] < OBSTACLE_DISTANCE) {
      detected_ = true;
      object_position_ = j;
      break;
    }
  }

  if (!detected_) {
    for (int j = max_pos; j < last_scan_->ranges.size(); j++) {
      if (!std::isinf(last_scan_->ranges[j]) && !std::isnan(last_scan_->ranges[j]) && last_scan_->ranges[j] < OBSTACLE_DISTANCE) {
        detected_ = true;
        object_position_ = j;
        break;
      }
    }
  }


  // if (max_pos < object_position_) {
  //   /*TURNING_LEFT*/
  //   side_ = -1;
  // }
  // else {
  //   side_ = 1;
    /*TURNING_RIGTH;*/

  //}

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
  return (now() - state_ts_) > TURNING_TIME;
}



}  // namespace avoid_obstacle_cibernots
