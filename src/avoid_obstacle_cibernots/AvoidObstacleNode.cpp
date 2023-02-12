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

  out_vel.angular.z = SPEED_ANGULAR;

  if (check_turn_2_forward()) {
    state_ts_ = now();
    out_vel.linear.x = -SPEED_LINEAR;
  }
  std::cout << "Tiempo de giro" << time_turn << std::endl;
  /*switch (state_) {
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;
      out_vel.angular.z = 0f;

      if (check_forward_2_stop()) {
        out_vel.linear.x = 0f;
        last_state_ = FORWARD;
        go_state(STOP);
      }

      // Si el ultimo estado fue turn, avanzar en arco durante x segundos
      if (last_state_ == TURN && now() - state_ts_ < 3s) {
        out_vel.linear.x = 0.8 * SPEED_ANGULAR*direction;
        out_vel.angular.z = SPEED_ANGULAR*direction;
      }

      if (check_forward_2_turn()) {
        last_state_ = FORWARD;
        go_state(TURN);
      }
      break;
    case TURN:
      out_vel.linear.x = 0f;
      out_vel.angular.z = SPEED_ANGULAR*direction;

      // Una vez gira los 90º procede a avanzar en arco
      if (check_turn_2_forward()) {
        last_state_ = TURN;
        go_state(FORWARD);
      }

      break;
    case STOP:
      out_vel.linear.x = 0f;
      out_vel.angular.z = 0f;

      if (check_stop_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }*/

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
  // Implementar lógica del laser para detectar objeto
  // en un abanico de 120 grados o menos.
  // Actualizar la variable direction en funcion del lado
  // en el que se detecte el objeto.
  size_t pos = last_scan_->ranges.size() / 2;
  return last_scan_->ranges[pos] < OBSTACLE_DISTANCE;
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
  // Una Opcion
  // En topic /odom, obtenemos orientacion y giramos 90º
  // Pos inicial, nav_msgs/msg/Odometry(tipo de mensajes)
  /*  x: 4.838973292918089e-05
      y: -0.006673698700173976
      z: -0.007270319308134201
      w: 0.9999512997447679*/
    
  return (now() - state_ts_) > TURNING_TIME;
}

}  // namespace avoid_obstacle_cibernots
