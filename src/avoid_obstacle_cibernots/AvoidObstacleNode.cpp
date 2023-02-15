// Copyright 2021 Intelligent Robotics Lab
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
: Node("bump_go"),
  state_(FORWARD)
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

      if (check_forward_2_stop()) {
        go_state(STOP);
      }

      if (check_forward_2_turn()) {
        go_state(TURN);
      }
      break;
    case TURN:
      out_vel.angular.z = SPEED_ANGULAR*side_;

      if (check_turn_2_forward()) {
        go_state(FORWARD);
      }

      break;
    case STOP:
      if (check_stop_2_forward()) {
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

void
AvoidObstacle::dist_med_obstacles()
{
  int n = 0;

  /* recorremos los valores del laser de 0-45º */
  for(int j = 0; j < min_pos; j++){
    if((last_scan_->ranges[j] < last_scan_->range_max) && (last_scan_->ranges[j] > last_scan_->range_min)){
      laser_dist_ += last_scan_->ranges[j];
      n++;
    }
  }
  /* recorremos los valores del laser de 315-360º*/
  for(int j = max_pos; j < LONG_MED; j++){
    if((last_scan_->ranges[j] < last_scan_->range_max) && (last_scan_->ranges[j] > last_scan_->range_min)){
      laser_dist_ += last_scan_->ranges[j];
      n++;
    }
  }

  laser_dist_ = laser_dist_/n;
}


bool
AvoidObstacle::check_forward_2_turn()
{
  bool detected_=false;

  for(int j = 0; j < min_pos; j++){
    if(last_scan_->ranges[j] < DISTANCE_DETECT && (last_scan_->ranges[j] < last_scan_->range_max) && (last_scan_->ranges[j] > last_scan_->range_min)){
      detected_ = true;
      object_position_ = j;
      break;
    }
  }

  if(!detected_){
    for(int j = max_pos; j < last_scan_->ranges.size(); j++){
      if(last_scan_->ranges[j] < DISTANCE_DETECT && (last_scan_->ranges[j] < last_scan_->range_max) && (last_scan_->ranges[j] > last_scan_->range_min)){
        detected_ = true;
        object_position_ = j;
        break;
      }
    }
  }

  if( max_pos < object_position_ && object_position_ < LONG_MED)
  {
    /*state_ = TURNING_RIGHT;*/
    side_ = -1;
  }
  else
  {
    side_ = 1;
    /*state_ = TURNING_LEFT;*/
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
AvoidObstacle::check_back_2_turn()
{
  // Going back for 2 seconds
  return (now() - state_ts_) > BACKING_TIME;
}

bool
AvoidObstacle::check_turn_2_forward()
{
  // Turning for 2 seconds
  return (now() - state_ts_) > TURNING_TIME;
}

}  // namespace avoid_obstacle_cibernots
