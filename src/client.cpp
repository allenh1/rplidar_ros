/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

constexpr auto M_PI{3.1415926535897932384626433832795};

constexpr float radians_to_degrees(float radians)
{
   return radians * 180 / M_PI;
}

void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{
   const auto& logger = rclcpp::get_logger("RPlidar Client");
   const auto count = static_cast<size_t>(scan->scan_time / scan->time_increment);

   RCLCPP_INFO(logger, "I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
   RCLCPP_INFO(logger, "angle_range, %f, %f", radians_to_degrees(scan->angle_min), radians_to_degrees(scan->angle_max));

   for (size_t index = 0; index < count; ++index) {
      float degree = radians_to_degrees(scan->angle_min + scan->angle_increment * index);
      RCLCPP_INFO(logger, ": [%f, %f]", degree, scan->ranges[index]);
   }
}

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("rplidar_node_client");
   rclcpp::QoS qos{1};
   auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos, scanCallback);
   rclcpp::spin(node);
   return 0;
}
