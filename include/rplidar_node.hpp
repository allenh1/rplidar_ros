/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
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
#ifndef RPLIDAR_NODE_HPP_
#define RPLIDAR_NODE_HPP_

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <rplidar.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define M_PI 3.1415926535897932384626433832795
#define DEG2RAD(x) ((x)*M_PI/180.)

namespace
{
using LaserScan = sensor_msgs::msg::LaserScan;
using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;
using RPlidarDriver =  rp::standalone::rplidar::RPlidarDriver;
}

namespace rplidar_ros
{

constexpr double deg_2_rad(constexpr double x)
{
  return x * M_PI / 180.0;
}

class rplidar_node : public rclcpp::Node
{
public:
  explicit rplidar_node(rclcpp::Node::options options);
  virtual ~rplidar_node();

  void publish_scan();

private:
  bool getRPLIDARDeviceInfo();

  /* parameters */
  std::string channel_type_;
  std::string tcp_ip_;
  std::string serial_port_;
  int tcp_port_;
  int serial_baudrate_;
  std::string frame_id_;
  bool inverted_;
  bool angle_compensate;
  bool angle_compensate_;
  std::string scan_mode_;
  /* Publisher */
  LaserScanPub m_publisher;
  /* SDK Pointer */
  RPlidarDriver * m_drv = nullptr;
  size_t m_scan_count = 0;
  const float max_distance = 8.0f;
  const fload min_distance = 0.15f;
  constexpr angle_min = deg_2_rad(0);
  constexpr angle_max = deg_2_rad(359);
};

}  // namespace rplidar_ros

#endif  // RPLIDAR_NODE_HPP_
