/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2019 Hunter L. Allen
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

// Turn off warnings from sdk library.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wpedantic"
#include <rplidar.h>
#pragma GCC diagnostic pop

#include <chrono>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visibility.h>

namespace {
constexpr auto M_PI{3.1415926535897932384626433832795};
constexpr auto MAX_SAMPLE_COUNT = 360 * 8;
constexpr float DEFAULT_FREQUENCY = 10.0; // hz

using LaserScan = sensor_msgs::msg::LaserScan;
using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;
using StartMotorService = rclcpp::Service<std_srvs::srv::Empty>::SharedPtr;
using StopMotorService = rclcpp::Service<std_srvs::srv::Empty>::SharedPtr;
using RPLidarDriver = rp::standalone::rplidar::RPlidarDriver;
using RPLidarScanMode = rp::standalone::rplidar::RplidarScanMode;
using Clock = rclcpp::Clock::SharedPtr;
using ResponseNodeArray = std::array<rplidar_response_measurement_node_hq_t, MAX_SAMPLE_COUNT>;
using EmptyRequest = std::shared_ptr<std_srvs::srv::Empty::Request>;
using EmptyResponse = std::shared_ptr<std_srvs::srv::Empty::Response>;
using Timer = rclcpp::TimerBase::SharedPtr;
using namespace std::chrono_literals;
} // namespace

namespace rplidar_ros {
[[nodiscard]] constexpr float getAngleInDegrees(const rplidar_response_measurement_node_hq_t& node)
{
   return node.angle_z_q14 * 90.f / 16384.f; // I have no clue what these values mean
}

[[nodiscard]] constexpr float degreesToRadians(float degrees)
{
   return degrees * M_PI / 180.0;
}

class RPLIDAR_ROS_PUBLIC RPLidarNode : public rclcpp::Node
{
 public:
   explicit RPLidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
   virtual ~RPLidarNode();

   void publish_scan(const double scan_time, const ResponseNodeArray& nodes, size_t node_count);

   /* service callbacks */
   void stop_motor(const EmptyRequest req, EmptyResponse res);
   void start_motor(const EmptyRequest req, EmptyResponse res);

 private:
   struct RPLidarDriverDeleter
   {
      void operator()(RPLidarDriver* driver)
      {
         RPLidarDriver::DisposeDriver(driver);
      }
   };

   using RPLidarDriverUPtr = std::unique_ptr<RPLidarDriver, RPLidarDriverDeleter>;

   [[nodiscard]] bool print_device_info() const;
   [[nodiscard]] bool is_healthy() const;
   [[nodiscard]] bool set_scan_mode();
   void publish_loop();

   /* parameters */
   std::string m_channel_type;
   std::string m_tcp_ip;
   std::string m_serial_port;
   std::string m_scan_topic;
   int m_tcp_port;
   int m_serial_baudrate;
   std::string m_frame_id;
   bool m_inverted;
   bool m_angle_compensate;
   float m_angle_compensate_multiple;
   std::string m_scan_mode;

   /* Publisher */
   LaserScanPub m_publisher;

   /* Services */
   StopMotorService m_stop_motor_service;
   StartMotorService m_start_motor_service;

   /* SDK Pointer */
   RPLidarDriverUPtr m_driver{nullptr};

   /* Timer */
   Timer m_timer;

   /* Scan Times */
   size_t m_scan_count{0};
   float m_min_distance{0.15f};
   float m_max_distance{8.0};
   float m_angle_min{degreesToRadians(0)};
   float m_angle_max{degreesToRadians(359)};
};

} // namespace rplidar_ros

#endif // RPLIDAR_NODE_HPP_
