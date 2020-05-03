/*
 *  Rplidar ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
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

#include <RPLidarNode.hpp>

namespace rplidar_ros {

RPLidarNode::RPLidarNode(const rclcpp::NodeOptions& options) : rclcpp::Node("rplidar_node", options)
{
   const auto& logger = this->get_logger();

   /* set parameters */
   m_channel_type = this->declare_parameter("channel_type", "serial");
   m_tcp_ip = this->declare_parameter("tcp_ip", "192.168.0.7");
   m_tcp_port = this->declare_parameter("tcp_port", 20108);
   m_serial_port = this->declare_parameter("serial_port", "/dev/ttyUSB0");
   m_serial_baudrate = this->declare_parameter("serial_baudrate", 115200);
   m_frame_id = this->declare_parameter("frame_id", std::string("laser_frame"));
   m_inverted = this->declare_parameter("inverted", false);
   m_angle_compensate = this->declare_parameter("angle_compensate", false);
   m_scan_mode = this->declare_parameter("scan_mode", std::string());
   m_scan_topic = this->declare_parameter("topic_name", std::string("scan"));

   RCLCPP_INFO(logger, "RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '%s'", RPLIDAR_SDK_VERSION);

   /* initialize SDK */
   if (m_channel_type == "tcp") {
      m_driver = RPLidarDriverUPtr(RPLidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP));
   } else {
      m_driver = RPLidarDriverUPtr(RPLidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT));
   }

   if (m_driver == nullptr) {
      /* don't start spinning without a driver object */
      RCLCPP_ERROR(logger, "Failed to construct driver");
      throw std::bad_alloc();
   }

   if (m_channel_type == "tcp") {
      // make connection...
      if (IS_FAIL(m_driver->connect(m_tcp_ip.c_str(), static_cast<uint32_t>(m_tcp_port)))) {
         std::stringstream error_string;
         error_string << "Cannot bind to the specified TCP host: " << m_tcp_ip << ":" << m_tcp_port;
         throw std::runtime_error(error_string.str());
      }

   } else {
      // make connection...
      if (IS_FAIL(m_driver->connect(m_serial_port.c_str(), (_u32)m_serial_baudrate))) {
         std::stringstream error_string;
         error_string << "Cannot bind to the specified serial port '" << m_serial_port << "'";
         throw std::runtime_error(error_string.str());
         return;
      }
   }

   if (!print_device_info()) {
      throw std::runtime_error("Failed to get device info.");
   }

   if (!is_healthy()) {
      throw std::runtime_error("Failed the health check.");
   }

   m_driver->startMotor();

   if (!set_scan_mode()) {
      m_driver->stop();
      m_driver->stopMotor();
      throw std::runtime_error("Failed to set the scan mode.");
   }

   m_publisher = this->create_publisher<LaserScan>(m_scan_topic, rclcpp::SystemDefaultsQoS());

   m_stop_motor_service = this->create_service<std_srvs::srv::Empty>(
      "stop_motor", std::bind(&RPLidarNode::stop_motor, this, std::placeholders::_1, std::placeholders::_2));

   m_start_motor_service = this->create_service<std_srvs::srv::Empty>(
      "start_motor", std::bind(&RPLidarNode::start_motor, this, std::placeholders::_1, std::placeholders::_2));

   m_timer = this->create_wall_timer(1ms, std::bind(&RPLidarNode::publish_loop, this));
}

RPLidarNode::~RPLidarNode()
{
   m_driver->stop();
   m_driver->stopMotor();
}

void RPLidarNode::publish_scan(const double scan_time, const ResponseNodeArray& nodes, size_t node_count)
{
   static size_t scan_count = 0;
   sensor_msgs::msg::LaserScan scan_msg;

   /* NOTE(allenh1): time was passed in as a parameter before */
   scan_msg.header.stamp = this->now();
   scan_msg.header.frame_id = m_frame_id;
   scan_count++;

   bool reversed = (m_angle_max > m_angle_min);
   if (reversed) {
      /* NOTE(allenh1): the other case seems impossible? */
      scan_msg.angle_min = M_PI - m_angle_max;
      scan_msg.angle_max = M_PI - m_angle_min;
   } else {
      scan_msg.angle_min = M_PI - m_angle_min;
      scan_msg.angle_max = M_PI - m_angle_max;
   }
   scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

   scan_msg.scan_time = scan_time;
   scan_msg.time_increment = scan_time / (double)(node_count - 1);
   scan_msg.range_min = m_min_distance;
   scan_msg.range_max = m_max_distance;

   scan_msg.intensities.resize(node_count);
   scan_msg.ranges.resize(node_count);
   bool reverse_data = (!m_inverted && reversed) || (m_inverted && !reversed);
   if (!reverse_data) {
      for (size_t i = 0; i < node_count; i++) {
         float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
         if (read_value == 0.0) {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
         } else {
            scan_msg.ranges[i] = read_value;
         }
         scan_msg.intensities[i] = (float)(nodes[i].quality >> 2);
      }
   } else {
      for (size_t i = 0; i < node_count; i++) {
         float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
         if (read_value == 0.0) {
            scan_msg.ranges[node_count - 1 - i] = std::numeric_limits<float>::infinity();
         } else {
            scan_msg.ranges[node_count - 1 - i] = read_value;
         }
         scan_msg.intensities[node_count - 1 - i] = (float)(nodes[i].quality >> 2);
      }
   }

   m_publisher->publish(scan_msg);
}

bool RPLidarNode::print_device_info() const
{
   const auto& logger = this->get_logger();

   u_result op_result;
   rplidar_response_device_info_t devinfo;

   op_result = m_driver->getDeviceInfo(devinfo);
   if (IS_FAIL(op_result)) {
      if (op_result == RESULT_OPERATION_TIMEOUT) {
         RCLCPP_ERROR(logger, "Operation time out!");
      } else {
         RCLCPP_ERROR(logger, "Error code: '%x'", op_result);
      }
      return false;
   }

   constexpr size_t SERIAL_NUMBER_SIZE{16};
   // print out the device serial number, firmware and hardware version number..
   std::stringstream serial_no{};
   serial_no << std::hex << std::uppercase;
   for (size_t index = 0; index < SERIAL_NUMBER_SIZE; ++index) {
      serial_no << static_cast<uint16_t>(devinfo.serialnum[index]);
   }

   struct firmware_version
   {
      uint8_t minor : 8;
      uint8_t major : 8;
   } version{*reinterpret_cast<firmware_version*>(&devinfo.firmware_version)};

   RCLCPP_INFO(logger, "RPLIDAR S/N: %s", serial_no.str().c_str());
   RCLCPP_INFO(logger, "Firmware Ver: %d.%02d", version.major, version.minor);
   RCLCPP_INFO(logger, "Hardware Rev: %d", static_cast<uint16_t>(devinfo.hardware_version));
   return true;
}

bool RPLidarNode::is_healthy() const
{
   rplidar_response_device_health_t healthinfo;
   u_result op_result = m_driver->getHealth(healthinfo);

   const auto& logger = this->get_logger();
   if (IS_OK(op_result)) {
      RCLCPP_INFO(logger, "RPLidar health status : '%d'", healthinfo.status);
      if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
         RCLCPP_ERROR(logger, "RPLidar internal error detected. Please reboot the device to retry");
         return false;
      }
      return true;
   }
   RCLCPP_ERROR(logger, "Cannot retrieve rplidar health code: '%x'", op_result);
   return false;
}

void RPLidarNode::stop_motor(const EmptyRequest, EmptyResponse)
{
   if (m_driver == nullptr) {
      return;
   }

   RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
   m_driver->stop();
   m_driver->stopMotor();
}

void RPLidarNode::start_motor(const EmptyRequest, EmptyResponse)
{
   if (m_driver == nullptr) {
      return;
   }

   RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
   m_driver->startMotor();
   m_driver->startScan(0, 1);
}

bool RPLidarNode::set_scan_mode()
{
   const auto& logger = this->get_logger();

   u_result options_results{};
   RPLidarScanMode current_scan_mode{};
   constexpr auto force_scan{false};
   constexpr auto typical_scan_mode{true};
   constexpr auto scan_options{0}; // don't change this

   if (m_scan_mode.empty()) {
      options_results = m_driver->startScan(force_scan, typical_scan_mode, scan_options, &current_scan_mode);
   } else {
      std::vector<RPLidarScanMode> supported_scan_modes;
      options_results = m_driver->getAllSupportedScanModes(supported_scan_modes);

      if (IS_OK(options_results)) {
         auto found_scan_mode =
            std::find_if(supported_scan_modes.begin(), supported_scan_modes.end(), [this](auto scan_mode) {
               return std::string_view(scan_mode.scan_mode) == m_scan_mode;
            });

         if (found_scan_mode == supported_scan_modes.end()) {
            RCLCPP_ERROR(logger,
                         "scan mode `%s' is not supported by lidar, supported modes ('%zd'):",
                         m_scan_mode.c_str(),
                         supported_scan_modes.size());

            for (const auto& scan_mode : supported_scan_modes) {
               RCLCPP_ERROR(logger,
                            "%s: max_distance: %.1f m, Point number: %.1fK",
                            scan_mode.scan_mode,
                            scan_mode.max_distance,
                            (1000 / scan_mode.us_per_sample));
            }

            options_results = RESULT_OPERATION_FAIL;
            return false;
         } else {
            options_results =
               m_driver->startScanExpress(force_scan, found_scan_mode->id, scan_options, &current_scan_mode);
         }
      }
   }

   /* verify we set the scan mode */
   if (!IS_OK(options_results)) {
      RCLCPP_ERROR(logger, "Cannot start scan: '%08x'", options_results);
      return false;
   }

   const float samples_per_microsecond = 1 / current_scan_mode.us_per_sample;
   const float samples_per_second = 1e6 * samples_per_microsecond;
   const float samples_per_rotation = samples_per_second / DEFAULT_FREQUENCY; // Each rotation is 1000 ms
   const float samples_per_degree = samples_per_rotation / 360.0f;

   m_angle_compensate_multiple = samples_per_degree; // minimum viable theoretical samples per degrees rotated
   m_angle_compensate_multiple = std::max(m_angle_compensate_multiple, 1.0f); // minimum of 1 sample per degree
   m_max_distance = current_scan_mode.max_distance;

   RCLCPP_INFO(logger,
               "current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %.1f",
               current_scan_mode.scan_mode,
               current_scan_mode.max_distance,
               (1000 / current_scan_mode.us_per_sample),
               m_angle_compensate_multiple);
   return true;
}

void RPLidarNode::publish_loop()
{
   const auto& logger = this->get_logger();
   ResponseNodeArray sample_nodes{};

   const auto start_scan_time = this->now();
   size_t sample_count = MAX_SAMPLE_COUNT; // will get adjusted to real sample size after grabbing scan data
   u_result op_result = m_driver->grabScanDataHq(&sample_nodes[0], sample_count);
   const double scan_duration = (this->now() - start_scan_time).seconds();

   if (op_result != RESULT_OK) {
      return;
   }

   op_result = m_driver->ascendScanData(&sample_nodes[0], sample_count);

   m_angle_min = degreesToRadians(0.0f);
   m_angle_max = degreesToRadians(359.0f);

   if (op_result == RESULT_OK) {
      constexpr auto is_valid_node = [](const auto& node) -> bool { return node.dist_mm_q2 != 0; };

      const auto end_node = std::find_if(sample_nodes.rbegin(), sample_nodes.rend(), is_valid_node);
      const auto start_node = std::find_if(sample_nodes.begin(), sample_nodes.end(), is_valid_node);

      m_angle_min = degreesToRadians(getAngleInDegrees(*start_node));
      m_angle_max = degreesToRadians(getAngleInDegrees(*end_node));

      /**
       * turn this -> [0, 0, 0, 1, 2, 3, 0, 0]
       * into this -> [1, 2, 3, 0, 0, 0, 0, 0]
       */
      std::rotate(sample_nodes.begin(), start_node, end_node.base());
      const size_t nodes_count = end_node.base() - start_node + 1;

      if (m_angle_compensate) {
         /**
          * Output a fixed number of lidar points and map them to the closest angle possible
          * RPlidar by default will output a variable number of samples around a fixed sample
          * Example:
          * Number of nodes -> 535 - 543 (not consistent)
          * Number of compensation nodes-> 525 (a nice fixed number of samples)
          *
          * We want 525 of the 543, that are evenly distributed
          * Our delta then will be 543/525 = 1.03
          */
         const size_t angle_compensate_nodes_count = 360 * m_angle_compensate_multiple;
         ResponseNodeArray angle_compensate_nodes{};

         float angle_compensation_position = 0;
         float angle_compensation_delta = static_cast<double>(nodes_count) / angle_compensate_nodes_count;
         for (size_t index = 0; index < angle_compensate_nodes_count; ++index) {
            angle_compensate_nodes[index] = sample_nodes[static_cast<size_t>(angle_compensation_position)];
            RCLCPP_DEBUG(logger, "%f", getAngleInDegrees(angle_compensate_nodes[index]));
            angle_compensation_position += angle_compensation_delta;
         }

         publish_scan(scan_duration, angle_compensate_nodes, angle_compensate_nodes_count);

      } else {
         publish_scan(scan_duration, sample_nodes, nodes_count);
      }

   } else if (op_result == RESULT_OPERATION_FAIL) {
      RCLCPP_WARN(logger, "Failed to organize data in ascending format. Publishing invalid dat anyways.");
      publish_scan(scan_duration, sample_nodes, sample_count);
   }
}

} // namespace rplidar_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rplidar_ros::RPLidarNode)
