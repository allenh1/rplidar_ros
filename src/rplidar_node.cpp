/*
 *  RPLIDAR ROS NODE
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

#include <rplidar_node.hpp>

namespace rplidar_ros
{

rplidar_node::rplidar_node(rclcpp::Node::options options)
  : rclcpp::Node(options, "rplidar_node")
{
  /* set parameters */
  node->get_parameter_or("channel_type", channel_type_, std::string("serial"));
  node->get_parameter_or("tcp_ip", tcp_ip_, std::string("192.168.0.7"));
  node->get_parameter_or("tcp_port", tcp_port_, 20108);
  node->get_parameter_or("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
  node->get_parameter_or("serial_baudrate", serial_baudrate_, 115200);
  node->get_parameter_or("frame_id", frame_id_, std::string("laser_frame"));
  node->get_parameter_or("inverted", inverted_, false);
  node->get_parameter_or("angle_compensate", angle_compensate_, false);
  node->get_parameter_or("scan_mode", scan_mode_, std::string());

  /* set QoS settings */
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  /* create the publisher for "/scan" */
  m_publisher = this->create_publisher<LaserScan>("scan", qos);

  RCLCPP_INFO(this->get_logger(),
    "RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '%s'", RPLIDAR_SDK_VERSION);

  /* initialize SDK */
  m_drv = (channel_type == "tcp")
    ? RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP)
    : RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
}

void rplidar_node::publish_scan()
{
  static size_t scan_count = 0;
  sensor_msgs::msg::LaserScan scan_msg;

  /* NOTE(allenh1): time was passed in as a parameter before */
  scan_msg.header.stamp = clock->now();
  scan_msg.header.frame_id = frame_id_;
  scan_count++;

  constexpr bool reversed = (angle_max > angle_min);
  if constexpr (reversed) {
    /* NOTE(allenh1): the other case seems impossible? */
    scan_msg.angle_min =  M_PI - angle_max;
    scan_msg.angle_max =  M_PI - angle_min;
  } else {
    scan_msg.angle_min =  M_PI - angle_min;
    scan_msg.angle_max =  M_PI - angle_max;
  }
  scan_msg.angle_increment =
    (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (double)(node_count-1);
  scan_msg.range_min = min_distance;
  scan_msg.range_max = max_distance;

  scan_msg.intensities.resize(node_count);
  scan_msg.ranges.resize(node_count);
  bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
  if (!reverse_data) {
    for (size_t i = 0; i < node_count; i++) {
      float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
      if (read_value == 0.0)
        scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
      else
        scan_msg.ranges[i] = read_value;
      scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);
    }
  } else {
    for (size_t i = 0; i < node_count; i++) {
      float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
      if (read_value == 0.0)
        scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
      else
        scan_msg.ranges[node_count-1-i] = read_value;
      scan_msg.intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
    }
  }

  pub->publish(scan_msg);
}


bool rplidar_node::getRPLIDARDeviceInfo()
{
  u_result op_result;
  rplidar_response_device_info_t devinfo;

  op_result = m_drv->getDeviceInfo(devinfo);
  if (IS_FAIL(op_result)) {
    if (op_result == RESULT_OPERATION_TIMEOUT) {
      RCLCPP_ERROR(this->get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: %x", op_result);
    }
    return false;
  }

  // print out the device serial number, firmware and hardware version number..
  std::string serial_no{"RPLIDAR S/N: "};
  for (int pos = 0; pos < 16 ;++pos) {
    char buff[3];
    snprintf(buff, sizeof(buff), "%02X", devinfo.serialnum[pos]);
    serial_no += buff;
  }
  RCLCPP_INFO(this->get_logger(), "%s", serial_no.c_str());
  RCLCPP_INFO(this->get_logger(), "Firmware Ver: %d.%02d", devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
  RCLCPP_INFO(this->get_logger(), "Hardware Rev: %d", static_cast<int>(devinfo.hardware_version));
  return true;
}

}  // namespace rplidar_ros
