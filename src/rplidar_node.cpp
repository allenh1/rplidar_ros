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

#include <rplidar_node.hpp>

namespace rplidar_ros
{

rplidar_node::rplidar_node(rclcpp::NodeOptions options)
  : rclcpp::Node("rplidar_node", options)
{
  /* set parameters */
  this->get_parameter_or("channel_type", channel_type_, std::string("serial"));
  this->get_parameter_or("tcp_ip", tcp_ip_, std::string("192.168.0.7"));
  this->get_parameter_or("tcp_port", tcp_port_, 20108);
  this->get_parameter_or("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
  this->get_parameter_or("serial_baudrate", serial_baudrate_, 115200);
  this->get_parameter_or("frame_id", frame_id_, std::string("laser_frame"));
  this->get_parameter_or("inverted", inverted_, false);
  this->get_parameter_or("angle_compensate", angle_compensate_, false);
  this->get_parameter_or("scan_mode", scan_mode_, std::string());
  this->get_parameter_or("topic_name", topic_name_, std::string("scan"));

  RCLCPP_INFO(this->get_logger(),
    "RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '%s'", RPLIDAR_SDK_VERSION);

  /* initialize SDK */
  m_drv = (channel_type_ == "tcp")
    ? RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP)
    : RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

  if (nullptr == m_drv) {
    /* don't start spinning without a driver object */
    RCLCPP_ERROR(this->get_logger(), "Failed to construct driver");
    return;
  }

  if(channel_type_ == "tcp"){
    // make connection...
    if (IS_FAIL(m_drv->connect(tcp_ip_.c_str(), (_u32)tcp_port_))) {
      RCLCPP_ERROR(this->get_logger(),
        "Error, cannot bind to the specified TCP host '%s:%ud'",
        tcp_ip_.c_str(), static_cast<unsigned int>(tcp_port_));
      RPlidarDriver::DisposeDriver(m_drv);
      return;
    }
  }
  else{
    // make connection...
    if (IS_FAIL(m_drv->connect(serial_port_.c_str(), (_u32)serial_baudrate_))) {
      RCLCPP_ERROR(this->get_logger(), "Error, cannot bind to the specified serial port '%s'.", serial_port_.c_str());
      RPlidarDriver::DisposeDriver(m_drv);
      return;
    }
  }

  // get rplidar device info
  if (!getRPLIDARDeviceInfo()) {
    /* don't continue */
    RPlidarDriver::DisposeDriver(m_drv);
    return;
  }

  // check health...
  if (!checkRPLIDARHealth()) {
    RPlidarDriver::DisposeDriver(m_drv);
    return;
  }

  /* start motor */
  m_drv->startMotor();

  /* TODO(allenh1): set scan mode */

  /* done setting up RPLIDAR stuff, now set up ROS 2 stuff */

  /* set QoS settings */
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = 1;
  qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  /* connect to ROS clock */
  rclcpp::TimeSource timesource;
  m_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  timesource.attachClock(m_clock);

  /* create the publisher for "/scan" */
  m_publisher = this->create_publisher<LaserScan>(topic_name_, qos);

  /* create stop motor service */
  m_stop_motor_service = this->create_service<std_srvs::srv::Empty>(
    "stop_motor",
    std::bind(&rplidar_node::stop_motor, this, std::placeholders::_1, std::placeholders::_2),
    qos);

  /* create start motor service */
  m_start_motor_service = this->create_service<std_srvs::srv::Empty>(
    "start_motor",
    std::bind(&rplidar_node::start_motor, this, std::placeholders::_1, std::placeholders::_2),
    qos);
}

void rplidar_node::publish_scan(
  const double scan_time, const ResponseNodeArray & nodes, size_t node_count)
{
  static size_t scan_count = 0;
  sensor_msgs::msg::LaserScan scan_msg;

  /* NOTE(allenh1): time was passed in as a parameter before */
  scan_msg.header.stamp = m_clock->now();
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

  m_publisher->publish(scan_msg);
}


bool rplidar_node::getRPLIDARDeviceInfo() const
{
  u_result op_result;
  rplidar_response_device_info_t devinfo;

  op_result = m_drv->getDeviceInfo(devinfo);
  if (IS_FAIL(op_result)) {
    if (op_result == RESULT_OPERATION_TIMEOUT) {
      RCLCPP_ERROR(this->get_logger(), "Error, operation time out. RESULT_OPERATION_TIMEOUT!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error, unexpected error, code: '%x'", op_result);
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

bool rplidar_node::checkRPLIDARHealth() const
{
  rplidar_response_device_health_t healthinfo;
  u_result op_result = m_drv->getHealth(healthinfo);

  if (IS_OK(op_result)) {
    RCLCPP_INFO(this->get_logger(), "RPLidar health status : '%d'", healthinfo.status);
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
      RCLCPP_ERROR(this->get_logger(), "Error, rplidar internal error detected. Please reboot the device to retry");
      return false;
    }
    return true;
  }
  RCLCPP_ERROR(this->get_logger(), "Error, cannot retrieve rplidar health code: '%x'", op_result);
  return false;
}

void rplidar_node::stop_motor(const EmptyRequest req, EmptyResponse res)
{
  if (nullptr == m_drv) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
  m_drv->stop();
  m_drv->stopMotor();
}

void rplidar_node::start_motor(const EmptyRequest req, EmptyResponse res)
{
  if (nullptr == m_drv) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Call to '%s'", __FUNCTION__);
  m_drv->startMotor();
  m_drv->startScan(0,1);
}

}  // namespace rplidar_ros
