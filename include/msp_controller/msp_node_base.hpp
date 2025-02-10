#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/log_message.hpp>
#include <msp_msgs/msg/heartbeat.hpp>
#include <msp_controller/msp_px4.h>



namespace msp
{

/**
 * @brief MSP NodeBase
 * Provides MSP Command receiving and sending PX4 Commands to
 * the flight controller both via the MSP Controller node
 */


class MSPNodeBase : public rclcpp::Node
{


public:
  explicit MSPNodeBase(std::string nodeName, uint8_t component_id) : Node(nodeName)
  {
    this->component_id = component_id;
    auto qos = getQos();
    log_message_publisher = this->create_publisher<px4_msgs::msg::LogMessage>("/msp/in/log_message", qos);
    heartbeat_publisher = this->create_publisher<msp_msgs::msg::Heartbeat>("/msp/in/heartbeat", qos);
    px4_command_client = this->create_client<px4_msgs::srv::VehicleCommand>("/px4/in/vehicle_command");
    msp_vehicle_command = this->create_service<px4_msgs::srv::VehicleCommand>(
        "/msp/in/vehicle_command",
        std::bind(&MSPNodeBase::receive_msp_command, this, std::placeholders::_1, std::placeholders::_2));

    hb_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MSPNodeBase::send_heartbeat, this));
  }

  void setStatus(uint8_t state ) {
    this->state = state;
  }

  virtual void receive_msp_command(const std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request,
                                   std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> response){};

  virtual void send_px4_vehicle_command(const uint16_t command, const float param1 = 0, const float param2 = 0,
                                        const float param3 = 0)
  {
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = MSP_SYSID;
    msg.source_component = MSP_COMP_CONTROLLER;
    request->request = msg;

    auto result = px4_command_client->async_send_request(request);
  }

  virtual void log_message(const std::string msg, const uint8_t severity)
  {
    auto message = px4_msgs::msg::LogMessage();
    std::copy_n(msg.begin(), std::min(msg.size(), message.text.size()), message.text.begin());
    message.set__severity(severity);
    log_message_publisher->publish(message);
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
  }

  virtual rclcpp::QoS getQos()
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    return rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
  }

  void send_heartbeat()
  {
    auto message = msp_msgs::msg::Heartbeat();

    message.system_id = MSP_SYSID;
    message.component_id = component_id;
    message.system_status = state;
    message.timestamp = this->get_clock()->now().nanoseconds() / 1000L;
    heartbeat_publisher->publish(message);
  }

private:
  rclcpp::Service<px4_msgs::srv::VehicleCommand>::SharedPtr msp_vehicle_command;
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr px4_command_client;
  rclcpp::Publisher<px4_msgs::msg::LogMessage>::SharedPtr log_message_publisher;
  rclcpp::Publisher<msp_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher;

  rclcpp::TimerBase::SharedPtr hb_timer;

  uint8_t component_id;
  uint8_t state = MAV_STATE_ACTIVE;
};

}  // namespace msp