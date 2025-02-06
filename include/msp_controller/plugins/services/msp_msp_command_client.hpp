#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspMspCommandClient : public msp::MavlinkMessageListener
{
public:
  explicit MspMspCommandClient(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher): ros2Node(node), dispatcher_(dispatcher)
  {
    dispatcher_->addListener(MAVLINK_MSG_ID_MSP_COMMAND,this);
    msp_command_client = ros2Node->create_client<px4_msgs::srv::VehicleCommand>("/msp/in/vehicle_command");
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    mavlink_msp_command_t cmd;
    mavlink_msg_msp_command_decode(&msg, &cmd);

    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    auto message = px4_msgs::msg::VehicleCommand();
    message.command = cmd.command;
    message.param1 = cmd.param1;
    message.param2 = cmd.param2;
    message.param3 = cmd.param3;
    message.param4 = cmd.param4;
    message.param5 = cmd.param5;
    message.param6 = cmd.param6;

    message.target_component = msg.compid;
    message.target_system = msg.sysid;

    request->request = message;
    auto result = msp_command_client->async_send_request(request);
  }

private:
  rclcpp::Node* ros2Node;
  msp::MspMavlinkDispatcher* dispatcher_;
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr msp_command_client;

};

}  // namespace msp