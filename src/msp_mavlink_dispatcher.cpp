
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/log_message.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <msp_msgs/msg/heartbeat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>
#include <msp_controller/msp_px4.h>
#include <msp_controller/msp_mavlink_dispatcher.hpp>

using namespace msp;

void MspMavlinkDispatcher::start()
{
  udp::endpoint local_endpoint(udp::v4(), 14670); // Bind to a local port for receiving messages
  socket.open(udp::v4());
  socket.bind(local_endpoint);

  timer = ros2Node->create_wall_timer(std::chrono::milliseconds(1000),
                                      std::bind(&MspMavlinkDispatcher::send_heartbeat, this));

  msp_state = MAV_STATE_ACTIVE;
  start_receive();
  io_thread = std::thread([this]()
                          { io_service.run(); });

  RCLCPP_INFO(ros2Node->get_logger(), "MSP Controller dispatcher started %i listeners",mavlinkMessageListeners.size());
}

bool MspMavlinkDispatcher::sendMavlinkMessage(mavlink_message_t &msg)
{
  size_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  asio::error_code ec;
  udp::endpoint remote_endpoint(asio::ip::address::from_string("127.0.0.1"), 14676);
  socket.send_to(asio::buffer(buffer, len), remote_endpoint, 0, ec);
  if (ec)
  {
    RCLCPP_ERROR(ros2Node->get_logger(), "Failed to send MAVLink message: %s", ec.message().c_str());
    return false;
  }

  return true;
}


void MspMavlinkDispatcher::sendMavlinkCommand(uint16_t command, float param1, float param2, float param3, float param4,
                                              float param5, float param6, float param7)
{
  mavlink_message_t msg;
  mavlink_command_long_t cmd;

  cmd.target_system    = PX4_SYSID;
  cmd.target_component = MAV_COMP_ID_AUTOPILOT1;

  cmd.command = command;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.param3 = param3;
  cmd.param4 = param4;
  cmd.param5 = param5;
  cmd.param6 = param6;
  cmd.param7 = param7;

  mavlink_msg_command_long_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &cmd);
  this->sendMavlinkMessage(msg);
}

void MspMavlinkDispatcher::handle_px4_command(const std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request,
                                              std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> response)
{
  if (request->request.target_component != MAV_COMP_ID_AUTOPILOT1)
    return;

  sendMavlinkCommand(request->request.command, request->request.param1, request->request.param2,
                     request->request.param3, request->request.param4, request->request.param5, request->request.param6,
                     request->request.param7);
}

void MspMavlinkDispatcher::start_receive()
{
  socket.async_receive_from(asio::buffer(recv_buffer), remote_endpoint,
                            [this](std::error_code ec, std::size_t bytes_recvd)
                            {
                              if (!ec && bytes_recvd > 0)
                                process_mavlink_message(bytes_recvd);
                              start_receive(); // Continue receiving
                            });
}

void MspMavlinkDispatcher::process_mavlink_message(std::size_t length)
{
  mavlink_message_t message;
  mavlink_status_t status;


  for (std::size_t i = 0; i < length; ++i)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, recv_buffer[i], &message, &status))
    {
      // Dispatch mavlink messages to listeners
      auto range = mavlinkMessageListeners.equal_range(message.msgid);
      for (auto it = range.first; it != range.second; ++it)
      {
        if (it->second)
          it->second->onMessageReceived(message);
      }
    }
  }
}

void MspMavlinkDispatcher::send_heartbeat()
{
  mavlink_message_t msg;

  mavlink_msg_heartbeat_pack(MSP_SYSID,
                             MAV_COMPONENT::MAV_COMP_ID_ONBOARD_COMPUTER, 
                             &msg,
                             MAV_TYPE::MAV_TYPE_ONBOARD_CONTROLLER,             // Type
                             MAV_AUTOPILOT::MAV_AUTOPILOT_PX4,                  // Autopilot type
                             0,                                                 // Base mode
                             0,                                                 // Custom mode
                             this->msp_state                                    // System state
  );

  this->sendMavlinkMessage(msg);
}

void MspMavlinkDispatcher::addListener(uint16_t msgid, MavlinkMessageListener *listener)
{
  mavlinkMessageListeners.emplace(msgid, listener);
}

void MspMavlinkDispatcher::removeListener(uint16_t msgid, MavlinkMessageListener *listener)
{
  auto range = mavlinkMessageListeners.equal_range(msgid);
  for (auto it = range.first; it != range.second; ++it)
  {
    if (it->second == listener)
    {
      mavlinkMessageListeners.erase(it);
      break;
    }
  }
}

uint64_t MspMavlinkDispatcher::getPX4TimeUs()
{
  if (time_offset > 0)
    return ((uint64_t)ros2Node->get_clock()->now().nanoseconds() - (uint64_t)time_offset) / 1000ULL;
  return 0;
}

uint64_t MspMavlinkDispatcher::getRos2TimeMs()
{
  return (uint64_t)ros2Node->get_clock()->now().nanoseconds() / 1000000ULL;
}
