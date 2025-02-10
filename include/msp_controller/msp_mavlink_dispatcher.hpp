#pragma once
#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <lquac/mavlink.h>
#include <msp_controller/msp_mavlink_listener.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

using asio::ip::udp;

namespace msp
{

class MspMavlinkDispatcher 
{
public:
  explicit MspMavlinkDispatcher(rclcpp::Node* node) : ros2Node(node), io_service(), socket(io_service)
  {
  
    px4_vehicle_command = ros2Node->create_service<px4_msgs::srv::VehicleCommand>(
        "/px4/in/vehicle_command",
        std::bind(&MspMavlinkDispatcher::handle_px4_command, this, std::placeholders::_1, std::placeholders::_2));
  };

  ~MspMavlinkDispatcher()
  {
    io_service.stop();
    if (io_thread.joinable())
    {
      io_thread.join();
    }
  }

  void start();
  bool sendMavlinkMessage(mavlink_message_t& msg);
  void sendMavlinkCommand(uint16_t command, float param1=0, float param2=0, float param3=0, float param4=0, float param5=0, float param6=0, float param7=0);
  void addListener(uint16_t msgid, MavlinkMessageListener* listener);
  void removeListener(uint16_t msgid, MavlinkMessageListener* listener);
  uint64_t getPX4TimeUs();
  uint64_t getRos2TimeMs();
  
  inline void setTimeOffset(double offset) {
    time_offset = offset;
  }

  inline rclcpp::Node* getRos2Node() {
    return ros2Node;
  }

  inline double getYawFromQuaternion(double q_x, double q_y, double q_z, double q_w) {
    double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
    return std::atan2(siny_cosp, cosy_cosp); 
}

private:

  void start_receive();
  void process_mavlink_message(std::size_t length);
  void send_heartbeat();

  void publish_vehicle_command(mavlink_message_t& msg);

  void handle_px4_command(const std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request,
                          std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> response);

  rclcpp::Node* ros2Node;
  rclcpp::TimerBase::SharedPtr timer;

  std::multimap<uint16_t,msp::MavlinkMessageListener*> mavlinkMessageListeners;

  uint8_t  msp_state = MAV_STATE_ACTIVE;
  double   time_offset = 0;


  rclcpp::Service<px4_msgs::srv::VehicleCommand>::SharedPtr px4_vehicle_command;

  asio::io_service io_service;
  udp::socket socket;
  udp::endpoint remote_endpoint;
  std::thread io_thread;

  std::array<uint8_t, 1024> recv_buffer;
  uint8_t buffer[1024];

};

}  // namespace msp