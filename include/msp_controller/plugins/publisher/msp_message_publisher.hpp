
#pragma once
#include <px4_msgs/msg/log_message.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspMessagePublisher : public msp::MavlinkMessageListener
{
public:
  explicit MspMessagePublisher(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher) : ros2Node(node), dispatcher_(dispatcher)
  {
    dispatcher_->addListener(MAVLINK_MSG_ID_STATUSTEXT,this);
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    px4_publisher = ros2Node->create_publisher<px4_msgs::msg::LogMessage>("/msp/out/log_message", qos);
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    mavlink_statustext_t mes;
    mavlink_msg_statustext_decode(&msg, &mes);

    if(mes.chunk_seq != 0)
      return;
    auto message = px4_msgs::msg::LogMessage();
    message.timestamp = ros2Node->get_clock()->now().nanoseconds() / 1000L;
    message.severity = mes.severity;
    std::string text = "[px4] "+ std::string(reinterpret_cast<const char*>(mes.text));
    std::memcpy(message.text.data(), text.data(), std::min(std::strlen(text.data())-1, message.text.size()));
    
    px4_publisher->publish(message);

  }

private: 
  rclcpp::Publisher<px4_msgs::msg::LogMessage>::SharedPtr px4_publisher;
  rclcpp::Node* ros2Node;
  msp::MspMavlinkDispatcher* dispatcher_;
 
};

}  // namespace msp