#pragma once
#include <px4_msgs/msg/log_message.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspMessageSubscriber
{
public:
  explicit MspMessageSubscriber(msp::MspMavlinkDispatcher* dispatcher) : dispatcher(dispatcher)
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    subscription = this->dispatcher->getRos2Node()->create_subscription<px4_msgs::msg::LogMessage>(
      "/msp/in/log_message", qos, [this](const px4_msgs::msg::LogMessage::UniquePtr message)
      {
        mavlink_message_t msg;
        mavlink_statustext_t statustext;

        std::strncpy(statustext.text, reinterpret_cast<const char*>(message->text.data()), sizeof(statustext.text) - 1);
        statustext.text[sizeof(statustext.text) - 1] = '\0';  // Null-terminate
        statustext.severity = message->severity;

        mavlink_msg_statustext_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &statustext);
        this->dispatcher->sendMavlinkMessage(msg); });
   
  }


private:
  rclcpp::Subscription<px4_msgs::msg::LogMessage>::SharedPtr subscription;
  msp::MspMavlinkDispatcher* dispatcher;
  
};

}