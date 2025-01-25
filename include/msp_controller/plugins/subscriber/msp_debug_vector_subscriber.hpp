#pragma once
#include <px4_msgs/msg/debug_vect.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

  class MspDebugVectorSubscriber
  {
  public:
    explicit MspDebugVectorSubscriber(msp::MspMavlinkDispatcher *dispatcher) : dispatcher(dispatcher)
    {
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
      subscription = this->dispatcher->getRos2Node()->create_subscription<px4_msgs::msg::DebugVect>(
          "/msp/in/debug_vector", qos, [this](const px4_msgs::msg::DebugVect::UniquePtr message)
          {
        mavlink_message_t msg;
        mavlink_debug_vect_t debug;

        debug.x = message->x;
        debug.y = message->y;
        debug.z = message->z;

        debug.time_usec =  this->dispatcher->getRos2Node()->get_clock()->now().nanoseconds() / 1000L;

        mavlink_msg_debug_vect_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &debug);
        
        this->dispatcher->sendMavlinkMessage(msg); });
    }

  private:
    rclcpp::Subscription<px4_msgs::msg::DebugVect>::SharedPtr subscription;
    msp::MspMavlinkDispatcher *dispatcher;
  };

}