#pragma once
#include <msp_msgs/msg/heartbeat.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspTrajectorySubscriber
{
public:
  explicit MspTrajectorySubscriber(msp::MspMavlinkDispatcher* dispatcher) : dispatcher(dispatcher)
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    subscription = this->dispatcher->getRos2Node()->create_subscription<msp_msgs::msg::Heartbeat>(
      "/msp/in/heartbeat", qos, [this](const msp_msgs::msg::Heartbeat::UniquePtr message)
      {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(message->system_id, message->component_id, &msg,
                                   MAV_TYPE::MAV_TYPE_ONBOARD_CONTROLLER,             
                                   MAV_AUTOPILOT::MAV_AUTOPILOT_PX4,                  
                                   0,                      // Base mode
                                   0,                      // Custom mode
                                   message->system_status  // System state
        );
        this->dispatcher->sendMavlinkMessage(msg); 
        
        });   
  }


private:
  rclcpp::Subscription<msp_msgs::msg::Heartbeat>::SharedPtr subscription;
  msp::MspMavlinkDispatcher* dispatcher;
  
};

}