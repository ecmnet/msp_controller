#pragma once
#include <msp_msgs/msg/micro_grid.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

  class MspMicroGridSubscriber
  {
  public:
    explicit MspMicroGridSubscriber(msp::MspMavlinkDispatcher *dispatcher) : dispatcher(dispatcher)
    {
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 50), qos_profile);
      subscription = this->dispatcher->getRos2Node()->create_subscription<msp_msgs::msg::MicroGrid>(
          "/msp/in/micro_grid", qos, [this](const msp_msgs::msg::MicroGrid::UniquePtr message)
          {
        mavlink_message_t msg;
        mavlink_msp_micro_grid_t grid;

        grid.cx = message->center[0];
        grid.cy = message->center[1];
        grid.cz = message->center[2];

        grid.count = message->count;
        grid.resolution = message->resolution;
        grid.extension = message->extension;

        for(int i=0;i<25;i++)
          grid.data[i] = static_cast<uint64_t>(message->data[i]);
         
        grid.tms = this->dispatcher->getRos2Node()->get_clock()->now().nanoseconds() / 1000L;

        mavlink_msg_msp_micro_grid_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &grid);
        
        this->dispatcher->sendMavlinkMessage(msg); });
    }

  private:
    rclcpp::Subscription<msp_msgs::msg::MicroGrid>::SharedPtr subscription;
    msp::MspMavlinkDispatcher *dispatcher;
  
  };

}