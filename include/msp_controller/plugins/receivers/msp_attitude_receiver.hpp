#pragma once
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspAttitudeReceiver : public msp::MavlinkMessageListener
{
public:
  explicit MspAttitudeReceiver(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher): ros2Node(node), dispatcher_(dispatcher)
  {
    dispatcher->addListener(MAVLINK_MSG_ID_ATTITUDE,this);
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(&msg, &att);

    msp::MavlinkMessageListener::model.yaw_speed = att.yawspeed;

  }

private:
 rclcpp::Node* ros2Node;
msp::MspMavlinkDispatcher* dispatcher_;
};

}  // namespace msp