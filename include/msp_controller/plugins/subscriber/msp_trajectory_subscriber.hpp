#pragma once
#include <msp_msgs/msg/trajectory.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspHeartbeatSubscriber
{
public:
  explicit MspHeartbeatSubscriber(msp::MspMavlinkDispatcher* dispatcher) : dispatcher(dispatcher)
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
       subscription = this->dispatcher->getRos2Node()->create_subscription<msp_msgs::msg::Trajectory>(
      "/msp/in/trajectory", qos, [this](const msp_msgs::msg::Trajectory::UniquePtr message)
      {
        mavlink_message_t msg;
        mavlink_msp_trajectory_t trajectory;

        trajectory.tms = message->timestamp;
        trajectory.id  = message->id;
        trajectory.ls  = message->total_secs;
        trajectory.fs  = message->done_secs;

        trajectory.ax  = message->alpha[0];
        trajectory.ay  = message->alpha[1];
        trajectory.az  = message->alpha[2];

        trajectory.bx  = message->beta[0];
        trajectory.by  = message->beta[1];
        trajectory.bz  = message->beta[2];

        trajectory.gx  = message->gamma[0];
        trajectory.gy  = message->gamma[1];
        trajectory.gz  = message->gamma[2];

        trajectory.sx  = message->pos0[0];
        trajectory.sy  = message->pos0[1];
        trajectory.sz  = message->pos0[2];

        trajectory.svx = message->vel0[0];
        trajectory.svy = message->vel0[1];
        trajectory.svz = message->vel0[2];

        trajectory.sax = message->acc0[0];
        trajectory.say = message->acc0[1];
        trajectory.saz = message->acc0[2];

        mavlink_msg_msp_trajectory_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &trajectory);
        
        this->dispatcher->sendMavlinkMessage(msg); });
  }


private:
  rclcpp::Subscription<msp_msgs::msg::Trajectory>::SharedPtr subscription;
  msp::MspMavlinkDispatcher* dispatcher;
  
};

}