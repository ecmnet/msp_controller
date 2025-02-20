#pragma once
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspOffboardSetpointSubscriber
{
public:
  explicit MspOffboardSetpointSubscriber(msp::MspMavlinkDispatcher* dispatcher) : dispatcher(dispatcher)
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
    subscription = this->dispatcher->getRos2Node()->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/msp/in/trajectory_setpoint", qos, [this](const px4_msgs::msg::TrajectorySetpoint::UniquePtr setpoint)
      {
        mavlink_message_t msg;
        mavlink_set_position_target_local_ned_t offboard;

        offboard.type_mask = 0;
        offboard.target_system = 1;
        offboard.target_component = 1;

        offboard.x = setpoint->position[0];
        offboard.y = setpoint->position[1];
        offboard.z = setpoint->position[2];

        offboard.vx = setpoint->velocity[0];
        offboard.vy = setpoint->velocity[1];
        offboard.vz = setpoint->velocity[2];

        offboard.afx = setpoint->acceleration[0];
        offboard.afy = setpoint->acceleration[1];
        offboard.afz = setpoint->acceleration[2];

        offboard.yaw = setpoint->yaw;
        offboard.yaw_rate = setpoint->yawspeed;

        offboard.time_boot_ms = this->dispatcher->getPX4TimeUs() / 1000;
        offboard.coordinate_frame = 1;

        mavlink_msg_set_position_target_local_ned_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &offboard);
        this->dispatcher->sendMavlinkMessage(msg); });
   
  }


private:
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr subscription;
  msp::MspMavlinkDispatcher* dispatcher;
  
};

}