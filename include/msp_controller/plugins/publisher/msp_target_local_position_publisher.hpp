
#pragma once
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>


namespace msp
{

class MspTargetLocalPositionPublisher : public msp::MavlinkMessageListener
{
public:
  explicit MspTargetLocalPositionPublisher(rclcpp::Node* node) : ros2Node( node )
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    px4_publisher = ros2Node->create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>("/msp/out/vehicle_target_local_position", qos);
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    mavlink_position_target_local_ned_t ned;
    mavlink_msg_position_target_local_ned_decode(&msg, &ned);
    
    auto message = px4_msgs::msg::VehicleLocalPositionSetpoint();

    message.x = ned.x;
    message.y = ned.y;
    message.z = ned.z;

    message.vx = ned.vx;
    message.vy = ned.vy;
    message.vz = ned.vz;

    message.acceleration[0] = ned.afx;
    message.acceleration[1] = ned.afy;
    message.acceleration[2] = ned.afz;

    message.yaw = ned.yaw;
    message.yawspeed = ned.yaw_rate;
    
    message.timestamp = ros2Node->get_clock()->now().nanoseconds() / 1000L;
    
    px4_publisher->publish(message);
  
  }

private:

  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr px4_publisher;
  rclcpp::Node* ros2Node;

};

}  // namespace msp