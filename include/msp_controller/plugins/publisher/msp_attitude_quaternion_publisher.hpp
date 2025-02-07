
#pragma once
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspAttitudeQuaternionPublisher : public msp::MavlinkMessageListener
{
public:
  explicit MspAttitudeQuaternionPublisher(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher)  : ros2Node(node), dispatcher_(dispatcher)
  {
    dispatcher_->addListener(MAVLINK_MSG_ID_ATTITUDE_QUATERNION,this);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    px4_publisher = ros2Node->create_publisher<px4_msgs::msg::VehicleAttitude>("/msp/out/vehicle_attitude", qos);
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    mavlink_attitude_quaternion_t att;
    mavlink_msg_attitude_quaternion_decode(&msg, &att);
    
    auto message = px4_msgs::msg::VehicleAttitude();

    message.q[0] = att.q1;
    message.q[1] = att.q2;
    message.q[2] = att.q3;
    message.q[3] = att.q4;

    MavlinkMessageListener::model.vehicle_attitude[0] = att.q1;
    MavlinkMessageListener::model.vehicle_attitude[1] = att.q2;
    MavlinkMessageListener::model.vehicle_attitude[2] = att.q3;
    MavlinkMessageListener::model.vehicle_attitude[3] = att.q4;

    message.timestamp = ros2Node->get_clock()->now().nanoseconds() / 1000L;
    message.timestamp_sample = att.time_boot_ms;

    px4_publisher->publish(message);
  }

private:
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr px4_publisher;
  rclcpp::Node* ros2Node;
  msp::MspMavlinkDispatcher* dispatcher_;
 
};

}  // namespace msp