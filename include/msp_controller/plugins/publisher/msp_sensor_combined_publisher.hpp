
#pragma once
#include <px4_msgs/msg/sensor_combined.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspSensorCombinedPublisher : public msp::MavlinkMessageListener
{
public:
  explicit MspSensorCombinedPublisher(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher): ros2Node(node), dispatcher_(dispatcher)
  {
    dispatcher_->addListener(MAVLINK_MSG_ID_HIGHRES_IMU,this);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    px4_publisher = ros2Node->create_publisher<px4_msgs::msg::SensorCombined>("/msp/out/sensor_combined", qos);
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    
    mavlink_highres_imu_t imu;
    mavlink_msg_highres_imu_decode(&msg, &imu);

    auto message = px4_msgs::msg::SensorCombined();
    message.accelerometer_m_s2[0] = imu.xacc;
    message.accelerometer_m_s2[1] = imu.yacc;
    message.accelerometer_m_s2[2] = imu.zacc;

    message.gyro_rad[0] = imu.xgyro;
    message.gyro_rad[1] = imu.xgyro;
    message.gyro_rad[2] = imu.xgyro;
    
    message.timestamp = imu.time_usec;

    px4_publisher->publish(message);
  }

private:
  rclcpp::Publisher<px4_msgs::msg::SensorCombined>::SharedPtr px4_publisher;
  rclcpp::Node* ros2Node;
  msp::MspMavlinkDispatcher* dispatcher_;
};

}  // namespace msp