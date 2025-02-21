
#pragma once
#include <px4_msgs/msg/battery_status.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspBatteryStatusPublisher : public msp::MavlinkMessageListener
{
public:
  explicit MspBatteryStatusPublisher(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher) : ros2Node(node), dispatcher_(dispatcher)
  {
    dispatcher_->addListener(MAVLINK_MSG_ID_BATTERY_STATUS,this);
    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    px4_publisher = ros2Node->create_publisher<px4_msgs::msg::BatteryStatus>("/msp/out/battery_status", qos);
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    mavlink_battery_status_t bat;
    mavlink_msg_battery_status_decode(&msg, &bat);
    MavlinkMessageListener::model.battery_status = bat;

    auto message = px4_msgs::msg::BatteryStatus();

    message.voltage_v = 0;
			for (uint8_t i = 0; i < 10; i++) {
				if (bat.voltages[i] < 65535)
					message.voltage_v += static_cast<float>(bat.voltages[i]);
			}
    message.voltage_v = message.voltage_v / 1000.0f;
    message.connected = true;
    message.id = 1;
    message.timestamp = ros2Node->get_clock()->now().nanoseconds() / 1000L;

    px4_publisher->publish(message);
  }

private: 
  rclcpp::Publisher<px4_msgs::msg::BatteryStatus>::SharedPtr px4_publisher;
  rclcpp::Node* ros2Node;
  msp::MspMavlinkDispatcher* dispatcher_;
 
};

}  // namespace msp