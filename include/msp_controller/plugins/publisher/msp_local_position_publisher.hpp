
#pragma once
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

  class MspLocalPositionPublisher : public msp::MavlinkMessageListener
  {
  public:
    explicit MspLocalPositionPublisher(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher) : ros2Node(node), dispatcher_(dispatcher)
    {
      dispatcher_->addListener(MAVLINK_MSG_ID_LOCAL_POSITION_NED,this );

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
      px4_publisher = ros2Node->create_publisher<px4_msgs::msg::VehicleLocalPosition>("/msp/out/vehicle_local_position", qos);
     
    }

    void onMessageReceived(mavlink_message_t msg) override
    {
      mavlink_local_position_ned_t ned;
      mavlink_msg_local_position_ned_decode(&msg, &ned);

      auto message = px4_msgs::msg::VehicleLocalPosition();

      message.x = ned.x;
      message.y = ned.y;
      message.z = ned.z;

      message.vx = ned.vx;
      message.vy = ned.vy;
      message.vz = ned.vz;

      // if(last_message.timestamp_sample == 0) {
      message.ax = 0;
      message.ay = 0;
      message.az = 0;
      // } else {
      //   float delta_s = ( ned.time_boot_ms - last_message.timestamp_sample ) / 1000.0f;
      //   message.ax = (message.vx - last_message.vx) / delta_s;
      //   message.ay = (message.vy - last_message.vy) / delta_s;
      //   message.az = (message.vz - last_message.vz) / delta_s;
      // }

      message.xy_valid = true;
      message.z_valid = true;
      message.v_xy_valid = true;
      message.v_z_valid = true;
      message.heading_good_for_control = false;

      message.timestamp = ros2Node->get_clock()->now().nanoseconds() / 1000L;
      message.timestamp_sample = ned.time_boot_ms;

      px4_publisher->publish(message);
      
    }

  private:
    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr px4_publisher;
    rclcpp::Node* ros2Node;
   msp::MspMavlinkDispatcher* dispatcher_;
    
  };

} // namespace msp