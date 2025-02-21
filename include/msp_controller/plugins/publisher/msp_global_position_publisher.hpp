
#pragma once
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

  class MspGlobalPositionPublisher : public msp::MavlinkMessageListener
  {
  public:
    explicit MspGlobalPositionPublisher(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher) : ros2Node(node), dispatcher_(dispatcher)
    {
      dispatcher_->addListener(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,this );

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
      px4_publisher = ros2Node->create_publisher<px4_msgs::msg::VehicleGlobalPosition>("/msp/out/vehicle_global_position", qos);
     
    }

    void onMessageReceived(mavlink_message_t msg) override
    {
      mavlink_global_position_int_t global;
      mavlink_msg_global_position_int_decode(&msg, &global);

      // Put into model
      MavlinkMessageListener::model.global_position = global;
      
    }

  private:
   rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr px4_publisher;
   rclcpp::Node* ros2Node;
   msp::MspMavlinkDispatcher* dispatcher_;
    
  };

} // namespace msp