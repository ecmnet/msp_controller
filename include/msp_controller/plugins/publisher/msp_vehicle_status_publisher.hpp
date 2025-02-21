
#pragma once
#include <px4_msgs/msg/vehicle_status.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>
#include <msp_controller/msp_px4.h>

namespace msp
{

  class MspVehicleStatusPublisher : public msp::MavlinkMessageListener
  {
  public:
    explicit MspVehicleStatusPublisher(rclcpp::Node *node, msp::MspMavlinkDispatcher *dispatcher) : ros2Node(node), dispatcher_(dispatcher)
    {
      dispatcher_->addListener(MAVLINK_MSG_ID_HEARTBEAT, this);

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
      px4_publisher = ros2Node->create_publisher<px4_msgs::msg::VehicleStatus>("/msp/out/vehicle_status", qos);
    }

    void onMessageReceived(mavlink_message_t msg) override
    {
      if (msg.sysid != PX4_SYSID)
        return;

      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(&msg, &hb);

      auto message = px4_msgs::msg::VehicleStatus();

      if ((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0)
      {
        if (time_armed == 0)
          time_armed = ros2Node->get_clock()->now().nanoseconds() / 1000L;
        message.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
      }

      if ((hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) == 0)
      {
        time_armed = 0;
        message.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
      }

      if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_OFFBOARD))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_POSCTL))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_ALTCTL))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL;
      else if (isMode(hb.custom_mode, PX4_CUSTOM_MAIN_MODE_STABILIZED))
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_STAB;
      else
        message.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL;

      message.timestamp = ros2Node->get_clock()->now().nanoseconds() / 1000L;
      message.nav_state_timestamp = message.timestamp;
      if (time_armed != 0)
        message.armed_time = time_armed;
      else
        message.armed_time = 0;

      message.set__gcs_connection_lost(!dispatcher_->isGCLConnected());

      message.component_id = msg.compid;
      message.system_id = msg.sysid;

      px4_publisher->publish(message);
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr px4_publisher;
    rclcpp::Node *ros2Node;
    msp::MspMavlinkDispatcher *dispatcher_;

    uint64_t time_armed = 0;
    
    inline bool isMode(uint64_t custom, uint16_t mode_auto, uint16_t mode = 0)
    {
      custom = custom & 0xFFFFFFFF;
      if (((custom >> 16) & 0xFF) == mode_auto)
      {
        return ((custom >> 24) & 0xFF) == mode;
      }
      else
        return false;
    }
  };

} // namespace msp