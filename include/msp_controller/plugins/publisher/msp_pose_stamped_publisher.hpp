
#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

  class MspPoseStampedPublisher : public msp::MavlinkMessageListener
  {
  public:
    explicit MspPoseStampedPublisher(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher): ros2Node(node), dispatcher_(dispatcher)
    {
      dispatcher_->addListener(MAVLINK_MSG_ID_LOCAL_POSITION_NED,this);
      
      px4_publisher = ros2Node->create_publisher<geometry_msgs::msg::PoseStamped>("/msp/out/pose", 1);
    }

    void onMessageReceived(mavlink_message_t msg) override
    {
      mavlink_local_position_ned_t ned;
      mavlink_msg_local_position_ned_decode(&msg, &ned);

      auto message = geometry_msgs::msg::PoseStamped();

      message.pose.position.x =  ned.x;
      message.pose.position.y = -ned.y;
      message.pose.position.z = -ned.z;

      // TODO: Orientation does not work properly
      message.pose.orientation.w = MavlinkMessageListener::model.vehicle_attitude[0];
      message.pose.orientation.x = MavlinkMessageListener::model.vehicle_attitude[1];
      message.pose.orientation.y = MavlinkMessageListener::model.vehicle_attitude[2];
      message.pose.orientation.z = -MavlinkMessageListener::model.vehicle_attitude[3];

      message.header.stamp = ros2Node->get_clock()->now();
      message.header.frame_id = "world"; 

      px4_publisher->publish(message);
      
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr px4_publisher;
    rclcpp::Node* ros2Node;
    msp::MspMavlinkDispatcher* dispatcher_;
   
  };

} // namespace msp