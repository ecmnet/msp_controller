

#pragma once
#include <tf2_ros/transform_broadcaster.h>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <msp_controller/frame_conversion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

  class MspBodyFrameBroadcaster : public msp::MavlinkMessageListener
  {
  public:
    explicit MspBodyFrameBroadcaster(rclcpp::Node *node, msp::MspMavlinkDispatcher *dispatcher) : ros2Node(node), dispatcher_(dispatcher)
    {

     // dispatcher_->addListener(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, this);
      dispatcher->addListener(MAVLINK_MSG_ID_ATTITUDE,this);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros2Node);    

      geometry_msgs::msg::TransformStamped transform;

      // Set header
      transform.header.stamp = ros2Node->get_clock()->now();
      transform.header.frame_id = "world";      // Parent frame
      transform.child_frame_id  = "base_link";  // Child frame

      // Translation 
      transform.transform.translation.x = 0;
      transform.transform.translation.y = 0;
      transform.transform.translation.z = 0;

      // Rotation 
      transform.transform.rotation.x = 0;
      transform.transform.rotation.y = 0;
      transform.transform.rotation.z = 0;
      transform.transform.rotation.w = 1.0f;;

      // Publish the transform
      tf_broadcaster_->sendTransform(transform);

      // Set header
      transform.header.stamp = ros2Node->get_clock()->now();
      transform.header.frame_id = "base_link";    // Parent frame
      transform.child_frame_id  = "camera_link";  // Child frame

      // Translation 
      transform.transform.translation.x = 0;
      transform.transform.translation.y = 0;
      transform.transform.translation.z = 0;

      // Rotation 
      transform.transform.rotation.x = 0;
      transform.transform.rotation.y = 0;
      transform.transform.rotation.z = 0;
      transform.transform.rotation.w = 1.0f;;

      // Publish the transform
      tf_broadcaster_->sendTransform(transform);
    }

    void onMessageReceived(mavlink_message_t msg) override
    {
      // mavlink_attitude_quaternion_t att;
      // mavlink_msg_attitude_quaternion_decode(&msg, &att);

      mavlink_attitude_t att;
      mavlink_msg_attitude_decode(&msg, &att);
      MavlinkMessageListener::model.attitude = att;

      geometry_msgs::msg::TransformStamped transform;

      Eigen::Vector3f rpy(att.roll,-att.pitch,-att.yaw);
      auto q_att = msp::quaternion_from_rpy(rpy);

      // Set header
      transform.header.stamp = ros2Node->get_clock()->now();
      transform.header.frame_id = "world";      // Parent frame
      transform.child_frame_id  = "base_link";  // Child frame

      // Translation from model
      transform.transform.translation.x =  MavlinkMessageListener::model.position.x();
      transform.transform.translation.y = -MavlinkMessageListener::model.position.y();
      transform.transform.translation.z = -MavlinkMessageListener::model.position.z();

      // Rotation 
      transform.transform.rotation.x =  q_att.x();
      transform.transform.rotation.y =  q_att.y();
      transform.transform.rotation.z =  q_att.z();
      transform.transform.rotation.w =  q_att.w();

      // Publish the transform
      tf_broadcaster_->sendTransform(transform);

      // Set model data
      msp::MavlinkMessageListener::model.yaw_speed = att.yawspeed;
      msp::MavlinkMessageListener::model.rpy       = rpy;

    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Node *ros2Node;
    msp::MspMavlinkDispatcher *dispatcher_;
  };

} // namespace msp