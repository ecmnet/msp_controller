

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

      dispatcher_->addListener(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, this);
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


      mavlink_attitude_quaternion_t att;
      mavlink_msg_attitude_quaternion_decode(&msg, &att);

      geometry_msgs::msg::TransformStamped transform;


      // Set header
      transform.header.stamp = ros2Node->get_clock()->now();
      transform.header.frame_id = "world";      // Parent frame
      transform.child_frame_id  = "base_link";  // Child frame

      // Translation 
      transform.transform.translation.x =  MavlinkMessageListener::model.position.x();
      transform.transform.translation.y =  MavlinkMessageListener::model.position.y();
      transform.transform.translation.z = -MavlinkMessageListener::model.position.z();

      // Rotation 
      transform.transform.rotation.y =  att.q2;
      transform.transform.rotation.x = -att.q3;
      transform.transform.rotation.z = -att.q4;
      transform.transform.rotation.w =  att.q1;

      // Publish the transform
      tf_broadcaster_->sendTransform(transform);
    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Node *ros2Node;
    msp::MspMavlinkDispatcher *dispatcher_;
  };

} // namespace msp