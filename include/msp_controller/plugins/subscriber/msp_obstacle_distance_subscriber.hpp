#pragma once
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

namespace msp
{

class MspObstacleDistanceSubscriber
{
public:
  explicit MspObstacleDistanceSubscriber(msp::MspMavlinkDispatcher* dispatcher) : dispatcher(dispatcher)
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    subscription = this->dispatcher->getRos2Node()->create_subscription<px4_msgs::msg::ObstacleDistance>(
      "/msp/in/obstacle_distance", qos, [this](const px4_msgs::msg::ObstacleDistance::UniquePtr message)
      {
        
        mavlink_message_t msg;
        mavlink_msg_obstacle_distance_pack(MSP_SYSID,MAV_COMPONENT::MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
               this->dispatcher->getPX4TimeUs(),
               MAV_DISTANCE_SENSOR_LASER,
               message->distances.data(),
               1,
               message->min_distance,
               message->max_distance,
               message->increment,
               message->angle_offset,
               MAV_FRAME::MAV_FRAME_BODY_FRD
               );
        
        this->dispatcher->sendMavlinkMessage(msg); 
            
        });   
  }


private:
  rclcpp::Subscription<px4_msgs::msg::ObstacleDistance>::SharedPtr subscription;
  msp::MspMavlinkDispatcher* dispatcher;
  
};

}