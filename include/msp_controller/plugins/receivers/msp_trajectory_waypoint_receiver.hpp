#pragma once
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>
#include <msp_controller/msp_px4.h>

namespace msp
{

class MspTrajectoryWaypointReceiver : public msp::MavlinkMessageListener
{
public:
  explicit MspTrajectoryWaypointReceiver(rclcpp::Node* node,msp::MspMavlinkDispatcher* dispatcher) : ros2Node(node), dispatcher_(dispatcher)
  {
    dispatcher_->addListener(MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS,this);
  }

  void onMessageReceived(mavlink_message_t msg) override
  {
    mavlink_trajectory_representation_waypoints_t traj;
    mavlink_msg_trajectory_representation_waypoints_decode(&msg, &traj);
    
    traj.time_usec = ros2Node->get_clock()->now().nanoseconds() / 1000;

    mavlink_message_t reply_msg;
    mavlink_msg_trajectory_representation_waypoints_encode(2,196, &reply_msg, &traj);
    dispatcher_->sendMavlinkMessage(reply_msg);


  }

private:
 rclcpp::Node* ros2Node;
msp::MspMavlinkDispatcher* dispatcher_;
};

}  // namespace msp