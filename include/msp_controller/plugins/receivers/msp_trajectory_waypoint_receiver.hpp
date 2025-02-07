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

   mavlink_msg_trajectory_representation_waypoints_encode(PX4_SYSID,MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS, &msg, &traj);
   dispatcher_->sendMavlinkMessage(msg);


  }

private:
 rclcpp::Node* ros2Node;
msp::MspMavlinkDispatcher* dispatcher_;
};

}  // namespace msp