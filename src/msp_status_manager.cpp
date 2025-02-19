
#include <msp_controller/msp_status_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <lquac/mavlink.h>
#include <msp_controller/msp_px4.h>

using namespace msp;

void MspStatusManager::onMessageReceived(mavlink_message_t msg)
{
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(&msg, &hb);
  switch (msg.sysid)
  {
    case PX4_SYSID:
    
      if(!px4_connected)
       initializePX4();

      px4_connect_tms = dispatcher->getRos2TimeMs();
      px4_connected = true;
      break;

    case GCL_SYSID:
      if(!gcl_connected)
        RCLCPP_INFO(dispatcher->getRos2Node()->get_logger(), "MSP regained GCL connection");
      gcl_connect_tms = dispatcher->getRos2TimeMs();
      gcl_connected = true;
      break;
    // TODO add more components
  }
}

void MspStatusManager::checkTimeOut()
{
  uint64_t now = dispatcher->getRos2TimeMs();

  if (px4_connected && (now - px4_connect_tms) > PX4_TIMEOUT_MS)
  {
    RCLCPP_ERROR(dispatcher->getRos2Node()->get_logger(), "MSP lost PX4 connection");
    px4_connected = false;
  }

  if (gcl_connected && (now - gcl_connect_tms) > GCL_TIMEOUT_MS)
  {
    RCLCPP_ERROR(dispatcher->getRos2Node()->get_logger(), "MSP lost GCL connection");
    gcl_connected = false;
    if(px4_connected) {
    ; //TODO: Action? Land Loiter? or leave it to PX4?
    }
  }
  
  dispatcher->setConnected(gcl_connected, px4_connected);

}

void MspStatusManager::initializePX4() {

  // disable some streams
   dispatcher->sendPX4MavlinkCommand( MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ESC_INFO, -1);
   dispatcher->sendPX4MavlinkCommand( MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SCALED_PRESSURE, -1);

  // set rates for some streams
  dispatcher->sendPX4MavlinkCommand( MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 10000);
  dispatcher->sendPX4MavlinkCommand( MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10000);
  dispatcher->sendPX4MavlinkCommand( MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 25000);
  dispatcher->sendPX4MavlinkCommand( MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ESTIMATOR_STATUS, 50000);

  RCLCPP_INFO(dispatcher->getRos2Node()->get_logger(), "PX4 setup performed");

}