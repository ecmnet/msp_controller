#pragma once
#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <lquac/mavlink.h>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>

#define GCL_TIMEOUT_US 2000000L;
#define PX4_TIMEOUT_US 2000000L;

namespace msp
{

class MspStatusManager : public msp::MavlinkMessageListener
{
public:
  explicit MspStatusManager(msp::MspMavlinkDispatcher* dispatcher_) : dispatcher(dispatcher_)
  {
    timer_out_timer = dispatcher->getRos2Node()->create_wall_timer(std::chrono::milliseconds(200),
                                                                   std::bind(&MspStatusManager::checkTimeOut, this));
    dispatcher->addListener(MAVLINK_MSG_ID_HEARTBEAT, this);
  }

  void onMessageReceived(mavlink_message_t msg) override;
  void checkTimeOut();
  void initializePX4();



  uint64_t gcl_connect_tms =0;
  uint64_t px4_connect_tms =0;
  bool gcl_connected = false;
  bool px4_connected = false;

private:

  rclcpp::TimerBase::SharedPtr timer_out_timer;
  msp::MspMavlinkDispatcher* dispatcher;
};

}  // namespace msp