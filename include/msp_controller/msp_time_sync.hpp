#pragma once
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#define CONVERGENCE_WINDOW 20
#define MAX_CONS_HIGH_DEVIATION 10
#define MAX_RTT_SAMPLE 10
#define MAX_DEVIATION_SAMPLE 1000
#define MAX_CONS_HIGH_RTT 5

#define FILTER_ALPHA_INITAL 0.05
#define FILTER_BETA_INITAL 0.05
#define FILTER_ALPHA_FINAL 0.003
#define FILTER_BETA_FINAL 0.003

namespace msp
{

class MspTimeSync : public MavlinkMessageListener
{
public:
  explicit MspTimeSync(msp::MspMavlinkDispatcher* dispatcher_) : dispatcher(dispatcher_)
  {
    timer_sync = dispatcher->getRos2Node()->create_wall_timer(std::chrono::milliseconds(100),
                                                         std::bind(&MspTimeSync::sendTimesyncRequest, this));
    dispatcher->addListener(MAVLINK_MSG_ID_TIMESYNC, this);

  }

  void sendTimesyncRequest();
  void onMessageReceived(mavlink_message_t msg) override;

  inline bool sync_converged()
  {
    return sequence >= CONVERGENCE_WINDOW;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_sync;
  msp::MspMavlinkDispatcher* dispatcher;

  double time_offset = 0.0;
  double time_skew = 0.0;

  uint16_t sequence = 0;
  double filter_alpha = FILTER_ALPHA_INITAL;
  double filter_beta = FILTER_BETA_INITAL;

  uint16_t high_rtt_count = 0;
  uint16_t high_deviation_count = 0;

  void add_timesync_observation(uint64_t offset_ns, uint64_t local_time_ns, uint64_t remote_time_ns);
  void add_sample(uint64_t offset_ns);
  void reset_filter();
};

}  // namespace msp
