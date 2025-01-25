#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>
#include <msp_controller/msp_time_sync.hpp>
#include <msp_controller/msp_px4.h>

using namespace msp;

void MspTimeSync::sendTimesyncRequest()
{
  mavlink_message_t msg;
  mavlink_timesync_t timesync;

  timesync.tc1 = 0;
  timesync.ts1 = (uint64_t)dispatcher->getRos2Node()->get_clock()->now().nanoseconds();
;

  mavlink_msg_timesync_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &timesync);
  dispatcher->sendMavlinkMessage(msg);

}

void MspTimeSync::onMessageReceived(mavlink_message_t msg)
{
  mavlink_message_t msg2;
  mavlink_timesync_t tsync;
  mavlink_timesync_t tsync_send;

  mavlink_msg_timesync_decode(&msg, &tsync);

  if (msg.sysid != PX4_SYSID)
    return;

  uint64_t now_ns = (uint64_t)dispatcher->getRos2Node()->get_clock()->now().nanoseconds();

  if (tsync.tc1 == 0)
  {
    tsync_send.tc1 = now_ns;
    tsync_send.ts1 = tsync.ts1;
    mavlink_msg_timesync_encode(MSP_SYSID, MAV_COMP_ID_ONBOARD_COMPUTER, &msg2, &tsync_send);
    dispatcher->sendMavlinkMessage(msg2);
  }
  else if (tsync.tc1 > 0)
  {
    // Time offset between this system and the remote system is calculated assuming RTT for
    // the timesync packet is roughly equal both ways.
    add_timesync_observation((tsync.ts1 + now_ns - tsync.tc1 * 2) / 2, tsync.ts1, tsync.tc1);
    uint64_t now_corr = now_ns - (uint64_t)time_offset;
  }
  
}

void MspTimeSync::add_timesync_observation(uint64_t offset_ns, uint64_t local_time_ns, uint64_t remote_time_ns)
{
  uint64_t now_ns = (uint64_t)dispatcher->getRos2Node()->get_clock()->now().nanoseconds();
  uint64_t rtt_ns = now_ns - local_time_ns;
  uint64_t deviation = llabs(int64_t(time_offset) - offset_ns);

  if (rtt_ns < MAX_RTT_SAMPLE * 1000000ULL)
  {
    if ((sequence >= CONVERGENCE_WINDOW) && (deviation > MAX_DEVIATION_SAMPLE * 1000000ULL))
    {
      high_deviation_count++;
      if (high_deviation_count > MAX_CONS_HIGH_DEVIATION)
      {
        RCLCPP_WARN(dispatcher->getRos2Node()->get_logger(), "Time jump detected. Resetting the time synchronizer");
        dispatcher->setTimeOffset(0);
        reset_filter();
      }
    }
    else
    {
      if (sequence < CONVERGENCE_WINDOW)
      {
        float progress = (float)sequence / CONVERGENCE_WINDOW;
        float p = 1.0f - (float)std::exp(0.5f * (1.0f - 1.0f / (1.0f - progress)));
        filter_alpha = p * FILTER_ALPHA_FINAL + (1.0f - p) * FILTER_ALPHA_INITAL;
        filter_beta = p * FILTER_BETA_FINAL + (1.0f - p) * FILTER_BETA_INITAL;
      }
      else
      {
        filter_alpha = FILTER_ALPHA_FINAL;
        filter_beta = FILTER_BETA_FINAL;
      }

      add_sample(offset_ns);
      dispatcher->setTimeOffset(time_offset);
      sequence++;
      high_deviation_count = 0;
      high_rtt_count = 0;
    }
  }
  else
  {
    high_rtt_count++;
    if (high_rtt_count > MAX_CONS_HIGH_RTT)
    {
      RCLCPP_WARN(dispatcher->getRos2Node()->get_logger(), "RTT too high for timesync: %f ms", rtt_ns / 1000000.0f);
      high_rtt_count = 0;
    }
  }
}

void MspTimeSync::add_sample(uint64_t offset_ns)
{
  // Online exponential smoothing filter. The derivative of the estimate is also
  // estimated in order to produce an estimate without steady state lag:
  // https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing

  double time_offset_prev = time_offset;
  if (sequence == 0)
  {  // First offset sample
    time_offset = offset_ns;
  }
  else
  {
    // Update the clock offset estimate
    time_offset = filter_alpha * offset_ns + (1.0 - filter_alpha) * (time_offset + time_skew);
    // Update the clock skew estimate
    time_skew = filter_beta * (time_offset - time_offset_prev) + (1.0 - filter_beta) * time_skew;
  }
}

void MspTimeSync::reset_filter()
{
  sequence = 0;
  time_offset = 0.0;
  time_skew = 0.0;
  filter_alpha = FILTER_ALPHA_INITAL;
  filter_beta = FILTER_BETA_INITAL;
  high_deviation_count = 0;
  high_rtt_count = 0;
}