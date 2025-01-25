#pragma once
#include <msp_controller/msp_controller.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <msp_controller/msp_time_sync.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <msp_controller/msp_status_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>


#include <msp_controller/plugins/publisher/msp_local_position_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_target_local_position_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_attitude_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_vehicle_status_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_battery_status_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_sensor_combined_publisher.hpp>

#include <msp_controller/plugins/services/msp_msp_command_client.hpp>

#include <msp_controller/plugins/subscriber/msp_offboard_setpoint_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_message_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_heartbeat_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_trajectory_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_debug_vector_subscriber.hpp>


class MavlinkEndpointNode : public rclcpp::Node
{
public:
   MavlinkEndpointNode() : Node("MSPController"), dispatcher( this ), timesync( &dispatcher ), status_manager( &dispatcher )

  {
    RCLCPP_INFO(this->get_logger(), "MSP Controller Endpoint Node started");

    dispatcher.addListener(MAVLINK_MSG_ID_LOCAL_POSITION_NED,             &p01 );
    dispatcher.addListener(MAVLINK_MSG_ID_ATTITUDE_QUATERNION,            &p02 );
    dispatcher.addListener(MAVLINK_MSG_ID_HEARTBEAT,                      &p03 );
    dispatcher.addListener(MAVLINK_MSG_ID_BATTERY_STATUS,                 &p04 );
    dispatcher.addListener(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED,      &p05 );
    dispatcher.addListener(MAVLINK_MSG_ID_HIGHRES_IMU,                    &p06 );

    dispatcher.addListener(MAVLINK_MSG_ID_MSP_COMMAND,         &c01 );
    dispatcher.start();
    
  }
 
   
private:

  msp::MspMavlinkDispatcher dispatcher;
  msp::MspTimeSync          timesync;
  msp::MspStatusManager     status_manager;

  msp::MspLocalPositionPublisher       p01 = msp::MspLocalPositionPublisher(this);
  msp::MspAttitudePublisher            p02 = msp::MspAttitudePublisher(this);
  msp::MspVehicleStatusPublisher       p03 = msp::MspVehicleStatusPublisher(this);
  msp::MspBatteryStatusPublisher       p04 = msp::MspBatteryStatusPublisher(this);
  msp::MspTargetLocalPositionPublisher p05 = msp::MspTargetLocalPositionPublisher(this);
  msp::MspSensorCombinedPublisher      p06 = msp::MspSensorCombinedPublisher(this);

  msp::MspMspCommandClient             c01 = msp::MspMspCommandClient(this);

  msp::MspOffboardSetpointSubscriber   s01 = msp::MspOffboardSetpointSubscriber(&dispatcher);
  msp::MspMessageSubscriber            s02 = msp::MspMessageSubscriber(&dispatcher);
  msp::MspHeartbeatSubscriber          s03 = msp::MspHeartbeatSubscriber(&dispatcher);
  msp::MspTrajectorySubscriber         s04 = msp::MspTrajectorySubscriber(&dispatcher);
  msp::MspDebugVectorSubscriber        s05 = msp::MspDebugVectorSubscriber(&dispatcher);
    

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavlinkEndpointNode>());
  rclcpp::shutdown();
  return 0;
}