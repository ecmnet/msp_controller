
#include <msp_controller/msp_controller.hpp>
#include <msp_controller/msp_mavlink_dispatcher.hpp>
#include <msp_controller/msp_time_sync.hpp>
#include <msp_controller/msp_mavlink_listener.hpp>
#include <msp_controller/msp_status_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lquac/mavlink.h>

#include <msp_controller/plugins/receivers/msp_trajectory_waypoint_receiver.hpp>

#include <msp_controller/plugins/publisher/msp_local_position_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_target_local_position_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_global_position_publisher.hpp>

#include <msp_controller/plugins/publisher/msp_vehicle_status_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_battery_status_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_sensor_combined_publisher.hpp>
#include <msp_controller/plugins/publisher/msp_message_publisher.hpp>

#include <msp_controller/plugins/services/msp_msp_command_client.hpp>

#include <msp_controller/plugins/subscriber/msp_offboard_setpoint_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_message_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_obstacle_distance_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_heartbeat_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_trajectory_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_debug_vector_subscriber.hpp>
#include <msp_controller/plugins/subscriber/msp_micro_grid_subscriber.hpp>

#include <msp_controller/plugins/tf/msp_body_frame_broadcaster.hpp>


class MavlinkEndpointNode : public rclcpp::Node
{
public:
   MavlinkEndpointNode() : Node("MSPController"), dispatcher( this ), timesync( &dispatcher ), status_manager( &dispatcher )

  {

    RCLCPP_INFO(this->get_logger(), "MSP Controller Endpoint Node started");
    dispatcher.start();
    
  }
 
   
private:

  msp::MspMavlinkDispatcher dispatcher;
  msp::MspTimeSync          timesync;
  msp::MspStatusManager     status_manager;

  msp::MspBodyFrameBroadcaster         t01 = msp::MspBodyFrameBroadcaster(this,&dispatcher);

  msp::MspTrajectoryWaypointReceiver   r02 = msp::MspTrajectoryWaypointReceiver(this,&dispatcher);

  msp::MspLocalPositionPublisher       p01 = msp::MspLocalPositionPublisher(this,&dispatcher);
  msp::MspVehicleStatusPublisher       p03 = msp::MspVehicleStatusPublisher(this,&dispatcher);
  msp::MspBatteryStatusPublisher       p04 = msp::MspBatteryStatusPublisher(this,&dispatcher);
  msp::MspTargetLocalPositionPublisher p05 = msp::MspTargetLocalPositionPublisher(this,&dispatcher);
  msp::MspSensorCombinedPublisher      p06 = msp::MspSensorCombinedPublisher(this,&dispatcher);
  msp::MspGlobalPositionPublisher      p07 = msp::MspGlobalPositionPublisher(this,&dispatcher);
  msp::MspMessagePublisher             p08 = msp::MspMessagePublisher(this,&dispatcher);

  msp::MspMspCommandClient             c01 = msp::MspMspCommandClient(this,&dispatcher);

  msp::MspOffboardSetpointSubscriber   s01 = msp::MspOffboardSetpointSubscriber(&dispatcher);
  msp::MspMessageSubscriber            s02 = msp::MspMessageSubscriber(&dispatcher);
  msp::MspHeartbeatSubscriber          s03 = msp::MspHeartbeatSubscriber(&dispatcher);
  msp::MspTrajectorySubscriber         s04 = msp::MspTrajectorySubscriber(&dispatcher);
  msp::MspDebugVectorSubscriber        s05 = msp::MspDebugVectorSubscriber(&dispatcher);
  msp::MspObstacleDistanceSubscriber   s06 = msp::MspObstacleDistanceSubscriber(&dispatcher);
  msp::MspMicroGridSubscriber          s07 = msp::MspMicroGridSubscriber(&dispatcher);

    

};

msp::DataModel msp::MavlinkMessageListener::model;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavlinkEndpointNode>());
  rclcpp::shutdown();
  return 0;
}