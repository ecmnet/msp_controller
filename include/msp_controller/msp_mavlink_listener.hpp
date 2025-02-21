#pragma once
#include <lquac/mavlink.h>
#include <Eigen/Geometry>

namespace msp
{

    struct DataModel
    {
        Eigen::Vector3f      position;
        Eigen::Vector3f      rpy;
        float                yaw_speed;
        float                altitude_relative;
        float                altitude;

        mavlink_global_position_int_t global_position;
        mavlink_local_position_ned_t  local_position;
        mavlink_battery_status_t      battery_status;
        mavlink_attitude_t            attitude;
    };

    class MavlinkMessageListener
    {

    public:
       

        virtual ~MavlinkMessageListener() = default;
        virtual void onMessageReceived(mavlink_message_t msg) = 0;

        static msp::DataModel model;

    };

}