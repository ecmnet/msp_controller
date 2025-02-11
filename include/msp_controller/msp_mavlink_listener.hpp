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
    };

    class MavlinkMessageListener
    {

    public:
       

        virtual ~MavlinkMessageListener() = default;
        virtual void onMessageReceived(mavlink_message_t msg) = 0;

        static msp::DataModel model;

    };

}