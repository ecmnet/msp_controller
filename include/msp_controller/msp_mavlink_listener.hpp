#pragma once
#include <lquac/mavlink.h>

namespace msp
{

    struct DataModel
    {
        std::array<float, 4> vehicle_attitude;
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