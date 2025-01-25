#pragma once
#include <lquac/mavlink.h>

namespace msp
{

class MavlinkMessageListener {
public:
    virtual ~MavlinkMessageListener() = default;
    virtual void onMessageReceived(mavlink_message_t msg) = 0;
};

}