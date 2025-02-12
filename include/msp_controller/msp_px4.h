//#pragma once
#define PX4_SYSID 1
#define MSP_SYSID 2
#define GCL_SYSID 255

#define MSP_COMP_CONTROLLER 191
#define MSP_COMP_OFFBOARD   25
#define MSP_COMP_VISION     26
#define MSP_COMP_ODOMETRY   27
#define MSP_COMP_MAP        28
#define MSP_COMP_AVOID      196
#define MSP_COMP_AI         29

#define PX4_CUSTOM_MAIN_MODE_MANUAL 1
#define PX4_CUSTOM_MAIN_MODE_ALTCTL 2
#define PX4_CUSTOM_MAIN_MODE_POSCTL 3
#define PX4_CUSTOM_MAIN_MODE_AUTO 4
#define PX4_CUSTOM_MAIN_MODE_ACRO 5
#define PX4_CUSTOM_MAIN_MODE_OFFBOARD 6
#define PX4_CUSTOM_MAIN_MODE_STABILIZED 7
#define PX4_CUSTOM_MAIN_MODE_RATTITUDE 8

#define PX4_CUSTOM_SUB_MODE_AUTO_READY 1
#define PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF 2
#define PX4_CUSTOM_SUB_MODE_AUTO_LOITER 3
#define PX4_CUSTOM_SUB_MODE_AUTO_MISSION 4
#define PX4_CUSTOM_SUB_MODE_AUTO_RTL 5
#define PX4_CUSTOM_SUB_MODE_AUTO_LAND 6
#define PX4_CUSTOM_SUB_MODE_AUTO_RTGS 7
#define PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_ME 8
#define PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND 9



#ifndef HAVE_ENUM_MAV_STATE
#define HAVE_ENUM_MAV_STATE
typedef enum MAV_STATE
{
   MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
   MAV_STATE_BOOT=1, /* System is booting up. | */
   MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
   MAV_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
   MAV_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
   MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
   MAV_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
   MAV_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
   MAV_STATE_FLIGHT_TERMINATION=8, /* System is terminating itself. | */
   MAV_STATE_ENUM_END=9, /*  | */
} MAV_STATE;

#endif

#ifndef HAVE_ENUM_MAV_SEVERITY
#define HAVE_ENUM_MAV_SEVERITY

typedef enum MAV_SEVERITY
{
   MAV_SEVERITY_EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
   MAV_SEVERITY_ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
   MAV_SEVERITY_CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
   MAV_SEVERITY_ERROR=3, /* Indicates an error in secondary/redundant systems. | */
   MAV_SEVERITY_WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
   MAV_SEVERITY_NOTICE=5, /* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
   MAV_SEVERITY_INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
   MAV_SEVERITY_DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
   MAV_SEVERITY_ENUM_END=8, /*  | */
} MAV_SEVERITY;

#endif


// MSP Command defintion
#ifndef HAVE_ENUM_MSP_CMD
#define HAVE_ENUM_MSP_CMD
typedef enum MSP_CMD
{
    MSP_CMD_OFFBOARD_SETLOCALPOS=73,
    SELECT_VIDEO_STREAM = 80,
} MSP_CMD;

#endif