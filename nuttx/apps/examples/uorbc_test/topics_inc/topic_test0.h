/* Auto-generated by genmsg_cpp from file /Volumes/SD/Git/PX4/Firmware_fmuv2/Firmware/msg/actuator_armed.msg */


#pragma once

#include <stdint.h>
#ifdef __cplusplus
#include <cstring>
#else
#include <string.h>
#endif

#include <nuttx/uorbc/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct test0_s {
#else
struct test0_s {
#endif
	uint16_t timestamp; // required for logger
	uint16_t magnetometer_timestamp;
	uint16_t baro_timestamp;
	uint16_t rng_timestamp;
	uint16_t flow_timestamp;
	uint16_t asp_timestamp;
	uint16_t ev_timestamp;
	uint16_t seq;
	uint16_t time_usec;
	float gyro_integral_dt;
	bool vel_ned_valid;

#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(test0);   //extern const struct orb_metadata __orb_test0

