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
struct test9_s {
#else
struct test9_s {
#endif
	uint64_t timestamp; // required for logger
	uint32_t seq;
	uint8_t _padding0[4]; // required for logger

#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(test9);


