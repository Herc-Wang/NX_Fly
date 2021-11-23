#ifndef __MOD_UORBC_UTILS_H__
#define __MOD_UORBC_UTILS_H__


#include <stdio.h>
#include <stdlib.h>
#include <semaphore.h>
#include <nuttx/uorbc/mod_uorbc_ringbuffer.h>
#include <nuttx/uorbc/mod_uorbc_platform.h>

//-------------------------------------------------------------
#define MULTI_ORB_NUM 		5
#define UORB_MAX_SUB   		10	/* orb消息最多可被UORB_MAX_SUB个线程订阅 */

#define uorbc_min(x, y) ({				\
	typeof(x) _min1 = (x);			\
	typeof(y) _min2 = (y);			\
	(void) (&_min1 == &_min2);		\
	_min1 < _min2 ? _min1 : _min2; })

#define uorbc_max(x, y) ({				\
	typeof(x) _max1 = (x);			\
	typeof(y) _max2 = (y);			\
	(void) (&_max1 == &_max2);		\
	_max1 > _max2 ? _max1 : _max2; })

typedef enum {
	ORBC_TEST0 = 0,
/*	ORBC_TEST1 = 1,*/
	ORBC_TEST2 = 2,
	ORBC_TEST3 = 3,
	ORBC_TEST4 = 4,
	ORBC_TEST5 = 5,
	ORBC_TEST6 = 6,
	ORBC_TEST7 = 7,
	ORBC_TEST8 = 8,
	ORBC_TEST9 = 9,
	ORBC_TEST10 = 10,
	TOTAL_UORBC_NUM =1,
}e_orbc_id;
//-------------------------------------------------------------
typedef struct _orb_data {
	void*                               data;
	int32_t                             orb_id;
	uint64_t                            interval;
	ring_buffer_t*			            queue;
	bool                                published; /* 表示有新消息发布 */
	uorbc_platform_sem_t                sem;
	bool                                sem_taken;
	uint64_t                            last_updated_time;

	/* 仅用于queue模式 */
	uint32_t							cur_gen; /* 产生的pub总数 */
	uint32_t							last_gen;  /**< last generation the subscriber has seen */
	uint 								queue_size; /* 队列深度 */
	size_t 								qbuf_size; /* 队列buffer大小 */

	/* 订阅此消息的线程列表 */
	int32_t*                            registered_list;

	/* 已取到此orb消息的线程列表,
	如果orb有更新才读取,否则不重复读取,
	advertise和publish的时候置为-1 */
	int32_t*                            authority_list;
} orb_data_t, *p_orb_data_t;
//-------------------------------------------------------------
bool mod_uorbc_utils_ismulti( const int orb_id );
int mod_uorbc_utils_get_orbid( const char* name );
//-------------------------------------------------------------


#endif

