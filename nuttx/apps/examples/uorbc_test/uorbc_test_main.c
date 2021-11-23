
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

#include <nuttx/uorbc/uORB.h>
#include <nuttx/uorbc/mod_uorbc.h>
#include <nuttx/uorbc/mod_uorbc_test.h>
#include <nuttx/uorbc/mod_uorbc_platform.h>

#include "./topics_inc/topic_test0.h"
#include "./topics_inc/topic_test1.h"
#include "./topics_inc/topic_test2.h"
#include "./topics_inc/topic_test3.h"
#include "./topics_inc/topic_test4.h"
#include "./topics_inc/topic_test5.h"
#include "./topics_inc/topic_test6.h"
#include "./topics_inc/topic_test7.h"
#include "./topics_inc/topic_test8.h"
#include "./topics_inc/topic_test9.h"
#include "./topics_inc/topic_test10.h"


//-----------------------------------------------------------------
static uorbc_platform_pthread_t thread_pub = 0;
static uorbc_platform_pthread_t thread_sub = 0;
//no multi advertise
static orb_advert_t adv_test0;
/*static orb_advert_t adv_test3;
static orb_advert_t adv_test4;
static orb_advert_t adv_test6;
static orb_advert_t adv_test7;
static orb_advert_t adv_test9;
//multi advertise
static orb_advert_t adv_test1;
static orb_advert_t adv_test2;
static orb_advert_t adv_test5;
static orb_advert_t adv_test8;
static orb_advert_t adv_test10;*/ //herc

static struct test0_s test0_t = {};
/*static struct test1_s test1_t = {};
static struct test2_s test2_t = {};
static struct test3_s test3_t = {};
static struct test4_s test4_t = {};
static struct test5_s test5_t = {};
static struct test6_s test6_t = {};		
static struct test7_s test7_t = {};
static struct test8_s test8_t = {};
static struct test9_s test9_t = {};
static struct test10_s test10_t = {};*/ //herc
//-----------------------------------------------------------------
static void* mod_uorbc_thread_pub( void );
static void* mod_uorbc_thread_sub( void );
static void mod_uorbc_test_init( void );
static void mod_uorbc_check_copy( const struct orb_metadata* meta, int handle, void* buffer );
//-----------------------------------------------------------------
static void* mod_uorbc_thread_pub( void )
{
	uint32_t pub_count = 0;
	int ret = -1;

	uorbc_platform_log( "mod_uorbc_thread_pub\r\n" );

	while( 1 ) {
		uorbc_platform_delay_ms( 1000 );
		pub_count++;
		uorbc_platform_log( "pub_count:%02X\r\n", pub_count );
		{
			test0_t.timestamp 				= pub_count * 100;
			test0_t.magnetometer_timestamp 	= pub_count * 100 + 1;
			test0_t.baro_timestamp 			= pub_count * 100 + 2;
			test0_t.rng_timestamp 			= pub_count * 100 + 3;
			test0_t.flow_timestamp 			= pub_count * 100 + 4;
			test0_t.asp_timestamp 			= pub_count * 100 + 5;
			test0_t.ev_timestamp			= pub_count * 100 + 6;
			test0_t.seq 					= pub_count;
			test0_t.time_usec 				= pub_count * 100 + 7;
			test0_t.gyro_integral_dt		= pub_count * 0.1;
		}

		ret = orb_publish( ORB_ID( test0 ), adv_test0, &test0_t );
		uorbc_platform_log( "mod_uorbc_thread_pub:  orb_publish:   ret = %d \r\n", ret);
		uorbc_platform_log( "mod_uorbc_thread_pub:  orb_publish:   ret = %d, testx_t.timestamp = %u  , \
							 .magnetometer_timestamp = %u  , .baro_timestamp = %u  ,\
							 .rng_timestamp = %u  , .flow_timestamp = %u  ,\
							 .asp_timestamp = %u  , .ev_timestamp = %u  ,\
							 .seq = %u  , .time_usec = %u  , .gyro_integral_dt = %f  .vel_ned_valid = %d\r\n", \
							 ret, test0_t.timestamp, test0_t.magnetometer_timestamp, test0_t.baro_timestamp, test0_t.rng_timestamp, \
							 test0_t.flow_timestamp, test0_t.asp_timestamp, test0_t.ev_timestamp, test0_t.seq, test0_t.time_usec, \
							 test0_t.gyro_integral_dt, test0_t.vel_ned_valid);
/*		test1_t.seq = pub_count+10;
		orb_publish(ORB_ID(test1),adv_test1,&test1_t);
		test2_t.seq = pub_count+20;
		orb_publish(ORB_ID(test2),adv_test2,&test2_t);
		test3_t.seq = pub_count+30;
		orb_publish(ORB_ID(test3),adv_test3,&test3_t);
		test4_t.seq = pub_count+40;
		orb_publish(ORB_ID(test4),adv_test4,&test4_t);
		test5_t.seq = pub_count+50;
		orb_publish(ORB_ID(test5),adv_test5,&test5_t);
		test6_t.seq = pub_count+60;
		orb_publish(ORB_ID(test6),adv_test6,&test6_t);
		test7_t.seq = pub_count+70;
		orb_publish(ORB_ID(test7),adv_test7,&test7_t);
		test8_t.seq = pub_count+80;
		orb_publish(ORB_ID(test8),adv_test8,&test8_t);
		test9_t.seq = pub_count+90;
		orb_publish(ORB_ID(test9),adv_test9,&test9_t);
		test10_t.seq = pub_count+100;
		orb_publish(ORB_ID(test10),adv_test10,&test10_t);*/  // herc
	}
}
//-----------------------------------------------------------------
static void mod_uorbc_check_copy( const struct orb_metadata* meta, int handle, void* buffer )
{
	int ret = 0;
	bool updated;
	
	do {
		updated = false;
		ret = orb_check( handle, &updated );
		uorbc_platform_log( "mod_uorbc_check_copy:  repeat ret:%d updated:%d\r\n", ret, updated );
		if( updated ) {
			if( 0 == orb_copy( meta, handle, buffer ) ) {
				uorbc_platform_log( "mod_uorbc_check_copy:  meta->o_name:%s\r\n", meta->o_name );
				//buffer_dump((char *)buffer, meta->o_size);											//test code by herc
				//uorbc_platform_log_bin(buffer, meta->o_size);
			}
			else {
				uorbc_platform_log_err( "mod_uorbc_check_copy:  orb_copy fail\r\n" );
			}
		}
	}
	while( ret == 1 );
}
//-----------------------------------------------------------------
static void* mod_uorbc_thread_sub( void )
{
	int sub_test0, sub_test1, sub_test2, sub_test3, sub_test4, sub_test5;
	int sub_test6, sub_test7, sub_test8, sub_test9, sub_test10;
	volatile int instance = 0;
	bool updated;

	uorbc_platform_log( "mod_uorbc_thread_sub\r\n" );

	struct test0_s sub_test0_t = {};
/*	struct test1_s sub_test1_t = {};
	struct test2_s sub_test2_t = {};
	struct test3_s sub_test3_t = {};
	struct test4_s sub_test4_t = {};
	struct test5_s sub_test5_t = {};
	struct test6_s sub_test6_t = {};
	struct test7_s sub_test7_t = {};
	struct test8_s sub_test8_t = {};
	struct test9_s sub_test9_t = {};
	struct test10_s sub_test10_t = {};*/  //herc

	sub_test0 = orb_subscribe( ORB_ID( test0 ) );   //  ORB_ID( test0 )   =   &__orb_test0
/*	sub_test3 = orb_subscribe( ORB_ID( test3 ) );
	sub_test4 = orb_subscribe( ORB_ID( test4 ) );
	sub_test6 = orb_subscribe( ORB_ID( test6 ) );
	sub_test7 = orb_subscribe( ORB_ID( test7 ) );
	sub_test9 = orb_subscribe( ORB_ID( test9 ) );

	instance++;
	sub_test1 = orb_subscribe_multi( ORB_ID( test1 ), instance );
	instance++;
	sub_test2 = orb_subscribe_multi( ORB_ID( test2 ), instance );
	instance++;
	sub_test5 = orb_subscribe_multi( ORB_ID( test5 ), instance );
	instance++;
	sub_test8 = orb_subscribe_multi( ORB_ID( test8 ), instance );
	instance++;
	sub_test10 = orb_subscribe_multi( ORB_ID( test10 ), instance );*/ //herc

	while( 1 ) {
		uorbc_platform_log( "*******************mod_uorbc_thread_sub  run onetime******************\r\n" );
		uorbc_platform_delay_ms( 1000 );

		mod_uorbc_check_copy( ORB_ID( test0 ), sub_test0, &sub_test0_t );
		uorbc_platform_log("*************************message**************************\r\n");
		//uorbc_platform_log("message: sub_test0_t[0]=%d  sub_test0_t[1]=%d\r\n", *((char *)&sub_test0_t), *((char *)&sub_test0_t + 1));
		//uorbc_platform_log("message: sub_test0_t.timestamp=%d  sub_test0_t.seq=%d  sub_test0_t.armed=%d   sub_test0_t._padding0[0]=%d\r\n", sub_test0_t.timestamp, sub_test0_t.seq, sub_test0_t.armed, sub_test0_t._padding0[0]);
		uorbc_platform_log( "message:  mod_uorbc_thread_sub:  testx_t.timestamp = %u  , \
							 .magnetometer_timestamp = %u  , .baro_timestamp = %u  ,\
							 .rng_timestamp = %u  , .flow_timestamp = %u  ,\
							 .asp_timestamp = %u  , .ev_timestamp = %u  ,\
							 .seq = %u  , .time_usec = %u  , .gyro_integral_dt = %f  .vel_ned_valid = %d\r\n", \
							 sub_test0_t.timestamp, sub_test0_t.magnetometer_timestamp, sub_test0_t.baro_timestamp, sub_test0_t.rng_timestamp, \
							 sub_test0_t.flow_timestamp, sub_test0_t.asp_timestamp, sub_test0_t.ev_timestamp, sub_test0_t.seq, sub_test0_t.time_usec, \
							 sub_test0_t.gyro_integral_dt, sub_test0_t.vel_ned_valid);
		//uorbc_platform_log("****************************************************\r\n\r\n");
	/*	mod_uorbc_check_copy( ORB_ID( test1 ), sub_test1, &sub_test1_t );
		mod_uorbc_check_copy( ORB_ID( test2 ), sub_test2, &sub_test2_t );
		mod_uorbc_check_copy( ORB_ID( test3 ), sub_test3, &sub_test3_t );
		mod_uorbc_check_copy( ORB_ID( test4 ), sub_test4, &sub_test4_t );
		mod_uorbc_check_copy( ORB_ID( test5 ), sub_test5, &sub_test5_t );
		mod_uorbc_check_copy( ORB_ID( test6 ), sub_test6, &sub_test6_t );
		mod_uorbc_check_copy( ORB_ID( test7 ), sub_test7, &sub_test7_t );
		mod_uorbc_check_copy( ORB_ID( test8 ), sub_test8, &sub_test8_t );
		mod_uorbc_check_copy( ORB_ID( test9 ), sub_test9, &sub_test0_t );
		mod_uorbc_check_copy( ORB_ID( test10 ), sub_test10, &sub_test10_t );*/ //herc
	}
}
//-----------------------------------------------------------------
static void mod_uorbc_test_init( void )
{
	volatile int instance = 0;

	uorbc_platform_log( "mod_uorbc_test_init\r\n" );

	/* 不同topic的内容 */
	test0_t.vel_ned_valid = true;
/*	test1_t.seq = 1;
	test2_t.seq = 2;
	test3_t.seq = 3;
	test4_t.seq = 4;
	test5_t.seq = 5;
	test6_t.seq = 6;
	test7_t.seq = 7;
	test8_t.seq = 8;
	test9_t.seq = 9;
	test10_t.seq = 10;*/  //herc

	/*	queue模式：
		orb_advertise_queue 测试OK
		orb_advertise_multi_queue 测试OK
	*/
	adv_test0 = orb_advertise_queue( ORB_ID( test0 ), &test0_t, 1 );		//ORB_ID( test0 ) =  &__orb_test0
/*	adv_test3 = orb_advertise_queue( ORB_ID( test3 ), &test3_t, 2 );
	adv_test4 = orb_advertise_queue( ORB_ID( test4 ), &test4_t, 3 );
	adv_test6 = orb_advertise_queue( ORB_ID( test6 ), &test6_t, 4 );
	adv_test7 = orb_advertise_queue( ORB_ID( test7 ), &test7_t, 5 );
	adv_test9 = orb_advertise_queue( ORB_ID( test9 ), &test9_t, 6 );

	instance++;
	adv_test1 = orb_advertise_multi_queue( ORB_ID( test1 ), &test1_t, &instance, instance );
	instance++;
	adv_test2 = orb_advertise_multi_queue( ORB_ID( test2 ), &test2_t, &instance, instance );
	instance++;
	adv_test5 = orb_advertise_multi_queue( ORB_ID( test5 ), &test5_t, &instance, instance );
	instance++;
	adv_test8 = orb_advertise_multi_queue( ORB_ID( test8 ), &test8_t, &instance, instance );
	instance++;
	adv_test10 = orb_advertise_multi_queue( ORB_ID( test10 ), &test10_t, &instance, instance );*/  //herc



	/*	非queue模式：
		orb_advertise 测试OK
		orb_advertise_multi	测试OK
	*/
//	adv_test0 = orb_advertise( ORB_ID( test0 ), &test0_t );
//	adv_test3 = orb_advertise( ORB_ID( test3 ), &test3_t );
//	adv_test4 = orb_advertise( ORB_ID( test4 ), &test4_t );
//	adv_test6 = orb_advertise( ORB_ID( test6 ), &test6_t );
//	adv_test7 = orb_advertise( ORB_ID( test7 ), &test7_t );
//	adv_test9 = orb_advertise( ORB_ID( test9 ), &test9_t );

//	instance++;
//	adv_test1 = orb_advertise_multi( ORB_ID( test1 ), &test1_t, &instance );
//	instance++;
//	adv_test2 = orb_advertise_multi( ORB_ID( test2 ), &test2_t, &instance );
//	instance++;
//	adv_test5 = orb_advertise_multi( ORB_ID( test5 ), &test5_t, &instance );
//	instance++;
//	adv_test8 = orb_advertise_multi( ORB_ID( test8 ), &test8_t, &instance );
//	instance++;
//	adv_test10 = orb_advertise_multi( ORB_ID( test10 ), &test10_t, &instance );
}
//-----------------------------------------------------------------
void mod_uorbc_test( void )
{
	orb_init();

	mod_uorbc_test_init();

	mod_uorbc_platform_thread_create( &thread_pub, mod_uorbc_thread_pub, NULL );

	mod_uorbc_platform_thread_create( &thread_sub, mod_uorbc_thread_sub, NULL );
}


//-----------------------------------------------------------------


/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, char *argv[])
{
  int ret; 

  mod_uorbc_test();

  return EXIT_SUCCESS;
}



//test code
int buffer_dump(char* buffer, size_t len)
{
	int i = 0;

	if(buffer == NULL)
	{
		uorbc_platform_log("buffer_dump:   error :   buffer is null");
		return -1;
	}

	if(len > 31)
	{
		uorbc_platform_log("buffer_dump:   error :  len > 31");
		return -1;
	}
	
	uorbc_platform_log("buffer_dump:   \r\n");
	for(i = 0; i < len; i++)
	{
		uorbc_platform_log("                    buffer[%d] = %d\r\n", i, *(buffer+i));
	}
	return 0;

}
