#include <nuttx/uorbc/mod_uorbc_utils.h>

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


//----------------------------------------------------------------
const e_orbc_id orb_multi_list[MULTI_ORB_NUM] = {
	ORBC_TEST3,
	ORBC_TEST4,
	ORBC_TEST5,
	ORBC_TEST8,
	ORBC_TEST10
};
//----------------------------------------------------------------
int mod_uorbc_utils_get_orbid( const char* name )
{
	if( 0 == strcmp( name, ( ORB_ID( test0 ) )->o_name ) ) {
		return ORBC_TEST0;
	}
	/*else if( 0 == strcmp( name, ( ORB_ID( test1 ) )->o_name ) ) {
		return ORBC_TEST1;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test2 ) )->o_name ) ) {
		return ORBC_TEST2;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test3 ) )->o_name ) ) {
		return ORBC_TEST3;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test4 ) )->o_name ) ) {
		return ORBC_TEST4;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test5 ) )->o_name ) ) {
		return ORBC_TEST5;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test6 ) )->o_name ) ) {
		return ORBC_TEST6;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test7 ) )->o_name ) ) {
		return ORBC_TEST7;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test8 ) )->o_name ) ) {
		return ORBC_TEST8;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test9 ) )->o_name ) ) {
		return ORBC_TEST9;
	}
	else if( 0 == strcmp( name, ( ORB_ID( test10 ) )->o_name ) ) {
		return ORBC_TEST10;
	}*/ //herc
	else {
		return -1;
	}
}
//----------------------------------------------------------------
bool mod_uorbc_utils_ismulti( const int orb_id )
{
	for( int i = 0; i < MULTI_ORB_NUM; ++i ) {		
		uorbc_platform_log( "mod_uorbc_utils_ismulti:   orb_multi_list[i]=%d\r\n", orb_multi_list[i]);
		if( orb_id == ( int ) orb_multi_list[i] ) {
			uorbc_platform_log( "mod_uorbc_utils_ismulti:   orb_id=%d\r\n", orb_id);
			return true;
		}
	}

	return false;
}
//----------------------------------------------------------------


