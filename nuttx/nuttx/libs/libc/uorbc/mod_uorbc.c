/********************************************************************************

 **** Copyright (C), 2021, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : mod_uorbc.c
 * Author        : vincent
 * Date          : 2021-06-02
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *
 * Record        :
 * 1.Date        : 2021-06-02
 *   Author      : vincent
 *   Modification: Created file

*************************************************************************************************************/
#include <time.h>

#include <nuttx/uorbc/mod_uorbc.h>
#include <nuttx/uorbc/uORB.h>
#include <nuttx/uorbc/mod_uorbc_utils.h>
#include <nuttx/uorbc/mod_uorbc_platform.h>




//----------------------------------------------------------------
static p_orb_data_t g_orb_buf[TOTAL_UORBC_NUM];
//----------------------------------------------------------------
static void orb_init_multi( int i );
static void orb_init_nomulti( int i );
//----------------------------------------------------------------
static void orb_init_multi( int i )
{
	int ret = -1;
	int j = 0;
	int k = 0;

	uorbc_platform_log( "orb_init_multi:  orb_init_multi\r\n" );

	g_orb_buf[i] = ( p_orb_data_t ) uorbc_platform_malloc( sizeof( orb_data_t ) * ORB_MULTI_MAX_INSTANCES );

	uorbc_platform_assert( g_orb_buf[i] );

	for( j = 0; j < ORB_MULTI_MAX_INSTANCES; ++j ) {
		g_orb_buf[i][j].data = NULL;
		g_orb_buf[i][j].orb_id = -1;
		g_orb_buf[i][j].interval = 0;
		g_orb_buf[i][j].queue = NULL;
		g_orb_buf[i][j].published = false;

		ret = uorbc_platform_sem_init( & ( g_orb_buf[i][j].sem ), 0, 1 );
		if( 0 != ret ) {
			uorbc_platform_perror( "orb_init_multi:  uorbc_platform_sem_init" );
		}

		g_orb_buf[i][j].sem_taken = false;
		g_orb_buf[i][j].last_updated_time = 0;
		g_orb_buf[i][j].registered_list = ( int32_t* ) uorbc_platform_malloc( UORB_MAX_SUB * sizeof( int32_t ) );
		uorbc_platform_assert( g_orb_buf[i]->registered_list );
		g_orb_buf[i][j].authority_list = ( int32_t* ) uorbc_platform_malloc( UORB_MAX_SUB * sizeof( int32_t ) );
		uorbc_platform_assert( g_orb_buf[i]->authority_list );

		for( k = 0; k < UORB_MAX_SUB; ++k ) {
			g_orb_buf[i][j].registered_list[k] = -1;
			g_orb_buf[i][j].authority_list[k] = -1;
		}
	}
}
//----------------------------------------------------------------
static void orb_init_nomulti( int i )
{
	int ret = -1;
	int k = 0;

	uorbc_platform_log( "orb_init_nomulti:  orb_init_nomulti\r\n" );

	g_orb_buf[i] = ( p_orb_data_t ) uorbc_platform_malloc( sizeof( orb_data_t ) );

	uorbc_platform_assert( g_orb_buf[i] );

	g_orb_buf[i]->data = NULL;
	g_orb_buf[i]->orb_id = -1;
	g_orb_buf[i]->interval = 0;
	g_orb_buf[i]->queue = NULL;
	g_orb_buf[i]->published = false;

	ret = uorbc_platform_sem_init( & ( g_orb_buf[i]->sem ), 0, 1 );
	if( 0 != ret ) {
		uorbc_platform_perror( "orb_init_nomulti:  uorbc_platform_sem_init" );
	}

	g_orb_buf[i]->sem_taken = false;
	g_orb_buf[i]->last_updated_time = 0;
	g_orb_buf[i]->registered_list = ( int32_t* ) uorbc_platform_malloc( UORB_MAX_SUB * sizeof( int32_t ) );
	uorbc_platform_assert( g_orb_buf[i]->registered_list );
	g_orb_buf[i]->authority_list = ( int32_t* ) uorbc_platform_malloc( UORB_MAX_SUB * sizeof( int32_t ) );
	uorbc_platform_assert( g_orb_buf[i]->authority_list );

	for( k = 0; k < UORB_MAX_SUB; ++k ) {
		g_orb_buf[i]->registered_list[k] = -1;
		g_orb_buf[i]->authority_list[k] = -1;
	}
}
//----------------------------------------------------------------
int orb_exists( const struct orb_metadata* meta, int instance )
{
	int orb_id = mod_uorbc_utils_get_orbid( meta->o_name );
	if( orb_id < 0 || orb_id >= TOTAL_UORBC_NUM ) {
		return -1;
	}
	if( !mod_uorbc_utils_ismulti( orb_id ) && instance > 0 ) {
		return -1;
	}

	if( ( g_orb_buf[orb_id][instance].data ) != NULL &&
	    ( g_orb_buf[orb_id][instance].orb_id ) != -1 ) {
		return 0;
	}

	return -1;
}
//----------------------------------------------------------------
orb_advert_t orb_advertise( const struct orb_metadata* meta, const void* data )
{
//	uorbc_platform_log( "orb_advertise" );

	return orb_advertise_multi( meta, data, NULL );
}
//----------------------------------------------------------------
orb_advert_t orb_advertise_queue( const struct orb_metadata* meta, const void* data, unsigned int queue_size )
{
	return orb_advertise_multi_queue( meta, data, NULL, queue_size );
}
//----------------------------------------------------------------
//Unqueued version
orb_advert_t orb_advertise_multi( const struct orb_metadata* meta, const void* data, int* instance )
{
	orb_advert_t advert = NULL;
	int inst = -1;
	int orb_id = -1;
	int task_id = -1;
	int i = 0;

//	uorbc_platform_log( "orb_advertise_multi" );

	orb_id = mod_uorbc_utils_get_orbid( meta->o_name );

	uorbc_platform_log( "orb_advertise_multi:  orb name:%s o_size：%d orb_id:%d\r\n", meta->o_name, meta->o_size, orb_id );

	if( orb_id < 0 || orb_id >= TOTAL_UORBC_NUM ) {
		return NULL;
	}
	else {
		if( NULL == instance ) {
			inst = 0;
		}
		else {
			inst = *instance;
		}

		uorbc_platform_log( "orb_advertise_multi:  inst:%d\r\n", inst );

		if( /* ( mod_uorbc_utils_ismulti( orb_id ) ) &&*/ ( inst >= 0 ) && ( inst < ORB_MULTI_MAX_INSTANCES ) ) {
			uorbc_platform_sem_wait( &( g_orb_buf[orb_id][inst].sem ) );
			//reset the published flag to false to prevent subscribing and checking
			g_orb_buf[orb_id][inst].published = false;
//            int atomic_state = orb_lock();
			if( g_orb_buf[orb_id][inst].data == NULL ) {
				g_orb_buf[orb_id][inst].data = ( uint8_t* )uorbc_platform_malloc( sizeof( uint8_t ) * meta->o_size );
				uorbc_platform_assert( g_orb_buf[orb_id][inst].data );
			}
			uorbc_platform_memcpy( g_orb_buf[orb_id][inst].data, data, meta->o_size );
//            orb_unlock(atomic_state);
			g_orb_buf[orb_id][inst].orb_id = orb_id;
			//name the advert as the pointer of the orb's internal data
			advert = ( void* )( &g_orb_buf[orb_id][inst] );			
			g_orb_buf[orb_id][inst].last_updated_time = time( NULL );

			g_orb_buf[orb_id][inst].cur_gen++;
			g_orb_buf[orb_id][inst].last_gen = 0;
			g_orb_buf[orb_id][inst].queue_size = 0;

			//update the published flag to true
			g_orb_buf[orb_id][inst].published = true;
			task_id = ( int )uorbc_platfrom_get_thread_id();
			for( i = 0; i < UORB_MAX_SUB; ++i ) {
				g_orb_buf[orb_id][inst].authority_list[i] = -1;
			}

			uorbc_platform_sem_post( &( g_orb_buf[orb_id][inst].sem ) );
		}
	}

	if( NULL == advert ) {
		uorbc_platform_log( "orb_advertise_multi:  advert fail\r\n" );
	}
	else {
		uorbc_platform_log( "orb_advertise_multi:  advert succ\r\n" );
	}

	return advert;
}
//----------------------------------------------------------------
orb_advert_t orb_advertise_multi_queue( const struct orb_metadata* meta, const void* data, int* instance,
                                        unsigned int queue_size )
{
	orb_advert_t advert = NULL;
	int inst = -1;
	int orb_id = -1;
	int task_id = -1;
	int i = 0;

	orb_id = mod_uorbc_utils_get_orbid( meta->o_name );

	uorbc_platform_log( "orb_advertise_multi_queue:  orb name:%s o_size:%d orb_id:%d queue_size:%d\r\n", meta->o_name, meta->o_size, orb_id, queue_size );

	if( orb_id < 0 || orb_id >= TOTAL_UORBC_NUM ) {
		return NULL;
	}
	else {
		if( NULL == instance ) {
			inst = 0;
		}
		else {
			inst = *instance;
		}

		uorbc_platform_log( "orb_advertise_multi_queue:  inst:%d\r\n", inst );

		if( /* ( mod_uorbc_utils_ismulti( orb_id ) ) && */ ( inst >= 0 ) && ( inst < ORB_MULTI_MAX_INSTANCES ) ) {
			uorbc_platform_sem_wait( &( g_orb_buf[orb_id][inst].sem ) );
			g_orb_buf[orb_id][inst].published = false;
			if( g_orb_buf[orb_id][inst].queue == NULL ) {
				g_orb_buf[orb_id][inst].queue = ring_buffer_init( queue_size, meta->o_size );
				uorbc_platform_assert( g_orb_buf[orb_id][inst].queue );
				g_orb_buf[orb_id][inst].qbuf_size = ring_buffer_get_size(g_orb_buf[orb_id][inst].queue);
				uorbc_platform_log("orb_advertise_multi_queue:  meta->o_name:%s qbuf_size:%d\r\n", meta->o_name, g_orb_buf[orb_id][inst].qbuf_size);
			}
			if( g_orb_buf[orb_id][inst].data == NULL ) {
				g_orb_buf[orb_id][inst].data = ( uint8_t* )uorbc_platform_malloc( sizeof( uint8_t ) * meta->o_size );
				uorbc_platform_assert( g_orb_buf[orb_id][inst].data );
			}

			/* 如果ringbuffer空间不足 */
			if( ring_buffer_left_space( g_orb_buf[orb_id][inst].queue ) < ( meta->o_size ) ) {
				ring_buffer_dequeue_arr( g_orb_buf[orb_id][inst].queue, g_orb_buf[orb_id][inst].data, meta->o_size );
			}
			ring_buffer_queue_arr( g_orb_buf[orb_id][inst].queue, data, meta->o_size );

			g_orb_buf[orb_id][inst].orb_id = orb_id;
			advert = ( void* )( &g_orb_buf[orb_id][inst] );
			g_orb_buf[orb_id][inst].last_updated_time = time( NULL );

			g_orb_buf[orb_id][inst].cur_gen++;
			g_orb_buf[orb_id][inst].last_gen = 0;
			g_orb_buf[orb_id][inst].queue_size = queue_size;
			uorbc_platform_log( "orb_advertise_multi_queue:  orb_advertise_multi_queue:cur_gen1:%u last_gen:%u queue_size:%u\r\n", g_orb_buf[orb_id][inst].cur_gen, g_orb_buf[orb_id][inst].last_gen, queue_size );

			g_orb_buf[orb_id][inst].published = true;
			task_id = ( int )uorbc_platfrom_get_thread_id();
			for( i = 0; i < UORB_MAX_SUB; ++i ) {
				g_orb_buf[orb_id][inst].authority_list[i] = -1;
			}
			uorbc_platform_sem_post( &( g_orb_buf[orb_id][inst].sem ) );
		}
	}

	if( NULL == advert ) {
		uorbc_platform_log( "orb_advertise_multi_queue:  advert fail\r\n" );
	}
	else {
		uorbc_platform_log( "orb_advertise_multi_queue:  advert succ\r\n" );
	}

	return advert;
}
//----------------------------------------------------------------
int orb_unadvertise( orb_advert_t* handle )
{
	*handle = NULL;
	return 0;
}
//----------------------------------------------------------------
int orb_publish_auto( const struct orb_metadata* meta, orb_advert_t* handle, const void* data, int* instance )
{
	int orb_id = -1;

	if( *handle == NULL ) {
		int orb_id = mod_uorbc_utils_get_orbid( meta->o_name );
		if( mod_uorbc_utils_ismulti( orb_id ) ) {
			*handle = orb_advertise_multi( meta, data, instance );
		}
		else {
			*handle = orb_advertise( meta, data );
			if( instance ) {
				*instance = 0;
			}
		}

		if( *handle != NULL ) {
			return 0;
		}
	}
	else {
		return orb_publish( meta, *handle, data );
	}

	return -1;
}
//----------------------------------------------------------------
int  orb_publish( const struct orb_metadata* meta, orb_advert_t handle, const void* data )
{
	int ret = -1;
	int orb_id = -1;
	int instance = 0;
	size_t buffer_space = 0;

	orb_id = mod_uorbc_utils_get_orbid( meta->o_name );

	uorbc_platform_log( "orb_publish:  orb name:%s orb_id:%d\r\n", meta->o_name, orb_id );  //meta->o_name = testx    orb_id = x

	if( orb_id < 0 || orb_id >= TOTAL_UORBC_NUM ) {
		uorbc_platform_log_err( "orb_publish:  invalid orb_id:%d\r\n", orb_id );
		return -1;
	}
	else {
        uorbc_platform_log( "orb_publish:  orb_id > 0 && orb_id < TOTAL_UORBC_NUM\r\n" );
		if( mod_uorbc_utils_ismulti( orb_id ) ) {
			for( int i = 0; i < ORB_MULTI_MAX_INSTANCES; ++i ) {
				if( handle == &g_orb_buf[orb_id][i] ) {
					instance = i;
					break;
				}
			}
		}
uorbc_platform_log("orb_publish:   now will run not a queued publish code \r\n");
		//Not a queued publish
		if( g_orb_buf[orb_id][instance].queue == NULL ) {
uorbc_platform_log("orb_publish:   (g_orb_buf[orb_id][instance].queue == NULL) \r\n");
			uorbc_platform_sem_wait( &( g_orb_buf[orb_id][instance].sem ) );
			g_orb_buf[orb_id][instance].published = false;
//            int atomic_state = orb_lock();
			uorbc_platform_memcpy( g_orb_buf[orb_id][instance].data, data, meta->o_size );
//            orb_unlock(atomic_state);
			g_orb_buf[orb_id][instance].orb_id = orb_id;
			g_orb_buf[orb_id][instance].last_updated_time = time( NULL );
			g_orb_buf[orb_id][instance].published = true;
			ret = 0;
			for( int i = 0; i < UORB_MAX_SUB; ++i ) {
				g_orb_buf[orb_id][instance].authority_list[i] = -1; /* 这是一条全新的消息 */
			}
			/* UORBCTODO: 更新poll_queue */
//			for( int i = 0; i < UORB_MAX_POLL; ++i ) {
//				if( poll_queue[i].fd == ( int )( ( orb_id << 4 ) | instance ) ) {
//					orb_poll_notify( poll_queue[i].fd, POLLIN );
//					break;
//				}
//			}
			uorbc_platform_sem_post( &( g_orb_buf[orb_id][instance].sem ) );
		}
		//We have a queued multi publish
		else {
uorbc_platform_log("orb_publish:   (g_orb_buf[orb_id][instance].queue != NULL) \r\n");
			uorbc_platform_sem_wait( &( g_orb_buf[orb_id][instance].sem ) );
//			uorbc_platform_log( "enqueue data:(%d)", ( meta->o_size ) );
//			uorbc_platform_log_bin( data, ( meta->o_size ) );

			/* 如果ringbuffer空间不足 */
			buffer_space = ring_buffer_left_space( g_orb_buf[orb_id][instance].queue );
//			uorbc_platform_log( "buffer_space:%d", buffer_space );
			if( buffer_space < ( meta->o_size ) ) {
				uorbc_platform_log( "orb_publish:  ringbuffer no space %d<%d\r\n", buffer_space, ( meta->o_size ) );
				ring_buffer_dequeue_arr( g_orb_buf[orb_id][instance].queue, g_orb_buf[orb_id][instance].data, meta->o_size );
			}
			ring_buffer_queue_arr( g_orb_buf[orb_id][instance].queue, data, meta->o_size );

			g_orb_buf[orb_id][instance].published = false;
			g_orb_buf[orb_id][instance].orb_id = orb_id;
			g_orb_buf[orb_id][instance].last_updated_time = time( NULL );
			g_orb_buf[orb_id][instance].cur_gen++;
			uorbc_platform_log( "orb_publish:  orb_publish:cur_gen2:%u\r\n", g_orb_buf[orb_id][instance].cur_gen );
			g_orb_buf[orb_id][instance].published = true;
			ret = 0;
			for( int i = 0; i < UORB_MAX_SUB; ++i ) {
				g_orb_buf[orb_id][instance].authority_list[i] = -1; /* 这是一条全新的消息 */
			}
			/* UORBCTODO: 更新poll_queue */
//				for( int i = 0; i < UORB_MAX_POLL; ++i ) {
//					if( poll_queue[i].fd == ( int )( ( orb_id << 4 ) | instance ) ) {
//						orb_poll_notify( poll_queue[i].fd, POLLIN );
//						break;
//					}
//				}
			uorbc_platform_sem_post( &( g_orb_buf[orb_id][instance].sem ) );
		}
	}

	return ret;
}
//----------------------------------------------------------------
int  orb_subscribe( const struct orb_metadata* meta )
{
	int ret = -1;
	int orb_id = -1;
	int instance = 0;
	int task_id = -1;
	uint32_t cur_gen = 0;
	uint queue_size = 0;

	if( meta == NULL || meta->o_name == NULL ) {
		uorbc_platform_log_err( "orb_subscribe:   meta == NULL || meta->o_name == NULL\r\n" );
		return -1;
	}

	orb_id = mod_uorbc_utils_get_orbid( meta->o_name );

	uorbc_platform_log( "orb_subscribe:   orb name:%s orb_id:%d\r\n", meta->o_name, orb_id );

	if( orb_id >= 0 && orb_id < TOTAL_UORBC_NUM ) {
		if( mod_uorbc_utils_ismulti( orb_id ) ) { /* 使用orb_subscribe_multi订阅multi topic */
            uorbc_platform_log( "orb_subscribe:  mod_uorbc_utils_ismulti( orb_id )    ret=%d\r\n", ret);
			return ret;
		}

		task_id = ( int )uorbc_platfrom_get_thread_id();

//        int atomic_state = orb_lock();

		for( int i = 0; i < UORB_MAX_SUB; ++i ) {
			if( g_orb_buf[orb_id][instance].registered_list[i] == task_id ) {
				ret = ( int )( ( orb_id << 4 ) | ( instance ) );
				break;
			}

			if( g_orb_buf[orb_id][instance].registered_list[i] == -1 ) {
				uorbc_platform_sem_wait( &( g_orb_buf[orb_id][instance].sem ) );
				g_orb_buf[orb_id][instance].registered_list[i] = task_id;

				// If there were any previous publications allow the subscriber to read them
				cur_gen = g_orb_buf[orb_id][instance].cur_gen;
				queue_size = g_orb_buf[orb_id][instance].queue_size;
				g_orb_buf[orb_id][instance].last_gen = cur_gen - uorbc_min( cur_gen, queue_size );

				uorbc_platform_log( "orb_subscribe:   orb_subscribe:cur_gen3:%u  queue_size:%u  last_gen:%u\r\n", g_orb_buf[orb_id][instance].cur_gen, g_orb_buf[orb_id][instance].queue_size, g_orb_buf[orb_id][instance].last_gen );

				uorbc_platform_sem_post( &( g_orb_buf[orb_id][instance].sem ) );
				ret = ( int )( ( orb_id << 4 ) | ( instance ) );
				break;
			}
		}

//        orb_unlock(atomic_state);
	}

	uorbc_platform_log( "orb_subscribe:   sub:%d\r\n", ret );

	return ret;
}
//----------------------------------------------------------------
int  orb_subscribe_multi( const struct orb_metadata* meta, unsigned instance )
{
	int ret = -1;
	int orb_id = -1;
	int task_id = -1;
	int i = 0;

	if( meta == NULL || meta->o_name == NULL ) {
		uorbc_platform_log_err( "orb_subscribe_multi:   meta == NULL || meta->o_name == NULL\r\n" );
		return -1;
	}

	orb_id = mod_uorbc_utils_get_orbid( meta->o_name );

	uorbc_platform_log( "orb_subscribe_multi:   orb name:%s orb_id:%d\r\n", meta->o_name, orb_id );

	if( orb_id >= 0 && orb_id < TOTAL_UORBC_NUM && mod_uorbc_utils_ismulti( orb_id ) ) {
		task_id = ( int )uorbc_platfrom_get_thread_id();
//        int atomic_state = orb_lock();
		for( i = 0; i < UORB_MAX_SUB; ++i ) {
			if( g_orb_buf[orb_id][instance].registered_list[i] == task_id ) {
				ret = ( int )( ( orb_id << 4 ) | ( instance ) );
				break;
			}

			if( g_orb_buf[orb_id][instance].registered_list[i] == -1 ) {
				uorbc_platform_sem_wait( &( g_orb_buf[orb_id][instance].sem ) );
				g_orb_buf[orb_id][instance].registered_list[i] = task_id;
				uorbc_platform_sem_post( &( g_orb_buf[orb_id][instance].sem ) );
				ret = ( int )( ( orb_id << 4 ) | ( instance ) );
				break;
			}
		}
//        orb_unlock(atomic_state);
	}

	uorbc_platform_log( "orb_subscribe_multi:   sub:%d\r\n", ret );

	return ret;
}
//----------------------------------------------------------------
int  orb_unsubscribe( int handle )
{
	int task_id = -1;
	int orb_id = -1;
	int instance = -1;
	int i = 0;

	if( handle < 0 ) {
		return -1;
	}

	orb_id = ( handle >> 4 );
	instance = handle - ( ( handle >> 4 ) << 4 );

	if( orb_id >= 0 && orb_id < TOTAL_UORBC_NUM ) {
		task_id = ( int ) uorbc_platfrom_get_thread_id();
//        int atomic_state = orb_lock();
		for( i = 0; i < UORB_MAX_SUB; ++i ) {
			if( g_orb_buf[orb_id][instance].registered_list[i] == task_id ) {
				uorbc_platform_sem_wait( &( g_orb_buf[orb_id][instance].sem ) );
				g_orb_buf[orb_id][instance].registered_list[i] = -1;
				uorbc_platform_sem_post( &( g_orb_buf[orb_id][instance].sem ) );
//                orb_unlock(atomic_state);
				return 0;
			}
		}
//        orb_unlock(atomic_state);
	}

	return -1;
}
//----------------------------------------------------------------
int orb_copy( const struct orb_metadata* meta, int handle, void* buffer )
{
	int ret = -1;
	int instance = -1;
	bool authorised = false;
	int task_id = -1;
	int i = 0;
	int registered_len = 0, authority_len = 0;
	uint32_t cur_gen = 0;
	uint32_t last_gen = 0;
	uint queue_size = 0;
	int sub_val = 0;
	ring_buffer_t* p_ring = NULL;
	size_t peek_pos = 0; 

	int orb_id = mod_uorbc_utils_get_orbid( meta->o_name );

	uorbc_platform_log( "orb_copy:   orb_id:%d o_name:%s o_size:%d handle:%d\r\n", orb_id, meta->o_name, meta->o_size, handle );

	if( orb_id != -1 && orb_id == ( handle >> 4 ) ) {
		instance = handle - ( ( handle >> 4 ) << 4 );               //test0   handle = 0   instance = 0

		uorbc_platform_sem_wait( & ( g_orb_buf[orb_id][instance].sem ) );

		if( instance > 0 && !mod_uorbc_utils_ismulti( orb_id ) ) {
			ret = -1;
			uorbc_platform_sem_post( & ( g_orb_buf[orb_id][instance].sem ) );
			uorbc_platform_log_err( "orb_copy:   instance:%d not multi\r\n", instance );
			return ret;
		}
		else if( instance >= 0 && instance < ORB_MULTI_MAX_INSTANCES &&
		         g_orb_buf[orb_id][instance].data != NULL ) {
			task_id = ( int ) uorbc_platfrom_get_thread_id();
			for( i = 0; i < UORB_MAX_SUB; ++i ) {
				/* queue模式可多次读取 */
				if( task_id == g_orb_buf[orb_id][instance].authority_list[i] ) {
					if( NULL != g_orb_buf[orb_id][instance].queue ) {
						authorised = true;
						break;
					}
				}

				if( g_orb_buf[orb_id][instance].authority_list[i] == -1 ) {
					g_orb_buf[orb_id][instance].authority_list[i] = task_id;
					authorised = true;
					break;
				}
			}

			if( authorised ) {
//				int atomic_state = orb_lock();
				if( NULL != g_orb_buf[orb_id][instance].queue ) { /* 如果是queue模式 */
					cur_gen = g_orb_buf[orb_id][instance].cur_gen;
					last_gen = g_orb_buf[orb_id][instance].last_gen;
					queue_size = g_orb_buf[orb_id][instance].queue_size;
					uorbc_platform_log( "orb_copy:   cur_gen4:%u last_gen:%u queue_size:%u\r\n", cur_gen, last_gen, queue_size );

					if (cur_gen > last_gen + queue_size) {
						// Reader is too far behind: some messages are lost
						g_orb_buf[orb_id][instance].last_gen = cur_gen - queue_size;
						last_gen = g_orb_buf[orb_id][instance].last_gen;
						uorbc_platform_log("orb_copy:   Reader is too far behind: some messages are lost, last_gen:%u\r\n", last_gen);
					}

					p_ring = g_orb_buf[orb_id][instance].queue;
					peek_pos = last_gen*(meta->o_size);
					peek_pos %= (g_orb_buf[orb_id][instance].qbuf_size);
					uorbc_platform_log("orb_copy:   meta->o_size:%d peek_pos:%d qbuf_size:%d\r\n", meta->o_size, peek_pos, g_orb_buf[orb_id][instance].qbuf_size);
					ring_buffer_just_peek( p_ring, buffer, peek_pos, meta->o_size );
					
					uorbc_platform_log( "got buffer:(%d)", meta->o_size );
                    
//					uorbc_platform_log_bin( buffer, meta->o_size );

//					uorbc_platform_log( "compare last_gen:%d cur_gen:%d", g_orb_buf[orb_id][instance].last_gen, g_orb_buf[orb_id][instance].cur_gen );
					if( g_orb_buf[orb_id][instance].last_gen < g_orb_buf[orb_id][instance].cur_gen ) {
						g_orb_buf[orb_id][instance].last_gen++;
//						uorbc_platform_log( "last_gen:%d", g_orb_buf[orb_id][instance].last_gen );
					}
				}
				else {
					uorbc_platform_memcpy( buffer, g_orb_buf[orb_id][instance].data, meta->o_size );
				}
//				orb_unlock ( atomic_state );
				ret = 0;
			}
			else {
				uorbc_platform_log( "orb_copy:   no authorised\r\n" );
			}

			for( int i = 0; i < UORB_MAX_SUB; ++i ) {
				if( g_orb_buf[orb_id][instance].registered_list[i] != -1 ) {
					++registered_len;
				}
				if( g_orb_buf[orb_id][instance].authority_list[i] != -1 ) {
					++authority_len;
				}
			}

//			uorbc_platform_log("authority_len:%d registered_len:%d", authority_len, registered_len);
//			uorbc_platform_log("last_gen:%d cur_gen:%d", g_orb_buf[orb_id][instance].last_gen, g_orb_buf[orb_id][instance].cur_gen);
			
			if(( authority_len >= registered_len )&&
				( g_orb_buf[orb_id][instance].last_gen == g_orb_buf[orb_id][instance].cur_gen )) {
				g_orb_buf[orb_id][instance].published = false;
				uorbc_platform_log("orb_copy:   published:%d\r\n", g_orb_buf[orb_id][instance].published);
			}
		}

		uorbc_platform_sem_post( & ( g_orb_buf[orb_id][instance].sem ) );
	}

	return ret;
}
//----------------------------------------------------------------
int orb_check( int handle, bool* updated )
{
	int ret = 0;

	if( handle < 0 ) {
        uorbc_platform_log( "orb_check:   error handle < 0\r\n");
		*updated = false;
		return -1;
	}

	int orb_id = ( handle >> 4 );
	int instance = handle - ( ( handle >> 4 ) << 4 );

	if( orb_id < 0 || orb_id >= TOTAL_UORBC_NUM ||
	    instance < 0 || instance >= ORB_MULTI_MAX_INSTANCES ) {
        uorbc_platform_log( "orb_check:   error  infomation  orb_id=%d, instance=%d\r\n", orb_id, instance);
		*updated = false;
		return -1;
	}

	uorbc_platform_sem_wait( & ( g_orb_buf[orb_id][instance].sem ) );

	bool authorised = false;
	int task_id = ( int ) uorbc_platfrom_get_thread_id();

	uorbc_platform_log( "orb_check:   cur_gen5:%u last_gen:%u\r\n", g_orb_buf[orb_id][instance].cur_gen, g_orb_buf[orb_id][instance].last_gen );

	if( NULL != g_orb_buf[orb_id][instance].queue ) {
		if( ( g_orb_buf[orb_id][instance].cur_gen != g_orb_buf[orb_id][instance].last_gen ) ) {
			/* 说明queue模式还有消息未取完 */
			uorbc_platform_log( "orb_check:   still more msg\r\n" );
			ret = 1;
		}
	}
	else {
		for( int i = 0; i < UORB_MAX_SUB; ++i ) {
			/* 此线程已获取到此消息,无需重复获取 */
			uorbc_platform_log( "orb_check:   task_id:%d\r\n", task_id );
			if( g_orb_buf[orb_id][instance].authority_list[i] == task_id ) {
				authorised = true;
				break;
			}
		}
	}

	if( g_orb_buf[orb_id][instance].published && !authorised ) {
		*updated = true;
		uorbc_platform_sem_post( & ( g_orb_buf[orb_id][instance].sem ) );
		return ret;
	}
	else {
		*updated = false;
	}

	uorbc_platform_sem_post( & ( g_orb_buf[orb_id][instance].sem ) );

	return ret;
}
//----------------------------------------------------------------
/* 获取multi   orb个数   */
int  orb_group_count( const struct orb_metadata* meta )
{
	unsigned instance = 0;

	for( int i = 0; i < ORB_MULTI_MAX_INSTANCES; ++i ) {
		if( orb_exists( meta, i ) == 0 ) {
			++instance;
		}
	}

	return instance;
}
//----------------------------------------------------------------
int orb_set_interval( int handle, unsigned interval )
{
	int orb_id = ( handle >> 4 );
	int instance = handle - ( ( handle >> 4 ) << 4 );

	if( orb_id >= 0 && orb_id < TOTAL_UORBC_NUM &&
	    instance >= 0 && instance < ORB_MULTI_MAX_INSTANCES ) {
		g_orb_buf[orb_id][instance].interval = interval;
		return 0;
	}
	return -1;
}
//----------------------------------------------------------------
int orb_get_interval( int handle, unsigned* interval )
{
	int orb_id = ( handle >> 4 );
	int instance = handle - ( ( handle >> 4 ) << 4 );

	if( orb_id >= 0 && orb_id < TOTAL_UORBC_NUM &&
	    instance >= 0 && instance < ORB_MULTI_MAX_INSTANCES ) {
		*interval = g_orb_buf[orb_id][instance].interval;
		return 0;   
	}
	return -1;
}
//----------------------------------------------------------------
void orb_init( void )
{
	uorbc_platform_log( "orb_init\r\n" );

	/* UORBCTODO: 更新poll_queue */

	for( int i = 0; i < TOTAL_UORBC_NUM; ++i ) {
		/* 如果是multi类型的orb */
		if( mod_uorbc_utils_ismulti( i ) ) {
			orb_init_multi( i );
		}
		else {
			orb_init_nomulti( i );
		}
	}
}
//----------------------------------------------------------------

