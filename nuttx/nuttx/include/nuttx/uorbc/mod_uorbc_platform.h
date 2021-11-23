#ifndef __MOD_UORBC_PLATFROM_H__
#define __MOD_UORBC_PLATFROM_H__

#include <pthread.h>
#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <inttypes.h>

/* 平台相关实现 */
//-------------------------------------------------------------------
#define	uorbc_platform_malloc				malloc
#define uorbc_platform_sem_init				sem_init
#define	uorbc_platform_perror				_err
#define	uorbc_platform_sem_wait				sem_wait
#define	uorbc_platform_sem_t				sem_t
#define	uorbc_platfrom_get_thread_id		pthread_self
#define	uorbc_platform_sem_post				sem_post
#define	uorbc_platform_memcpy				memcpy
#define	uorbc_platform_free					free
#define uorbc_platform_assert(exp)			do{							\
					if(!(exp)) {										\
						printf("\n%s:%d:%s: Assertion '%s' failed.\n",	\
							   __FILE__, __LINE__, __func__, #exp);		\
						abort();										\
					}													\
				}while(0)
#define	uorbc_platform_pthread_t			pthread_t
#define uorbc_platform_delay_ms				msleep
#define	uorbc_platform_log					printf
#define	uorbc_platform_log_bin				//TTI_BIN
#define	uorbc_platform_log_err				_err
//-------------------------------------------------------------------
int mod_uorbc_platform_thread_create ( void* id, void* ( *start_rtn ) ( void* ), void* arg );


#endif

