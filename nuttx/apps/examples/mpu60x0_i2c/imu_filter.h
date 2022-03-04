#ifndef _IMU_FILTER_H
#define _IMU_FILTER_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <math.h>
#include "imu_types.h"

#define N 20      //滤波缓存数组大小
//#define M_PI_F 3.1416f

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )


typedef struct
{
	float lpf_1;

	float out;
}_lf_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
float FindPos(float*a,int low,int high);
void QuiteSort(float* a,int low,int high);
void  SortAver_Filter(float value,float *filter,uint8_t n);
void  SortAver_Filter1(float value,float *filter,uint8_t n);
u_char  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n);
void Aver_FilterXYZ6(INT16_XYZ *acc,INT16_XYZ *gry,FLOAT_XYZ *Acc_filt,FLOAT_XYZ *Gry_filt,uint8_t n);
void Aver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n);
void Aver_Filter(float data,float *filt_data,uint8_t n);
void Aver_Filter1(float data,float *filt_data,uint8_t n);


#endif