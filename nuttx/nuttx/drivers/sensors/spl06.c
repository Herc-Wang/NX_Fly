/****************************************************************************
 * drivers/sensors/spl06.c
 *
 * 
 *
 ****************************************************************************/

/* Character driver for the stm32 spl06 Barometer Sensor */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <limits.h>
#include <inttypes.h>
#include <stdlib.h>
#include <fixedmath.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sensors/spl06.h>
#include <nuttx/random.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>


#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_SPL06)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define SPL06_I2C_ADDR					        (0x76)
#define SPL06_DEFAULT_CHIP_ID			      (0x10)


#define SPL06_CALIB_COEFFICIENT_LENGTH	(18)
#define SPL06_DATA_FRAME_SIZE			      (6)

#define SPL06_CONTINUOUS_MODE			      (0x07)

#define TEMPERATURE_INTERNAL_SENSOR	  	(0)
#define TEMPERATURE_EXTERNAL_SENSOR	  	(1)

//测量次数 times / S
#define SPL06_MWASURE_1					(0x00)
#define SPL06_MWASURE_2					(0x01)
#define SPL06_MWASURE_4					(0x02)
#define SPL06_MWASURE_8					(0x03)
#define SPL06_MWASURE_16				(0x04)
#define SPL06_MWASURE_32				(0x05)
#define SPL06_MWASURE_64				(0x06)
#define SPL06_MWASURE_128				(0x07)

//过采样率
#define SPL06_OVERSAMP_1				(0x00)
#define SPL06_OVERSAMP_2				(0x01)
#define SPL06_OVERSAMP_4				(0x02)
#define SPL06_OVERSAMP_8				(0x03)
#define SPL06_OVERSAMP_16				(0x04)
#define SPL06_OVERSAMP_32				(0x05)
#define SPL06_OVERSAMP_64				(0x06)
#define SPL06_OVERSAMP_128			(0x07)


//set the local definitions
#define P_MEASURE_RATE 			SPL06_MWASURE_16 	//每秒测量次数
#define P_OVERSAMP_RATE 		SPL06_OVERSAMP_64	//过采样率
#define SPL06_PRESSURE_CFG		(P_MEASURE_RATE<<4 | P_OVERSAMP_RATE)

#define T_MEASURE_RATE 			SPL06_MWASURE_16 	//每秒测量次数
#define T_OVERSAMP_RATE 		SPL06_OVERSAMP_8	//过采样率
#define SPL06_TEMPERATURE_CFG	(TEMPERATURE_EXTERNAL_SENSOR<<7 | T_MEASURE_RATE<<4 | T_OVERSAMP_RATE)

#define SPL06_MODE				(SPL06_CONTINUOUS_MODE)


/* Sets bit @n */

#define BIT(n) (1 << (n))

/* Creates a mask of @m bits, i.e. MASK(2) -> 00000011 */

#define MASK(m) (BIT((m) + 1) - 1)

/* Masks and shifts @v into bit field @m */

#define TO_BITFIELD(m,v) (((v) & MASK(m ##__WIDTH)) << (m ##__SHIFT))

/* Un-masks and un-shifts bit field @m from @v */

#define FROM_BITFIELD(m,v) (((v) >> (m ##__SHIFT)) & MASK(m ##__WIDTH))



/*这部分宏定义后面需要放到menuconfig中设置*/
#define CONFIG_SPL06_I2C_FREQ 400000


/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

enum spl_regaddr_e
{
  SPL06_PRESSURE_MSB_REG	    = 0x00,   /* Pressure MSB Register */
  SPL06_PRESSURE_LSB_REG		  = 0x01,   /* Pressure LSB Register */
  SPL06_PRESSURE_XLSB_REG		  = 0x02,   /* Pressure XLSB Register */
  SPL06_TEMPERATURE_MSB_REG		= 0x03,   /* Temperature MSB Reg */
  SPL06_TEMPERATURE_LSB_REG		= 0x04,   /* Temperature LSB Reg */
  SPL06_TEMPERATURE_XLSB_REG	= 0x05,   /* Temperature XLSB Reg */
  SPL06_PRESSURE_CFG_REG			= 0x06,	  /* Pressure configuration Reg */
  SPL06_TEMPERATURE_CFG_REG		= 0x07,	  /* Temperature configuration Reg */
  SPL06_MODE_CFG_REG				  = 0x08,   /* Mode and Status Configuration */
  SPL06_INT_FIFO_CFG_REG			= 0x09,	  /* Interrupt and FIFO Configuration */
  SPL06_INT_STATUS_REG			  = 0x0A,	  /* Interrupt Status Reg */
  SPL06_FIFO_STATUS_REG			  = 0x0B,	  /* FIFO Status Reg */
  SPL06_RST_REG					      = 0x0C,   /* Softreset Register */
  SPL06_CHIP_ID					      = 0x0D,   /* Chip ID Register */
  SPL06_COEFFICIENT_CALIB_REG	= 0x10,   /* Coeffcient calibraion Register */
 
};

enum spl06Sensor_e
{
	PRESURE_SENSOR, 
	TEMPERATURE_SENSOR
};

typedef struct 
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06CalibCoefficient_t;

begin_packed_struct struct spl06_data_s
{
  float pressure;
  float temperature;
  float ASL;          //altitude above sea level(ASL) in meters   converted by pressure.
}end_packed_struct;

struct spl_dev_s
{
  mutex_t lock;               /* mutex for this structure */
  struct spl_config_s config; /* board-specific information */

  struct spl06_data_s buf;    /* temporary buffer (for read(), etc.) */
  size_t bufpos;              /* cursor into @buf, in bytes (!) */

};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int spl06_open(FAR struct file *filep);
static int spl06_close(FAR struct file *filep);
static ssize_t spl06_read(FAR struct file *filep, FAR char *buf, size_t len);
static ssize_t spl06_write(FAR struct file *filep, FAR const char *buf,
                         size_t len);
static off_t spl06_seek(FAR struct file *filep, off_t offset, int whence);
static int spl06_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
 
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_spl06_fops =
{
  spl06_open,                  /* open */
  spl06_close,                 /* close */
  spl06_read,                  /* read */
  spl06_write,                 /* write */
  spl06_seek,                  /* seek */
  spl06_ioctl,                 /* ioctl */
  NULL                         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                       /* unlink */
#endif
};

const uint32_t scaleFactor[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};

spl06CalibCoefficient_t  spl06Calib;

static uint8_t devAddr;

static bool isInit = false;


int32_t kp = 0;
int32_t kt = 0;
int32_t SPL06RawPressure = 0;
int32_t SPL06RawTemperature = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* NOTE :
 *
 * In all of the following code, functions named with a double leading
 * underscore '__' must be invoked ONLY if the spl_dev_s lock is
 * already held. Failure to do this might cause the transaction to get
 * interrupted, which will likely confuse the data you get back.
 *
 */

/* __spl_read_reg(), but for i2c-connected devices. */

static int __spl_read_reg_i2c(FAR struct spl_dev_s *dev,
                              enum spl_regaddr_e reg_addr,
                              FAR uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_SPL06_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_SPL06_I2C_FREQ;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(read) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

static int __spl_write_reg_i2c(FAR struct spl_dev_s *dev,
                               enum spl_regaddr_e reg_addr,
                               FAR const uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_SPL06_I2C_FREQ;
  msg[0].addr      = dev->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;
  msg[1].frequency = CONFIG_SPL06_I2C_FREQ;
  msg[1].addr      = dev->config.addr;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (FAR uint8_t *)buf;
  msg[1].length    = len;
  ret = I2C_TRANSFER(dev->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER(write) failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/* __spl_read_reg()
 *
 * Reads a block of @len byte-wide registers, starting at @reg_addr,
 * from the device connected to @dev. Bytes are returned in @buf,
 * which must have a capacity of at least @len bytes.
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes read, or a negative errno.
 */

static inline int __spl_read_reg(FAR struct spl_dev_s *dev,
                                 enum spl_regaddr_e reg_addr,
                                 FAR uint8_t *buf, uint8_t len)
{
  /* If we're wired to I2C, use that function. */

  if (dev->config.i2c != NULL)
    {
      return __spl_read_reg_i2c(dev, reg_addr, buf, len);
    }

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* __spl_write_reg()
 *
 * Writes a block of @len byte-wide registers, starting at @reg_addr,
 * using the values in @buf to the device connected to @dev. Register
 * values are taken in numerical order from @buf, i.e.:
 *
 *   buf[0] -> register[@reg_addr]
 *   buf[1] -> register[@reg_addr + 1]
 *   ...
 *
 * Note: The caller must hold @dev->lock before calling this function.
 *
 * Returns number of bytes written, or a negative errno.
 */

static inline int __spl_write_reg(FAR struct spl_dev_s *dev,
                                  enum spl_regaddr_e reg_addr,
                                  FAR const uint8_t *buf, uint8_t len)
{
  if (dev->config.i2c != NULL)
    {
      return __spl_write_reg_i2c(dev, reg_addr, buf, len);
    }

  /* If we get this far, it's because we can't "find" our device. */

  return -ENODEV;
}

/* Locks and unlocks the @dev data structure (mutex).
 *
 * Use these functions any time you call one of the lock-dependent
 * helper functions defined above.
 */

static void inline spl_lock(FAR struct spl_dev_s *dev)
{
  nxmutex_lock(&dev->lock);
}

static void inline spl_unlock(FAR struct spl_dev_s *dev)
{
  nxmutex_unlock(&dev->lock);
}

/****************************************************************************
 * Name: spl0601_get_calib_param
 *
 * Description:
 *    
 *    通过I2C读取校准参数，并保存到spl06Calib结构体中。
 *
 ****************************************************************************/

void spl0601_get_calib_param(FAR struct spl_dev_s *dev)
{
	uint8_t buffer[SPL06_CALIB_COEFFICIENT_LENGTH] = {0};
	spl_lock(dev);
	__spl_read_reg(dev, SPL06_COEFFICIENT_CALIB_REG, (uint8_t *) buffer, sizeof(buffer));
  spl_unlock(dev);
  for(int i=0; i<18; i++)
    printf("buffer[%d] = %d   ", i,buffer[i]);

	spl06Calib.c0 = (int16_t)buffer[0]<<4 | buffer[1]>>4;
	spl06Calib.c0 = (spl06Calib.c0 & 0x0800) ? (spl06Calib.c0 | 0xF000) : spl06Calib.c0;
	
	spl06Calib.c1 = (int16_t)(buffer[1] & 0x0F)<<8 | buffer[2];
	spl06Calib.c1 = (spl06Calib.c1 & 0x0800) ? (spl06Calib.c1 | 0xF000) : spl06Calib.c1;
	
	spl06Calib.c00 = (int32_t)buffer[3]<<12 | (int32_t)buffer[4]<<4 | (int32_t)buffer[5]>>4;
	spl06Calib.c00 = (spl06Calib.c00 & 0x080000) ? (spl06Calib.c00 | 0xFFF00000) : spl06Calib.c00;
	
	spl06Calib.c10 = (int32_t)(buffer[5] & 0x0F)<<16 | (int32_t)buffer[6]<<8 | (int32_t)buffer[7];
	spl06Calib.c10 = (spl06Calib.c10 & 0x080000) ? (spl06Calib.c10 | 0xFFF00000) : spl06Calib.c10;
	
	spl06Calib.c01 = (int16_t)buffer[8]<<8 | buffer[9];
	spl06Calib.c11 = (int16_t)buffer[10]<<8 | buffer[11];
	spl06Calib.c20 = (int16_t)buffer[12]<<8 | buffer[13];
	spl06Calib.c21 = (int16_t)buffer[14]<<8 | buffer[15];
	spl06Calib.c30 = (int16_t)buffer[16]<<8 | buffer[17];

  _alert("spl0601_get_calib_param \n c0=%ld\n c1=%ld\n c00=%ld\n c10=%ld\n c01=%ld\n c11=%ld\n c20=%ld\n c21=%ld\n c30=%ld\n", \
            spl06Calib.c0, spl06Calib.c1, spl06Calib.c00, spl06Calib.c10, spl06Calib.c01, spl06Calib.c11, spl06Calib.c20, spl06Calib.c21, spl06Calib.c30);
}

/****************************************************************************
 * Name: spl0601_rateset
 *
 * Description:
 *    
 *    根据对应传感器的宏，设置其采样次数、过采样率等参数。
 *
 ****************************************************************************/

void spl0601_rateset(FAR struct spl_dev_s *dev, 
                     enum spl06Sensor_e sensor, 
                     uint8_t measureRate, 
                     uint8_t oversamplRate)
{
	uint8_t reg;
	if (sensor == PRESURE_SENSOR)
	{
		kp = scaleFactor[oversamplRate];
    reg = (measureRate<<4 | oversamplRate);
		__spl_write_reg(dev, SPL06_PRESSURE_CFG_REG, &reg, 1);

    if (oversamplRate > SPL06_OVERSAMP_8)
		{
      __spl_read_reg(dev, SPL06_INT_FIFO_CFG_REG, &reg, 1);
      reg = reg | 0x04;
		  __spl_write_reg(dev, SPL06_INT_FIFO_CFG_REG, &reg, 1);
		}
	}
	else if (sensor == TEMPERATURE_SENSOR)
	{
		kt = scaleFactor[oversamplRate];
    reg = measureRate<<4 | oversamplRate | 0x80;
		__spl_write_reg(dev, SPL06_TEMPERATURE_CFG_REG, &reg, 1);//Using mems temperature

		if (oversamplRate > SPL06_OVERSAMP_8)
		{
      __spl_read_reg(dev, SPL06_INT_FIFO_CFG_REG, &reg, 1);
      reg = reg | 0x08;
		  __spl_write_reg(dev, SPL06_INT_FIFO_CFG_REG, &reg, 1);
		}
	}
}

/****************************************************************************
 * Name: spl_init
 *
 * Description:
 *    
 *    init the spl, sets it to a default configuration
 *
 ****************************************************************************/

static int spl_init(FAR struct spl_dev_s *dev)
{
  int ret;
  uint8_t SPL06ID = 0;
  uint8_t reg = 0;

  if (dev->config.i2c == NULL)
  {
    return -EINVAL;
  }

  if (isInit)
        return true;
        
  //nxsig_usleep(50000000); //50ms
  nxsig_usleep(500000); //50ms

  spl_lock(dev);
  __spl_read_reg(dev, SPL06_CHIP_ID, &SPL06ID, 1);  /* 读取SPL06 ID*/
  spl_unlock(dev);
  
  if(SPL06ID == SPL06_DEFAULT_CHIP_ID)
		printf("SPL06 ID IS: 0x%X\n",SPL06ID);
  else
  {
    snerr("Could not find SPL06 : ID is not match!  now SPL06ID = %d,  expected ID is SPL06_CHIP_ID = %d\n", SPL06ID, SPL06_CHIP_ID);
    return SPL06ID;
  }

  //读取校准数据
	spl0601_get_calib_param(dev);
  spl_lock(dev);
	spl0601_rateset(dev, PRESURE_SENSOR, SPL06_MWASURE_16, SPL06_OVERSAMP_64);
	spl0601_rateset(dev, TEMPERATURE_SENSOR, SPL06_MWASURE_16, SPL06_OVERSAMP_64);

  reg = SPL06_MODE;
  __spl_write_reg(dev, SPL06_MODE_CFG_REG, &reg, 1);
  spl_unlock(dev);
  
  isInit = true;
 
  //nxsig_usleep(2000);
  return 0;
}

/****************************************************************************
 * Name: SPL06GetPressure
 *
 * Description:
 *    
 *    读取设备，获得压力和温度MSB LSB XLSB,并组合成为校准的压力温度raw值
 *
 ****************************************************************************/

void SPL06GetPressure(FAR struct spl_dev_s *dev)
{
    uint8_t data[SPL06_DATA_FRAME_SIZE];

    __spl_read_reg(dev, SPL06_PRESSURE_MSB_REG, data, SPL06_DATA_FRAME_SIZE);
	  SPL06RawPressure = (int32_t)data[0]<<16 | (int32_t)data[1]<<8 | (int32_t)data[2];
    SPL06RawPressure = (SPL06RawPressure & 0x800000) ? (0xFF000000 | SPL06RawPressure) : SPL06RawPressure;
	
    SPL06RawTemperature = (int32_t)data[3]<<16 | (int32_t)data[4]<<8 | (int32_t)data[5];
  	SPL06RawTemperature = (SPL06RawTemperature & 0x800000) ? (0xFF000000 | SPL06RawTemperature) : SPL06RawTemperature;
}
/****************************************************************************
 * Name: spl0601_get_temperature
 *
 * Description:
 *    
 *    对传入的为校准温度值进行校准后返回。
 *
 ****************************************************************************/

float spl0601_get_temperature(int32_t rawTemperature)
{
    float fTCompensate;
    float fTsc;
    
    fTsc = rawTemperature / (float)kt;
    fTCompensate =  spl06Calib.c0 * 0.5 + spl06Calib.c1 * fTsc;

    return fTCompensate;
}

/****************************************************************************
 * Name: spl0601_get_pressure
 *
 * Description:
 *    
 *    计算有偿测量压力值
 *
 ****************************************************************************/

float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = rawTemperature / (float)kt;
    fPsc = rawPressure / (float)kp;
    qua2 = spl06Calib.c10 + fPsc * (spl06Calib.c20 + fPsc* spl06Calib.c30);
    qua3 = fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);
	//qua3 = 0.9f *fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);

    fPCompensate = spl06Calib.c00 + fPsc * qua2 + fTsc * spl06Calib.c01 + qua3;
	//fPCompensate = spl06Calib.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06Calib.c01 + qua3;
    return fPCompensate;
}

/****************************************************************************
 * Name: SPL06GetData
 *
 * Description:
 *    
 *    获得有偿测量的压力值和温度值。
 *
 ****************************************************************************/

void SPL06GetData(FAR struct spl_dev_s *dev, float* pressure, float* temperature, float* asl)
{
    static float t;
    static float p;
	
	SPL06GetPressure(dev);     //获取SPL06RawTemperature 、 SPL06RawPressure

	t = spl0601_get_temperature(SPL06RawTemperature);		
	p = spl0601_get_pressure(SPL06RawPressure, SPL06RawTemperature);		

//	pressureFilter(&p,pressure);
	*temperature  = (float)t;/*单位度*/
	*pressure     = (float)p ;	/*单位hPa*/	
	
	// *asl          = SPL06PressureToAltitude(*pressure);	/*转换成海拔*/	
  *asl          = (float)(44330.f * (powf((1015.7f / p), 0.190295f) - 1.0f));
}

/****************************************************************************
 * Name: SPL06PressureToAltitude
 *
 * Description:
 *    
 *    把压力转换成海拔高度  单位m
 *
 ****************************************************************************/

float* SPL06PressureToAltitude(float pressure/*, float* groundPressure, float* groundTemp*/)
{	
  static float * ASL_tmp;

    if(pressure)
    {
        *ASL_tmp = 44330.f * (powf((1015.7f / pressure), 0.190295f) - 1.0f);
		    return ASL_tmp;
    }
    else
    {
        return NULL;
    }
}

/* __spl_read__press_temp_asl()
 *
 * Reads the whole spl data file from @dev in one uninterrupted pass,
 * placing the sampled values into @buf. This function is the only way
 * to guarantee that the measured values are sampled as closely-spaced
 * in time as the hardware permits, which is almost always what you
 * want.
 */

static inline void __spl_read__press_temp_ASL(FAR struct spl_dev_s *dev,
                                 FAR struct spl06_data_s *buf)
{
  //SPL06GetData(dev, &(buf->pressure), &(buf->temperature), $(buf->ASL));

  SPL06GetPressure(dev);     //获取SPL06RawTemperature 、 SPL06RawPressure

  buf->pressure    =  spl0601_get_pressure(SPL06RawPressure, SPL06RawTemperature);
  buf->temperature =  spl0601_get_temperature(SPL06RawTemperature);                               //温度部分测试有小问题，后续不会使用这里测出的温度值，如果有更新会再次上传
  buf->ASL         =  (float)(44330.f * (powf((1015.7f / buf->pressure), 0.190295f) - 1.0f));     //海拔高度计算公式估计有问题

}

/****************************************************************************
 * Name: spl06_open
 *
 * Note: we don't deal with multiple users trying to access this interface at
 * the same time. Until further notice, don't do that.
 *
 * And no, it's not as simple as just prohibiting concurrent opens or
 * reads with a mutex: there are legit reasons for truy concurrent
 * access, but they must be treated carefully in this interface lest a
 * partial reader end up with a mixture of old and new samples. This
 * will make some users unhappy.
 *
 ****************************************************************************/

static int spl06_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct spl_dev_s *dev = inode->i_private;

  /* Reset the register cache */
  spl_lock(dev);
  dev->bufpos = 0;
  spl_unlock(dev);

  return 0;
}

/****************************************************************************
 * Name: spl06_close
 ****************************************************************************/

static int spl06_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct spl_dev_s *dev = inode->i_private;

  /* Reset (clear) the register cache. */
  spl_lock(dev);
  dev->bufpos = 0;
  spl_unlock(dev);

  return 0;
}

/****************************************************************************
 * Name: spl06_read
 *
 *
 ****************************************************************************/

static ssize_t spl06_read(FAR struct file *filep, FAR char *buf, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct spl_dev_s *dev = inode->i_private;
  size_t send_len = 0;

  spl_lock(dev);
  /* Populate the register cache if it seems empty. */

  if (!dev->bufpos)
    {
      __spl_read__press_temp_ASL(dev, &dev->buf);
    }

  /* Send the lesser of: available bytes, or amount requested. */

  send_len = sizeof(dev->buf) - dev->bufpos;
  if (send_len > len)
    {
      send_len = len;
    }

  if (send_len)
    {
      memcpy(buf, ((uint8_t *)&dev->buf) + dev->bufpos, send_len);
    }

  /* Move the cursor, to mark them as sent. */

  dev->bufpos += send_len;

  /* If we've sent the last byte, reset the buffer. */

  if (dev->bufpos >= sizeof(dev->buf))
    {
      dev->bufpos = 0;
    }

  spl_unlock(dev);

  return send_len;
}

/****************************************************************************
 * Name: spl06_write
 ****************************************************************************/

static ssize_t spl06_write(FAR struct file *filep, FAR const char *buf,
                         size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct spl_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);
  snerr("ERROR: %p %p %d\n", inode, dev, len);

  return len;
}

/****************************************************************************
 * Name: spl06_seek
 ****************************************************************************/

static off_t spl06_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct spl_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);

  snerr("ERROR: %p %p\n", inode, dev);

  return 0;
}

/****************************************************************************
 * Name: spl06_ioctl
 ****************************************************************************/

static int spl06_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct spl_dev_s *dev = inode->i_private;

  UNUSED(inode);
  UNUSED(dev);

  snerr("ERROR: %p %p\n", inode, dev);

  /* ENOTTY is the standard return if an IOCTL command is not supported. */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spl06_register
 *
 * Description:
 *   Registers the spl06 interface as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the interface to register. E.g., "/dev/spl0"
 *   config   - Configuration information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int spl06_register(FAR const char *path, FAR struct spl_config_s *config)
{
  FAR struct spl_dev_s *priv;
  int ret;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  priv = (FAR struct spl_dev_s *)kmm_malloc(sizeof(struct spl_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate spl06 device instance\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(*priv));
  nxmutex_init(&priv->lock);

  /* Keep a copy of the config structure, in case the caller discards
   * theirs.
   */

  priv->config = *config;

  /* Reset the chip, to give it an initial configuration. */
_alert("spl_init: arg   priv->config->i2c:%x   ,priv->config->addr=%x\n\n",priv->config.i2c,priv->config.addr);
  ret = spl_init(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure spl06: %d\n", ret);
      nxmutex_destroy(&priv->lock);

      kmm_free(priv);
      return ret;
    }

  /* Register the device node. */

  ret = register_driver(path, &g_spl06_fops, 0666, priv);
  if (ret < 0)
    {
_alert("register_driver failed\n");
      snerr("ERROR: Failed to register spl06 interface: %d\n", ret);

      nxmutex_destroy(&priv->lock);
_alert("register_driver failed\n");
      kmm_free(priv);
      return ret;
    }

  return OK;
}


#endif /* CONFIG_I2C && CONFIG_SENSORS_SPL06 */
