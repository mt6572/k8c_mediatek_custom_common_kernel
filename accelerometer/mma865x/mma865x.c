/* drivers/i2c/chips/mma865x.c - MMA865X motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>


#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "mma865x.h"
#include <linux/hwmsen_helper.h>

//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE



/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_MMA8652 0x4a
#define I2C_DRIVERID_MMA8653 0x5a

/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_MMA865X_LOWPASS   /*apply low pass filter on output*/       
/*----------------------------------------------------------------------------*/
#define MMA865X_AXIS_X          0
#define MMA865X_AXIS_Y          1
#define MMA865X_AXIS_Z          2
#define MMA865X_AXES_NUM        3
#define MMA865X_DATA_LEN        6
#define MMA865X_DEV_NAME        "MMA865X"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mma865x_i2c_id[] = {{MMA865X_DEV_NAME,0},{}};
/*the adapter id will be available in customization*/
//static unsigned short mma865x_force[] = {0x00, MMA865X_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const mma865x_forces[] = { mma865x_force, NULL };
//static struct i2c_client_address_data mma865x_addr_data = { .forces = mma865x_forces,};
static struct i2c_board_info __initdata i2c_MMA865X={ I2C_BOARD_INFO("MMA865X", 0X1D)};


/*----------------------------------------------------------------------------*/
static int mma865x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int mma865x_i2c_remove(struct i2c_client *client);
static int mma865x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

/*----------------------------------------------------------------------------*/
static int MMA865X_SetPowerMode(struct i2c_client *client, bool enable);


/*------------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
    ADX_TRC_REGXYZ	= 0X20,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][MMA865X_AXES_NUM];
    int sum[MMA865X_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct mma865x_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[MMA865X_AXES_NUM+1];

    /*data*/
    s8                      offset[MMA865X_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[MMA865X_AXES_NUM+1];

#if 1//defined(CONFIG_MMA865X_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver mma865x_i2c_driver = {
    .driver = {
//        .owner          = THIS_MODULE,
        .name           = MMA865X_DEV_NAME,
    },
	.probe      		= mma865x_i2c_probe,
	.remove    			= mma865x_i2c_remove,
	.detect				= mma865x_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = mma865x_suspend,
    .resume             = mma865x_resume,
#endif
	.id_table = mma865x_i2c_id,
//	.address_data = &mma865x_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *mma865x_i2c_client = NULL;
static struct platform_driver mma865x_gsensor_driver;
static struct mma865x_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
static char selftestRes[10] = {0};



/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution mma865x_data_resolution[] = {
 /*8 combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},   /*+/-2g  in 12-bit resolution:  3.9 mg/LSB*/
    {{ 2, 0}, 512},   /*+/-4g  in 12-bit resolution:  7.8 mg/LSB*/
    {{3, 9},  256},   /*+/-8g  in 12-bit resolution: 15.6 mg/LSB*/
    {{ 15, 6}, 64},   /*+/-2g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 31, 3}, 32},   /*+/-4g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 62, 5}, 16},   /*+/-8g  in 8-bit resolution:  3.9 mg/LSB (full-resolution)*/            
};
/*----------------------------------------------------------------------------*/
static struct data_resolution mma865x_offset_resolution = {{2, 0}, 512};

/*--------------------ADXL power control function----------------------------------*/

int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
   u8 buf;
    int ret = 0;
	
    client->addr = client->addr& I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG;
    buf = addr;
	ret = i2c_master_send(client, (const char*)&buf, 1<<8 | 1);
    //ret = i2c_master_send(client, (const char*)&buf, 1);
    if (ret < 0) {
        HWM_ERR("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}

void dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x00;
  u8 regdata=0;
  for(i=0; i<49 ; i++)
  {
    //dump all
    hwmsen_read_byte_sr(client,addr,&regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;
	if(addr ==01)
		addr=addr+0x06;
	if(addr==0x09)
		addr++;
	if(addr==0x0A)
		addr++;
  }
  
  /*
  for(i=0; i<5 ; i++)
  {
    //dump ctrol_reg1~control_reg5
    hwmsen_read_byte_sr(client,addr,regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;
  }
  
  addr = MMA865X_REG_OFSX;
  for(i=0; i<5 ; i++)
  {
    //dump offset
    hwmsen_read_byte_sr(client,addr,regdata);
	HWM_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++
  }
  */
}

int hwmsen_read_block_sr(struct i2c_client *client, u8 addr, u8 *data)
{
   u8 buf[10];
    int ret = 0;
	memset(buf, 0, sizeof(u8)*10); 
	
    client->addr = client->addr& I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG;
    buf[0] = addr;
	ret = i2c_master_send(client, (const char*)&buf, 6<<8 | 1);
    //ret = i2c_master_send(client, (const char*)&buf, 1);
    if (ret < 0) {
        HWM_ERR("send command error!!\n");
        return -EFAULT;
    }

    *data = buf;
	client->addr = client->addr& I2C_MASK_FLAG;
    return 0;
}

static void MMA865X_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "MMA865X"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "MMA865X"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}
/*----------------------------------------------------------------------------*/
//this function here use to set resolution and choose sensitivity
static int MMA865X_SetDataResolution(struct i2c_client *client ,u8 dataresolution)
{
    GSE_LOG("fwq set resolution  dataresolution= %d!\n", dataresolution);
	int err;
	u8  dat, reso;
    u8 databuf[10];    
    int res = 0;
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);

	if(hwmsen_read_byte_sr(client, MMA865X_REG_CTL_REG2, databuf))
	{
		GSE_ERR("read power ctl register err!\n");
		return -1;
	}
	GSE_LOG("fwq read MMA865X_REG_CTL_REG2 =%x in %s \n",databuf[0],__FUNCTION__);
	if(dataresolution == MMA865X_12BIT_RES)
	{
		databuf[0] |= MMA865X_12BIT_RES;
	}
	else
	{
		databuf[0] &= (~MMA865X_12BIT_RES);//8 bit resolution
	}
	databuf[1] = databuf[0];
	databuf[0] = MMA865X_REG_CTL_REG2;
	

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_LOG("set resolution  failed!\n");
		return -1;
	}
	else
	{
		GSE_LOG("set resolution mode ok %x!\n", databuf[1]);
	}
	
    //choose sensitivity depend on resolution and detect range
	//read detect range
	if(err = hwmsen_read_byte_sr(client, MMA865X_REG_XYZ_DATA_CFG, &dat))
	{
		GSE_ERR("read detect range  fail!!\n");
		return err;
	}
	reso  = (dataresolution & MMA865X_12BIT_RES) ? (0x00) : (0x03);
	
	
    if(dat & MMA865X_RANGE_2G)
    {
      reso = reso + MMA865X_RANGE_2G;
    }
	if(dat & MMA865X_RANGE_4G)
    {
      reso = reso + MMA865X_RANGE_4G;
    }
	if(dat & MMA865X_RANGE_8G)
    {
      reso = reso + MMA865X_RANGE_8G;
    }

	if(reso < sizeof(mma865x_data_resolution)/sizeof(mma865x_data_resolution[0]))
	{        
		obj->reso = &mma865x_data_resolution[reso];
		GSE_LOG("reso=%x!! OK \n",reso);
		return 0;
	}
	else
	{   
	    GSE_ERR("choose sensitivity  fail!!\n");
		return -EINVAL;
	}
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ReadData(struct i2c_client *client, s16 data[MMA865X_AXES_NUM])
{
	struct mma865x_i2c_data *priv = i2c_get_clientdata(client);        
	u8 addr = MMA865X_REG_DATAX0;
	u8 buf[MMA865X_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else
		
	{
	  // hwmsen_read_block(client, addr, buf, 0x06);
       // dumpReg(client);
	
		buf[0] = MMA865X_REG_DATAX0;
	    client->addr = client->addr& I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG;
        i2c_master_send(client, (const char*)&buf, 6<<8 | 1);
	    client->addr = client->addr& I2C_MASK_FLAG;


		data[MMA865X_AXIS_X] = (s16)((buf[MMA865X_AXIS_X*2] << 8) |
		         (buf[MMA865X_AXIS_X*2+1]));
		data[MMA865X_AXIS_Y] = (s16)((buf[MMA865X_AXIS_Y*2] << 8) |
		         (buf[MMA865X_AXIS_Y*2+1]));
		data[MMA865X_AXIS_Z] = (s16)((buf[MMA865X_AXIS_Z*2] << 8) |
		         (buf[MMA865X_AXIS_Z*2+1]));
	    

		if(atomic_read(&priv->trace) & ADX_TRC_REGXYZ)
		{
			GSE_LOG("raw from reg(SR) [%08X %08X %08X] => [%5d %5d %5d]\n", data[MMA865X_AXIS_X], data[MMA865X_AXIS_Y], data[MMA865X_AXIS_Z],
		                               data[MMA865X_AXIS_X], data[MMA865X_AXIS_Y], data[MMA865X_AXIS_Z]);
		}
		//GSE_LOG("raw from reg(SR) [%08X %08X %08X] => [%5d %5d %5d]\n", data[MMA865X_AXIS_X], data[MMA865X_AXIS_Y], data[MMA865X_AXIS_Z],
		  //                             data[MMA865X_AXIS_X], data[MMA865X_AXIS_Y], data[MMA865X_AXIS_Z]);
		//add to fix data, refer to datasheet
//leenli
#if 0		
		data[MMA865X_AXIS_X] = data[MMA865X_AXIS_X]>>6;
		data[MMA865X_AXIS_Y] = data[MMA865X_AXIS_Y]>>6;
		data[MMA865X_AXIS_Z] = data[MMA865X_AXIS_Z]>>6;
#endif		
		data[MMA865X_AXIS_X] = data[MMA865X_AXIS_X]>>4;
		data[MMA865X_AXIS_Y] = data[MMA865X_AXIS_Y]>>4;
		data[MMA865X_AXIS_Z] = data[MMA865X_AXIS_Z]>>4;

		data[MMA865X_AXIS_X] += priv->cali_sw[MMA865X_AXIS_X];
		data[MMA865X_AXIS_Y] += priv->cali_sw[MMA865X_AXIS_Y];
		data[MMA865X_AXIS_Z] += priv->cali_sw[MMA865X_AXIS_Z];

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("raw >>6it:[%08X %08X %08X] => [%5d %5d %5d]\n", data[MMA865X_AXIS_X], data[MMA865X_AXIS_Y], data[MMA865X_AXIS_Z],
		                               data[MMA865X_AXIS_X], data[MMA865X_AXIS_Y], data[MMA865X_AXIS_Z]);
		}
		
#ifdef CONFIG_MMA865X_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][MMA865X_AXIS_X] = data[MMA865X_AXIS_X];
					priv->fir.raw[priv->fir.num][MMA865X_AXIS_Y] = data[MMA865X_AXIS_Y];
					priv->fir.raw[priv->fir.num][MMA865X_AXIS_Z] = data[MMA865X_AXIS_Z];
					priv->fir.sum[MMA865X_AXIS_X] += data[MMA865X_AXIS_X];
					priv->fir.sum[MMA865X_AXIS_Y] += data[MMA865X_AXIS_Y];
					priv->fir.sum[MMA865X_AXIS_Z] += data[MMA865X_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][MMA865X_AXIS_X], priv->fir.raw[priv->fir.num][MMA865X_AXIS_Y], priv->fir.raw[priv->fir.num][MMA865X_AXIS_Z],
							priv->fir.sum[MMA865X_AXIS_X], priv->fir.sum[MMA865X_AXIS_Y], priv->fir.sum[MMA865X_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[MMA865X_AXIS_X] -= priv->fir.raw[idx][MMA865X_AXIS_X];
					priv->fir.sum[MMA865X_AXIS_Y] -= priv->fir.raw[idx][MMA865X_AXIS_Y];
					priv->fir.sum[MMA865X_AXIS_Z] -= priv->fir.raw[idx][MMA865X_AXIS_Z];
					priv->fir.raw[idx][MMA865X_AXIS_X] = data[MMA865X_AXIS_X];
					priv->fir.raw[idx][MMA865X_AXIS_Y] = data[MMA865X_AXIS_Y];
					priv->fir.raw[idx][MMA865X_AXIS_Z] = data[MMA865X_AXIS_Z];
					priv->fir.sum[MMA865X_AXIS_X] += data[MMA865X_AXIS_X];
					priv->fir.sum[MMA865X_AXIS_Y] += data[MMA865X_AXIS_Y];
					priv->fir.sum[MMA865X_AXIS_Z] += data[MMA865X_AXIS_Z];
					priv->fir.idx++;
					data[MMA865X_AXIS_X] = priv->fir.sum[MMA865X_AXIS_X]/firlen;
					data[MMA865X_AXIS_Y] = priv->fir.sum[MMA865X_AXIS_Y]/firlen;
					data[MMA865X_AXIS_Z] = priv->fir.sum[MMA865X_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][MMA865X_AXIS_X], priv->fir.raw[idx][MMA865X_AXIS_Y], priv->fir.raw[idx][MMA865X_AXIS_Z],
						priv->fir.sum[MMA865X_AXIS_X], priv->fir.sum[MMA865X_AXIS_Y], priv->fir.sum[MMA865X_AXIS_Z],
						data[MMA865X_AXIS_X], data[MMA865X_AXIS_Y], data[MMA865X_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ReadOffset(struct i2c_client *client, s8 ofs[MMA865X_AXES_NUM])
{    
	int err;
    GSE_ERR("fwq read offset+: \n");
	if(err = hwmsen_read_byte_sr(client, MMA865X_REG_OFSX, &ofs[MMA865X_AXIS_X]))
	{
		GSE_ERR("error: %d\n", err);
	}
	if(err = hwmsen_read_byte_sr(client, MMA865X_REG_OFSY, &ofs[MMA865X_AXIS_Y]))
	{
		GSE_ERR("error: %d\n", err);
	}
	if(err = hwmsen_read_byte_sr(client, MMA865X_REG_OFSZ, &ofs[MMA865X_AXIS_Z]))
	{
		GSE_ERR("error: %d\n", err);
	}
	GSE_LOG("fwq read off:  offX=%x ,offY=%x ,offZ=%x\n",ofs[MMA865X_AXIS_X],ofs[MMA865X_AXIS_Y],ofs[MMA865X_AXIS_Z]);
	
	return err;    
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ResetCalibration(struct i2c_client *client)
{
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	s8 ofs[MMA865X_AXES_NUM] = {0x00, 0x00, 0x00};
	int err;

	//goto standby mode to clear cali
	MMA865X_SetPowerMode(obj->client,false);
	if(err = hwmsen_write_block(client, MMA865X_REG_OFSX, ofs, MMA865X_AXES_NUM))
	{
		GSE_ERR("error: %d\n", err);
	}
    MMA865X_SetPowerMode(obj->client,true);
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ReadCalibration(struct i2c_client *client, int dat[MMA865X_AXES_NUM])
{
    struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    int mul;
    
    if ((err = MMA865X_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }    
    
    //mul = obj->reso->sensitivity/mma865x_offset_resolution.sensitivity;
    mul = mma865x_offset_resolution.sensitivity/obj->reso->sensitivity;
    dat[obj->cvt.map[MMA865X_AXIS_X]] = obj->cvt.sign[MMA865X_AXIS_X]*(obj->offset[MMA865X_AXIS_X]/mul);
    dat[obj->cvt.map[MMA865X_AXIS_Y]] = obj->cvt.sign[MMA865X_AXIS_Y]*(obj->offset[MMA865X_AXIS_Y]/mul);
    dat[obj->cvt.map[MMA865X_AXIS_Z]] = obj->cvt.sign[MMA865X_AXIS_Z]*(obj->offset[MMA865X_AXIS_Z]/mul);                        
    GSE_LOG("fwq:read cali offX=%x ,offY=%x ,offZ=%x\n",obj->offset[MMA865X_AXIS_X],obj->offset[MMA865X_AXIS_Y],obj->offset[MMA865X_AXIS_Z]);
	//GSE_LOG("fwq:read cali swX=%x ,swY=%x ,swZ=%x\n",obj->cali_sw[MMA865X_AXIS_X],obj->cali_sw[MMA865X_AXIS_Y],obj->cali_sw[MMA865X_AXIS_Z]);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ReadCalibrationEx(struct i2c_client *client, int act[MMA865X_AXES_NUM], int raw[MMA865X_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;

	if(err = MMA865X_ReadOffset(client, obj->offset))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}    

	//mul = obj->reso->sensitivity/mma865x_offset_resolution.sensitivity;
	mul = mma865x_offset_resolution.sensitivity/obj->reso->sensitivity;
	raw[MMA865X_AXIS_X] = obj->offset[MMA865X_AXIS_X]/mul + obj->cali_sw[MMA865X_AXIS_X];
	raw[MMA865X_AXIS_Y] = obj->offset[MMA865X_AXIS_Y]/mul + obj->cali_sw[MMA865X_AXIS_Y];
	raw[MMA865X_AXIS_Z] = obj->offset[MMA865X_AXIS_Z]/mul + obj->cali_sw[MMA865X_AXIS_Z];

	act[obj->cvt.map[MMA865X_AXIS_X]] = obj->cvt.sign[MMA865X_AXIS_X]*raw[MMA865X_AXIS_X];
	act[obj->cvt.map[MMA865X_AXIS_Y]] = obj->cvt.sign[MMA865X_AXIS_Y]*raw[MMA865X_AXIS_Y];
	act[obj->cvt.map[MMA865X_AXIS_Z]] = obj->cvt.sign[MMA865X_AXIS_Z]*raw[MMA865X_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_WriteCalibration(struct i2c_client *client, int dat[MMA865X_AXES_NUM])
{
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	u8 testdata=0;
	int err;
	int cali[MMA865X_AXES_NUM], raw[MMA865X_AXES_NUM];
	int lsb = mma865x_offset_resolution.sensitivity;
	u8 databuf[2]; 
	int res = 0;
	//int divisor = obj->reso->sensitivity/lsb;
	int divisor = lsb/obj->reso->sensitivity;
	GSE_LOG("fwq obj->reso->sensitivity=%d\n", obj->reso->sensitivity);
	GSE_LOG("fwq lsb=%d\n", lsb);
	

	if(err = MMA865X_ReadCalibrationEx(client, cali, raw))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[MMA865X_AXIS_X], raw[MMA865X_AXIS_Y], raw[MMA865X_AXIS_Z],
		obj->offset[MMA865X_AXIS_X], obj->offset[MMA865X_AXIS_Y], obj->offset[MMA865X_AXIS_Z],
		obj->cali_sw[MMA865X_AXIS_X], obj->cali_sw[MMA865X_AXIS_Y], obj->cali_sw[MMA865X_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[MMA865X_AXIS_X] += dat[MMA865X_AXIS_X];
	cali[MMA865X_AXIS_Y] += dat[MMA865X_AXIS_Y];
	cali[MMA865X_AXIS_Z] += dat[MMA865X_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[MMA865X_AXIS_X], dat[MMA865X_AXIS_Y], dat[MMA865X_AXIS_Z]);

	obj->offset[MMA865X_AXIS_X] = (s8)(obj->cvt.sign[MMA865X_AXIS_X]*(cali[obj->cvt.map[MMA865X_AXIS_X]])*(divisor));
	obj->offset[MMA865X_AXIS_Y] = (s8)(obj->cvt.sign[MMA865X_AXIS_Y]*(cali[obj->cvt.map[MMA865X_AXIS_Y]])*(divisor));
	obj->offset[MMA865X_AXIS_Z] = (s8)(obj->cvt.sign[MMA865X_AXIS_Z]*(cali[obj->cvt.map[MMA865X_AXIS_Z]])*(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[MMA865X_AXIS_X] =0; //obj->cvt.sign[MMA865X_AXIS_X]*(cali[obj->cvt.map[MMA865X_AXIS_X]])%(divisor);
	obj->cali_sw[MMA865X_AXIS_Y] =0; //obj->cvt.sign[MMA865X_AXIS_Y]*(cali[obj->cvt.map[MMA865X_AXIS_Y]])%(divisor);
	obj->cali_sw[MMA865X_AXIS_Z] =0;// obj->cvt.sign[MMA865X_AXIS_Z]*(cali[obj->cvt.map[MMA865X_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[MMA865X_AXIS_X] + obj->cali_sw[MMA865X_AXIS_X], 
		obj->offset[MMA865X_AXIS_Y] + obj->cali_sw[MMA865X_AXIS_Y], 
		obj->offset[MMA865X_AXIS_Z] + obj->cali_sw[MMA865X_AXIS_Z], 
		obj->offset[MMA865X_AXIS_X], obj->offset[MMA865X_AXIS_Y], obj->offset[MMA865X_AXIS_Z],
		obj->cali_sw[MMA865X_AXIS_X], obj->cali_sw[MMA865X_AXIS_Y], obj->cali_sw[MMA865X_AXIS_Z]);
	//
	//go to standby mode to set cali
    MMA865X_SetPowerMode(obj->client,false);
	if(err = hwmsen_write_block(obj->client, MMA865X_REG_OFSX, obj->offset, MMA865X_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	MMA865X_SetPowerMode(obj->client,true);
	
	//
	/*
	MMA865X_SetPowerMode(obj->client,false);
	msleep(20);
	if(err = hwmsen_write_byte(obj->client, MMA865X_REG_OFSX, obj->offset[MMA865X_AXIS_X]))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
    msleep(20);
	hwmsen_read_byte_sr(obj->client,MMA865X_REG_OFSX,&testdata);
	GSE_LOG("write offsetX: %x\n", testdata);
	
	if(err = hwmsen_write_byte(obj->client, MMA865X_REG_OFSY, obj->offset[MMA865X_AXIS_Y]))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	msleep(20);
	hwmsen_read_byte_sr(obj->client,MMA865X_REG_OFSY,&testdata);
	GSE_LOG("write offsetY: %x\n", testdata);
	
	if(err = hwmsen_write_byte(obj->client, MMA865X_REG_OFSZ, obj->offset[MMA865X_AXIS_Z]))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	msleep(20);
	hwmsen_read_byte_sr(obj->client,MMA865X_REG_OFSZ,&testdata);
	GSE_LOG("write offsetZ: %x\n", testdata);
	MMA865X_SetPowerMode(obj->client,true);
*/
	return err;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MMA865X_REG_DEVID;    

	res = hwmsen_read_byte_sr(client,MMA865X_REG_DEVID,databuf);
    GSE_LOG("fwq mma865x id %x!\n",databuf[0]);
	
	//res = hwmsen_read_byte_sr(client,MMA865X_REG_CTL_REG1,databuf);
    //GSE_LOG("fwq mma865x MMA865X_REG_CTL_REG1 %x!\n",databuf[0]);

	//res = hwmsen_read_byte_sr(client,MMA865X_REG_DEVID,databuf);
   // GSE_LOG("fwq mma865x id %x!\n",databuf[0]);
	if(databuf[0]!=MMA8652_FIXED_DEVID && databuf[0]!=MMA8653_FIXED_DEVID)
	{
		return MMA865X_ERR_IDENTIFICATION;
	}

	exit_MMA865X_CheckDeviceID:
	if (res < 0)
	{
		return MMA865X_ERR_I2C;
	}
	
	return MMA865X_SUCCESS;
}
/*----------------------------------------------------------------------------*/
//normal
//High resolution
//low noise low power
//low power

/*---------------------------------------------------------------------------*/
static int MMA865X_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = MMA865X_REG_CTL_REG1;
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	
	GSE_FUN();
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status need not to be set again!!!\n");
		return MMA865X_SUCCESS;
	}

	if(hwmsen_read_byte_sr(client, addr, databuf))
	{
		GSE_ERR("read power ctl register err!\n");
		return MMA865X_ERR_I2C;
	}

	databuf[0] &= ~MMA865X_MEASURE_MODE;
	
	if(enable == TRUE)
	{
		databuf[0] |= MMA865X_MEASURE_MODE;
	}
	else
	{
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = MMA865X_REG_CTL_REG1;
	

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		GSE_LOG("fwq set power mode failed!\n");
		return MMA865X_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("fwq set power mode ok %d!\n", databuf[1]);
	}

	sensor_power = enable;
	
	return MMA865X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
//set detect range

static int MMA865X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
    
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MMA865X_REG_XYZ_DATA_CFG;    
	databuf[1] = dataformat;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MMA865X_ERR_I2C;
	}

	return 0;

	//return MMA865X_SetDataResolution(obj,dataformat);    
}
/*----------------------------------------------------------------------------*/
static int MMA865X_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MMA865X_REG_CTL_REG1;    
	//databuf[1] = bwrate;
	
	if(hwmsen_read_byte_sr(client, MMA865X_REG_CTL_REG1, databuf))
	{
		GSE_ERR("read power ctl register err!\n");
		return MMA865X_ERR_I2C;
	}
	GSE_LOG("fwq read MMA865X_REG_CTL_REG1 =%x in %s \n",databuf[0],__FUNCTION__);

	databuf[0] &=0xC7;//clear original  data rate 
		
	databuf[0] |= bwrate; //set data rate
	databuf[1]= databuf[0];
	databuf[0]= MMA865X_REG_CTL_REG1;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MMA865X_ERR_I2C;
	}
	
	return MMA865X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int MMA865X_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MMA865X_REG_CTL_REG4;    
	databuf[1] = intenable;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MMA865X_ERR_I2C;
	}
	
	return MMA865X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int MMA865X_Init(struct i2c_client *client, int reset_cali)
{
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
    GSE_LOG("2010-11-03-11:43 fwq mma865x addr %x!\n",client->addr);
	
	res = MMA865X_CheckDeviceID(client); 
	if(res != MMA865X_SUCCESS)
	{
	    GSE_LOG("fwq mma865x check id error\n");
		return res;
	}	

	res = MMA865X_SetPowerMode(client, false);
	if(res != MMA865X_SUCCESS)
	{
	    GSE_LOG("fwq mma865x set power error\n");
		return res;
	}
	

	res = MMA865X_SetBWRate(client, MMA865X_BW_100HZ);
	if(res != MMA865X_SUCCESS ) 
	{
	    GSE_LOG("fwq mma865x set BWRate error\n");
		return res;
	}

	res = MMA865X_SetDataFormat(client, MMA865X_RANGE_2G);
	if(res != MMA865X_SUCCESS)
	{
	    GSE_LOG("fwq mma865x set data format error\n");
		return res;
	}
	//add by fwq
	res = MMA865X_SetDataResolution(client, MMA865X_12BIT_RES);
	if(res != MMA865X_SUCCESS) 
	{
	    GSE_LOG("fwq mma865x set data reslution error\n");
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;
/*//we do not use interrupt
	res = MMA865X_SetIntEnable(client, MMA865X_DATA_READY);        
	if(res != MMA865X_SUCCESS)//0x2E->0x80
	{
		return res;
	}
*/
    
	if(NULL != reset_cali)
	{ 
		/*reset calibration only in power on*/
		GSE_LOG("fwq mma865x  set cali\n");
		res = MMA865X_ResetCalibration(client);
		if(res != MMA865X_SUCCESS)
		{
		    GSE_LOG("fwq mma865x set cali error\n");
			return res;
		}
	}

#ifdef CONFIG_MMA865X_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
    GSE_LOG("fwq mma865x Init OK\n");
	return MMA865X_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "MMA865X Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct mma865x_i2c_data *obj = (struct mma865x_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[MMA865X_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == FALSE)
	{
		res = MMA865X_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on mma865x error %d!\n", res);
		}
	}

	if(res = MMA865X_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		obj->data[MMA865X_AXIS_X] += obj->cali_sw[MMA865X_AXIS_X];
		obj->data[MMA865X_AXIS_Y] += obj->cali_sw[MMA865X_AXIS_Y];
		obj->data[MMA865X_AXIS_Z] += obj->cali_sw[MMA865X_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[MMA865X_AXIS_X]] = obj->cvt.sign[MMA865X_AXIS_X]*obj->data[MMA865X_AXIS_X];
		acc[obj->cvt.map[MMA865X_AXIS_Y]] = obj->cvt.sign[MMA865X_AXIS_Y]*obj->data[MMA865X_AXIS_Y];
		acc[obj->cvt.map[MMA865X_AXIS_Z]] = obj->cvt.sign[MMA865X_AXIS_Z]*obj->data[MMA865X_AXIS_Z];

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[MMA865X_AXIS_X], acc[MMA865X_AXIS_Y], acc[MMA865X_AXIS_Z]);

		//Out put the mg
		acc[MMA865X_AXIS_X] = acc[MMA865X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[MMA865X_AXIS_Y] = acc[MMA865X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[MMA865X_AXIS_Z] = acc[MMA865X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
		

		sprintf(buf, "%04x %04x %04x", acc[MMA865X_AXIS_X], acc[MMA865X_AXIS_Y], acc[MMA865X_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
			GSE_LOG("gsensor data:  sensitivity x=%d \n",gsensor_gain.z);
			 
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_ReadRawData(struct i2c_client *client, char *buf)
{
	struct mma865x_i2c_data *obj = (struct mma865x_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}

	if(sensor_power == FALSE)
	{
		res = MMA865X_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on mma865x error %d!\n", res);
		}
	}
	
	if(res = MMA865X_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[MMA865X_AXIS_X], 
			obj->data[MMA865X_AXIS_Y], obj->data[MMA865X_AXIS_Z]);
	
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_InitSelfTest(struct i2c_client *client)
{
	int res = 0;
	u8  data;
	u8 databuf[10]; 
    GSE_LOG("fwq init self test\n");
/*
	res = MMA865X_SetPowerMode(client,true);
	if(res != MMA865X_SUCCESS ) //
	{
		return res;
	}
	*/
	res = MMA865X_SetBWRate(client, MMA865X_BW_100HZ);
	if(res != MMA865X_SUCCESS ) //
	{
		return res;
	}	
	
	res = MMA865X_SetDataFormat(client, MMA865X_RANGE_2G);
	if(res != MMA865X_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
	res = MMA865X_SetDataResolution(client, MMA865X_12BIT_RES);
	if(res != MMA865X_SUCCESS) 
	{
	    GSE_LOG("fwq mma865x set data reslution error\n");
		return res;
	}

	//set self test reg
	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MMA865X_REG_CTL_REG2;//set self test    
	if(hwmsen_read_byte_sr(client, MMA865X_REG_CTL_REG2, databuf))
	{
		GSE_ERR("read power ctl register err!\n");
		return MMA865X_ERR_I2C;
	}

	databuf[0] &=~0x80;//clear original    	
	databuf[0] |= 0x80; //set self test
	
	databuf[1]= databuf[0];
	databuf[0]= MMA865X_REG_CTL_REG2;

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
	    GSE_LOG("fwq set selftest error\n");
		return MMA865X_ERR_I2C;
	}
	
	GSE_LOG("fwq init self test OK\n");
	return MMA865X_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MMA865X_JudgeTestResult(struct i2c_client *client, s32 prv[MMA865X_AXES_NUM], s32 nxt[MMA865X_AXES_NUM])
{
    struct criteria {
        int min;
        int max;
    };
	
    struct criteria self[6][3] = {
        {{-2, 11}, {-2, 16}, {-20, 105}},
        {{-10, 89}, {0, 125}, {0, 819}},
        {{12, 135}, {-135, -12}, {19, 219}},            
        {{ 6,  67}, {-67,  -6},  {10, 110}},
        {{ 6,  67}, {-67,  -6},  {10, 110}},
        {{ 50,  540}, {-540,  -50},  {75, 875}},
    };
    struct criteria (*ptr)[3] = NULL;
    u8 detectRage;
	u8 tmp_resolution;
    int res;
	GSE_LOG("fwq judge test result\n");
    if(res = hwmsen_read_byte_sr(client, MMA865X_REG_XYZ_DATA_CFG, &detectRage))
        return res;
	if(res = hwmsen_read_byte_sr(client, MMA865X_REG_CTL_REG2, &tmp_resolution))
        return res;

	GSE_LOG("fwq tmp_resolution=%x , detectRage=%x\n",tmp_resolution,detectRage);
	if((tmp_resolution&MMA865X_12BIT_RES) && (detectRage==0x00))
		ptr = &self[0];
	else if((tmp_resolution&MMA865X_12BIT_RES) && (detectRage&MMA865X_RANGE_4G))
	{
		ptr = &self[1];
		GSE_LOG("fwq self test choose ptr1\n");
	}
	else if((tmp_resolution&MMA865X_12BIT_RES) && (detectRage&MMA865X_RANGE_8G))
		ptr = &self[2];
	else if(detectRage&MMA865X_RANGE_2G)//8 bit resolution
		ptr = &self[3];
	else if(detectRage&MMA865X_RANGE_4G)//8 bit resolution
		ptr = &self[4];
	else if(detectRage&MMA865X_RANGE_8G)//8 bit resolution
		ptr = &self[5];
	

    if (!ptr) {
        GSE_ERR("null pointer\n");
		GSE_LOG("fwq ptr null\n");
        return -EINVAL;
    }

    if (((nxt[MMA865X_AXIS_X] - prv[MMA865X_AXIS_X]) > (*ptr)[MMA865X_AXIS_X].max) ||
        ((nxt[MMA865X_AXIS_X] - prv[MMA865X_AXIS_X]) < (*ptr)[MMA865X_AXIS_X].min)) {
        GSE_ERR("X is over range\n");
        res = -EINVAL;
    }
    if (((nxt[MMA865X_AXIS_Y] - prv[MMA865X_AXIS_Y]) > (*ptr)[MMA865X_AXIS_Y].max) ||
        ((nxt[MMA865X_AXIS_Y] - prv[MMA865X_AXIS_Y]) < (*ptr)[MMA865X_AXIS_Y].min)) {
        GSE_ERR("Y is over range\n");
        res = -EINVAL;
    }
    if (((nxt[MMA865X_AXIS_Z] - prv[MMA865X_AXIS_Z]) > (*ptr)[MMA865X_AXIS_Z].max) ||
        ((nxt[MMA865X_AXIS_Z] - prv[MMA865X_AXIS_Z]) < (*ptr)[MMA865X_AXIS_Z].min)) {
        GSE_ERR("Z is over range\n");
        res = -EINVAL;
    }
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    GSE_LOG("fwq show_chipinfo_value \n");
	struct i2c_client *client = mma865x_i2c_client;
	char strbuf[MMA865X_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	MMA865X_ReadChipInfo(client, strbuf, MMA865X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mma865x_i2c_client;
	char strbuf[MMA865X_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	MMA865X_ReadSensorData(client, strbuf, MMA865X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
    GSE_LOG("fwq show_cali_value \n");
	struct i2c_client *client = mma865x_i2c_client;
	struct mma865x_i2c_data *obj;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	int err, len = 0, mul;
	int tmp[MMA865X_AXES_NUM];

	if(err = MMA865X_ReadOffset(client, obj->offset))
	{
		return -EINVAL;
	}
	else if(err = MMA865X_ReadCalibration(client, tmp))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/mma865x_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[MMA865X_AXIS_X], obj->offset[MMA865X_AXIS_Y], obj->offset[MMA865X_AXIS_Z],
			obj->offset[MMA865X_AXIS_X], obj->offset[MMA865X_AXIS_Y], obj->offset[MMA865X_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[MMA865X_AXIS_X], obj->cali_sw[MMA865X_AXIS_Y], obj->cali_sw[MMA865X_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[MMA865X_AXIS_X]*mul + obj->cali_sw[MMA865X_AXIS_X],
			obj->offset[MMA865X_AXIS_Y]*mul + obj->cali_sw[MMA865X_AXIS_Y],
			obj->offset[MMA865X_AXIS_Z]*mul + obj->cali_sw[MMA865X_AXIS_Z],
			tmp[MMA865X_AXIS_X], tmp[MMA865X_AXIS_Y], tmp[MMA865X_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = mma865x_i2c_client;  
	int err, x, y, z;
	int dat[MMA865X_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(err = MMA865X_ResetCalibration(client))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[MMA865X_AXIS_X] = x;
		dat[MMA865X_AXIS_Y] = y;
		dat[MMA865X_AXIS_Z] = z;
		if(err = MMA865X_WriteCalibration(client, dat))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mma865x_i2c_client;
	struct mma865x_i2c_data *obj;
	int result =0;
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	GSE_LOG("fwq  selftestRes value =%s\n",selftestRes); 
	return snprintf(buf, 10, "%s\n", selftestRes);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, char *buf, size_t count)
{   /*write anything to this register will trigger the process*/
	struct item{
	s16 raw[MMA865X_AXES_NUM];
	};
	
	struct i2c_client *client = mma865x_i2c_client;  
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	int idx, res, num;
	struct item *prv = NULL, *nxt = NULL;
	s32 avg_prv[MMA865X_AXES_NUM] = {0, 0, 0};
	s32 avg_nxt[MMA865X_AXES_NUM] = {0, 0, 0};
    u8 databuf[10];

	if(1 != sscanf(buf, "%d", &num))
	{
		GSE_ERR("parse number fail\n");
		return count;
	}
	else if(num == 0)
	{
		GSE_ERR("invalid data count\n");
		return count;
	}

	prv = kzalloc(sizeof(*prv) * num, GFP_KERNEL);
	nxt = kzalloc(sizeof(*nxt) * num, GFP_KERNEL);
	if (!prv || !nxt)
	{
		goto exit;
	}

	res = MMA865X_SetPowerMode(client,true);
	if(res != MMA865X_SUCCESS ) //
	{
		return res;
	}

	GSE_LOG("NORMAL:\n");
	for(idx = 0; idx < num; idx++)
	{
		if(res = MMA865X_ReadData(client, prv[idx].raw))
		{            
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		
		avg_prv[MMA865X_AXIS_X] += prv[idx].raw[MMA865X_AXIS_X];
		avg_prv[MMA865X_AXIS_Y] += prv[idx].raw[MMA865X_AXIS_Y];
		avg_prv[MMA865X_AXIS_Z] += prv[idx].raw[MMA865X_AXIS_Z];        
		GSE_LOG("[%5d %5d %5d]\n", prv[idx].raw[MMA865X_AXIS_X], prv[idx].raw[MMA865X_AXIS_Y], prv[idx].raw[MMA865X_AXIS_Z]);
	}
	
	avg_prv[MMA865X_AXIS_X] /= num;
	avg_prv[MMA865X_AXIS_Y] /= num;
	avg_prv[MMA865X_AXIS_Z] /= num; 

	res = MMA865X_SetPowerMode(client,false);
	if(res != MMA865X_SUCCESS ) //
	{
		return res;
	}

	/*initial setting for self test*/
	MMA865X_InitSelfTest(client);
	GSE_LOG("SELFTEST:\n");  
/*
	MMA865X_ReadData(client, nxt[0].raw);
	GSE_LOG("nxt[0].raw[MMA865X_AXIS_X]: %d\n", nxt[0].raw[MMA865X_AXIS_X]);
	GSE_LOG("nxt[0].raw[MMA865X_AXIS_Y]: %d\n", nxt[0].raw[MMA865X_AXIS_Y]);
	GSE_LOG("nxt[0].raw[MMA865X_AXIS_Z]: %d\n", nxt[0].raw[MMA865X_AXIS_Z]);
	*/
	for(idx = 0; idx < num; idx++)
	{
		if(res = MMA865X_ReadData(client, nxt[idx].raw))
		{            
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		avg_nxt[MMA865X_AXIS_X] += nxt[idx].raw[MMA865X_AXIS_X];
		avg_nxt[MMA865X_AXIS_Y] += nxt[idx].raw[MMA865X_AXIS_Y];
		avg_nxt[MMA865X_AXIS_Z] += nxt[idx].raw[MMA865X_AXIS_Z];        
		GSE_LOG("[%5d %5d %5d]\n", nxt[idx].raw[MMA865X_AXIS_X], nxt[idx].raw[MMA865X_AXIS_Y], nxt[idx].raw[MMA865X_AXIS_Z]);
	}

	//softrestet

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MMA865X_REG_CTL_REG2;//set self test    
	if(hwmsen_read_byte_sr(client, MMA865X_REG_CTL_REG2, databuf))
	{
		GSE_ERR("read power ctl2 register err!\n");
		return MMA865X_ERR_I2C;
	}

	databuf[0] &=~0x40;//clear original    	
	databuf[0] |= 0x40; 
	
	databuf[1]= databuf[0];
	databuf[0]= MMA865X_REG_CTL_REG2;

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
	    GSE_LOG("fwq softrest error\n");
		return MMA865X_ERR_I2C;
	}

	// 
	MMA865X_Init(client, 0);

	avg_nxt[MMA865X_AXIS_X] /= num;
	avg_nxt[MMA865X_AXIS_Y] /= num;
	avg_nxt[MMA865X_AXIS_Z] /= num;    

	GSE_LOG("X: %5d - %5d = %5d \n", avg_nxt[MMA865X_AXIS_X], avg_prv[MMA865X_AXIS_X], avg_nxt[MMA865X_AXIS_X] - avg_prv[MMA865X_AXIS_X]);
	GSE_LOG("Y: %5d - %5d = %5d \n", avg_nxt[MMA865X_AXIS_Y], avg_prv[MMA865X_AXIS_Y], avg_nxt[MMA865X_AXIS_Y] - avg_prv[MMA865X_AXIS_Y]);
	GSE_LOG("Z: %5d - %5d = %5d \n", avg_nxt[MMA865X_AXIS_Z], avg_prv[MMA865X_AXIS_Z], avg_nxt[MMA865X_AXIS_Z] - avg_prv[MMA865X_AXIS_Z]); 

	if(!MMA865X_JudgeTestResult(client, avg_prv, avg_nxt))
	{
		GSE_LOG("SELFTEST : PASS\n");
		atomic_set(&obj->selftest, 1); 
		strcpy(selftestRes,"y");
		
	}	
	else
	{
		GSE_LOG("SELFTEST : FAIL\n");
		atomic_set(&obj->selftest, 0);
		strcpy(selftestRes,"n");
	}
	
	exit:
	/*restore the setting*/    
	MMA865X_Init(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
    GSE_LOG("fwq show_firlen_value \n");
#ifdef CONFIG_MMA865X_LOWPASS
	struct i2c_client *client = mma865x_i2c_client;
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][MMA865X_AXIS_X], obj->fir.raw[idx][MMA865X_AXIS_Y], obj->fir.raw[idx][MMA865X_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[MMA865X_AXIS_X], obj->fir.sum[MMA865X_AXIS_Y], obj->fir.sum[MMA865X_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[MMA865X_AXIS_X]/len, obj->fir.sum[MMA865X_AXIS_Y]/len, obj->fir.sum[MMA865X_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, char *buf, size_t count)
{
    GSE_LOG("fwq store_firlen_value \n");
#ifdef CONFIG_MMA865X_LOWPASS
	struct i2c_client *client = mma865x_i2c_client;  
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    GSE_LOG("fwq show_trace_value \n");
	ssize_t res;
	struct mma865x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
    GSE_LOG("fwq store_trace_value \n");
	struct mma865x_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    GSE_LOG("fwq show_status_value \n");
	ssize_t len = 0;    
	struct mma865x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(selftest,       S_IWUSR | S_IRUGO, show_selftest_value,          store_selftest_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *mma865x_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_selftest,         /*self test demo*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,        
};
/*----------------------------------------------------------------------------*/
static int mma865x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mma865x_attr_list)/sizeof(mma865x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, mma865x_attr_list[idx]))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", mma865x_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int mma865x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(mma865x_attr_list)/sizeof(mma865x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mma865x_attr_list[idx]);
	}
	

	return err;
}

/*----------------------------------------------------------------------------*/
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct mma865x_i2c_data *priv = (struct mma865x_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[MMA865X_BUFSIZE];
	
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			//GSE_LOG("fwq set delay\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = MMA865X_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = MMA865X_BW_100HZ;
				}
				else
				{
					sample_delay = MMA865X_BW_50HZ;
				}
				
				err = MMA865X_SetBWRate(priv->client, MMA865X_BW_100HZ); //err = MMA865X_SetBWRate(priv->client, sample_delay);
				if(err != MMA865X_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{					
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[MMA865X_AXIS_X] = 0;
					priv->fir.sum[MMA865X_AXIS_Y] = 0;
					priv->fir.sum[MMA865X_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				}
			}
			break;

		case SENSOR_ENABLE:
			GSE_LOG("fwq sensor enable gsensor\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					err = MMA865X_SetPowerMode( priv->client, !sensor_power);
				}
			}
			break;

		case SENSOR_GET_DATA:
			//GSE_LOG("fwq sensor operate get data\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				MMA865X_ReadSensorData(priv->client, buff, MMA865X_BUFSIZE);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
				//GSE_LOG("X :%d,Y: %d, Z: %d\n",gsensor_data->values[0],gsensor_data->values[1],gsensor_data->values[2]);
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int mma865x_open(struct inode *inode, struct file *file)
{
	file->private_data = mma865x_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int mma865x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int mma865x_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
   //    unsigned long arg)
static long mma865x_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct mma865x_i2c_data *obj = (struct mma865x_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[MMA865X_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			//GSE_LOG("fwq GSENSOR_IOCTL_INIT\n");
			MMA865X_Init(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			//GSE_LOG("fwq GSENSOR_IOCTL_READ_CHIPINFO\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MMA865X_ReadChipInfo(client, strbuf, MMA865X_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			//GSE_LOG("fwq GSENSOR_IOCTL_READ_SENSORDATA\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MMA865X_ReadSensorData(client, strbuf, MMA865X_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			//GSE_LOG("fwq GSENSOR_IOCTL_READ_GAIN\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			//GSE_LOG("fwq GSENSOR_IOCTL_READ_OFFSET\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, &gsensor_offset, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			//GSE_LOG("fwq GSENSOR_IOCTL_READ_RAW_DATA\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MMA865X_ReadRawData(client, &strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			//GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{   
			    GSE_LOG("fwq going to set cali\n");
				cali[MMA865X_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[MMA865X_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[MMA865X_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = MMA865X_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			//GSE_LOG("fwq GSENSOR_IOCTL_CLR_CALI!!\n");
			err = MMA865X_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			//GSE_LOG("fwq GSENSOR_IOCTL_GET_CALI\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(err = MMA865X_ReadCalibration(client, cali))
			{
				break;
			}
			
			sensor_data.x = cali[MMA865X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[MMA865X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[MMA865X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations mma865x_fops = {
	.owner = THIS_MODULE,
	.open = mma865x_open,
	.release = mma865x_release,
	.unlocked_ioctl = mma865x_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mma865x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &mma865x_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int mma865x_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	u8  dat=0;
	GSE_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		//read old data
		if ((err = hwmsen_read_byte_sr(client, MMA865X_REG_CTL_REG1, &dat))) 
		{
           GSE_ERR("read ctl_reg1  fail!!\n");
           return err;
        }
		dat = dat&0b11111110;//stand by mode
		atomic_set(&obj->suspend, 1);
		if(err = hwmsen_write_byte(client, MMA865X_REG_CTL_REG1, dat))
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}        
		MMA865X_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int mma865x_resume(struct i2c_client *client)
{
	struct mma865x_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	MMA865X_power(obj->hw, 1);
	if(err = MMA865X_Init(client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void mma865x_early_suspend(struct early_suspend *h) 
{
	struct mma865x_i2c_data *obj = container_of(h, struct mma865x_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	/*
	if(err = hwmsen_write_byte(obj->client, MMA865X_REG_POWER_CTL, 0x00))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}  
	*/
	if(err = MMA865X_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;
	
	MMA865X_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void mma865x_late_resume(struct early_suspend *h)
{
	struct mma865x_i2c_data *obj = container_of(h, struct mma865x_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	MMA865X_power(obj->hw, 1);
	if(err = MMA865X_Init(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int mma865x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, MMA865X_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mma865x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mma865x_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct mma865x_i2c_data));

	obj->hw = get_cust_acc_hw();
	
	if(err = hwmsen_get_convert(obj->hw->direction, &obj->cvt))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
#ifdef CONFIG_MMA865X_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	mma865x_i2c_client = new_client;	

	if(err = MMA865X_Init(new_client, 1))
	{
		goto exit_init_failed;
	}
	

	if(err = misc_register(&mma865x_device))
	{
		GSE_ERR("mma865x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(err = mma865x_create_attr(&mma865x_gsensor_driver.driver))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = gsensor_operate;
	if(err = hwmsen_attach(ID_ACCELEROMETER, &sobj))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = mma865x_early_suspend,
	obj->early_drv.resume   = mma865x_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	misc_deregister(&mma865x_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int mma865x_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	if(err = mma865x_delete_attr(&mma865x_gsensor_driver.driver))
	{
		GSE_ERR("mma865x_delete_attr fail: %d\n", err);
	}
	
	if(err = misc_deregister(&mma865x_device))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if(err = hwmsen_detach(ID_ACCELEROMETER))
	    

	mma865x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mma865x_probe(struct platform_device *pdev) 
{
	struct acc_hw *hw = get_cust_acc_hw();
	GSE_FUN();

	MMA865X_power(hw, 1);
	//mma865x_force[0] = hw->i2c_num;
	if(i2c_add_driver(&mma865x_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mma865x_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc_hw();

    GSE_FUN();    
    MMA865X_power(hw, 0);    
    i2c_del_driver(&mma865x_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver mma865x_gsensor_driver = {
	.probe      = mma865x_probe,
	.remove     = mma865x_remove,    
	.driver     = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};

/*----------------------------------------------------------------------------*/
static int __init mma865x_init(void)
{
	GSE_FUN();
	i2c_register_board_info(1, &i2c_MMA865X, 1);

	if(platform_driver_register(&mma865x_gsensor_driver))
	{
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit mma865x_exit(void)
{
	GSE_FUN();
	platform_driver_unregister(&mma865x_gsensor_driver);
}
/*----------------------------------------------------------------------------*/
module_init(mma865x_init);
module_exit(mma865x_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMA865X I2C driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");
