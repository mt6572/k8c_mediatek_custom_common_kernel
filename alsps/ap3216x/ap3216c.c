/*
 * This file is part of the AP3216C sensor driver for MTK platform.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: YC Hou <yc.hou@liteonsemi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3216c_mtk.c
 *
 * Summary:
 *	AP3216C sensor dirver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 05/11/12 YC		 Original Creation (Test version:1.0)
 * 05/30/12 YC		 Modify AP3216C_check_and_clear_intr return value and exchange
 *                   AP3216C_get_ps_value return value to meet our spec.
 * 05/30/12 YC		 Correct shift number in AP3216C_read_ps.
 * 05/30/12 YC		 Correct ps data formula.
 * 05/31/12 YC		 1. Change the reg in clear int function from low byte to high byte 
 *                      and modify the return value.
 *                   2. Modify the eint_work function to filter als int.
 * 06/04/12 YC		 Add PS high/low threshold instead of using the same value.
 * 07/12/12 YC		 Add wakelock to prevent entering suspend when early suspending.
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
#include <linux/wakelock.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "ap3216c.h"

/*
#if 0
#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif
#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif

#ifdef MT6577
#include <mach/mt6577_devs.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_gpio.h>
#include <mach/mt6577_pm_ldo.h>
#endif

#ifdef MT6573
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

#endif
#ifdef MT6575
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

#endif

#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
#ifdef MT6575
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
#ifdef MT6577
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#endif
*/


extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);



/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define AP3216C_DEV_NAME     "AP3216C"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 

/*----------------------------------------------------------------------------*/
static struct i2c_client *AP3216C_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id AP3216C_i2c_id[] = {{AP3216C_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_ap3216c={ I2C_BOARD_INFO("AP3216C", (0x3C >> 1))};
//static unsigned short AP3216C_force[] = {0x00, 0x3C, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const AP3216C_forces[] = { AP3216C_force, NULL };
//static struct i2c_client_address_data AP3216C_addr_data = { .forces = AP3216C_forces,};
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int AP3216C_i2c_remove(struct i2c_client *client);
//static int AP3216C_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int AP3216C_i2c_resume(struct i2c_client *client);

static struct AP3216C_priv *g_AP3216C_ptr = NULL;


#define MS_ENABLE
//#define USE_WAKELOCK        //20130718 add
//#define UNUSE_ENREAD

#if defined(USE_WAKELOCK)
struct wake_lock chrg_lock;
#endif
#ifdef MS_ENABLE
static int AP3216C_Detect(struct i2c_client *client);
#endif
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
#ifdef MS_ENABLE
	CMC_BIT_MS	   = 4,
#endif
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct AP3216C_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct AP3216C_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
	  struct mutex lock;
    /*i2c address group*/
    struct AP3216C_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;   /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;   /*the cmd value can't be read, stored in ram*/
#if defined(UNUSE_ENREAD)
    atomic_t    enable;
#else
    ulong       enable;         /*enable mask*/    
#endif
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver AP3216C_i2c_driver = {	
	.probe      = AP3216C_i2c_probe,
	.remove     = AP3216C_i2c_remove,
//	.detect     = AP3216C_i2c_detect,
	.suspend    = AP3216C_i2c_suspend,
	.resume     = AP3216C_i2c_resume,
	.id_table   = AP3216C_i2c_id,
//	.address_data = &AP3216C_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = AP3216C_DEV_NAME,
	},
};

static struct AP3216C_priv *AP3216C_obj = NULL;
static struct platform_driver AP3216C_alsps_driver;
static int LSC_read_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift)
{
	int ret = 0;
	char tmp[1];
	tmp[0]=reg;
	mutex_lock(&AP3216C_obj->lock);

	ret = i2c_master_send(client, tmp, 0x01);
	if(ret <= 0)
	{
		printk("LSC_read_reg 1 ret=%x\n",ret);
		goto EXIT_ERR;
	}
	ret = i2c_master_recv(client, tmp, 0x01);
	if(ret <= 0)
	{
		printk("LSC_read_reg 2 ret=%d\n",ret);
		goto EXIT_ERR;
	}

	mutex_unlock(&AP3216C_obj->lock);
	return ( tmp[0] & mask ) >> shift;

EXIT_ERR:
		APS_ERR("LSC_read_reg fail\n");
        mutex_unlock(&AP3216C_obj->lock);
		return ret;
}

static int LSC_write_reg(struct i2c_client *client,
		char reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0x00;
	char tmp[2];

    mutex_lock(&AP3216C_obj->lock);

    tmp[0]=reg;
    tmp[1]=val;
	ret = i2c_master_send(client, tmp, 0x02);
	if(ret <= 0)
	{
		printk("LSC_write_reg ret=%d\n",ret);
		goto EXIT_ERR;
	}

    mutex_unlock(&AP3216C_obj->lock);
	return ret;

EXIT_ERR:
		APS_ERR("LSC_write_reg fail\n");
    mutex_unlock(&AP3216C_obj->lock);
		return ret;
}
/*----------------------------------------------------------------------------*/
int AP3216C_get_addr(struct alsps_hw *hw, struct AP3216C_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void AP3216C_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "AP3216C")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "AP3216C")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_enable_als(struct i2c_client *client, int enable)
{
		struct AP3216C_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];	  
		int res = 0;
		u8 buffer[1];
		int reg_value[1];
	
		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUAL NULL\n");
			return -1;
		}
	
#if defined(UNUSE_ENREAD)
        reg_value[0]=atomic_read(&obj->enable);
#else
		buffer[0]=AP3216C_LSC_ENABLE;
		reg_value[0] = LSC_read_reg(client,buffer[0], 0xFF, 0x00);
		if(res < 0)
		{
			goto EXIT_ERR;
		}        
#endif
		if(enable)
		{
			databuf[0] = AP3216C_LSC_ENABLE;	
			databuf[1] = reg_value[0] |0x01;
			res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("AP3216C ALS enable\n");
		}
		else
		{
			databuf[0] = AP3216C_LSC_ENABLE;	
			databuf[1] = reg_value[0] &0xFE;
			res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("AP3216C ALS disable\n");
		}
		return 0;
		
	EXIT_ERR:
		APS_ERR("AP3216C_enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
static int AP3216C_enable_ps(struct i2c_client *client, int enable)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

#if defined(UNUSE_ENREAD)
    reg_value[0]=atomic_read(&obj->enable);
#else
	buffer[0]=AP3216C_LSC_ENABLE;
	reg_value[0] = LSC_read_reg(client,buffer[0], 0xFF, 0x00);
	if(res < 0)
	{
		goto EXIT_ERR;
	}    
#endif
	if(enable)
	{
		//wake_lock(&ps_lock);
		databuf[0] = AP3216C_LSC_ENABLE;    
		databuf[1] = reg_value[0] |0x02;
		res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));

		mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
		APS_DBG("AP3216C PS enable\n");
	}
	else
	{
		databuf[0] = AP3216C_LSC_ENABLE;    
		databuf[1] = reg_value[0] &0xfd;
		res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("AP3216C PS disable\n");

		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt65xx_eint_mask(CUST_EINT_ALS_NUM);
		}
		//wake_unlock(&ps_lock);
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("AP3216C_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_check_and_clear_intr(struct i2c_client *client) 
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 ints[1];

	/* Get Int status */
	ints[0] = LSC_read_reg(client,AP3216C_LSC_INT_STATUS, 0xFF, 0x00);
	if(ints[0] < 0)
	{
		goto EXIT_ERR;
	}

	/* Clear ALS int flag */
	res = LSC_read_reg(client,AP3216C_LSC_ADATA_H, 0xFF, 0x00);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

	/* Clear PS int flag */
	res =LSC_read_reg(client,AP3216C_LSC_PDATA_H, 0xFF, 0x00);
	if(res < 0)
	{
		goto EXIT_ERR;
	}

	return ints[0];

EXIT_ERR:
	APS_ERR("AP3216C_check_and_clear_intr fail\n");
	return -1;
}
static int LSC_set_plthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 2;
	lsb = val & 0x03;

	err = LSC_write_reg(client, 0x2A,
			0xFF, 0x00, lsb);
	if (err<=0)
		return err;

	err = LSC_write_reg(client, 0x2B,
			0xFF, 0x00, msb);

	return err;
}

static int LSC_set_phthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;

	msb = val >> 2;
	lsb = val & 0x03;

	err = LSC_write_reg(client, 0x2C,
			0xFF, 0x00, lsb);
	if (err<=0)
		return err;

	err = LSC_write_reg(client, 0x2D,
			0xFF, 0x00, msb);

	return err;
}

static int LSC_set_pcrosstalk(struct i2c_client *client, int val)
{
    int lsb, msb, err;
	msb = val >> 1;
	lsb = val & 0x01;
    err =LSC_write_reg(client, 0x28,
            0xFF, 0x00, lsb);    
	if (err<=0)
		return err;            
    err =LSC_write_reg(client, 0x29,
            0xFF, 0x00, msb);
	return err;
}
/*----------------------------------------------------------------------------*/
void AP3216C_eint_func(void)
{
	printk("AP3216C_eint_func\n");
	struct AP3216C_priv *obj = g_AP3216C_ptr;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
// This function depends the real hw setting, customers should modify it. 2012/5/10 YC. 
int AP3216C_setup_eint(struct i2c_client *client)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);        

	g_AP3216C_ptr = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, AP3216C_eint_func, 0);

	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LSC_init_client(struct i2c_client *client)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = AP3216C_LSC_ENABLE;    
	databuf[1] = 0x00;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
	/*
	databuf[0] = AP3216C_LSC_INT_LOW_THD_LOW;
	databuf[1] = atomic_read(&obj->ps_thd_val_l) & 0x03;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_HIGH_THD_LOW;
	databuf[1] = atomic_read(&obj->ps_thd_val_h) & 0x03;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_LOW_THD_HIGH;
	databuf[1] = (atomic_read(&obj->ps_thd_val_l) & 0x3FC) >> 2;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

	databuf[0] = AP3216C_LSC_INT_HIGH_THD_HIGH;
	databuf[1] = (atomic_read(&obj->ps_thd_val_h) & 0x3FC) >> 2;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
*/	
    	LSC_set_plthres(client,atomic_read(&obj->ps_thd_val_low));
    	LSC_set_phthres(client,atomic_read(&obj->ps_thd_val_high));
#if 1
/*******************************************/
	 LSC_set_pcrosstalk(client,50);//set crosstalk 100
	 
	databuf[0] = 0x20;   //modify the gain 2->8
	databuf[1] = 0x05;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
	databuf[0] = 0x21;   //modify the led plus 1->3
	databuf[1] = 0x33;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}
	databuf[0] = 0x23;   //modify the mean time 12.5->50.
	databuf[1] = 0x03;
	res = LSC_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return AP3216C_ERR_I2C;
	}

#endif
	
	return AP3216C_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int AP3216C_read_als(struct i2c_client *client, u16 *data)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);	 
	u8 als_value_low[1], als_value_high[1];
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	// get ALS adc count
	als_value_low[0] = LSC_read_reg(client,AP3216C_LSC_ADATA_L, 0xFF, 0x00);
	if(als_value_low[0] < 0)
	{
		goto EXIT_ERR;
	}

	als_value_high[0] = LSC_read_reg(client,AP3216C_LSC_ADATA_H, 0xFF, 0x00);
	if(als_value_high[0] < 0)
	{
		goto EXIT_ERR;
	}

	*data = als_value_low[0] | (als_value_high[0]<<8);

	if (*data < 0)
	{
		*data = 0;
		APS_DBG("als_value is invalid!!\n");
		goto EXIT_ERR;
	}

	return 0;

EXIT_ERR:
	APS_ERR("AP3216C_read_als fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_get_als_value(struct AP3216C_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
int AP3216C_read_ps(struct i2c_client *client, u16 *data)
{
	struct AP3216C_priv *obj = i2c_get_clientdata(client);       
	u8 ps_value_low[1], ps_value_high[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}
	ps_value_low[0] = LSC_read_reg(client,AP3216C_LSC_PDATA_L, 0xFF, 0x00);
	if(ps_value_low[0] < 0)
	{
		goto EXIT_ERR;
	}

	ps_value_high[0] = LSC_read_reg(client,AP3216C_LSC_PDATA_H, 0xFF, 0x00);
	if(ps_value_high[0] < 0)
	{
		goto EXIT_ERR;
	}

	*data = (ps_value_low[0] & 0x0f) | ((ps_value_high[0] & 0x3f) << 4);

	return 0;    

EXIT_ERR:
	printk("AP3216C_read_ps fail\n");
	return -1;
}
/*----------------------------------------------------------------------------*/
/* 
   for AP3216C_get_ps_value:
	return 1 = object close,
	return 0 = object far away. 2012/5/10 YC   // exchange 0 and 1 2012/5/30 YC 
*/
static int AP3216C_get_ps_value(struct AP3216C_priv *obj, u16 ps)
{
	int val;
	int invalid = 0;

	if(ps > atomic_read(&obj->ps_thd_val_high))
		val = 0;  /*close*/
	else if(ps < atomic_read(&obj->ps_thd_val_low))
		val = 1;  /*far away*/

	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

static int AP3216C_get_OBJ(struct i2c_client *client)
{
	//struct AP3216C_priv *obj = i2c_get_clientdata(client);       
	u8 ps_value_high[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUAL NULL\n");
		return -1;
	}

	ps_value_high[0] = LSC_read_reg(client,AP3216C_LSC_PDATA_H, 0xFF, 0x00);
	if(ps_value_high[0] < 0)
	{
		goto EXIT_ERR;
	}
	// printk("the ps_value_h>>7 is %d\n",ps_value_high[0]>>7);
	return !(ps_value_high[0]>>7);
  
EXIT_ERR:
	APS_ERR("AP3216C_get_obj fail\n");
	return 0;
}


/*----------------------------------------------------------------------------*/
static void AP3216C_eint_work(struct work_struct *work)
{
	struct AP3216C_priv *obj = (struct AP3216C_priv *)container_of(work, struct AP3216C_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;

	if((err = AP3216C_check_and_clear_intr(obj->client)) < 0)
	{
		APS_ERR("AP3216C_eint_work check intrs: %d\n", err);
	}
	else if (err & 0x01)
	{
		// ALS interrupt. User should add their code here if they want to handle ALS Int.
	}
	else if (err & 0x02)
	{
#ifdef MS_ENABLE	
		AP3216C_Detect(obj->client);
#endif
	//	AP3216C_read_ps(obj->client, &obj->ps);
	//	sensor_data.values[0] = AP3216C_get_ps_value(obj, obj->ps);
		sensor_data.values[0] = AP3216C_get_OBJ(obj->client);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);      
}
/*----------------------------------------------------------------------------*/

static ssize_t AP3216C_em_read(struct device_driver *ddri, char *buf)
{
	  #define AP3216C_NUM_CACHABLE_REGS	26 
    int idx=0,count=0;
		int reg_value[1];

		u8 ap3216c_reg[AP3216C_NUM_CACHABLE_REGS] = 
		{0x00,0x01,0x02,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
	   0x10,0x19,0x1a,0x1b,0x1c,0x1d,
	   0x20,0x21,0x22,0x23,0x24,0x28,0x29,0x2a,0x2b,0x2c,0x2d};
	 
    if(!AP3216C_obj)
    {
        APS_ERR("AP3216C_obj is null!!\n");
    		count+=sprintf(buf+count, "AP3216C_obj is null!! \n");
    		return count;
    }
    for(idx=0;idx<AP3216C_NUM_CACHABLE_REGS;idx++){

			reg_value[0] = LSC_read_reg(AP3216C_obj->client,ap3216c_reg[idx], 0xFF, 0x00);
		if(reg_value[0] < 0)
		{
    		count+=sprintf(buf+count, "i2c read_reg err \n");
    		return count;
		}
		  count+=sprintf(buf+count, "[%x]=0x%x \n",ap3216c_reg[idx],reg_value[0]);
    }
    AP3216C_read_ps(AP3216C_obj->client,&idx);
    count+=sprintf(buf+count, "[ps]=%d \n",idx);
    return count;
}

static ssize_t AP3216C_em_write(struct device_driver *ddri,char *buf, size_t count)
{

	int addr,val;
	int ret = 0;

    if(!AP3216C_obj)
    {
        APS_ERR("AP3216C_obj is null!!\n");
        return 0;
    }

	sscanf(buf, "%x %x", &addr, &val);

	printk("Reg[%x].Write [%x]..\n",addr,val);

    ret = LSC_write_reg(AP3216C_obj->client,addr, 0xFF,0x00,val);

	return count;
}
static DRIVER_ATTR(em,      S_IWUSR | S_IRUGO, AP3216C_em_read,AP3216C_em_write);
static struct device_attribute *AP3216C_attr_list[] = {
            &driver_attr_em
};

static int AP3216C_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(AP3216C_attr_list)/sizeof(AP3216C_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, AP3216C_attr_list[idx]))
        {
            APS_ERR("driver_create_file (%s) = %d\n", AP3216C_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}

/*----------------------------------------------------------------------------*/
static int AP3216C_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(AP3216C_attr_list)/sizeof(AP3216C_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, AP3216C_attr_list[idx]);
    }

    return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int AP3216C_open(struct inode *inode, struct file *file)
{
	file->private_data = AP3216C_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int AP3216C_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int AP3216C_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static int AP3216C_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct AP3216C_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = AP3216C_enable_ps(obj->client, 1))
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)|0x02);
#else
				set_bit(CMC_BIT_PS, &obj->enable);                
#endif
			}
			else
			{
				if(err = AP3216C_enable_ps(obj->client, 0))
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFD);
#else
				clear_bit(CMC_BIT_PS, &obj->enable);                
#endif
			}
			break;

		case ALSPS_GET_PS_MODE:
#if defined(UNUSE_ENREAD)
            enable = (atomic_read(&obj->enable)&0x02) ? (1) : (0);
#else
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);            	
#endif
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if(err = AP3216C_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = AP3216C_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if(err = AP3216C_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = AP3216C_enable_als(obj->client, 1))
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)|0x01);
#else
				set_bit(CMC_BIT_ALS, &obj->enable);                
#endif
			}
			else
			{
				if(err = AP3216C_enable_als(obj->client, 0))
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFE);
#else
				clear_bit(CMC_BIT_ALS, &obj->enable);                
#endif
			}
			break;

		case ALSPS_GET_ALS_MODE:
#if defined(UNUSE_ENREAD)
            enable=(atomic_read(&obj->enable)&0x01) ? (1) : (0);
#else
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);            	
#endif
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if(err = AP3216C_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = AP3216C_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if(err = AP3216C_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations AP3216C_fops = {
//	.owner = THIS_MODULE,
	.open = AP3216C_open,
	.release = AP3216C_release,
//	.ioctl = AP3216C_ioctl,
	.unlocked_ioctl = AP3216C_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice AP3216C_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &AP3216C_fops,
};
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	APS_FUN();    
	return 0;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_resume(struct i2c_client *client)
{
	APS_FUN();
	return 0;
}
/*----------------------------------------------------------------------------*/
static void AP3216C_early_suspend(struct early_suspend *h) 
{   
	struct AP3216C_priv *obj = container_of(h, struct AP3216C_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 1);
#if defined(UNUSE_ENREAD)
    if(atomic_read(&obj->enable)&0x01)
#else
	if(test_bit(CMC_BIT_ALS, &obj->enable))    	
#endif
	{
		if(err = AP3216C_enable_als(obj->client, 0))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
#if defined(USE_WAKELOCK)		
	if (atomic_read(&obj->enable)&0x02)
	{
		APS_DBG("USE_WAKELOCK\n");
		wake_lock(&chrg_lock);
	}
#endif
}
/*----------------------------------------------------------------------------*/
static void AP3216C_late_resume(struct early_suspend *h)
{   
	struct AP3216C_priv *obj = container_of(h, struct AP3216C_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
#if defined(UNUSE_ENREAD)
    if(atomic_read(&obj->enable)&0x01)
#else
	if(test_bit(CMC_BIT_ALS, &obj->enable))    	
#endif
	{
		if(err = AP3216C_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
#if defined(USE_WAKELOCK)
	if (atomic_read(&obj->enable)&0x02)
		{
		wake_unlock(&chrg_lock);
	  }
#endif		
}

int AP3216C_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct AP3216C_priv *obj = (struct AP3216C_priv *)self;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if(err = AP3216C_enable_ps(obj->client, 1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)|0x02);
#else
					set_bit(CMC_BIT_PS, &obj->enable);                
#endif
				}
				else
				{
					if(err = AP3216C_enable_ps(obj->client, 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
                atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFD);
#else
					clear_bit(CMC_BIT_PS, &obj->enable);                
#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				AP3216C_read_ps(obj->client, &obj->ps);
				
				sensor_data->values[0] = AP3216C_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			#ifdef MS_ENABLE	
			AP3216C_Detect(obj->client);
			#endif
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
static int temp_als = 0;
int AP3216C_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct AP3216C_priv *obj = (struct AP3216C_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if(err = AP3216C_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
					atomic_set(&obj->enable,atomic_read(&obj->enable)|0x01);
#else
					set_bit(CMC_BIT_ALS, &obj->enable);					
#endif
				}
				else
				{
					if(err = AP3216C_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
#if defined(UNUSE_ENREAD)
					atomic_set(&obj->enable,atomic_read(&obj->enable)&0xFE);
#else
					clear_bit(CMC_BIT_ALS, &obj->enable);					
#endif
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				AP3216C_read_als(obj->client, &obj->als);
				APS_LOG("ap3216c ALS level=%d\n",obj->als);
				#if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				#else			
				if(obj->als == 0)
				{
					sensor_data->values[0] = temp_als;
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
						AP3216C_read_als(obj->client, &obj->als);
						b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = AP3216C_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, AP3216C_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct AP3216C_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	AP3216C_obj = obj;

	mutex_init(&obj->lock);
	obj->hw = get_cust_alsps_hw();
	AP3216C_get_addr(obj->hw, &obj->addr);

	INIT_WORK(&obj->eint_work, AP3216C_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
#if defined(UNUSE_ENREAD)
    atomic_set(&obj->enable,0);
#else
	obj->enable = 0;    
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);
#endif
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	obj->als_modulus = (400*100*40)/(1*1500);

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	
	AP3216C_i2c_client = client;

	
	if(err = LSC_init_client(client))
	{
		goto exit_init_failed;
	}
	APS_LOG("LSC_init_client() OK!\n");

	if(err = misc_register(&AP3216C_device))
	{
		APS_ERR("AP3216C_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	if(err = AP3216C_create_attr(&AP3216C_alsps_driver.driver))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_ps.self = AP3216C_obj;	
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
		if(err = AP3216C_setup_eint(client))
		{
			APS_ERR("setup eint: %d\n", err);
		}
		if((err = AP3216C_check_and_clear_intr(client)) < 0)
		{
			APS_ERR("check/clear intr: %d\n", err);
		}			
	}
		obj_ps.sensor_operate = AP3216C_ps_operate;
		if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
		{
			APS_ERR("attach fail = %d\n", err);
			goto exit_create_attr_failed;
		}		

		
	    obj_als.self = AP3216C_obj;
		obj_als.polling = 1;
		obj_als.sensor_operate = AP3216C_als_operate;
		if(err = hwmsen_attach(ID_LIGHT, &obj_als))
		{
			APS_ERR("attach fail = %d\n", err);
			goto exit_create_attr_failed;
		}

APS_LOG("hwmsen_attach OK.%s: \n", __func__);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = AP3216C_early_suspend,
	obj->early_drv.resume   = AP3216C_late_resume,    
	register_early_suspend(&obj->early_drv);

#endif
#if defined(USE_WAKELOCK)
	wake_lock_init(&chrg_lock, WAKE_LOCK_SUSPEND, "ap3216c_wake_lock");
#endif	
	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&AP3216C_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	mutex_destroy(&AP3216C_obj->lock);		
	exit_kfree:
	kfree(obj);
	exit:
	AP3216C_i2c_client = NULL;           
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_i2c_remove(struct i2c_client *client)
{
	int err;	

	if(err = misc_deregister(&AP3216C_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	AP3216C_i2c_client = NULL;
	i2c_unregister_device(client);
	mutex_destroy(&AP3216C_obj->lock);
	kfree(i2c_get_clientdata(client));
#if defined(USE_WAKELOCK)	
	wake_lock_destroy(&chrg_lock);
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	AP3216C_power(hw, 1);    
	//AP3216C_force[0] = hw->i2c_num;
	//AP3216C_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",AP3216C_force[0],AP3216C_force[1]);
	if(i2c_add_driver(&AP3216C_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int AP3216C_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	AP3216C_power(hw, 0);    
	i2c_del_driver(&AP3216C_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver AP3216C_alsps_driver = {
	.probe      = AP3216C_probe,
	.remove     = AP3216C_remove,    
	.driver     = {
		.name  = "als_ps",
//		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init AP3216C_init(void)
{
	APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_ap3216c, 1);
	//i2c_register_board_info(0, &i2c_ap3216c, 1);
	if(platform_driver_register(&AP3216C_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit AP3216C_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&AP3216C_alsps_driver);
}
#ifdef MS_ENABLE
/*----------------------------------------------------------------------------*/
/*-----Infrared Shake Hands-------------------------------------------------*/
/*---------------------------------------------------------------------------=*/
enum{
	SHAKE_NONE,
	SHAKE_LEFT,
	SHAKE_RIGHT,

};
//extern void alps_send_keyinfo(int keycode);//add zhumw

static int AP3216C_Detect(struct i2c_client *client)
{
		//alps_send_keyinfo(KEY_LEFTALT);//add zhumw
    return SHAKE_LEFT;
}

#endif
module_init(AP3216C_init);
module_exit(AP3216C_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("YC Hou");
MODULE_DESCRIPTION("AP3216C driver");
MODULE_LICENSE("GPL");
