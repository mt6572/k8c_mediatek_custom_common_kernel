
#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "tpd_custom_ft6206.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"

//#undef TPD_PS_SUPPORT
 #ifdef TPD_PS_SUPPORT
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

//bingo,
//#define MT65XX_POWER_LDO_VGP 1
//#define MT65XX_POWER_LDO_VGP2 2
 
extern struct tpd_device *tpd;
 
static struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
#ifdef MT6575 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
#endif
#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;


//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 3

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif


#if defined(TPD_PS_SUPPORT)
static unsigned char tpd_ps_flag = 0; //enable--1;disanle--0
static unsigned char tpd_ps_value=1;// 1->far,0->close

#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
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
		TPD_DEBUG("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;


		default:
			TPD_DEBUG("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

#ifdef  TPD_PS_SUPPORT//TPD_PROXIMITY
extern int getcall_staus(); //ugrec_tky 
static s32 tpd_get_ps_value(void)
{
    return tpd_ps_value;
}


static s32 tpd_enable_ps(s32 enable)
{
	s32 ret = 0;
	static char write_data = 0x01;
	int call_status=getcall_staus();
	
	if (tpd_load_status==0)
		return;

	/*unsigned char bWriteData[1] =
	{
	    0xB0
	};*/

	if(enable==1)
	{
		write_data = 0x01;
		tpd_ps_flag=1;

	}
	else
	{	
		write_data = 0x00;
		if(call_status==0)
			tpd_ps_flag=0;

	}

       TPD_DEBUG("ainen5555555555555555555555555555555555555555tpd_enable_ps tpd_ps_flag=%d \n", tpd_ps_flag);
       tpd_ps_value = 1;
       
     //  i2c_client->addr = TOUCH_ADDR_MSG20XX;
     //  i2c_client->addr = ((i2c_client->addr & I2C_MASK_FLAG ) | I2C_ENEXT_FLAG );//LK@add
       //i2c_write_bytes(i2c_client, &bWriteData[0], 4);
    //   ret = i2c_master_send(i2c_client, &bWriteData[0], 4);

	 
	ret=i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &write_data);
	 
    if (ret < 0)
    {
        TPD_DEBUG("TPD %s proximity cmd failed." ,"disable");
        return ret;
    }

    TPD_DEBUG("TPD %s proximity cmd ok.", "ok");
    return 0;	   

}

static s32 tpd_ps_operate(void *self, u32 command, void *buff_in, s32 size_in,
                   void *buff_out, s32 size_out, s32 *actualout)
{
    s32 err = 0;
    s32 value;
    hwm_sensor_data *sensor_data;
	
       //TPD_DEBUG("ainen11111111111 tpd_ps_operate command=%d\n", command);

    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DEBUG("Set delay parameter error!");
                err = -EINVAL;
            }

            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DEBUG("Enable sensor parameter error!");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                err = tpd_enable_ps(value);
            }

            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
            {
                TPD_DEBUG("Get sensor data parameter error!");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
			
                sensor_data->values[0] = tpd_get_ps_value();
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
	          //TPD_DEBUG("ainen112222222222 tpd_ps_operate sensor_data->values[0]  =%d\n", sensor_data->values[0] );
            }

            break;

        default:
            TPD_DEBUG("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}
#endif

static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[5];
    int x[5];
    int p[5];
    int id[5];
    int count;
};
 
 static const struct i2c_device_id ft5206_tpd_id[] = {{"ft5206",0},{}};
 unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
 static const unsigned short * const forces[] = { force, NULL };
 //static struct i2c_client_address_data addr_data = { .forces = forces, };
 static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO("ft5206", (0x70>>1))};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "ft5206",//.name = TPD_DEVICE,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = ft5206_tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
   .address_list = (const unsigned short*) forces,   
 };
 

static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 //TPD_DEBUG("D[%4d %4d %4d] ", x, y, p);
	 /* track id Start 0 */
       input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	 input_mt_sync(tpd->dev);
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
       tpd_button(x, y, 1);  
     }
	 if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	 {
         msleep(50);
		 TPD_DEBUG("D virtual key \n");
	 }
	 TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }
 
static  void tpd_up(int x, int y,int *count) {
	 //if(*count>0) {
		 //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		// input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 //printk("U[%4d %4d %4d] ", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_EM_PRINT(x, y, x, y, 0, 0);
	//	 (*count)--;
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
        tpd_button(x, y, 0); 
     }   		 

 }

 static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {

	int i = 0;
	
/*start:modify by zhangwei 2013-03-31-11:14:53  for touch ctp, at moment restart  BSP_MODIFY_RECORD*/
	char data[35] = {0};
/*end:modify by zhangwei 2013-03-31-13:14:53*/

#ifdef TPD_PS_SUPPORT
	//s32 err = 0;
	hwm_sensor_data sensor_data;
	//u8 proximity_status;
	//char drv_ps_data[2] = {0};
	//static char write_data = 0x01;
#endif

    u16 high_byte,low_byte;
	u8 report_rate =0;

	p_point_num = point_num;

	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));



	//ps read data
#ifdef TPD_PS_SUPPORT
	//i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &write_data);
	//i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 2, &(drv_ps_data[0]));
#endif

	
	////i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[24]));
	////i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);
	//TPD_DEBUG("FW version=%x]\n",data[24]);
	
	//TPD_DEBUG("received raw data from touch panel as following:\n");
	//TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
	//TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
	//TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);


	 //we have  to re update report rate
    // TPD_DMESG("report rate =%x\n",report_rate);
	 if(report_rate < 8)
	 {
	   report_rate = 0x8;
	   if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	   {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	   }
	 }
	 
	
	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if((data[0] & 0x70) != 0) return false; 


	//printk("ainentp tpd_ps_flag = %d data[0] = %x data[1] = %x  \n",tpd_ps_flag,data[0],data[1]);
	
	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;


	//printk("ainentp point_num = %d  \n",point_num);
	for(i = 0; i < point_num; i++)
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
	           cinfo->id[i] = (data[5+6*i] >> 4); //id//cinfo->id[i] = data[5+6*i+2]>>4; //touch id
	   /*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;

			//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra

		/*get the Y coordinate, 2 bytes*/
		
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;

		  //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;

		cinfo->count++;
		
	}

	#ifdef TPD_PS_SUPPORT

		#if 0
		//printk("ainentp drv_ps_data[0] = %x drv_ps_data[1] = %x  \n",drv_ps_data[0],drv_ps_data[1]);
		//data[1]  = 00;
		//tpd_ps_flag = 0;
		if(data[1] == 0xC0 && tpd_ps_flag == 1) // close and ps enable
		{

			s32 ret = -1;
			//msg2133_set_ps_data_to_alsps(0);
			tpd_ps_value = 0;


			
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

			//report to the up-layer
			ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);

			if (ret)
			{
				//printk("ainen..............................tpd_touchinfo err = %d \n", err);
			}
			//printk("[tpd_touchinfo] ps close\n");
			return true;
		}
		else if(data[1] == 0xE0 && tpd_ps_flag == 1) // leave and and ps enable
		{
			s32 ret = -1;
			
			tpd_ps_value = 1;


			
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

			//report to the up-layer
			ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);

			
			if (ret)
			{
				//printk("ainen..............................tpd_touchinfo err = %d \n", err);
			}
			//printk("[tpd_touchinfo] ps leave\n");
			return true;
		}

		#else
		TPD_DEBUG("tky tp tpd_ps_flag= %x data[1] = %x  \n",tpd_ps_flag,data[1]);
		printk("TKYTP tpd_ps_flag= %x data[1] = %x  \n",tpd_ps_flag,data[1]);
		
		if((data[1] == 0xC0 && tpd_ps_flag == 1) || (data[1] == 0xE0 && tpd_ps_flag == 1)) // close and ps enable
		{
			s32 ret = -1;

			if(data[1] == 0xC0 && tpd_ps_flag == 1)
			{
				tpd_ps_value = 0;
			}
			else if(data[1] == 0xE0 && tpd_ps_flag == 1) 
			{
				tpd_ps_value = 1;
			}
			printk("TKYTP tpd_ps_valueAAAA= %x   \n",tpd_ps_value);
			sensor_data.values[0] = tpd_get_ps_value();
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

			//report to the up-layer
			ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);

			
			if (ret<0)
			{
				TPD_DEBUG("ainen..............................tpd_touchinfo err = %d \n", ret);
			}
			TPD_DEBUG("[tpd_touchinfo] ps leave\n");
			return true;
			
		}
		printk("TKYTP tpd_ps_valueBBBBBBBBB= %x   \n",tpd_ps_value);
		#endif
	#endif

	//printk("ainentp data[0] = %x data[1] = %x data[2] = %x data[3] = %x data[4] = %x data[5] = %x  data[6] = %x data[7] = %x \n",drv_ps_data[0],drv_ps_data[1],drv_ps_data[2],drv_ps_data[3],drv_ps_data[4],drv_ps_data[5],drv_ps_data[6],drv_ps_data[7]);
	  
	 return true;

 };

 static int touch_event_handler(void *unused)
 {
  
    struct touch_info cinfo, pinfo;
	 int i=0;

	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 set_current_state(TASK_INTERRUPTIBLE); 
		  wait_event_interruptible(waiter,tpd_flag!=0);
						 
			 tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 

		  if (tpd_touchinfo(&cinfo, &pinfo)) 
		  {
		    //TPD_DEBUG("point_num = %d\n",point_num);
			TPD_DEBUG_SET_TIME;
			if(point_num >0) 
			{
			    for(i =0; i<point_num && i<5; i++)//only support 3 point
			    {

			         tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
			       
			    }
			    input_sync(tpd->dev);
			}

			else  
            {
			    tpd_up(cinfo.x[0], cinfo.y[0], 0);
                //TPD_DEBUG("release --->\n"); 
                //input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
        }

        if(tpd_mode==12)
        {
           //power down for desence debug
           //power off, need confirm with SA
          // hwPowerDown(MT65XX_POWER_LDO_VGP2,  "TP");
          // hwPowerDown(MT65XX_POWER_LDO_VGP,  "TP");
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#else
	hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif 
	    msleep(20);
          
        }

 }while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 //TPD_DEBUG("TPD interrupt has been triggered\n");
	 TPD_DEBUG_PRINT_INT;
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }



#ifdef  FT6206_AUTO_CFG_UPGRADE

//#define AUTO_CLB


#define IC_FT5X06	0
#define IC_FT5606	1
#define IC_FT5316	2
#define IC_FT5X36	3
#define IC_FT6X06	4

#define FT_UPGRADE_AA	0xAA
#define FT_UPGRADE_55 	0x55
#define FT_UPGRADE_EARSE_DELAY		1500

#define FT5x0x_REG_FW_VER		0xA6

/*upgrade config of FT5606*/
#define FT5606_UPGRADE_AA_DELAY 		50
#define FT5606_UPGRADE_55_DELAY 		10
#define FT5606_UPGRADE_ID_1			0x79
#define FT5606_UPGRADE_ID_2			0x06
#define FT5606_UPGRADE_READID_DELAY 	100

/*upgrade config of FT5316*/
#define FT5316_UPGRADE_AA_DELAY 		50
#define FT5316_UPGRADE_55_DELAY 		40
#define FT5316_UPGRADE_ID_1			0x79
#define FT5316_UPGRADE_ID_2			0x07
#define FT5316_UPGRADE_READID_DELAY 	1

/*upgrade config of FT5x06(x=2,3,4)*/
#define FT5X06_UPGRADE_AA_DELAY 		50
#define FT5X06_UPGRADE_55_DELAY 		30
#define FT5X06_UPGRADE_ID_1			0x79
#define FT5X06_UPGRADE_ID_2			0x03
#define FT5X06_UPGRADE_READID_DELAY 	1

/*upgrade config of FT5X36*/
#define FT5X36_UPGRADE_AA_DELAY 		30
#define FT5X36_UPGRADE_55_DELAY 		30
#define FT5X36_UPGRADE_ID_1			0x79
#define FT5X36_UPGRADE_ID_2			0x11
#define FT5X36_UPGRADE_READID_DELAY 	10

/*upgrade config of FT5X36*/
#define FT6X06_UPGRADE_AA_DELAY 		100
#define FT6X06_UPGRADE_55_DELAY 		10
#define FT6X06_UPGRADE_ID_1			0x79
#define FT6X06_UPGRADE_ID_2			0x08
#define FT6X06_UPGRADE_READID_DELAY 	10

#define DEVICE_IC_TYPE	IC_FT6X06

#define    FTS_PACKET_LENGTH       2 //ugrec_tky  128
#define    FTS_SETTING_BUF_LEN    2 //ugrec_tky     128

#define FTS_UPGRADE_LOOP	3 //ugrec_tky 

struct Upgrade_Info{
	u16		delay_aa;		/*delay of write FT_UPGRADE_AA*/
	u16		delay_55;		/*delay of write FT_UPGRADE_55*/
	u8		upgrade_id_1;	/*upgrade id 1*/
	u8		upgrade_id_2;	/*upgrade id 2*/
	u16		delay_readid;	/*delay of read id*/
};

void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}

int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			printk("CTP %s: i2c read error.\n",	__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			printk("CTP %s:i2c read error.\n", __func__);
	}
	return ret;
}

int ft5x0x_read_reg(struct i2c_client * client, u8 regaddr, u8 * regvalue)
{
	return ft5x0x_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

static unsigned char CTPM_FW[]=
{
	#include "SY1277_SP01E_0X0C_mouse_20131230_app.i"
};


u8 fts_ctpm_get_i_file_ver(void)
{
    u16 ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        return 0x00; /*default value*/
    }
}

int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		printk("CTP %s i2c write error.\n", __func__);

	return ret;
}

int ft5x0x_write_reg(struct i2c_client * client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;
	
	return ft5x0x_i2c_Write(client, buf, sizeof(buf));
}

static void fts_get_upgrade_info(struct Upgrade_Info * upgrade_info)
{
	switch(DEVICE_IC_TYPE)
	{
	case IC_FT5X06:
		upgrade_info->delay_55 = FT5X06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X06_UPGRADE_READID_DELAY;
		break;
	case IC_FT5606:
		upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
		break;
	case IC_FT5316:
		upgrade_info->delay_55 = FT5316_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5316_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5316_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5316_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5316_UPGRADE_READID_DELAY;
		break;
	case IC_FT5X36:
		upgrade_info->delay_55 = FT5X36_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X36_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X36_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X36_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X36_UPGRADE_READID_DELAY;
		break;
	case IC_FT6X06:
		upgrade_info->delay_55 = FT6X06_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT6X06_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT6X06_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT6X06_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT6X06_UPGRADE_READID_DELAY;
		break;		
	default:
		break;
	}
}

int  fts_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	
	u8 reg_val[2] = {0};
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u32  packet_number;
	u32  j;
	u32  temp;
	u32  lenght;
	u8 	packet_buf[FTS_PACKET_LENGTH + 6];
	u8  	auc_i2c_write_buf[10];
	u8  	bt_ecc;
	int      i_ret;
	struct Upgrade_Info upgradeinfo;

	fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
    	/*********Step 1:Reset  CTPM *****/
    	/*write 0xaa to register 0xfc*/
	//   	ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_AA);
	//	msleep(upgradeinfo.delay_aa);
		
		 /*write 0x55 to register 0xfc*/
	//	ft5x0x_write_reg(client, 0xfc, FT_UPGRADE_55);   
	//	msleep(upgradeinfo.delay_55);   

		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
		msleep(1);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
		msleep(40);
		TPD_DMESG(" ft5306 reset\n");
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(40);
		/*********Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		
	    	i_ret = ft5x0x_i2c_Write(client, auc_i2c_write_buf, 2);
	  
	    /*********Step 3:check READ-ID***********************/   
		msleep(upgradeinfo.delay_readid);
	   	auc_i2c_write_buf[0] = 0x90; 
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		ft5x0x_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);
		
		if (reg_val[0] == upgradeinfo.upgrade_id_1 
			&& reg_val[1] == upgradeinfo.upgrade_id_2)
		{
		    	printk("CTP  [FTS] Step 3 OK: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
		    	break;
		}
		else
		{
			printk("CTP  [FTS] Step 3 FAIL: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	    		continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
		
	auc_i2c_write_buf[0] = 0xcd;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] > 4)
		is_5336_new_bootloader = 1;

     /*********Step 4:erase app and panel paramenter area ********************/
	auc_i2c_write_buf[0] = 0x61;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1); /*erase app area*/	
       delay_qt_ms(FT_UPGRADE_EARSE_DELAY);  //msleep

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(lenght>>8);
		packet_buf[5] = (u8)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
		    packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}

		ft5x0x_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH+6);
		//msleep(FTS_PACKET_LENGTH/6 + 1);
		udelay(100);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;

		for (i=0;i<temp;i++)
		{
		    packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}
  
		ft5x0x_i2c_Write(client, packet_buf, temp+6);
		msleep(20);
	}

	/*send the last six byte*/
	for (i = 0; i<6; i++)
	{
		if (is_5336_new_bootloader && DEVICE_IC_TYPE==IC_FT5X36) 
			temp = 0x7bfa + i;
		else
			temp = 0x6ffa + i;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		temp =1;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;
		packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		bt_ecc ^= packet_buf[6];
  
		ft5x0x_i2c_Write(client, packet_buf, 7);
		msleep(10);
	}

	/*********Step 6: read out checksum***********************/
	/*send the opration head*/
	auc_i2c_write_buf[0] = 0xcc;
	ft5x0x_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1); 

	if(reg_val[0] != bt_ecc)
	{
		printk("CTP  [FTS]--ecc error! FW=%02x bt_ecc=%02x\n", reg_val[0], bt_ecc);
	    	return -EIO;
	}

	/*********Step 7: reset the new FW***********************/
	auc_i2c_write_buf[0] = 0x07;
	ft5x0x_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(300);  /*make sure CTP startup normally*/  //msleep

	return 0;
}

#if 0
int fts_ctpm_auto_clb(struct i2c_client * client)
{
	unsigned char uc_temp;
	unsigned char i ;

	/*start auto CLB*/
	msleep(200);
	ft5x0x_write_reg(client, 0, 0x40);  
	msleep(100);   /*make sure already enter factory mode*/
	ft5x0x_write_reg(client, 2, 0x4);  /*write command to start calibration*/
	msleep(300);
	if (DEVICE_IC_TYPE == IC_FT5X36) {
		for(i=0;i<100;i++)
		{
			ft5x0x_read_reg(client, 0x02, &uc_temp);
			if (0x02 == uc_temp ||
				0xFF == uc_temp)
			{
				/*if 0x02, then auto clb ok, else 0xff, auto clb failure*/
			    break;
			}
			msleep(20);	    
		}
	} else {
		for(i=0;i<100;i++)
		{
			ft5x0x_read_reg(client, 0, &uc_temp);
			if (0x0 == ((uc_temp&0x70)>>4))  /*return to normal mode, calibration finish*/
			{
			    break;
			}
			msleep(20);	    
		}
	}
	/*calibration OK*/
	ft5x0x_write_reg(client, 0, 0x40);  /*goto factory mode for store*/
	msleep(200);   /*make sure already enter factory mode*/
	ft5x0x_write_reg(client, 2, 0x5);  /*store CLB result*/
	msleep(300);
	ft5x0x_write_reg(client, 0, 0x0); /*return to normal mode*/ 
	msleep(300);
	/*store CLB result OK*/
	
	return 0;
}
#endif

/*
upgrade with *.i file
*/
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client * client)
{
	u8 * pbt_buf = NULL;
	int i_ret;
	int fw_len = sizeof(CTPM_FW);

	/*judge the fw that will be upgraded
	 * if illegal, then stop upgrade and return.
	*/
	if(fw_len<8 || fw_len>32*1024)
	{
		pr_err("FW length error\n");
		return -EIO;
	}	
	if((CTPM_FW[fw_len-8]^CTPM_FW[fw_len-6])==0xFF
		&& (CTPM_FW[fw_len-7]^CTPM_FW[fw_len-5])==0xFF
		&& (CTPM_FW[fw_len-3]^CTPM_FW[fw_len-4])==0xFF)
	{
		/*FW upgrade*/
		pbt_buf = CTPM_FW;
		/*call the upgrade function*/
		i_ret =  fts_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
		{
			printk("CTP  [FTS] upgrade failed. err=%d.\n", i_ret);
		}
		else
		{
			#ifdef AUTO_CLB
				fts_ctpm_auto_clb(client);  /*start auto CLB*/
			#endif
		}
	}
	else
	{
		printk("CTP  FW format error\n");
		return -EBADFD;
	}
	return i_ret;
}


 int fts_ctpm_auto_upgrade(struct i2c_client * client)
{
	u8 uc_host_fm_ver=FT5x0x_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int           i_ret;
	
	ft5x0x_read_reg(client, FT5x0x_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	printk("CTP  uc_tp_fm_ver==0x%x  uc_host_fm_ver===0x%x\n",uc_tp_fm_ver,uc_host_fm_ver);
	if ( uc_tp_fm_ver != uc_host_fm_ver /*the firmware in host flash is new, need upgrade*/	    )  ///*the firmware in touch panel maybe corrupted*/  uc_tp_fm_ver == FT5x0x_REG_FW_VER  ||
	{
		printk("CTP AAAAAAAAAAAAAAAA\n");
		msleep(100);; //ugrec_tky 100
		printk("CTP [FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = fts_ctpm_fw_upgrade_with_i_file(client);    
		if (i_ret == 0)
		{
		    	msleep(300);; //ugrec_tky 300
		    	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		    	printk("CTP  [FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
		}
		else
		{
		    	printk("CTP  [FTS] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}
	}
	printk("CTP BBBBBBBBBBBBBBBBBBB\n");
	return 0;
}
#endif

 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;
#ifdef TPD_PS_SUPPORT
    struct hwmsen_object obj_ps;
#endif

reset_proc:   
	i2c_client = client;

   
		//power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
	hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#else
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
#endif
#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 

	#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
	#else
	#ifdef MT6573
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
  mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(100);
	#endif
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(1);
	TPD_DMESG(" ft5306 reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	#endif

	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	  mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	  mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	  mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
	msleep(100);

	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
		   return -1; 
	}

	//set report rate 80Hz
	report_rate = 0x8; 
	if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	{
	    if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	    {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	    }
		   
	}

	tpd_load_status = 1;

	#ifdef VELOCITY_CUSTOM_FT5206
	if((err = misc_register(&tpd_misc_device)))
	{
		TPD_DEBUG("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}

	TPD_DMESG("ft5206 Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#ifdef TPD_PS_SUPPORT
	obj_ps.polling = 0;         //0--interrupt mode;1--polling mode;
	obj_ps.sensor_operate = tpd_ps_operate;

	if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
	    TPD_DEBUG("hwmsen attach fail, return:%d.", err);
	}
#endif


#ifdef  FT6206_AUTO_CFG_UPGRADE
	printk("CTP  FTS_UPGRADE begin\n");
	fts_ctpm_auto_upgrade(i2c_client);
	printk("CTP  FTS_UPGRADE end\n");
#endif

	
   return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 
 {
   
	 TPD_DEBUG("TPD removed\n");
 
   return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG("Focaltech FT5206 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("ft5206 unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("ft5206 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }

 static void tpd_resume( struct early_suspend *h )
 {
  //int retval = TPD_OK;
  char data;
 
   //TPD_DMESG("TPD wake up tpd_ps_flag=%d  \n",tpd_ps_flag);

   #if defined(TPD_PS_SUPPORT)
	if((tpd_ps_flag==1))
	{
		return 0;
	}
   #endif


#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP"); 
#else
	#ifdef MT6573
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
		msleep(100);
	#endif	

	#ifdef MT6577 //added by maxyu 120924   
		pmic_ldo_vol_sel(MT65XX_POWER_LDO_VGP2, VOL_2800);
		pmic_ldo_enable(MT65XX_POWER_LDO_VGP2, KAL_TRUE);	
		msleep(150);
	#endif
	

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(10);  
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	
       msleep(20);
	TPD_DMESG("TPD wake up done11\n");
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: 914\n" );
	}
	tpd_up(0,0,0);
	input_sync(tpd->dev);
	TPD_DMESG("TPD wake up done\n");

	 //return retval;
 }

 static void tpd_suspend( struct early_suspend *h )
 {
	// int retval = TPD_OK;
	 static char data = 0x3;
 
	 //TPD_DMESG("TPD enter sleep  tpd_ps_flag==%d  \n",tpd_ps_flag);

#if defined(TPD_PS_SUPPORT)
	printk("TKYTP tpd_suspendCCCCCCCCCCCCCCCCCCCC= %x   \n",tpd_ps_flag);
	if(tpd_ps_flag==1)
	{
		return 0;
	}
#endif

	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
	
	#ifdef MT6573
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	#endif

	#ifdef MT6577 //added by maxyu 120924
	    pmic_ldo_enable(MT65XX_POWER_LDO_VGP2, KAL_FALSE);
	#endif
 #endif
        TPD_DMESG("TPD enter sleep done\n");
	 //return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "FT5206",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
	 TPD_DEBUG("MediaTek FT5206 touch panel driver init\n");
	   i2c_register_board_info(1, &ft5206_i2c_tpd, 1); //ugrec_tky
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add FT5206 driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek FT5206 touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);


