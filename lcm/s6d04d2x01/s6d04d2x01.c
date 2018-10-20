/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
	
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
#endif
	

#define FRAME_WIDTH  (240)
#define FRAME_HEIGHT (240)
#define LCM_ID		 (0x9806)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

static __inline void send_ctrl_cmd(unsigned int cmd)
{
   lcm_util.send_cmd(cmd);
}


static __inline void send_data_cmd(unsigned int data)
{
   lcm_util.send_data(data&0xff);
}


static __inline unsigned int read_data_cmd(void)
{
   return 0xFF&lcm_util.read_data();
}


static __inline void set_lcm_register(unsigned int regIndex,
                                                            unsigned int regData)
{
   send_ctrl_cmd(regIndex);
   send_data_cmd(regData);
}

static inline void set_lcm_gpio_output_low(unsigned int GPIO)
{
	lcm_util.set_gpio_mode(GPIO, GPIO_MODE_00);
	lcm_util.set_gpio_dir(GPIO, GPIO_DIR_OUT);
	lcm_util.set_gpio_out(GPIO, GPIO_OUT_ZERO);
}

static inline void set_lcm_gpio_mode(unsigned int GPIO, unsigned int mode)
{
	lcm_util.set_gpio_mode(GPIO, mode);
}

static inline int get_lcm_gpio_mode(unsigned int GPIO)
{
	return mt_get_gpio_mode(GPIO);
}


static void sw_clear_panel(unsigned int color)
{
   unsigned short x0, y0, x1, y1, x, y;
   unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;
   
   x0 = (unsigned short)0;
   y0 = (unsigned short)0;
   x1 = (unsigned short)FRAME_WIDTH-1;
   y1 = (unsigned short)FRAME_HEIGHT-1;
   
   h_X_start=((x0&0xFF00)>>8);
   l_X_start=(x0&0x00FF);
   h_X_end=((x1&0xFF00)>>8);
   l_X_end=(x1&0x00FF);
   
   h_Y_start=((y0&0xFF00)>>8);
   l_Y_start=(y0&0x00FF);
   h_Y_end=((y1&0xFF00)>>8);
   l_Y_end=(y1&0x00FF);
   
   send_ctrl_cmd(0x2A);
   send_data_cmd(h_X_start); 
   send_data_cmd(l_X_start); 
   send_data_cmd(h_X_end); 
   send_data_cmd(l_X_end); 
   
   send_ctrl_cmd(0x2B);
   send_data_cmd(h_Y_start); 
   send_data_cmd(l_Y_start); 
   send_data_cmd(h_Y_end); 
   send_data_cmd(l_Y_end); 
   
   send_ctrl_cmd(0x29); 
   
   send_ctrl_cmd(0x2C);
   for (y = y0; y <= y1; ++ y) {
      for (x = x0; x <= x1; ++ x) {
         //lcm_util.send_data(0x00);
         lcm_util.send_data(color);
      }
   }
   
  // send_ctrl_cmd(0x2C);
}


static void init_lcm_registers(void)
{
//************* Start Initial Sequence **********//
#if 0
 int i,j;


MDELAY(120);          //Delay 120ms   
send_ctrl_cmd(0xB2);     
send_data_cmd(0x0C);   
send_data_cmd(0x0C);   
send_data_cmd(0x00);   
send_data_cmd(0x33);   
send_data_cmd(0x33);   

send_ctrl_cmd(0xB7);     
send_data_cmd(0x02);   

send_ctrl_cmd(0xC2);     
send_data_cmd(0x01);   

send_ctrl_cmd(0xC3);     
send_data_cmd(0x16);

send_ctrl_cmd(0xbb);     
send_data_cmd(0x3e);   

send_ctrl_cmd(0xC5);     
send_data_cmd(0x20);      

send_ctrl_cmd(0xD0);     
send_data_cmd(0xA4);   
send_data_cmd(0xA2);       

send_ctrl_cmd(0xD2);     
send_data_cmd(0x4c);   
  
send_ctrl_cmd(0xe8);     
send_data_cmd(0x83);     

send_ctrl_cmd(0xE9); 
send_data_cmd(0x09);
send_data_cmd(0x09);
send_data_cmd(0x08);

send_ctrl_cmd(0x36);     
send_data_cmd(0x00);     

send_ctrl_cmd(0x35);     
send_data_cmd(0x00);     

send_ctrl_cmd(0x3a);     
send_data_cmd(0x06);     

send_ctrl_cmd(0xc6);     
send_data_cmd(0x0f);     

send_ctrl_cmd(0xE0);     
send_data_cmd(0xd0);   
send_data_cmd(0x06);   
send_data_cmd(0x0c);   
send_data_cmd(0x09); 
send_data_cmd(0x09);   
send_data_cmd(0x25);   
send_data_cmd(0x2e);   
send_data_cmd(0x33);   
send_data_cmd(0x45);   
send_data_cmd(0x36);   
send_data_cmd(0x12);   
send_data_cmd(0x12);   
send_data_cmd(0x2e);   
send_data_cmd(0x34);   
//send_data_cmd(0x23);   

send_ctrl_cmd(0xE1);     
send_data_cmd(0xd0);   

send_data_cmd(0x06);   

send_data_cmd(0x0c);   
send_data_cmd(0x09); 
send_data_cmd(0x09);   

send_data_cmd(0x25);   
send_data_cmd(0x2e);   
send_data_cmd(0x33);   

send_data_cmd(0x45);   
send_data_cmd(0x36);   
send_data_cmd(0x12);   
send_data_cmd(0x12);   

send_data_cmd(0x2e);   

send_data_cmd(0x34); 



//windows 240*240
send_ctrl_cmd(0x2A);     
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0xEF);   

send_ctrl_cmd(0x2B);     
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0xEF); 
//end 

send_ctrl_cmd(0x2c);
for (i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
		send_data_cmd(0xff);
		send_data_cmd(0xff);
		send_data_cmd(0xff);
              
	}
send_ctrl_cmd(0x11);     
MDELAY(120); 


send_ctrl_cmd(0x29);        // Display On        // Display On 

#else
int i,j;


MDELAY(120);          //Delay 120ms
send_ctrl_cmd(0x36);     
send_data_cmd(0x00);

send_ctrl_cmd(0x3A);     
send_data_cmd(0x06);//05--->65K;06---->262K
   
send_ctrl_cmd(0xB2);     
send_data_cmd(0x0C);   
send_data_cmd(0x0C);   
send_data_cmd(0x00);   
send_data_cmd(0x33);   
send_data_cmd(0x33);   

send_ctrl_cmd(0xB7);     
send_data_cmd(0x75);   

send_ctrl_cmd(0xBB);     
send_data_cmd(0x2d);   

//send_ctrl_cmd(0xC0);     
//send_data_cmd(0x2C);   

send_ctrl_cmd(0xC2);     
send_data_cmd(0x01);   

send_ctrl_cmd(0xC3);     
send_data_cmd(0x19);   

send_ctrl_cmd(0xC4);     
send_data_cmd(0x20);   

send_ctrl_cmd(0xC6);     
send_data_cmd(0x0F);   

send_ctrl_cmd(0xD0);     
send_data_cmd(0xA4);   
send_data_cmd(0xA1);       

//send_ctrl_cmd(0xE9); 
//send_data_cmd(0x11);
//send_data_cmd(0x11);
//send_data_cmd(0x03);

send_ctrl_cmd(0xE0);     
send_data_cmd(0x70);   
send_data_cmd(0x06);   
send_data_cmd(0x0b);   
send_data_cmd(0x07);   
send_data_cmd(0x08);   
send_data_cmd(0x05);   
send_data_cmd(0x2b);   
send_data_cmd(0x43);   
send_data_cmd(0x42);   
send_data_cmd(0x06);   
send_data_cmd(0x14);   
send_data_cmd(0x15);   
send_data_cmd(0x29);   
send_data_cmd(0x2d);   

send_ctrl_cmd(0xE1);     
send_data_cmd(0x70);   
send_data_cmd(0x05);   
send_data_cmd(0x0b);   
send_data_cmd(0x06);   
send_data_cmd(0x07);   
send_data_cmd(0x05);   
send_data_cmd(0x2a);   
send_data_cmd(0x33);   
send_data_cmd(0x41);   
send_data_cmd(0x07);   
send_data_cmd(0x14);   
send_data_cmd(0x15);   
send_data_cmd(0x29);   
send_data_cmd(0x2c); 
 



//windows 240*240
send_ctrl_cmd(0x2A);     
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0xEF);   

send_ctrl_cmd(0x2B);     
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0x00);   
send_data_cmd(0xEF); 
//end 

send_ctrl_cmd(0x2c);
for (i = 0; i < FRAME_WIDTH * FRAME_HEIGHT; i++) {
		send_data_cmd(0xff);
		send_data_cmd(0xff);
		send_data_cmd(0xff);
              
	}
send_ctrl_cmd(0x21); 
send_ctrl_cmd(0x11);     
MDELAY(120); 


send_ctrl_cmd(0x29);        // Display On        // Display On 
#endif
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
   memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DBI;
	params->ctrl = LCM_CTRL_PARALLEL_DBI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dbi.port = 0;
	params->dbi.clock_freq              = LCM_DBI_CLOCK_FREQ_52M;
	params->dbi.data_width = LCM_DBI_DATA_WIDTH_8BITS;
	params->dbi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dbi.data_format.trans_seq = LCM_DBI_TRANS_SEQ_MSB_FIRST;
	params->dbi.data_format.padding = LCM_DBI_PADDING_ON_LSB;
	params->dbi.data_format.format = LCM_DBI_FORMAT_RGB666;
	params->dbi.data_format.width = LCM_DBI_DATA_WIDTH_8BITS;
	params->dbi.cpu_write_bits = LCM_DBI_CPU_WRITE_8_BITS;
	params->dbi.io_driving_current = LCM_DRIVING_CURRENT_8MA;

	params->dbi.parallel.write_setup    = 2;
       params->dbi.parallel.write_hold     = 2;
       params->dbi.parallel.write_wait     = 10;
       params->dbi.parallel.read_setup     = 4;
       params->dbi.parallel.read_hold      = 2;
       params->dbi.parallel.read_latency   = 31;
       params->dbi.parallel.wait_period    = 2;
	params->dbi.parallel.cs_high_width = 0;

	params->dbi.te_mode = 0;
	params->dbi.te_edge_polarity = 0;
}	

static void lcm_init(void)
{
	

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(10);

	init_lcm_registers();
}

static void lcm_suspend(void)
{
      // sw_clear_panel(0);
	send_ctrl_cmd(0x28);
	send_ctrl_cmd(0x10);
	MDELAY(5);

	
}

static void lcm_resume(void)
{
	

	send_ctrl_cmd(0x11);
	MDELAY(120);
	send_ctrl_cmd(0x29);
}

static void lcm_update(unsigned int x, unsigned int y,
                                       unsigned int width, unsigned int height)
{
	unsigned short x0, y0, x1, y1;
	unsigned short h_X_start, l_X_start, h_X_end, l_X_end, h_Y_start, l_Y_start, h_Y_end, l_Y_end;

	x0 = (unsigned short)x;
	y0 = (unsigned short)y;
	x1 = (unsigned short)x + width - 1;
	y1 = (unsigned short)y + height - 1;

	h_X_start = (x0 & 0xFF00) >> 8;
	l_X_start = x0 & 0x00FF;
	h_X_end = (x1 & 0xFF00) >> 8;
	l_X_end = x1 & 0x00FF;

	h_Y_start = (y0 & 0xFF00) >> 8;
	l_Y_start = y0 & 0x00FF;
	h_Y_end = (y1 & 0xFF00) >> 8;
	l_Y_end = y1 & 0x00FF;

	send_ctrl_cmd(0x2A);
	send_data_cmd(h_X_start);
	send_data_cmd(l_X_start);
	send_data_cmd(h_X_end);
	send_data_cmd(l_X_end);

	send_ctrl_cmd(0x2B);
	send_data_cmd(h_Y_start);
	send_data_cmd(l_Y_start);
	send_data_cmd(h_Y_end);
	send_data_cmd(l_Y_end);

	send_ctrl_cmd(0x2C);
}



static void lcm_setbacklight(unsigned int level)
{
   if(level > 255) level = 255;

   send_ctrl_cmd(0x51);
   send_data_cmd(level);	
}


static unsigned int lcm_compare_id(void)
{	
    

   //return model;//(LCM_ID == model)?1:0;
   return 1;
}

LCM_DRIVER s6d04d2x01_lcm_drv = 
{
    .name = "st7789h2_dbi",
   .set_util_funcs = lcm_set_util_funcs,
   .get_params     = lcm_get_params,
   .init           = lcm_init,
   .suspend        = lcm_suspend,
   .resume         = lcm_resume,
   .update         = lcm_update,
  // .set_backlight	= lcm_setbacklight,
   .compare_id     = lcm_compare_id,
};

