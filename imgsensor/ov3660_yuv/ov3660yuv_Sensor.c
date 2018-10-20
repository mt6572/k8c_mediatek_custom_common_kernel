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
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
//#include <windows.h>
//#include <memory.h>
//#include <nkintr.h>
//#include <ceddk.h>
//#include <ceddk_exp.h>

//#include "kal_release.h"
//#include "i2c_exp.h"
//#include "gpio_exp.h"
//#include "msdk_exp.h"
//#include "msdk_sensor_exp.h"
//#include "msdk_isp_exp.h"
//#include "base_regs.h"
//#include "Sensor.h"
//#include "camera_sensor_para.h"
//#include "CameraCustomized.h"

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ov3660yuv_Sensor.h"
#include "ov3660yuv_Camera_Sensor_para.h"
#include "ov3660yuv_CameraCustomized.h"

#define OV3660YUV_DEBUG
#ifdef OV3660YUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define OV3660_TEST_PATTERN_CHECKSUM (0x7ba87eae)
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
kal_uint16 OV3660_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	  char puSendCmd[3] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)}; 	  
	  iWriteRegI2C(puSendCmd , 3,OV3660_WRITE_ID);
}
kal_uint16 OV3660_read_cmos_sensor(kal_uint32 addr)
{
	  kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	  iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,OV3660_WRITE_ID);
    return ((get_byte&0x00ff));
}
static int sensor_id_fail = 0; 
static int temp_count = 0; 
//#define OV3660_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV3660_WRITE_ID)
//#define OV3660_write_cmos_sensor_2(addr, para, bytes) iWriteReg((u16) addr , (u32) para ,bytes,OV3660_WRITE_ID)
#if 0
kal_uint16 OV3660_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV3660_WRITE_ID);
    return get_byte;
}
#endif

/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT

/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/

#define	OV3660_LIMIT_EXPOSURE_LINES				         (1253)
#define	OV3660_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	OV3660_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	OV3660_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 

static kal_uint8 OV3660_exposure_line_h = 0, OV3660_exposure_line_l = 0,OV3660_extra_exposure_line_h = 0, OV3660_extra_exposure_line_l = 0;

static kal_bool OV3660_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool OV3660_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool OV3660_sensor_cap_state = KAL_FALSE; //Preview or Capture

static kal_uint16 OV3660_dummy_pixels=0, OV3660_dummy_lines=0;
kal_uint32 OV3660_FULL_dummy_pixels = 0;
kal_uint32 OV3660_FULL_dummy_lines = 0;


static kal_uint16 OV3660_exposure_lines=0, OV3660_extra_exposure_lines = 0;

static kal_int8 OV3660_DELAY_AFTER_PREVIEW = -1;

static kal_uint8 OV3660_Banding_setting = AE_FLICKER_MODE_50HZ;  //Wonder add

/****** OVT 6-18******/
static kal_uint16 OV3660_Capture_Max_Gain16= 6*16;
static kal_uint16 OV3660_Capture_Gain16=0 ;    
static kal_uint16 OV3660_Capture_Shutter=0;
static kal_uint16 OV3660_Capture_Extra_Lines=0;

static kal_uint16  OV3660_PV_Dummy_Pixels =0, OV3660_Capture_Dummy_Pixels =0, OV3660_Capture_Dummy_Lines =0;
static kal_uint16  OV3660_PV_Gain16 = 0;
static kal_uint16  OV3660_PV_Shutter = 0;
static kal_uint16  OV3660_PV_Extra_Lines = 0;

kal_uint16 OV3660_sensor_gain_base=0,OV3660_FAC_SENSOR_REG=0,OV3660_iOV3660_Mode=0,OV3660_max_exposure_lines=0;
kal_uint32 OV3660_capture_pclk_in_M=520,OV3660_preview_pclk_in_M=390,OV3660_PV_dummy_pixels=0,OV3660_PV_dummy_lines=0,OV3660_isp_master_clock=0;
static kal_uint32  OV3660_preview_pclk = 0, OV3660_capture_pclk = 0;
kal_bool OV3660_Night_mode = KAL_FALSE;
kal_bool OV3660_Y_Target_L = 64; 
kal_bool OV3660_Y_Target_H = 72; 

OV3660_OP_TYPE OV3660_g_iOV3660_Mode = OV3660_MODE_NONE;

static kal_uint32  OV3660_sensor_pclk=390;
kal_bool OV3660_VEDIO_MPEG4 = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4);


UINT8 OV3660_PixelClockDivider=0;

static kal_bool OV3660_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV3660_AE_ENABLE = KAL_TRUE; 

static kal_uint32 Capture_Shutter = 0; 
static kal_uint32 Capture_Gain = 0; 

int XVCLK = 2600;
// real clock/10000
int preview_sysclk, preview_HTS;
int AE_Target = 52;
int AE_high, AE_low;
int m_iCombo_NightMode = 0;


MSDK_SENSOR_CONFIG_STRUCT OV3660SensorConfigData;

void OV3660_set_dummy(kal_uint16 pixels, kal_uint16 lines)
{
    kal_uint16 Total_Pixels,Total_Lines;

    if (OV3660_sensor_cap_state==KAL_FALSE) //1024x768
    {
    	  Total_Pixels = 0x08fc + pixels;
		    Total_Lines = 0x0310 + lines;
	  }
	  else //2048x1536
	  {
		    Total_Pixels = 0x08fc + pixels;
		    Total_Lines = 0x0620 + lines;
	  }

    OV3660_write_cmos_sensor(0x380d,(Total_Pixels&0xFF));         //VTS[7:0]
    OV3660_write_cmos_sensor(0x380c,((Total_Pixels&0xFF00)>>8));  //VTS[15:8]

    OV3660_write_cmos_sensor(0x380f,(Total_Lines&0xFF));         //VTS[7:0]
    OV3660_write_cmos_sensor(0x380e,((Total_Lines&0xFF00)>>8));  //VTS[15:8]
   
}    /* OV3660_set_dummy */

kal_uint16 OV3660_read_OV3660_gain(void)
{
	// read gain, 16 = 1x
	int gain16;
	gain16 = OV3660_read_cmos_sensor(0x350a) & 0x03;
	gain16 = (gain16<<8) + OV3660_read_cmos_sensor(0x350b);
	SENSORDB("[OV3660YUV] OV3660_read_OV3660_gain:  gain16 = %d\n", gain16);
	return gain16;
}  /* OV3660_read_OV3660_gain */


kal_uint16 OV3660_read_shutter(void)
{
	// read shutter, in number of line period
	int shutter;
	shutter = (OV3660_read_cmos_sensor(0x03500) & 0x0f);
	shutter = (shutter<<8) + OV3660_read_cmos_sensor(0x3501);
	shutter = (shutter<<4) + (OV3660_read_cmos_sensor(0x3502)>>4);
	SENSORDB("[OV3660YUV] OV3660_read_shutter:	shutter = %d\n", shutter);

	return shutter;
}    /* OV3660_read_shutter */

void OV3660_write_OV3660_gain(kal_uint16 gain16)
{    
	// write gain, 16 = 1x
	
	int temp;
	SENSORDB("[OV3660YUV] OV3660_write_OV3660_gain:  gain16 = %d\n", gain16);
	gain16 = gain16 & 0x3ff;
	temp = gain16 & 0xff;
	OV3660_write_cmos_sensor(0x350b, temp);
	temp = gain16>>8;
	OV3660_write_cmos_sensor(0x350a, temp);
	//return 0;
}  /* OV3660_write_OV3660_gain */

static void OV3660_write_shutter(kal_uint16 shutter)
{
	// write shutter, in number of line period
	int temp;
	SENSORDB("[OV3660YUV] OV3660_write_shutter:	shutter = %d\n", shutter);
	shutter = shutter&0xffff;
	temp = shutter&0x0f;
	temp = temp<<4;
	OV3660_write_cmos_sensor(0x3502, temp);
	temp = shutter&0xfff;
	temp = temp>>4;
	OV3660_write_cmos_sensor(0x3501, temp);
	temp = shutter>>12;
	OV3660_write_cmos_sensor(0x3500, temp);
	//return 0;

}    /* OV3660_write_shutter */
int OV3660_get_sysclk()
{
	// calculate sysclk
	bool pllbypass, PclkManual;
	int temp1, temp2;
	int Multiplier, SysDiv, PreDiv2x, RootDiv, Seld52x, PclkDiv, VCO, PLLCLK, sysclk, PCLK;
	int Pre_div2x_map[] = {
	2, 3, 4, 6};
	int Seld52x_map[] = {
	2, 2, 4, 5};
	temp1 = OV3660_read_cmos_sensor(0x303a);
	if (temp1 & 0x80) {
	pllbypass = true;
	}
	else {
	pllbypass = false;
	}
	temp1 = OV3660_read_cmos_sensor(0x303b);
	Multiplier = temp1 & 0x1f;
	temp1 = OV3660_read_cmos_sensor(0x303c);
	SysDiv = temp1 & 0x0f;
	temp1 = OV3660_read_cmos_sensor(0x303d);
	temp2 = (temp1>>4) & 0x03;
	PreDiv2x = Pre_div2x_map[temp2];
	if (temp1 & 0x40) {
	RootDiv = 2;
	}
	else {
	RootDiv = 1;
	}
	temp2 = temp1 & 0x03;
	Seld52x = Seld52x_map[temp2];
	temp1 = OV3660_read_cmos_sensor(0x3824);
	PclkDiv = temp1 & 0x1f;
	temp1 = OV3660_read_cmos_sensor(0x460c);
	if (temp1 & 0x02) {
		PclkManual = true;
	}
	else {
		PclkManual = false;
	}
	VCO = XVCLK * Multiplier * RootDiv * 2 / PreDiv2x;
	if (pllbypass) {
		PLLCLK = XVCLK;
	}
	else {
		PLLCLK = VCO * 2 / SysDiv / Seld52x;
	}
	sysclk = PLLCLK / 4;
	if (PclkManual) {
		PCLK = PLLCLK / 2 / PclkDiv;
	}
	else {
		PCLK = PLLCLK / 2;
	}
	SENSORDB("[OV3660YUV] OV3660_get_sysclk:	sysclk = %d\n", sysclk);
	return sysclk;
}

int OV3660_get_HTS()
{
	// read HTS from register settings
	int HTS;
	HTS = OV3660_read_cmos_sensor(0x380c);
	HTS = (HTS<<8) + OV3660_read_cmos_sensor(0x380d);
	SENSORDB("[OV3660YUV] OV3660_get_HTS:	HTS = %d\n", HTS);
	return HTS;
}

int OV3660_get_VTS()
{
	// read VTS from register settings
	int VTS;
	VTS = OV3660_read_cmos_sensor(0x380e);
	VTS = (VTS<<8) + OV3660_read_cmos_sensor(0x380f);
	
	SENSORDB("[OV3660YUV] OV3660_get_VTS:	VTS = %d\n", VTS);
	return VTS;
}

int OV3660_set_VTS(int VTS)
{
	// write VTS to registers
	int temp;
	SENSORDB("[OV3660YUV] OV3660_set_VTS:	VTS = %d\n", VTS);
	temp = VTS & 0xff;
	OV3660_write_cmos_sensor(0x380f, temp);
	temp = VTS>>8;
	OV3660_write_cmos_sensor(0x380e, temp);
	return 0;
}


int OV3660_get_light_frequency()
{
	// get banding filter value
	int temp, temp1, light_frequency;
	temp = OV3660_read_cmos_sensor(0x3c01);
	if (temp & 0x80) 
	{
		// manual
		temp1 = OV3660_read_cmos_sensor(0x3c00);
		if (temp1 & 0x04) {
		// 50Hz
			light_frequency = 50;
		}
		else {
		// 60Hz
			light_frequency = 60;
		}
	}
	else {
		// auto
		temp1 = OV3660_read_cmos_sensor(0x3c0c);
		if (temp1 & 0x01) {
			// 50Hz
			light_frequency = 50;
		}
		else {
			// 60Hz
			light_frequency = 60;
		}
	}
	
	SENSORDB("[OV3660YUV] OV3660_get_light_frequency:	light_frequency = %d\n", light_frequency);
	return light_frequency;
}

void OV3660_set_bandingfilter()
{
	int preview_VTS;
	int band_step60, max_band60, band_step50, max_band50;
	
	SENSORDB("[OV3660YUV] OV3660_set_bandingfilter:\n");
	// read preview PCLK
	preview_sysclk = OV3660_get_sysclk();
	// read preview HTS
	preview_HTS = OV3660_get_HTS();
	// read preview VTS
	preview_VTS = OV3660_get_VTS();
	// calculate banding filter
	// 60Hz
	band_step60 = preview_sysclk * 100/preview_HTS * 100/120;
	OV3660_write_cmos_sensor(0x3a0a, (band_step60 >> 8));
	OV3660_write_cmos_sensor(0x3a0b, (band_step60 & 0xff));
	max_band60 = ((preview_VTS-4)/band_step60);
	OV3660_write_cmos_sensor(0x3a0d, max_band60);
	// 50Hz
	band_step50 = preview_sysclk * 100/preview_HTS;
	OV3660_write_cmos_sensor(0x3a08, (band_step50 >> 8));
	OV3660_write_cmos_sensor(0x3a09, (band_step50 & 0xff));
	max_band50 = ((preview_VTS-4)/band_step50);
	OV3660_write_cmos_sensor(0x3a0e, max_band50);
}
int OV3660_set_AE_target(int target)
{
	// stable in high
	int fast_high, fast_low;
	
	SENSORDB("[OV3660YUV] OV3660_set_AE_target:	target = %d\n", target);
	AE_low = target * 23 / 25; // 0.92
	AE_high = target * 27 / 25; // 1.08
	fast_high = AE_high<<1;
	if(fast_high>255)
	fast_high = 255;
	fast_low = AE_low>>1;
	OV3660_write_cmos_sensor(0x3a0f, AE_high);
	OV3660_write_cmos_sensor(0x3a10, AE_low);
	OV3660_write_cmos_sensor(0x3a1b, AE_high);
	OV3660_write_cmos_sensor(0x3a1e, AE_low);
	OV3660_write_cmos_sensor(0x3a11, fast_high);
	OV3660_write_cmos_sensor(0x3a1f, fast_low);
	return 0;
}

void OV3660_stream_on()
{	
	SENSORDB("[OV3660YUV] OV3660_stream_on:\n");
	OV3660_write_cmos_sensor(0x4202, 0x00);
}
void OV3660_stream_off()
{	
	SENSORDB("[OV3660YUV] OV3660_stream_off:\n");
	OV3660_write_cmos_sensor(0x4202, 0x0f);
}

void OV3660_Computer_AECAGC(kal_uint16 preview_clk_in_M, kal_uint16 capture_clk_in_M)
{
    kal_uint16 PV_Line_Width;
    kal_uint16 Capture_Line_Width;
    kal_uint16 Capture_Maximum_Shutter;
    kal_uint16 Capture_Exposure;
    kal_uint16 Capture_Gain16;
    kal_uint32 Capture_Banding_Filter;
    kal_uint32 Gain_Exposure=0;
	  SENSORDB("[OV3660YUV] OV3660_Computer_AECAGC:\n");

    PV_Line_Width = OV3660_PV_PERIOD_PIXEL_NUMS + OV3660_PV_Dummy_Pixels;   

    Capture_Line_Width = OV3660_FULL_PERIOD_PIXEL_NUMS + OV3660_Capture_Dummy_Pixels;
    Capture_Maximum_Shutter = OV3660_FULL_EXPOSURE_LIMITATION + OV3660_Capture_Dummy_Lines;

    if (OV3660_Banding_setting == AE_FLICKER_MODE_50HZ)
        Capture_Banding_Filter = (kal_uint32)(capture_clk_in_M*100000/100/(2*Capture_Line_Width));
    else
        Capture_Banding_Filter = (kal_uint32)(capture_clk_in_M*100000/120/(2*Capture_Line_Width) );

    /*   Gain_Exposure = OV3660_PV_Gain16*(OV3660_PV_Shutter+OV3660_PV_Extra_Lines)*PV_Line_Width/g_Preview_PCLK_Frequency/Capture_Line_Width*g_Capture_PCLK_Frequency
    ;*/
    OV3660_PV_Gain16 = OV3660_read_OV3660_gain();
    Gain_Exposure = 1 * OV3660_PV_Gain16;  //For OV3660
    ///////////////////////
    Gain_Exposure *=(OV3660_PV_Shutter+OV3660_PV_Extra_Lines);
    Gain_Exposure *=PV_Line_Width;  //970
    //   Gain_Exposure /=g_Preview_PCLK_Frequency;
    Gain_Exposure /=Capture_Line_Width;//1940
    Gain_Exposure = Gain_Exposure*capture_clk_in_M/preview_clk_in_M;// for clock   

    //redistribute gain and exposure
    if (Gain_Exposure < (kal_uint32)(Capture_Banding_Filter * 16))     // Exposure < 1/100/120
    {
       if(Gain_Exposure<16){//exposure line smaller than 2 lines and gain smaller than 0x08 
            Gain_Exposure = Gain_Exposure*4;     
            Capture_Exposure = 1;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2/4;
        }
        else
        {
            Capture_Exposure = Gain_Exposure /16;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
        }
    }
    else 
    {
        if (Gain_Exposure >(kal_uint32)( Capture_Maximum_Shutter * 16)) // Exposure > Capture_Maximum_Shutter
        {
           
            Capture_Exposure = Capture_Maximum_Shutter/Capture_Banding_Filter*Capture_Banding_Filter;
            Capture_Gain16 = (Gain_Exposure*2 + 1)/Capture_Exposure/2;
            if (Capture_Gain16 > OV3660_Capture_Max_Gain16) 
            {
                // gain reach maximum, insert extra line
                Capture_Exposure = (kal_uint16)(Gain_Exposure*11 /10 /OV3660_Capture_Max_Gain16);
                
                // Exposure = n/100/120
                Capture_Exposure = Capture_Exposure/Capture_Banding_Filter * Capture_Banding_Filter;
                Capture_Gain16 = ((Gain_Exposure *4)/ Capture_Exposure+3)/4;
            }
        }
        else  // 1/100 < Exposure < Capture_Maximum_Shutter, Exposure = n/100/120
        {
            Capture_Exposure = Gain_Exposure/16/Capture_Banding_Filter;
            Capture_Exposure = Capture_Exposure * Capture_Banding_Filter;
            Capture_Gain16 = (Gain_Exposure*2 +1) / Capture_Exposure/2;
        }
    }
    
       OV3660_Capture_Gain16 = Capture_Gain16;
       OV3660_Capture_Extra_Lines = (Capture_Exposure > Capture_Maximum_Shutter)?
       (Capture_Exposure - Capture_Maximum_Shutter/Capture_Banding_Filter*Capture_Banding_Filter):0;     
    
       OV3660_Capture_Shutter = Capture_Exposure - OV3660_Capture_Extra_Lines;
}




/*************************************************************************
* FUNCTION
*	OV3660_NightMode
*
* DESCRIPTION
*	This function night mode of OV3660.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void OV3660_night_mode(kal_bool enable)
{
	    OV3660_write_cmos_sensor(0x3a00,0x38);
		  OV3660_write_cmos_sensor(0x3a02,0x03);
		  OV3660_write_cmos_sensor(0x3a03,0x10);
		  OV3660_write_cmos_sensor(0x3a14,0x03);
		  OV3660_write_cmos_sensor(0x3a15,0x10);		 
		  mDELAY(50);
		  
	if (enable)
	{
		  OV3660_write_cmos_sensor(0x3212,0x03);					
		  //night mode
	    OV3660_write_cmos_sensor(0x3a00,0x3c);
		  OV3660_write_cmos_sensor(0x3a02,0x12);//decrease 1/6
		  OV3660_write_cmos_sensor(0x3a03,0x60);
		  OV3660_write_cmos_sensor(0x3a14,0x12);
		  OV3660_write_cmos_sensor(0x3a15,0x60);
		 
		  OV3660_write_cmos_sensor(0x3212,0x13);	
		  OV3660_write_cmos_sensor(0x3212,0xa3);	
	}
	else 
	{
		  OV3660_write_cmos_sensor(0x3212,0x03);			 
	    //normal mode
	    OV3660_write_cmos_sensor(0x3a00,0x3c);
		  OV3660_write_cmos_sensor(0x3a02,0x09);
		  OV3660_write_cmos_sensor(0x3a03,0x30);
		  OV3660_write_cmos_sensor(0x3a14,0x09);
		  OV3660_write_cmos_sensor(0x3a15,0x30);
		
		  OV3660_write_cmos_sensor(0x3212,0x13);	
		  OV3660_write_cmos_sensor(0x3212,0xa3);	
	}

}	/* OV3660_NightMode */
/* Register setting from capture to preview. */
static void OV3660_set_SVGA_mode(void)
{    /*1024x768*/
	   OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)&0xdf);//bit[2]='0'
	   
	   OV3660_write_cmos_sensor(0x3800,0x00);
     OV3660_write_cmos_sensor(0x3801,0x00);
     OV3660_write_cmos_sensor(0x3802,0x00);
     OV3660_write_cmos_sensor(0x3803,0x00);
     OV3660_write_cmos_sensor(0x3804,0x08);
     OV3660_write_cmos_sensor(0x3805,0x1f);
     OV3660_write_cmos_sensor(0x3806,0x06);
     OV3660_write_cmos_sensor(0x3807,0x09);
     OV3660_write_cmos_sensor(0x3808,0x04);//1024
     OV3660_write_cmos_sensor(0x3809,0x00);
     OV3660_write_cmos_sensor(0x380a,0x03);//768
     OV3660_write_cmos_sensor(0x380b,0x00);
     OV3660_write_cmos_sensor(0x3810,0x00);
     OV3660_write_cmos_sensor(0x3811,0x08);
     OV3660_write_cmos_sensor(0x3812,0x00);
     OV3660_write_cmos_sensor(0x3813,0x02);
     OV3660_write_cmos_sensor(0x3814,0x31);
     OV3660_write_cmos_sensor(0x3815,0x31);
     
      //modify by FAE 20150713
     /*OV3660_write_cmos_sensor(0x3826,0x23);
     OV3660_write_cmos_sensor(0x303a,0x00);
     OV3660_write_cmos_sensor(0x303b,0x19);
     OV3660_write_cmos_sensor(0x303c,0x11);//30fps
     OV3660_write_cmos_sensor(0x303d,0x30);
     OV3660_write_cmos_sensor(0x3824,0x02);
     OV3660_write_cmos_sensor(0x460c,0x22);*/
     OV3660_write_cmos_sensor(0x3826,0x23);
     OV3660_write_cmos_sensor(0x303a,0x00);
     OV3660_write_cmos_sensor(0x303b,0x1b);// 0x19
     OV3660_write_cmos_sensor(0x303c,0x11);//30fps 0x11 /0x12 modify by FAE 20150714  
     OV3660_write_cmos_sensor(0x303d,0x30);
     OV3660_write_cmos_sensor(0x3824,0x01);//0x02
     OV3660_write_cmos_sensor(0x460c,0x22);
     
     OV3660_write_cmos_sensor(0x380c,0x08);
     OV3660_write_cmos_sensor(0x380d,0xfc);
     OV3660_write_cmos_sensor(0x380e,0x03);
     OV3660_write_cmos_sensor(0x380f,0x10);
     
     OV3660_write_cmos_sensor(0x3a08,0x00);//50hz
     OV3660_write_cmos_sensor(0x3a09,0xeb);
     OV3660_write_cmos_sensor(0x3a0e,0x03);
     OV3660_write_cmos_sensor(0x3a0a,0x00);//60hz
     OV3660_write_cmos_sensor(0x3a0b,0xc4);
     OV3660_write_cmos_sensor(0x3a0d,0x04);	   
  
     OV3660_write_cmos_sensor(0x5302,0x30);//28
     OV3660_write_cmos_sensor(0x5303,0x20);//18
     OV3660_write_cmos_sensor(0x5306,0x18);
     OV3660_write_cmos_sensor(0x5307,0x28);     
     OV3660_write_cmos_sensor(0x3820,0x01);
     OV3660_write_cmos_sensor(0x3821,0x07);
     OV3660_write_cmos_sensor(0x4514,0xbb);
     OV3660_write_cmos_sensor(0x3618,0x00);
     OV3660_write_cmos_sensor(0x3708,0x66);
     OV3660_write_cmos_sensor(0x3709,0x12);
     OV3660_write_cmos_sensor(0x4520,0x0b);   
														
}
static void OV3660_YUV_sensor_initial_setting(void)
{	
	   OV3660_write_cmos_sensor(0x3103,0x13);
     OV3660_write_cmos_sensor(0x3008,0x42);
	
	   OV3660_write_cmos_sensor(0x3017,0xff);
     OV3660_write_cmos_sensor(0x3018,0xff);
     OV3660_write_cmos_sensor(0x302c,0x03);//83
     
     OV3660_write_cmos_sensor(0x3031,0x00);
     
     OV3660_write_cmos_sensor(0x3611,0x01);
     OV3660_write_cmos_sensor(0x3612,0x2d);
     
     OV3660_write_cmos_sensor(0x3032,0x00);
     OV3660_write_cmos_sensor(0x3614,0x80);
     OV3660_write_cmos_sensor(0x3618,0x00);
     OV3660_write_cmos_sensor(0x3619,0x75);
     OV3660_write_cmos_sensor(0x3622,0x80);
     OV3660_write_cmos_sensor(0x3623,0x00);
     OV3660_write_cmos_sensor(0x3624,0x03);
     OV3660_write_cmos_sensor(0x3630,0x52);
     OV3660_write_cmos_sensor(0x3632,0x07);
     OV3660_write_cmos_sensor(0x3633,0xd2);
     OV3660_write_cmos_sensor(0x3704,0x80);
     OV3660_write_cmos_sensor(0x3708,0x66);
     OV3660_write_cmos_sensor(0x3709,0x12);
     OV3660_write_cmos_sensor(0x370b,0x12);
     OV3660_write_cmos_sensor(0x3717,0x00);
     OV3660_write_cmos_sensor(0x371b,0x60);
     OV3660_write_cmos_sensor(0x371c,0x00);
     OV3660_write_cmos_sensor(0x3901,0x13);

     OV3660_write_cmos_sensor(0x3600,0x08);
     OV3660_write_cmos_sensor(0x3620,0x43);
     OV3660_write_cmos_sensor(0x3702,0x20);
     OV3660_write_cmos_sensor(0x3739,0x48);
     OV3660_write_cmos_sensor(0x3730,0x20);
     OV3660_write_cmos_sensor(0x370c,0x0c);
     
     OV3660_write_cmos_sensor(0x3a18,0x00);
     OV3660_write_cmos_sensor(0x3a19,0xf8);
     
     OV3660_write_cmos_sensor(0x3000,0x10);
     OV3660_write_cmos_sensor(0x3002,0x1c);
     OV3660_write_cmos_sensor(0x3004,0xef);
     OV3660_write_cmos_sensor(0x3006,0xc3);
     
     OV3660_write_cmos_sensor(0x6700,0x05);
     OV3660_write_cmos_sensor(0x6701,0x19);
     OV3660_write_cmos_sensor(0x6702,0xfd);
     OV3660_write_cmos_sensor(0x6703,0xd1);
     OV3660_write_cmos_sensor(0x6704,0xff);
     OV3660_write_cmos_sensor(0x6705,0xff);

     OV3660_write_cmos_sensor(0x3800,0x00);
     OV3660_write_cmos_sensor(0x3801,0x00);
     OV3660_write_cmos_sensor(0x3802,0x00);
     OV3660_write_cmos_sensor(0x3803,0x00);
     OV3660_write_cmos_sensor(0x3804,0x08);
     OV3660_write_cmos_sensor(0x3805,0x1f);
     OV3660_write_cmos_sensor(0x3806,0x06);
     OV3660_write_cmos_sensor(0x3807,0x09);
     OV3660_write_cmos_sensor(0x3808,0x04);
     OV3660_write_cmos_sensor(0x3809,0x00);
     OV3660_write_cmos_sensor(0x380a,0x03);
     OV3660_write_cmos_sensor(0x380b,0x00);
     OV3660_write_cmos_sensor(0x3810,0x00);
     OV3660_write_cmos_sensor(0x3811,0x08);
     OV3660_write_cmos_sensor(0x3812,0x00);
     OV3660_write_cmos_sensor(0x3813,0x02);
     OV3660_write_cmos_sensor(0x3814,0x31);
     OV3660_write_cmos_sensor(0x3815,0x31);
     
     OV3660_write_cmos_sensor(0x3826,0x23);
     OV3660_write_cmos_sensor(0x303a,0x00);
     OV3660_write_cmos_sensor(0x303b,0x1b);// 0x19   //modify by FAE 20150713
     OV3660_write_cmos_sensor(0x303c,0x11);//30fps 0x11   //modify by FAE 20150713 0x12 modify by FAE 20150714
     OV3660_write_cmos_sensor(0x303d,0x30);
     OV3660_write_cmos_sensor(0x3824,0x01);//0x02   //modify by FAE 20150713
     OV3660_write_cmos_sensor(0x460c,0x22);
     
     
     OV3660_write_cmos_sensor(0x380c,0x08);
     OV3660_write_cmos_sensor(0x380d,0xfc);
     OV3660_write_cmos_sensor(0x380e,0x03);
     OV3660_write_cmos_sensor(0x380f,0x10);//51
     
     OV3660_write_cmos_sensor(0x3a08,0x00);//50hz
     OV3660_write_cmos_sensor(0x3a09,0xeb);
     OV3660_write_cmos_sensor(0x3a0e,0x03);
     OV3660_write_cmos_sensor(0x3a0a,0x00);//60hz
     OV3660_write_cmos_sensor(0x3a0b,0xc4);
     OV3660_write_cmos_sensor(0x3a0d,0x04);	   
     
     OV3660_write_cmos_sensor(0x3a00,0x3c);
     OV3660_write_cmos_sensor(0x3a14,0x09);
     OV3660_write_cmos_sensor(0x3a15,0x30);
     OV3660_write_cmos_sensor(0x3a02,0x09);
     OV3660_write_cmos_sensor(0x3a03,0x30);

     OV3660_write_cmos_sensor(0x3c00,0x04);		
     OV3660_write_cmos_sensor(0x3c01,0x80);	
     
     OV3660_write_cmos_sensor(0x4300,0x30);//0x32
     OV3660_write_cmos_sensor(0x440e,0x08);
     OV3660_write_cmos_sensor(0x4520,0x0b);
     OV3660_write_cmos_sensor(0x460b,0x37);
     OV3660_write_cmos_sensor(0x4713,0x02);
     OV3660_write_cmos_sensor(0x471c,0xd0);
     OV3660_write_cmos_sensor(0x5086,0x00);
     
     OV3660_write_cmos_sensor(0x5001,0x03);
     OV3660_write_cmos_sensor(0x5002,0x00);
     OV3660_write_cmos_sensor(0x501f,0x00);
     
     OV3660_write_cmos_sensor(0x3820,0x01);
     OV3660_write_cmos_sensor(0x3821,0x07);
     OV3660_write_cmos_sensor(0x4514,0xbb);
     OV3660_write_cmos_sensor(0x3008,0x02);
     
     OV3660_write_cmos_sensor(0x5180,0xff);
     OV3660_write_cmos_sensor(0x5181,0xf2);
     OV3660_write_cmos_sensor(0x5182,0x00); 
     OV3660_write_cmos_sensor(0x5183,0x14);
     OV3660_write_cmos_sensor(0x5184,0x25);
     OV3660_write_cmos_sensor(0x5185,0x24);
     OV3660_write_cmos_sensor(0x5186,0x16);
     OV3660_write_cmos_sensor(0x5187,0x16); 
     OV3660_write_cmos_sensor(0x5188,0x16); 
     OV3660_write_cmos_sensor(0x5189,0x68);
     OV3660_write_cmos_sensor(0x518a,0x60);
     OV3660_write_cmos_sensor(0x518b,0xe0);
     OV3660_write_cmos_sensor(0x518c,0xb2);
     OV3660_write_cmos_sensor(0x518d,0x42);
     OV3660_write_cmos_sensor(0x518e,0x35);
     OV3660_write_cmos_sensor(0x518f,0x56);
     OV3660_write_cmos_sensor(0x5190,0x56);
     OV3660_write_cmos_sensor(0x5191,0xf8);
     OV3660_write_cmos_sensor(0x5192,0x04); 
     OV3660_write_cmos_sensor(0x5193,0x70);
     OV3660_write_cmos_sensor(0x5194,0xf0);
     OV3660_write_cmos_sensor(0x5195,0xf0);
     OV3660_write_cmos_sensor(0x5196,0x03); 
     OV3660_write_cmos_sensor(0x5197,0x01); 
     OV3660_write_cmos_sensor(0x5198,0x04); 
     OV3660_write_cmos_sensor(0x5199,0x12);
     OV3660_write_cmos_sensor(0x519a,0x04); 
     OV3660_write_cmos_sensor(0x519b,0x00); 
     OV3660_write_cmos_sensor(0x519c,0x06); 
     OV3660_write_cmos_sensor(0x519d,0x82);
     OV3660_write_cmos_sensor(0x519e,0x38);
     
     OV3660_write_cmos_sensor(0x5381,0x1c);  
     OV3660_write_cmos_sensor(0x5382,0x5a);
     OV3660_write_cmos_sensor(0x5383,0x10); 
     OV3660_write_cmos_sensor(0x5384,0x09); 
     OV3660_write_cmos_sensor(0x5385,0x70);
     OV3660_write_cmos_sensor(0x5386,0x79);
     OV3660_write_cmos_sensor(0x5387,0x82);
     OV3660_write_cmos_sensor(0x5388,0x6e);
     OV3660_write_cmos_sensor(0x5389,0x14);
     OV3660_write_cmos_sensor(0x538a,0x01); 
     OV3660_write_cmos_sensor(0x538b,0x98);     

     OV3660_write_cmos_sensor(0x5481,0x06); 
     OV3660_write_cmos_sensor(0x5482,0x0e);
     OV3660_write_cmos_sensor(0x5483,0x1e);
     OV3660_write_cmos_sensor(0x5484,0x42);
     OV3660_write_cmos_sensor(0x5485,0x54);
     OV3660_write_cmos_sensor(0x5486,0x68);
     OV3660_write_cmos_sensor(0x5487,0x76);
     OV3660_write_cmos_sensor(0x5488,0x82);
     OV3660_write_cmos_sensor(0x5489,0x8e);
     OV3660_write_cmos_sensor(0x548a,0x9b);
     OV3660_write_cmos_sensor(0x548b,0xac);
     OV3660_write_cmos_sensor(0x548c,0xba);
     OV3660_write_cmos_sensor(0x548d,0xd0);
     OV3660_write_cmos_sensor(0x548e,0xe3);
     OV3660_write_cmos_sensor(0x548f,0xf6);
     OV3660_write_cmos_sensor(0x5490,0x11);
     
     OV3660_write_cmos_sensor(0x5000,0xa7);  
     OV3660_write_cmos_sensor(0x5800,0x27);  
     OV3660_write_cmos_sensor(0x5801,0x15);  
     OV3660_write_cmos_sensor(0x5802,0x14);  
     OV3660_write_cmos_sensor(0x5803,0x14);  
     OV3660_write_cmos_sensor(0x5804,0x18);  
     OV3660_write_cmos_sensor(0x5805,0x1C);  
     OV3660_write_cmos_sensor(0x5806,0x0E);  
     OV3660_write_cmos_sensor(0x5807,0x08);  
     OV3660_write_cmos_sensor(0x5808,0x05);  
     OV3660_write_cmos_sensor(0x5809,0x06);  
     OV3660_write_cmos_sensor(0x580A,0x0A);  
     OV3660_write_cmos_sensor(0x580B,0x13);  
     OV3660_write_cmos_sensor(0x580C,0x08);  
     OV3660_write_cmos_sensor(0x580D,0x02);  
     OV3660_write_cmos_sensor(0x580E,0x00);  
     OV3660_write_cmos_sensor(0x580F,0x00);  
     OV3660_write_cmos_sensor(0x5810,0x04);  
     OV3660_write_cmos_sensor(0x5811,0x0A);  
     OV3660_write_cmos_sensor(0x5812,0x08);  
     OV3660_write_cmos_sensor(0x5813,0x02);  
     OV3660_write_cmos_sensor(0x5814,0x00);  
     OV3660_write_cmos_sensor(0x5815,0x01);  
     OV3660_write_cmos_sensor(0x5816,0x04);  
     OV3660_write_cmos_sensor(0x5817,0x0B);  
     OV3660_write_cmos_sensor(0x5818,0x0E);  
     OV3660_write_cmos_sensor(0x5819,0x09);  
     OV3660_write_cmos_sensor(0x581A,0x05);  
     OV3660_write_cmos_sensor(0x581B,0x06);  
     OV3660_write_cmos_sensor(0x581C,0x0A);  
     OV3660_write_cmos_sensor(0x581D,0x12);  
     OV3660_write_cmos_sensor(0x581E,0x1B);  
     OV3660_write_cmos_sensor(0x581F,0x15);  
     OV3660_write_cmos_sensor(0x5820,0x15);  
     OV3660_write_cmos_sensor(0x5821,0x15);  
     OV3660_write_cmos_sensor(0x5822,0x19);  
     OV3660_write_cmos_sensor(0x5823,0x1E);  
     OV3660_write_cmos_sensor(0x5824,0x32);  
     OV3660_write_cmos_sensor(0x5825,0x24);  
     OV3660_write_cmos_sensor(0x5826,0x15);  
     OV3660_write_cmos_sensor(0x5827,0x24);  
     OV3660_write_cmos_sensor(0x5828,0x32);  
     OV3660_write_cmos_sensor(0x5829,0x16);  
     OV3660_write_cmos_sensor(0x582A,0x23);  
     OV3660_write_cmos_sensor(0x582B,0x22);  
     OV3660_write_cmos_sensor(0x582C,0x33);  
     OV3660_write_cmos_sensor(0x582D,0x25);  
     OV3660_write_cmos_sensor(0x582E,0x14);  
     OV3660_write_cmos_sensor(0x582F,0x31);  
     OV3660_write_cmos_sensor(0x5830,0x50);  
     OV3660_write_cmos_sensor(0x5831,0x41);  
     OV3660_write_cmos_sensor(0x5832,0x03);  
     OV3660_write_cmos_sensor(0x5833,0x16);  
     OV3660_write_cmos_sensor(0x5834,0x23);  
     OV3660_write_cmos_sensor(0x5835,0x32);  
     OV3660_write_cmos_sensor(0x5836,0x23);  
     OV3660_write_cmos_sensor(0x5837,0x15);  
     OV3660_write_cmos_sensor(0x5838,0x33);  
     OV3660_write_cmos_sensor(0x5839,0x25);  
     OV3660_write_cmos_sensor(0x583A,0x15);  
     OV3660_write_cmos_sensor(0x583B,0x24);  
     OV3660_write_cmos_sensor(0x583C,0x43);  
     OV3660_write_cmos_sensor(0x583D,0xCF);  
     
     OV3660_write_cmos_sensor(0x3a0f,0x38);//40
     OV3660_write_cmos_sensor(0x3a10,0x30);//38
     OV3660_write_cmos_sensor(0x3a1b,0x38);//40
     OV3660_write_cmos_sensor(0x3a1e,0x30);//38
     OV3660_write_cmos_sensor(0x3a11,0x70);
     OV3660_write_cmos_sensor(0x3a1f,0x14);
     
     OV3660_write_cmos_sensor(0x5302,0x30);//28
     OV3660_write_cmos_sensor(0x5303,0x20);//18
     OV3660_write_cmos_sensor(0x5306,0x18);
     OV3660_write_cmos_sensor(0x5307,0x28);
     
     OV3660_write_cmos_sensor(0x4002,0xc5);
     OV3660_write_cmos_sensor(0x4003,0x81);
     OV3660_write_cmos_sensor(0x4005,0x12);
     
     OV3660_write_cmos_sensor(0x5688,0x11);
     OV3660_write_cmos_sensor(0x5689,0x11);
     OV3660_write_cmos_sensor(0x568a,0x11);
     OV3660_write_cmos_sensor(0x568b,0x11);
     OV3660_write_cmos_sensor(0x568c,0x11);
     OV3660_write_cmos_sensor(0x568d,0x11);
     OV3660_write_cmos_sensor(0x568e,0x11);
     OV3660_write_cmos_sensor(0x568f,0x11);
     
     OV3660_write_cmos_sensor(0x5001,0x83);
     OV3660_write_cmos_sensor(0x5580,0x06);
     OV3660_write_cmos_sensor(0x5588,0x00);
     OV3660_write_cmos_sensor(0x5583,0x40);
     OV3660_write_cmos_sensor(0x5584,0x2c);      

} /* OV3660_YUV_sensor_initial_setting */

static void OV3660_set_AE_mode(kal_bool AE_enable)
{
	kal_uint8 temp_AE_reg = 0;	
	if (AE_enable == 0)
	{
		// turn off AEC/AGC
		temp_AE_reg = OV3660_read_cmos_sensor(0x3503);
		OV3660_write_cmos_sensor(0x3503, temp_AE_reg|0x03);
	}
	else
	{
		// turn on AEC/AGC
		temp_AE_reg = OV3660_read_cmos_sensor(0x3503);
		OV3660_write_cmos_sensor(0x3503, temp_AE_reg&0xfc);
	}
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	OV3660Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV3660Open(void)
{
    volatile signed char i;
    kal_uint16 sensor_id=0;
    sensor_id_fail = 0;        
	 
    OV3660_write_cmos_sensor(0x3008,0x82);// Reset sensor
    Sleep(10);

    zoom_factor = 0;     
    i = 3; 
    while (i > 0)
    {
        sensor_id = (OV3660_read_cmos_sensor(0x300A) << 8) | OV3660_read_cmos_sensor(0x300B);
        if (sensor_id == OV3660_SENSOR_ID)
        {
            break;
        }
        i --; 
    }
    if(sensor_id != OV3660_SENSOR_ID)
    {
        printk("[OV3660YUV]:Read Sensor ID fail:0x%x\n", sensor_id); 	    
        sensor_id_fail = 1; 
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    /*9. Apply sensor initail setting*/
    OV3660_YUV_sensor_initial_setting();
    OV3660_set_AE_target(AE_Target);
     
    return ERROR_NONE;
}	/* OV3660Open() */

UINT32 OV3660GetSensorID(UINT32 *sensorID) 
{
	volatile signed char i;
	SENSORDB("[OV3660YUV] OV3660GetSensorID: \n");
	
	OV3660_write_cmos_sensor(0x3008,0x82);// Reset sensor
	Sleep(10);
	
	i = 3; 
	while (i > 0)
	{
		*sensorID = (OV3660_read_cmos_sensor(0x300A) << 8) | OV3660_read_cmos_sensor(0x300B);
		if (*sensorID == OV3660_SENSOR_ID)
		{
			break;
		}
		printk("[OV3640YUV]:Error Sensor ID:0x%x\n", *sensorID); 
		i --; 
	}

	if(*sensorID != OV3660_SENSOR_ID)
	{
		printk("[OV3640YUV]:Read Sensor ID fail:0x%x\n", *sensorID);		
		return ERROR_SENSOR_CONNECT_FAIL;
	}

    return ERROR_NONE;
}
/*************************************************************************
* FUNCTION
*	OV3660Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV3660Close(void)
{
	SENSORDB("[OV3660YUV] OV3660Close: \n");

	return ERROR_NONE;
}	/* OV3660Close() */

static kal_uint16 OV3660_Reg2Gain(const kal_uint8 iReg)
{
    kal_uint8 iI;
    kal_uint16 iGain = BASEGAIN;     
   
    for (iI = 7; iI >= 4; iI--) {
        iGain *= (((iReg >> iI) & 0x01) + 1);
    }
    return iGain +  iGain * (iReg & 0x0F) / 16;
}
static kal_uint8 OV3660_Gain2Reg(const kal_uint16 iGain)
{
    kal_uint8 iReg = 0x00;

    if (iGain < 2 * BASEGAIN) {
        // Gain = 1 + GAIN[3:0](0x00) / 16
        iReg = 16 * (iGain - BASEGAIN) / BASEGAIN;
    }else if (iGain < 4 * BASEGAIN) {
        // Gain = 2 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x10;
        iReg |= 8 * (iGain - 2 * BASEGAIN) / BASEGAIN;
    }else if (iGain < 8 * BASEGAIN) {
        // Gain = 4 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x30;
        iReg |= 4 * (iGain - 4 * BASEGAIN) / BASEGAIN;
    }else if (iGain < 16 * BASEGAIN) {
        // Gain = 8 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0x70;
        iReg |= 2 * (iGain - 8 * BASEGAIN) / BASEGAIN;
    }else if (iGain < 32 * BASEGAIN) {
        // Gain = 16 * (1 + GAIN[3:0](0x00) / 16)
        iReg |= 0xF0;
        iReg |= (iGain - 16 * BASEGAIN) / BASEGAIN;
    }else {
        ASSERT(0);
    }
    return iReg;
}
kal_uint16 OV3660_write_gain(kal_uint16 gain)
{
    kal_uint16 temp_reg;
	
  	SENSORDB("[OV3660YUV] OV3660_write_gain: gain = %d\n", gain);
   
   if(gain > 248)  return ;//ASSERT(0);
   
    temp_reg = 0;
    if (gain > 31)
    {
        temp_reg |= 0x10;
        gain = gain >> 1;
    }
    if (gain > 31)
    {
        temp_reg |= 0x20;
        gain = gain >> 1;
    }

    if (gain > 31)
    {
        temp_reg |= 0x40;
        gain = gain >> 1;
    }
    if (gain > 31)
    {
        temp_reg |= 0x80;
        gain = gain >> 1;
    }
    
    if (gain > 16)
    {
        temp_reg |= ((gain -16) & 0x0f);
    }     
   OV3660_write_cmos_sensor(0x3001,temp_reg);
}

kal_uint16 OV3660_read_gain(void)
{
    kal_uint8  temp_reg;
    kal_uint16 sensor_gain;

    temp_reg=OV3660_read_cmos_sensor(0x3001);  


    sensor_gain=(16+(temp_reg&0x0F));
    if(temp_reg&0x10)
        sensor_gain<<=1;
    if(temp_reg&0x20)
        sensor_gain<<=1;
      
    if(temp_reg&0x40)
        sensor_gain<<=1;
      
    if(temp_reg&0x80)
        sensor_gain<<=1;
	  SENSORDB("[OV3660YUV] OV3660_read_gain: sensor_gain = %d\n", sensor_gain);
	       
    return sensor_gain;	
}  /* read_OV3660_gain */
static void OV3660_set_mirror_flip(kal_uint8 image_mirror, kal_uint16 *iStartX, kal_uint16 *iStartY)
{
	       kal_uint8 iTemp = 0;
	       kal_uint8 iTemp2 = 0;
	       
         SENSORDB("[OV3660YUV]:image_mirror:0x%x\n", image_mirror); 
    switch (image_mirror) {
    case IMAGE_NORMAL:    //mirror
	                    OV3660_write_cmos_sensor(0x3820, 0x01);
	                    OV3660_write_cmos_sensor(0x3821, 0x01);
	                    OV3660_write_cmos_sensor(0x4514, 0xaa);		// To mirror the Array, or there is horizontal lines when turn on mirror.
                      break;
	  case IMAGE_H_MIRROR:  //normal
	                    OV3660_write_cmos_sensor(0x3820, 0x01);
	                    OV3660_write_cmos_sensor(0x3821, 0x07);
	                    OV3660_write_cmos_sensor(0x4514, 0xbb);		// To mirror the Array, or there is horizontal lines when turn on mirror.
                      break;
	  case IMAGE_V_MIRROR:   //mirror & flip  //Flip Register 0x04[6] and 0x04[4] (FF = 01)
	                    OV3660_write_cmos_sensor(0x3820, 0x07);
	                    OV3660_write_cmos_sensor(0x3821, 0x01);
	                    OV3660_write_cmos_sensor(0x4514, 0xbb);		// To mirror the Array, or there is horizontal lines when turn on mirror.
                      break;
    case IMAGE_HV_MIRROR:   //flip
	                    OV3660_write_cmos_sensor(0x3820, 0x07);
	                    OV3660_write_cmos_sensor(0x3821, 0x07);
	                    OV3660_write_cmos_sensor(0x4514, 0xaa);		// To mirror the Array, or there is horizontal lines when turn on mirror.
                      break;
    default:
                      ASSERT(0);
    }
}

static void OV3660WBcalibattion(kal_uint32 color_r_gain,kal_uint32 color_b_gain)
{
		kal_uint32 color_r_gain_w = 0;
		kal_uint32 color_b_gain_w = 0;
		printk("[OV5645MIPI]enter OV5645WBcalibattion function:\n ");
		kal_uint8 temp = OV3660_read_cmos_sensor(0x350b); 
		
		if(temp>=0xb0)
		{	
			color_r_gain_w=color_r_gain*97/100;																																														
			color_b_gain_w=color_b_gain*99/100;  
		}
		else if (temp>=0x70)
		{
			color_r_gain_w=color_r_gain *97/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else if (temp>=0x30)
		{
			color_r_gain_w=color_r_gain*98/100;																																														
			color_b_gain_w=color_b_gain*99/100;
		}
		else
		{
			color_r_gain_w=color_r_gain*98/100;																																														
			color_b_gain_w=color_b_gain*99/100; 
		}																																																																						
		OV3660_write_cmos_sensor(0x3400,(color_r_gain_w & 0xff00)>>8);																																														
		OV3660_write_cmos_sensor(0x3401,color_r_gain_w & 0xff); 			
		OV3660_write_cmos_sensor(0x3404,(color_b_gain_w & 0xff00)>>8);																																														
		OV3660_write_cmos_sensor(0x3405,color_b_gain_w & 0xff); 
		
}	
/*************************************************************************
* FUNCTION
*	OV3660Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 OV3660Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	     //kal_uint8 iTemp = 0;
	     kal_uint16 iStartX = 0, iStartY = 0;
	     kal_uint16 	shutter = 0,pv_gain = 0;
	     SENSORDB("[OV3660YUV]:OV3660Preview enter"); 
       
       
	     OV3660_sensor_cap_state = KAL_FALSE;
	     OV3660_gPVmode = KAL_TRUE;
       
	     OV3660_stream_off();
       
	     /*Step1. set output size*/
	     OV3660_set_SVGA_mode();
       
	     OV3660_VEDIO_encode_mode = KAL_FALSE;
	     OV3660_VEDIO_MPEG4 = KAL_FALSE;
	     //4  <2> if preview of capture PICTURE
	     OV3660_g_iOV3660_Mode = OV3660_MODE_PREVIEW;
	     OV3660_PV_dummy_pixels = 1;
	     OV3660_PV_dummy_lines = 0;	
       
	     //Set preview pixel clock freqency and pixel clock division.
	     //	OV3660_g_Preview_PCLK_Frequency = ;		
	     //	OV3660_g_Preview_PCLK_Division = 0;	 

	     //Step 3. record preview ISP_clk
	     OV3660_preview_pclk = 563;//56330000;   //22500000;
       
	     //4 <3> set mirror and flip
	     //OV3660_set_mirror_flip(sensor_config_data->SensorImageMirror, &iStartX, &iStartY);
	     //OV3660_set_mirror_flip(IMAGE_V_MIRROR, &iStartX, &iStartY);
	     
       
       
	     // set banding filter
	     // OV3660_set_bandingfilter();
	     // set ae_target
       // OV3660_set_AE_target(AE_Target);
	     // update night mode setting
       // OV3660_night_mode(m_iCombo_NightMode);
       
	     OV3660_stream_on();
	     //OV3660_set_AE_mode(1);
	     OV3660_write_cmos_sensor(0x3503,0x00);
       OV3660_write_cmos_sensor(0x3a00,0x3c);
       OV3660_write_cmos_sensor(0x3406,OV3660_read_cmos_sensor(0x3406)&0xfe);  
       Sleep(200);
       
	     image_window->GrabStartX = IMAGE_SENSOR_START_GRAB_X;    
	     image_window->GrabStartY = IMAGE_SENSOR_START_GRAB_Y;       
	     image_window->ExposureWindowWidth = OV3660_IMAGE_SENSOR_PV_WIDTH;
	     image_window->ExposureWindowHeight = OV3660_IMAGE_SENSOR_PV_HEIGHT;
       
	     //debug 
	     //shutter = OV3660_read_shutter();
	     //pv_gain = OV3660_read_gain();

	     //Enable Color bar   
       SENSORDB("[OV3660YUV]:OV3660Preview exit.\n"); 

	     return TRUE; 
}	/* OV3660_Preview */

UINT32 OV3660Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	     int preview_shutter, preview_gain16;
	     kal_uint32 color_r_gain = 0;
			 kal_uint32 color_b_gain = 0;
       if(KAL_TRUE == OV3660_sensor_cap_state)
	        return;
	         
       SENSORDB("[OV3660YUV]:OV3660Capture Normal capture.\n"); 
	     //OV3660_set_AE_mode(0);
	     OV3660_write_cmos_sensor(0x3503,OV3660_read_cmos_sensor(0x3503)|0x03);      
	     OV3660_write_cmos_sensor(0x3a00,OV3660_read_cmos_sensor(0x3a00)&0xfb); 
	           
	     OV3660_write_cmos_sensor(0x3406,OV3660_read_cmos_sensor(0x3406)|0x01);
	     color_r_gain=((OV3660_read_cmos_sensor(0x3401)&0xFF)+((OV3660_read_cmos_sensor(0x3400)&0xFF)*256));  
			 color_b_gain=((OV3660_read_cmos_sensor(0x3405)&0xFF)+((OV3660_read_cmos_sensor(0x3404)&0xFF)*256)); 
	     preview_shutter = OV3660_read_shutter();	        
	     preview_gain16 = OV3660_read_OV3660_gain();	        

	     OV3660_stream_off();
	         
	     OV3660_write_cmos_sensor(0x3826,0x23);
       OV3660_write_cmos_sensor(0x303a,0x00);
       OV3660_write_cmos_sensor(0x303b,0x19);
       OV3660_write_cmos_sensor(0x303c,0x12);//7.5fps
       OV3660_write_cmos_sensor(0x303d,0x30);//30   
       OV3660_write_cmos_sensor(0x3824,0x01);
       OV3660_write_cmos_sensor(0x460c,0x20);
       
       OV3660_write_cmos_sensor(0x380c,0x08);
       OV3660_write_cmos_sensor(0x380d,0xfc);
       OV3660_write_cmos_sensor(0x380e,0x06);
       OV3660_write_cmos_sensor(0x380f,0x20);
       
       OV3660_write_cmos_sensor(0x3a08,0x00);//50hz
       OV3660_write_cmos_sensor(0x3a09,0x76);
       OV3660_write_cmos_sensor(0x3a0e,0x0d);
       OV3660_write_cmos_sensor(0x3a0a,0x00);//60hz
       OV3660_write_cmos_sensor(0x3a0b,0x62);
       OV3660_write_cmos_sensor(0x3a0d,0x10);	 
 	     
       OV3660_write_cmos_sensor(0x5302,0x30);//40
       OV3660_write_cmos_sensor(0x5303,0x10);//30
       OV3660_write_cmos_sensor(0x5306,0x1c);//20
       OV3660_write_cmos_sensor(0x5307,0x2c);//30
       
       OV3660_write_cmos_sensor(0x3800,0x00);
       OV3660_write_cmos_sensor(0x3801,0x00);
       OV3660_write_cmos_sensor(0x3802,0x00);
       OV3660_write_cmos_sensor(0x3803,0x00);
       OV3660_write_cmos_sensor(0x3804,0x08);
       OV3660_write_cmos_sensor(0x3805,0x1f);
       OV3660_write_cmos_sensor(0x3806,0x06);
       OV3660_write_cmos_sensor(0x3807,0x0b);
       OV3660_write_cmos_sensor(0x3808,0x08);//2048
       OV3660_write_cmos_sensor(0x3809,0x00);
       OV3660_write_cmos_sensor(0x380a,0x06);//1536
       OV3660_write_cmos_sensor(0x380b,0x00);
       OV3660_write_cmos_sensor(0x3810,0x00);
       OV3660_write_cmos_sensor(0x3811,0x10);
       OV3660_write_cmos_sensor(0x3812,0x00);
       OV3660_write_cmos_sensor(0x3813,0x06);
       OV3660_write_cmos_sensor(0x3814,0x11);
       OV3660_write_cmos_sensor(0x3815,0x11);       
       OV3660_write_cmos_sensor(0x3820,0x40);
       OV3660_write_cmos_sensor(0x3821,0x06);
       OV3660_write_cmos_sensor(0x4514,0x00);
       OV3660_write_cmos_sensor(0x3618,0x78);
       OV3660_write_cmos_sensor(0x3708,0x63);
       OV3660_write_cmos_sensor(0x3709,0x12);
       OV3660_write_cmos_sensor(0x4520,0xb0);
       
       OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)&0xdf);//bit[2]='0'
	   
	     preview_shutter = preview_shutter/2;
	     
	     if(preview_shutter < 1)
	        preview_shutter = 1;
	         
	     OV3660_write_shutter(preview_shutter);
	     OV3660WBcalibattion(color_r_gain,color_b_gain);
	     OV3660_stream_on();
	     Sleep(100);
	     
	     OV3660_sensor_cap_state = KAL_TRUE;
	     // skip 2 vysnc
	     // start capture at 3rd vsync
	     SENSORDB("[OV3660YUV]:OV3660Capture exit.\n");
	     return TRUE;     
}	/* OV3660_Capture */
UINT32 OV3660GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{	
	       SENSORDB("[OV3660YUV]:OV3660GetResolution.\n");
	       pSensorResolution->SensorFullWidth=OV3660_IMAGE_SENSOR_FULL_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;  //modify by yanxu
	       pSensorResolution->SensorFullHeight=OV3660_IMAGE_SENSOR_FULL_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
	       pSensorResolution->SensorPreviewWidth=OV3660_IMAGE_SENSOR_PV_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
	       pSensorResolution->SensorPreviewHeight=OV3660_IMAGE_SENSOR_PV_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
		   pSensorResolution->SensorVideoWidth=OV3660_IMAGE_SENSOR_VIDEO_WIDTH - 2 * IMAGE_SENSOR_START_GRAB_X;
		   pSensorResolution->SensorVideoHeight=OV3660_IMAGE_SENSOR_VIDEO_HEIGHT - 2 * IMAGE_SENSOR_START_GRAB_Y;
       
	     return ERROR_NONE;
}	/* OV3660GetResolution() */

UINT32 OV3660GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_INFO_STRUCT *pSensorInfo,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	       SENSORDB("[OV3660YUV]:OV3660GetInfo.\n");
	       pSensorInfo->SensorPreviewResolutionX=OV3660_IMAGE_SENSOR_PV_WIDTH;
	       pSensorInfo->SensorPreviewResolutionY=OV3660_IMAGE_SENSOR_PV_HEIGHT;
	       pSensorInfo->SensorFullResolutionX=OV3660_IMAGE_SENSOR_FULL_WIDTH;
	       pSensorInfo->SensorFullResolutionY=OV3660_IMAGE_SENSOR_FULL_HEIGHT;
         
	       pSensorInfo->SensorCameraPreviewFrameRate=30;
	       pSensorInfo->SensorVideoFrameRate=30;
	       pSensorInfo->SensorStillCaptureFrameRate=10;
	       pSensorInfo->SensorWebCamCaptureFrameRate=15;
	       pSensorInfo->SensorResetActiveHigh=FALSE;
	       pSensorInfo->SensorResetDelayCount=1;
	       pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	       pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	       pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	       pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	       pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	       pSensorInfo->SensorInterruptDelayLines = 1;
	       pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_PARALLEL;
         
	       pSensorInfo->CaptureDelayFrame = 2; 
	       pSensorInfo->PreviewDelayFrame = 3; 
	       pSensorInfo->VideoDelayFrame = 20; 
	       pSensorInfo->SensorMasterClockSwitch = 0; 
         pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   		
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			   pSensorInfo->SensorClockFreq=26;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
         pSensorInfo->SensorGrabStartX = IMAGE_SENSOR_START_GRAB_X; 
         pSensorInfo->SensorGrabStartY = IMAGE_SENSOR_START_GRAB_Y;  		
		     break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			   pSensorInfo->SensorClockFreq=26;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
         pSensorInfo->SensorGrabStartX = IMAGE_FULL_GRAB_START_X; 
         pSensorInfo->SensorGrabStartY = IMAGE_FULL_GRAB_START_Y;     			
		     break;
		default:
			   pSensorInfo->SensorClockFreq=26;
			   pSensorInfo->SensorClockDividCount=3;
			   pSensorInfo->SensorClockRisingCount=0;
			   pSensorInfo->SensorClockFallingCount=2;
			   pSensorInfo->SensorPixelClockCount=3;
			   pSensorInfo->SensorDataLatchCount=2;
         pSensorInfo->SensorGrabStartX = 0; 
         pSensorInfo->SensorGrabStartY = 0;     			
		     break;
	}
	OV3660_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	memcpy(pSensorConfigData, &OV3660SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* OV3660GetInfo() */

UINT32 OV3660Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:		
			              SENSORDB("[OV3660YUV]:OV3660Control Preview.\n");
			              OV3660Preview(pImageWindow, pSensorConfigData);
		                break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:			
			              SENSORDB("[OV3660YUV]:OV3660Control Capture.\n");
			              OV3660Capture(pImageWindow, pSensorConfigData);
		                break;
		default:
		                break; 
	}
	return TRUE;
}	/* OV3660Control() */

/* [TC] YUV sensor */	

/*************************************************************************
* FUNCTION
*	OV3660_set_param_wb
*
* DESCRIPTION
*	wb setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV3660_set_param_wb(UINT16 para)
{
	  
	switch (para)
	{
	  case AWB_MODE_OFF:
			                OV3660_write_cmos_sensor(0x3212, 0x03);
			                
			                OV3660_write_cmos_sensor(0x3406, 0x00);
			                
			                OV3660_write_cmos_sensor(0x3212, 0x13);
			                OV3660_write_cmos_sensor(0x3212, 0xa3);
	                    break;             
		case AWB_MODE_AUTO:
			                OV3660_write_cmos_sensor(0x3212, 0x03);
			                
			                OV3660_write_cmos_sensor(0x3406, 0x00);
			                
			                OV3660_write_cmos_sensor(0x3212, 0x13);
			                OV3660_write_cmos_sensor(0x3212, 0xa3);
		                  break;
		case AWB_MODE_CLOUDY_DAYLIGHT:
			                OV3660_write_cmos_sensor(0x3212, 0x03);
			                
			                OV3660_write_cmos_sensor(0x3406, 0x01);
			                OV3660_write_cmos_sensor(0x3400, 0x04);
			                OV3660_write_cmos_sensor(0x3401, 0xa0);
			                OV3660_write_cmos_sensor(0x3402, 0x04);
			                OV3660_write_cmos_sensor(0x3403, 0x00);
			                OV3660_write_cmos_sensor(0x3404, 0x06);
			                OV3660_write_cmos_sensor(0x3405, 0xac);
			                
			                OV3660_write_cmos_sensor(0x3212, 0x13);
			                OV3660_write_cmos_sensor(0x3212, 0xa3);
		                  break;
		case AWB_MODE_DAYLIGHT:
			                OV3660_write_cmos_sensor(0x3212, 0x03);
			                
			                OV3660_write_cmos_sensor(0x3406, 0x01);
			                OV3660_write_cmos_sensor(0x3400, 0x07);
			                OV3660_write_cmos_sensor(0x3401, 0x1d);
			                OV3660_write_cmos_sensor(0x3402, 0x04);
			                OV3660_write_cmos_sensor(0x3403, 0x00);
			                OV3660_write_cmos_sensor(0x3404, 0x04);
			                OV3660_write_cmos_sensor(0x3405, 0xb4);
			                
			                OV3660_write_cmos_sensor(0x3212, 0x13);
			                OV3660_write_cmos_sensor(0x3212, 0xa3);
		                  break;
		case AWB_MODE_INCANDESCENT:	
			                OV3660_write_cmos_sensor(0x3212, 0x03);
			                
			                OV3660_write_cmos_sensor(0x3406, 0x01);
			                OV3660_write_cmos_sensor(0x3400, 0x05);
			                OV3660_write_cmos_sensor(0x3401, 0x4f);
			                OV3660_write_cmos_sensor(0x3402, 0x04);
			                OV3660_write_cmos_sensor(0x3403, 0x00);
			                OV3660_write_cmos_sensor(0x3404, 0x07);
			                OV3660_write_cmos_sensor(0x3405, 0x35);
			                
			                OV3660_write_cmos_sensor(0x3212, 0x13);
			                OV3660_write_cmos_sensor(0x3212, 0xa3);  
			                break;  
		case AWB_MODE_FLUORESCENT:
			                OV3660_write_cmos_sensor(0x3212, 0x03);
			                
			                OV3660_write_cmos_sensor(0x3406, 0x01);
			                OV3660_write_cmos_sensor(0x3400, 0x06);
			                OV3660_write_cmos_sensor(0x3401, 0x75);
			                OV3660_write_cmos_sensor(0x3402, 0x04);
			                OV3660_write_cmos_sensor(0x3403, 0x00);
			                OV3660_write_cmos_sensor(0x3404, 0x07);
			                OV3660_write_cmos_sensor(0x3405, 0xad);
			                
			                OV3660_write_cmos_sensor(0x3212, 0x13);
			                OV3660_write_cmos_sensor(0x3212, 0xa3);
		                  break;  
		case AWB_MODE_TUNGSTEN:
			                OV3660_write_cmos_sensor(0x3212, 0x03);
			                
			                OV3660_write_cmos_sensor(0x3406, 0x01);
			                OV3660_write_cmos_sensor(0x3400, 0x04);
			                OV3660_write_cmos_sensor(0x3401, 0xc7);
		                 	OV3660_write_cmos_sensor(0x3402, 0x04);
			                OV3660_write_cmos_sensor(0x3403, 0x00);
			                OV3660_write_cmos_sensor(0x3404, 0x07);
			                OV3660_write_cmos_sensor(0x3405, 0xed);
			                
			                OV3660_write_cmos_sensor(0x3212, 0x13);
			                OV3660_write_cmos_sensor(0x3212, 0xa3);
		                  break;
		default:
			                return FALSE;
	}

	return TRUE;
	
} /* OV3660_set_param_wb */

/*************************************************************************
* FUNCTION
*	OV3660_set_param_effect
*
* DESCRIPTION
*	effect setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV3660_set_param_effect(UINT16 para)
{
	kal_uint32  ret = TRUE;
	
	switch (para)
	{
		case MEFFECT_OFF:
			              OV3660_write_cmos_sensor(0x3212,0x03);	
	  		            OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			              OV3660_write_cmos_sensor(0x5580,0x06);	
			              OV3660_write_cmos_sensor(0x5583,0x40);	
			              OV3660_write_cmos_sensor(0x5584,0x2c);	
			              OV3660_write_cmos_sensor(0x3212,0x13);	
			              OV3660_write_cmos_sensor(0x3212,0xa3);	
	                  break;
		case MEFFECT_SEPIA:
			              OV3660_write_cmos_sensor(0x3212,0x03);	
		  	            OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			              OV3660_write_cmos_sensor(0x5580,0x1e);
			              OV3660_write_cmos_sensor(0x5583,0x40);
			              OV3660_write_cmos_sensor(0x5584,0xa0);
			              OV3660_write_cmos_sensor(0x3212,0x13);	
			              OV3660_write_cmos_sensor(0x3212,0xa3);	
			              break;  
		case MEFFECT_NEGATIVE:
			              OV3660_write_cmos_sensor(0x3212,0x03);	
		  	            OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);	                 	
			              OV3660_write_cmos_sensor(0x5580,0x46);			                
			              OV3660_write_cmos_sensor(0x3212,0x13);	
			              OV3660_write_cmos_sensor(0x3212,0xa3);	    
			              break; 
		case MEFFECT_SEPIAGREEN:		
			              OV3660_write_cmos_sensor(0x3212,0x03);	
		  	            OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			              OV3660_write_cmos_sensor(0x5580,0x1e);
			              OV3660_write_cmos_sensor(0x5583,0x60);
			              OV3660_write_cmos_sensor(0x5584,0x60);	
			              OV3660_write_cmos_sensor(0x3212,0x13);	
			              OV3660_write_cmos_sensor(0x3212,0xa3);	
			              break;
		case MEFFECT_SEPIABLUE:
			              OV3660_write_cmos_sensor(0x3212,0x03);	
		  	            OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			              OV3660_write_cmos_sensor(0x5580,0x1e);
			              OV3660_write_cmos_sensor(0x5583,0xa0);
			              OV3660_write_cmos_sensor(0x5584,0x40);
			              OV3660_write_cmos_sensor(0x3212,0x13);	
			              OV3660_write_cmos_sensor(0x3212,0xa3);
			              break;        
		case MEFFECT_MONO:			
			              OV3660_write_cmos_sensor(0x3212,0x03);	
		  	            OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			              OV3660_write_cmos_sensor(0x5580,0x26);			                 
			              OV3660_write_cmos_sensor(0x3212,0x13);	
			              OV3660_write_cmos_sensor(0x3212,0xa3);	 	
			              break;
		default:
			              ret = FALSE;
	}

	return ret;

} /* OV3660_set_param_effect */

/*************************************************************************
* FUNCTION
*	OV3660_set_param_banding
*
* DESCRIPTION
*	banding setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV3660_set_param_banding(UINT16 para)
{
	int temp;
	SENSORDB("[OV3660YUV]:OV3660_set_param_banding: para = %d\n", para);

	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			                       OV3660_write_cmos_sensor(0x3c00,0x04);		
			                       OV3660_write_cmos_sensor(0x3c01,0x80);
			                       break;
		case AE_FLICKER_MODE_60HZ:		
			                       OV3660_write_cmos_sensor(0x3c00,0x00);			
			                       OV3660_write_cmos_sensor(0x3c01,0x80);		
			                       break;
    default:
	                           return KAL_FALSE;
	}
	return KAL_TRUE;
} /* OV3660_set_param_banding */
/*************************************************************************
* FUNCTION
*	OV3660_set_param_exposure
*
* DESCRIPTION
*	exposure setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL OV3660_set_param_exposure(UINT16 para)
{
  switch (para)
	{
		case AE_EV_COMP_13:
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                 
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x00);	
			                 OV3660_write_cmos_sensor(0x5587,0x20);	
			                 	
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);	
			                 break;  
		case AE_EV_COMP_10:
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                 
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x00);	
			                 OV3660_write_cmos_sensor(0x5587,0x18);		
			                 
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);	 			
			                 break;    
		case AE_EV_COMP_07:
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                  
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x00);	
			                 OV3660_write_cmos_sensor(0x5587,0x10);	
			                 	
			                 OV3660_write_cmos_sensor(0x3212,0x13);				                 
			                 OV3660_write_cmos_sensor(0x3212,0xa3);				
			                 break;    
		case AE_EV_COMP_03:			
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                 
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x00);	
			                 OV3660_write_cmos_sensor(0x5587,0x08);	
			                 	
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);			
			                 break;    
		case AE_EV_COMP_00:
			                 OV3660_write_cmos_sensor(0x3212,0x03);
			                 	
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x00);	
			                 OV3660_write_cmos_sensor(0x5587,0x00);	
			                 		               
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);	 		
			                 break;    
		case AE_EV_COMP_n03:
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                 
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x08);	
			                 OV3660_write_cmos_sensor(0x5587,0x08);	
			                 	
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);				
			                 break;    
		case AE_EV_COMP_n07:			
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                 
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x08);	
			                 OV3660_write_cmos_sensor(0x5587,0x10);	
			                 	
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);				
			                 break;    
		case AE_EV_COMP_n10:
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                 
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x08);	
			                 OV3660_write_cmos_sensor(0x5587,0x18);	
			                 	
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);	
			                 break;
		case AE_EV_COMP_n13:
			                 OV3660_write_cmos_sensor(0x3212,0x03);	
			                 
			                 OV3660_write_cmos_sensor(0x5001,OV3660_read_cmos_sensor(0x5001)|0x80);
			                 OV3660_write_cmos_sensor(0x5580,OV3660_read_cmos_sensor(0x5580)|0x04);
			                 OV3660_write_cmos_sensor(0x5588,0x08);	
			                 OV3660_write_cmos_sensor(0x5587,0x20);	
			                 	
			                 OV3660_write_cmos_sensor(0x3212,0x13);	
			                 OV3660_write_cmos_sensor(0x3212,0xa3);	
			                 break;
		default:
			                 return false;
	}
	return TRUE;	
} /* OV3660_set_param_exposure */

BOOL OV3660_set_param_contrast(UINT16 para)
{
    kal_uint8 temp_reg;	
	 
    temp_reg = OV3660_read_cmos_sensor(0x5580);

	  switch (para)
    {
        case ISP_CONTRAST_LOW:
                              OV3660_write_cmos_sensor(0x3212, 0x03);
			                        
                              OV3660_write_cmos_sensor(0x5580, temp_reg|0x4); 
                              
                              OV3660_write_cmos_sensor(0x5586, 0x18);
                              OV3660_write_cmos_sensor(0x5585, 0x18);                               
                              
			                        OV3660_write_cmos_sensor(0x3212, 0x13);
			                        OV3660_write_cmos_sensor(0x3212, 0xa3);  
                              break;
        case ISP_CONTRAST_MIDDLE:
                             
			                        OV3660_write_cmos_sensor(0x3212, 0x03);
			                        
                              OV3660_write_cmos_sensor(0x5580, temp_reg|0x4); 
                              
                              OV3660_write_cmos_sensor(0x5586, 0x20);
                              OV3660_write_cmos_sensor(0x5585, 0x20);                               
                              
			                        OV3660_write_cmos_sensor(0x3212, 0x13);
			                        OV3660_write_cmos_sensor(0x3212, 0xa3);                              
                              break;
        case ISP_CONTRAST_HIGH:
                              OV3660_write_cmos_sensor(0x3212, 0x03);
			                        
                              OV3660_write_cmos_sensor(0x5580, temp_reg|0x4); 
                              
                              OV3660_write_cmos_sensor(0x5586, 0x28);
                              OV3660_write_cmos_sensor(0x5585, 0x28);                               
                              
			                        OV3660_write_cmos_sensor(0x3212, 0x13);
			                        OV3660_write_cmos_sensor(0x3212, 0xa3);  
                              break;
        default:
                              return KAL_FALSE;	                                    
    }
    return KAL_TRUE;
}
UINT32 OV3660YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	SENSORDB("[OV3660YUV]:OV3660YUVSensorSetting: iCmd = %d, iPara = %d\n", iCmd, iPara);
      
	switch (iCmd) {
	case FID_SCENE_MODE:	    
	                 printk("[OV3660YUV] Set Scene Mode:%d\n", iPara); 
	                 if (iPara == SCENE_MODE_OFF)
	                 {
	                     OV3660_night_mode(0); 
	                 }
	                 else if (iPara == SCENE_MODE_NIGHTSCENE)
	                 {
                            OV3660_night_mode(1); 
	                 }	    
	                 break; 	    
	case FID_AWB_MODE:
	                 printk("[OV3660YUV] Set AWB Mode:%d\n", iPara); 	    
                   OV3660_set_param_wb(iPara);
	                 break;
	case FID_COLOR_EFFECT:
	                 printk("[OV3660YUV] Set Color Effect:%d\n", iPara); 	    	    
                   OV3660_set_param_effect(iPara);
	                 break;
	case FID_AE_EV:
                   printk("[OV3660YUV] Set EV:%d\n", iPara); 	    	    
                   OV3660_set_param_exposure(iPara);
	                 break;
	case FID_AE_FLICKER:
                   printk("[OV3660YUV] Set Flicker:%d\n", iPara); 	    	    	    
                   OV3660_set_param_banding(iPara);
	                 break;
	case FID_AE_SCENE_MODE: 
	  	             printk("[OV3660YUV] Set AE_SCENE_MODE:%d\n", iPara);
	                 if (iPara == AE_MODE_OFF)
	                 {
                       OV3660_AE_ENABLE = 0; 
                   }
                   else 
                   {
                       OV3660_AE_ENABLE = 1; 
	                 }
                   OV3660_set_AE_mode(OV3660_AE_ENABLE);
                   break; 
	case FID_ZOOM_FACTOR:
		               printk("[OV3660YUV] Set zoom factor:%d\n", iPara);
	                 zoom_factor = iPara; 		
	                 break; 
  case FID_ISP_CONTRAST:
		               printk("[OV3660YUV] Set contrast:%d\n", iPara);
	                 OV3660_set_param_contrast(iPara);
	                 break;	
	default:
	                 break;
	}
	return TRUE;
}   /* OV3660YUVSensorSetting */

UINT32 OV3660YUVSetVideoMode(UINT16 u2FrameRate)
{
     OV3660_VEDIO_encode_mode = KAL_TRUE;
     
    if (u2FrameRate == 30)
    {
		 OV3660_write_cmos_sensor(0x3826,0x23);
     OV3660_write_cmos_sensor(0x303a,0x00);
     OV3660_write_cmos_sensor(0x303b,0x19);
     OV3660_write_cmos_sensor(0x303c,0x11);//30fps
     OV3660_write_cmos_sensor(0x303d,0x30);
     OV3660_write_cmos_sensor(0x3824,0x02);
     OV3660_write_cmos_sensor(0x460c,0x22);
     
     OV3660_write_cmos_sensor(0x380c,0x08);
     OV3660_write_cmos_sensor(0x380d,0xfc);
     OV3660_write_cmos_sensor(0x380e,0x03);
     OV3660_write_cmos_sensor(0x380f,0x10);
     
     OV3660_write_cmos_sensor(0x3a08,0x00);//50hz
     OV3660_write_cmos_sensor(0x3a09,0xeb);
     OV3660_write_cmos_sensor(0x3a0e,0x03);
     OV3660_write_cmos_sensor(0x3a0a,0x00);//60hz
     OV3660_write_cmos_sensor(0x3a0b,0xc4);
     OV3660_write_cmos_sensor(0x3a0d,0x04);		
     
     OV3660_write_cmos_sensor(0x3a00,0x38);
	   OV3660_write_cmos_sensor(0x3a02,0x03);
	   OV3660_write_cmos_sensor(0x3a03,0x10);
	   OV3660_write_cmos_sensor(0x3a14,0x03);
	   OV3660_write_cmos_sensor(0x3a15,0x10);	
    }
    else if (u2FrameRate == 15)
    {
		 OV3660_write_cmos_sensor(0x3826,0x23);
     OV3660_write_cmos_sensor(0x303a,0x00);
     OV3660_write_cmos_sensor(0x303b,0x19);
     OV3660_write_cmos_sensor(0x303c,0x12);//15fps
     OV3660_write_cmos_sensor(0x303d,0x30);
     OV3660_write_cmos_sensor(0x3824,0x02);
     OV3660_write_cmos_sensor(0x460c,0x22);
     
     OV3660_write_cmos_sensor(0x380c,0x08);
     OV3660_write_cmos_sensor(0x380d,0xfc);
     OV3660_write_cmos_sensor(0x380e,0x03);
     OV3660_write_cmos_sensor(0x380f,0x10);
     
     OV3660_write_cmos_sensor(0x3a08,0x00);//50hz
     OV3660_write_cmos_sensor(0x3a09,0x76);
     OV3660_write_cmos_sensor(0x3a0e,0x06);
     OV3660_write_cmos_sensor(0x3a0a,0x00);//60hz
     OV3660_write_cmos_sensor(0x3a0b,0x62);
     OV3660_write_cmos_sensor(0x3a0d,0x08);		
     
     OV3660_write_cmos_sensor(0x3a00,0x38);
	   OV3660_write_cmos_sensor(0x3a02,0x03);
	   OV3660_write_cmos_sensor(0x3a03,0x10);
	   OV3660_write_cmos_sensor(0x3a14,0x03);
	   OV3660_write_cmos_sensor(0x3a15,0x10);
    }
    else
    {
        printk("Wrong Frame Rate \n");
    }

    return TRUE;
}

UINT32 OV3660SetTestPatternMode(kal_bool bEnable)
{
	//OV5645MIPISENSORDB("[OV5645MIPI_OV5645SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	if(bEnable)
	{
		OV3660_write_cmos_sensor(0x503d,0x80);
		run_test_potten=1;
	}
	else
	{
		OV3660_write_cmos_sensor(0x503d,0x00);
		run_test_potten=0;
	}
	return ERROR_NONE;
}

UINT32 OV3660FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
  UINT16 u2Temp = 0; 
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	SENSORDB("[OV3660YUV]:OV3660FeatureControl: FeatureId = %d\n", FeatureId);

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			                                *pFeatureReturnPara16++=OV3660_IMAGE_SENSOR_FULL_WIDTH;
			                                *pFeatureReturnPara16=OV3660_IMAGE_SENSOR_FULL_HEIGHT;
			                                *pFeatureParaLen=4;
		                                  break;
		case SENSOR_FEATURE_GET_PERIOD:
			                                *pFeatureReturnPara16++=OV3660_PV_PERIOD_PIXEL_NUMS+OV3660_PV_dummy_pixels;
			                                *pFeatureReturnPara16=OV3660_PV_PERIOD_LINE_NUMS+OV3660_PV_dummy_lines;
			                                *pFeatureParaLen=4;
		                                  break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			                                *pFeatureReturnPara32 = OV3660_sensor_pclk/10;
			                                *pFeatureParaLen=4;
		                                  break;
		case SENSOR_FEATURE_SET_ESHUTTER:
                                      //u2Temp = OV3660_read_shutter(); 
                                      //printk("Shutter:%d\n", u2Temp); 			
		                                  break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			                                OV3660_night_mode((BOOL) *pFeatureData16);
		                                  break;
		case SENSOR_FEATURE_SET_GAIN:
                                      //u2Temp = OV3660_read_gain(); 
                                      //printk("Gain:%d\n", u2Temp); 
                                      //printk("y_val:%d\n", OV3660_read_cmos_sensor(0x301B));
			                                break; 
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		                                  break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			                                OV3660_isp_master_clock=*pFeatureData32;
		                                  break;
		case SENSOR_FEATURE_SET_REGISTER:
			                                OV3660_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		                                  break;
		case SENSOR_FEATURE_GET_REGISTER:
			                                pSensorRegData->RegData = OV3660_read_cmos_sensor(pSensorRegData->RegAddr);
		                                  break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			                                memcpy(pSensorConfigData, &OV3660SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			                                *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		                                  break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		                                 break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
	                                   *pFeatureReturnPara32++=0;
			                               *pFeatureParaLen=4;
		                                 break; 		
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			                               // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			                               // if EEPROM does not exist in camera module.
			                               *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			                               *pFeatureParaLen=4;
		                                 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
                                     //printk("OV3660 YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			                               OV3660YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		                                 break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		                                 OV3660YUVSetVideoMode(*pFeatureData16);
		                                 break; 
    case SENSOR_FEATURE_CHECK_SENSOR_ID:
            		                     OV3660GetSensorID(pFeatureReturnPara32); 
			                               break;	
        case SENSOR_FEATURE_SET_TEST_PATTERN: 
			OV3660SetTestPatternMode((BOOL)*pFeatureData16);            
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=OV3660_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;				
		
		default:
			                               break;			
	}
	return ERROR_NONE;
}	/* OV3660FeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncOV3660=
{
	     OV3660Open,
	     OV3660GetInfo,
	     OV3660GetResolution,
	     OV3660FeatureControl,
	     OV3660Control,
	     OV3660Close
};

UINT32 OV3660_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	     /* To Do : Check Sensor status here */
	     if (pfFunc!=NULL)
		   *pfFunc=&SensorFuncOV3660;       
	     return ERROR_NONE;
}	/* SensorInit() */
