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

#ifdef BUILD_LK
extern int g_lcm_ratio;
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  										(320)
#define FRAME_HEIGHT 										(320)
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE	1
#define ILI9488_LCM_ID 	   (0x9488) 

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   		    lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static struct LCM_setting_table
{
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
};


static struct LCM_setting_table lcm_initialization_setting[] =
{
    // Setting ending by predefined flag
    {0xB6, 3, { 0x02, 0x02, 0x27}},
    {0x2A, 4, { 0x00, 0x00, 0x01, 0x3F}},                               
    {0x2B, 4, { 0x00, 0x00, 0x01, 0x3F}},                               
    {0x3A, 1, {0x66}},                                  
    {0xB1, 2, {0xB0, 0x12}}, 
    {0xB2, 2, {0x00, 0x11}},                                    
    {0xB5, 4, {0x02, 0x02, 0x0A, 0x04}},                                    
    {0xB4, 1, {0x02}},                                    
    //{0xB6, 3, {0x02, 0x02, 0x27}},                                    
    {0x35, 1, {0x00}},                                    
    {0x36, 1, {0x48}},                                    
    {0x21, 1, {0x00}},                                      
    {0xFC, 3, {0x00, 0x05, 0x08}},                                
    {0xF2, 10,{0x58, 0x7E, 0x12, 0x02, 0x12, 0x12, 0xFF, 0x0A, 0x90, 0x10}},   
    {0xC0, 2, {0x0f, 0x0f}},                                    
    {0xC1, 1, {0x41}},                                    
    {0xC5, 4, {0x00, 0x33, 0x00, 0x40}},                                    
    {0xC2, 1, {0x22}},                                
    {0xE0, 15,{0x00, 0x09, 0x0F, 0x03, 0x0F, 0x05, 0x39, 0x9B, 0x51, 0x07, 0x0E, 0x16, 0x2F, 0x35, 0x0F}},                          
    {0xE1, 15,{0x00, 0x09, 0x0E, 0x01, 0x13, 0x09, 0x2C, 0x67, 0x43, 0x06, 0x10, 0x0E, 0x31, 0x35, 0x0F}},                                  
    {0x11, 1, {0x00}},                                      
    {REGFLAG_DELAY, 120, {}},
    {0x29, 1, {0x00}},                                      
    {REGFLAG_DELAY, 10, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_sleep_out_setting[] =
{
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
{
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 150, {}},

    {0xF1, 2, {0x5A, 0x5A}},

    {0xF4, 14, {0x07, 0x00, 0x00, 0x00, 0x21, 0x4F, 0x01, 0x02, 0x2A, 0x66, 0x02, 0x2A, 0x00, 0x02}},

    {0xF1, 2, {0x5A, 0x5A}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 100, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd)
        {
        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE :
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
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

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_FALLING;


#if defined(LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_ONE_LANE;
    params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

    // Video mode setting
    params->dsi.PS						= LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active	= 2;
    params->dsi.vertical_backporch		= 100;
    params->dsi.vertical_frontporch 	= 100;
    params->dsi.vertical_active_line	= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active	= 2;
    params->dsi.horizontal_backporch	= 4;
    params->dsi.horizontal_frontporch	= 4;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    //params->dsi.pll_select = 1;
    params->dsi.pll_div1   = 0;
    params->dsi.pll_div2   = 1;
    params->dsi.fbk_div    = 20;//16;
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(1);	// Reset complete time 5ms
#ifdef BUILD_LK
    //VGP1 3.0V
    pmic_config_interface(DIGLDO_CON7, 0x1, PMIC_RG_VGP1_EN_MASK, PMIC_RG_VGP1_EN_SHIFT);
    pmic_config_interface(DIGLDO_CON28, 0x06, PMIC_RG_VGP1_VOSEL_MASK, PMIC_RG_VGP1_VOSEL_SHIFT);
#else
    printk("lcm MT65XX_POWER_LDO_VGP1 VOL_3000\n");
    hwPowerOn(MT65XX_POWER_LDO_VGP1, VOL_3000, "LCM");
    MDELAY(10);
#endif
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
    //VGP1 3.0V
    pmic_config_interface(DIGLDO_CON7, 0x0, PMIC_RG_VGP1_EN_MASK, PMIC_RG_VGP1_EN_SHIFT);
    pmic_config_interface(DIGLDO_CON28, 0x0, PMIC_RG_VGP1_VOSEL_MASK, PMIC_RG_VGP1_VOSEL_SHIFT);
#else
    printk("lcm MT65XX_POWER_LDO_VGP1 hwPowerDown\n");
    hwPowerDown(MT65XX_POWER_LDO_VGP1, "LCM");
#endif
}

static unsigned int lcm_compare_id();

static void lcm_resume(void)
{
    lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
    unsigned char x0_LSB = (x0 & 0xFF);
    unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
    unsigned char x1_LSB = (x1 & 0xFF);
    unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
    unsigned char y0_LSB = (y0 & 0xFF);
    unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
    unsigned char y1_LSB = (y1 & 0xFF);

    unsigned int data_array[16];

    data_array[0] = 0x00053902;
    data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
    data_array[2] = (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00053902;
    data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
    data_array[2] = (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    //data_array[0]= 0x00290508; //HW bug, so need send one HS packet
    //dsi_set_cmdq(data_array, 1, 1);

    data_array[0] = 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}

// ---------------------------------------------------------------------------
//  Get LCM ID Information
// ---------------------------------------------------------------------------
static unsigned int lcm_compare_id()
{
	unsigned char buffer[4] = {0,0,0,0};
	unsigned int data_array[16];		
    int ili9488_id = 0;
    
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
		
	//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00043902;
	data_array[1]=0x0698ffff;
	dsi_set_cmdq(data_array, 2, 1);

	MDELAY(10);
	data_array[0] = 0x00083700;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(10);
 	read_reg_v2(0xD3, buffer, 4);
	ili9488_id = (buffer[1] << 8 | buffer[2]);	
	
#if defined(BUILD_LK)
	printf(" buffer[0] = 0x%x, buffer[1] = 0x%x, buffer[2] = 0x%x, buffer[3] = 0x%x \n",
		   buffer[0] ,buffer[1],buffer[2] ,buffer[3] );
#else
	printk(" buffer[0] = 0x%x, buffer[1] = 0x%x, buffer[2] = 0x%x, buffer[3] = 0x%x \n",
		   buffer[0] ,buffer[1],buffer[2] ,buffer[3] );
#endif

#if defined(BUILD_LK)
    if(ILI9488_LCM_ID == ili9488_id)
    {
        g_lcm_ratio = LCM_RATIO_320X320;
    }
#endif

	return (ILI9488_LCM_ID == ili9488_id) ? 1 : 0;
}


LCM_DRIVER ili9488_320x320_dsi_cmd_boe_lcm_drv =
{
    .name			= "320x320_ili9488_dsi_cmd_boe",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
	.compare_id     = lcm_compare_id,
};

