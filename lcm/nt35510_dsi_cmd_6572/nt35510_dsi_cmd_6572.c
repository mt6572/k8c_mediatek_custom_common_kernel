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


#if defined(BUILD_LK)
    #define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
    #define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (320)  // pixel
#define FRAME_HEIGHT (320)  // pixel
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

//#define PHYSICAL_WIDTH  (56)  // mm
//#define PHYSICAL_HEIGHT (93)  // mm

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
#if 1//def LCD_1
    
    {0xf0, 1, { 0xc3}},
    {0xf0, 1, { 0x96}},               
    {0x2a, 4, { 0x00, 0x00, 0x01, 0x3F}},                
    {0x2B, 4, { 0x00, 0x00, 0x01, 0x3F}}, 
    {0xB6, 3, {0x8a, 0x07, 0x3b}},      
    {0xB5, 4, {0x20, 0x20, 0x00, 0x20}},                         
                                     
    {0xB1, 2, {0x80, 0x10}},                                  
                                       
    {0x36, 1, {0x48}}, 
    {0x3A, 1, {0x77}},
     
    {0xb4, 1, {0x00}}, 
    {0xC5, 1, {0x28}},                            
    {0x21, 1, {0x00}},    
    {0xe4, 1, {0x31}},                                             
    {0xe8, 8,{0x40, 0x84, 0x1b, 0x1b, 0x10, 0x03, 0xb8, 0x33}},                                
    {0xE0, 14,{0xf0, 0x07, 0x0e, 0x0a, 0x08, 0x25, 0x38, 0x43, 0x51, 0x38, 0x14, 0x12, 0x32, 0x3f}},                         
    {0xE1, 14,{0xf0, 0x08, 0x0d, 0x09, 0x09, 0x26, 0x39, 0x45, 0x52, 0x07, 0x13, 0x16, 0x32, 0x3f}}, 
    {0x12, 1, {0x00}},   

    {0x30, 4,{0x00, 0x00, 0x01, 0x3f}},                    
    {0xf0, 1, {0x3c}}, 
    {0xf0, 1, {0x69}},        
    {0x11, 1, {0x00}},                             
    {REGFLAG_DELAY, 120, {}},
    {0x29, 1, {0x00}},                                      
    {REGFLAG_DELAY, 10, {}},
#else
    {0x11, 0, {}},  
    {REGFLAG_DELAY, 120, {}},
    {0xf0, 1, { 0xc3}},
    {0xf0, 1, { 0x96}},               
    {0x36, 1, {0x40}}, 
    {0x3A, 1, {0x55}}, 
    {0x2a, 4, { 0x00, 0x00, 0x01, 0x3F}},                
    {0x2B, 4, { 0x00, 0x00, 0x01, 0x3F}}, 
    {0xb4, 1, {0x01}},
    {0xB1, 2, {0xa0, 0x10}},  

    {0xB6, 3, {0x80, 0x07, 0x27}},      
    {0xB5, 4, {0x20, 0x20, 0x00, 0x20}},                         
    {0xb7, 1, {0xc6}},                               
    {0xe8, 8,{0x40, 0x84, 0x1b, 0x1b, 0x20, 0x15, 0xb8, 0x53}},                                                  
    {0xc0, 2, {0xe0,0x45}},
    {0xc1, 1, {0x19}}, 
    {0xc2, 1, {0xa7}},
    {0xc5, 1, {0x28}}, 
    {0x35, 1, {0x00}},                                                                           
    {0xE0, 14,{0xf0, 0x07, 0x0c, 0x08, 0x08, 0x05, 0x2f, 0x44, 0x46, 0x18, 0x14, 0x14, 0x2a, 0x30}},                         
    {0xE1, 14,{0xf0, 0x07, 0x0c, 0x08, 0x08, 0x05, 0x2f, 0x44, 0x46, 0x18, 0x14, 0x14, 0x2a, 0x30}},                   
    {0xf0, 1, {0x3c}}, 
    {0xf0, 1, {0x69}},        
    {0x21, 0, {}},                           
    {REGFLAG_DELAY, 120, {}},
    {0x29, 0, {}},                                      
    {REGFLAG_DELAY, 10, {}},
#endif
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

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
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{

    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

   // physical size
    //params->physical_width = PHYSICAL_WIDTH;
   // params->physical_height = PHYSICAL_HEIGHT;

     params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

    params->dsi.mode = SYNC_EVENT_VDO_MODE;//CMD_MODE;SYNC_PULSE_VDO_MODE; BURST_VDO_MODE;//

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_ONE_LANE;
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 1;
		params->dsi.vertical_backporch					= 1;//50
		params->dsi.vertical_frontporch					= 1;//20
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 1;
		params->dsi.horizontal_backporch				= 1;
		params->dsi.horizontal_frontporch				= 1;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


		// Bit rate calculation
		params->dsi.pll_div1=32;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)

		
		params->dsi.lcm_int_te_monitor = FALSE; 
		params->dsi.lcm_int_te_period = 1; // Unit : frames 
 
		// Need longer FP for more opportunity to do int. TE monitor applicably. 
		if(params->dsi.lcm_int_te_monitor) 
			params->dsi.vertical_frontporch *= 2; 
 
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.) 
		params->dsi.lcm_ext_te_monitor = FALSE; 
		// Non-continuous clock 
		params->dsi.noncont_clock = TRUE; 
		params->dsi.noncont_clock_period = 2; // Unit : frames	
}

static void lcm_init(void)
{
     SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);	// Reset complete time 5ms
   
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
   
}


static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    
    
    return 1;
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER nt35510_dsi_cmd_6572_drv = {
    .name = "nt35510_dsi_cmd_6572",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    //.update = lcm_update,
   .compare_id    = lcm_compare_id,
};
