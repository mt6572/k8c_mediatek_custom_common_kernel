#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (320)
#define FRAME_HEIGHT (480)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ILI9488_ID       (0x9488)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
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
    
    for(i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            
            case REGFLAG_END_OF_TABLE :
                break;
            
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);				
            
                if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
                    //#if defined(BUILD_UBOOT)
                    //	printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
                    //#endif
                    while(read_reg(cmd) != table[i].para_list[0]);		
                }
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
    
    // enable tearing-free
    params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = CMD_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_ONE_LANE; // LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;
    params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
    
    // Bit rate calculation
#ifdef CONFIG_MT6589_FPGA
    params->dsi.pll_div1=2;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=2;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =8;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =21; // 17;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
#endif
}


static void init_lcm_registers(void)
{
    unsigned int data_array[16];


    data_array[0]= 0x00053902;
    data_array[1]= 0x2c51a9f7;
    data_array[2]= 0x00000082;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00033902;
    data_array[1]= 0x001f1fc0;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000041c1;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00043902;
    data_array[1]= 0x800c00c5;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x0000a0b1;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000002b4;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00033902;
    data_array[1]= 0x002202b6;//0202
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x00000836;//48
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000000b0;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x0000c6b7;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000409be;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x000000e9;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x00000836;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]= 0x00023902;
    data_array[1]= 0x0000773a;
    dsi_set_cmdq(data_array, 2, 1);

	////   gamma 2.0  /////
    data_array[0]= 0x00103902;
    data_array[1]= 0x090300e0;
    data_array[2]= 0x3f0a1608;
    data_array[3]= 0x0a094c78;
    data_array[4]= 0x0f1a1608;
    dsi_set_cmdq(data_array, 5, 1);

    data_array[0]= 0x00103902;
    data_array[1]= 0x191600e1;
    data_array[2]= 0x32050f03;
    data_array[3]= 0x0e044645;
    data_array[4]= 0x0f37350d;
    dsi_set_cmdq(data_array, 5, 1);

    data_array[0]= 0x00351500;
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);

}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(20);

    init_lcm_registers();
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
}

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
    
    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);
    unsigned char y0_MSB = ((y0>>8)&0xFF);
    unsigned char y0_LSB = (y0&0xFF);
    unsigned char y1_MSB = ((y1>>8)&0xFF);
    unsigned char y1_LSB = (y1&0xFF);
    
    unsigned int data_array[16];
    

    data_array[0]= 0x00053902;
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


#if defined(BUILD_LK)
    printf("%s, %d\n", __func__, level);
#elif defined(BUILD_UBOOT)
    printf("%s, %d\n", __func__, level);
#else
    printk("lcm_setbacklight = %d\n", level);
#endif
  
    if(level > 255) 
        level = 255;
    
    data_array[0]= 0x00023902;
    data_array[1] =(0x51|(level<<8));
    dsi_set_cmdq(data_array, 2, 1);
}


static void lcm_setpwm(unsigned int divider)
{
    // TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
    // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
    // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
    unsigned int pwm_clk = 23706 / (1<<divider);	


    return pwm_clk;
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[3];
    unsigned int data_array[16];
    

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);	
    
    data_array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10); 
    
    read_reg_v2(0xD3, buffer, 3);
    id = (buffer[1]<<8) | buffer[2]; //we only need ID

	#if defined(BUILD_LK)
		printf("ili9488 Reg(0xC5) LK %s id = 0x%x 0x%x 0x%x \n", __func__, buffer[0], buffer[1], buffer[2]);
	#else
		printk("ili9488 Reg(0xC5) kernel %s id = 0x%x 0x%x 0x%x \n", __func__, buffer[0], buffer[1], buffer[2]);
	#endif
    
    return (LCM_ILI9488_ID == id)?1:0;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9488_lcm_drv = 
{
    .name			= "ili9488_lcm",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .set_backlight	= lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .compare_id    = lcm_compare_id,
    .update         = lcm_update
};

