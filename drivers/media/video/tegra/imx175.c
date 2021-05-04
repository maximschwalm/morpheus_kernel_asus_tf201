/*
 * imx175.c - imx175 sensor driver
 *
 * Copyright (C) 2012 Nvidia Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/imx175.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>

#include <linux/proc_fs.h>

/* #define ENABLE_4LANE_STREAM */

#ifndef ENABLE_4LANE_STREAM
/* #define FULLRES_2LANE_SCRIPT_FROM_ASUS */
#endif

#define IMX175_REG_GLOBAL_GAIN		0x205
#define IMX175_REG_GLOBAL_COARSE_TIME	0x202
#define IMX175_REG_GLOBAL_FRAME_LENGTH	0x340
#define IMX175_REG_GLOBAL_LINE_LENGTH	0x342
#define IMX175_REG_GLOBAL_GRPHOLD	0x104

struct imx175_reg {
	u16 addr;
	u16 val;
};
static int g_under_the_table = 0;
struct imx175_info {
	struct miscdevice		miscdev_info;
	struct imx175_power_rail	power;
	struct i2c_client		*i2c_client;
	struct imx175_platform_data	*pdata;
        struct imx175_sensordata        *sensor_data;
	struct kobject			*kobj_imx175;
	atomic_t			in_use;
	int				mode;
};    

#define IMX175_TABLE_WAIT_MS 0
#define IMX175_TABLE_END 1

#ifdef ENABLE_4LANE_STREAM
/* 3280 x 2464 x 30fps, 4 lanes, MCLK = 24MHz, PLL freq 640MHz */
static struct imx175_reg mode_3280x2464_4lane[] = {
	{0x0100, 0x00}, /* mode_select */
	{0x0103, 0x01},
	{0x0202, 0x09},	/* coarse _integration_time[15:8] */
	{0x0203, 0xAD},	/* coarse _integration_time[7:0] */
	{0x0301, 0x05},	/* vt_pix_clk_div[7:0] */
	{0x0303, 0x01},	/* vt_sys_clk_div[7:0] */
	{0x0305, 0x06},	/* pre_pll_clk_div[7:0] */
	{0x0309, 0x05},	/* op_pix_clk_div[7:0] */
	{0x030B, 0x01},	/* op_sys_clk_div[7:0] */
	{0x030C, 0x00},
	{0x030D, 0xA0},
	{0x0340, 0x09},	/* frame_length_lines[15:8] */
	{0x0341, 0xB1},	/* frame_length_lines[7:0] */
	{0x0342, 0x0D},	/* line_length_pck[15:8] */
	{0x0343, 0x70},	/* line_length_pck[7:0] */
	{0x0344, 0x00},	/* x_addr_start[15:8] */
	{0x0345, 0x00},	/* x_addr_start[7:0] */
	{0x0346, 0x00},	/* y_addr_start[15:8] */
	{0x0347, 0x00},	/* y_addr_start[7:0] */
	{0x0348, 0x0C},	/* x_addr_end[15:8] */
	{0x0349, 0xCF},	/* x_addr_end[7:0] */
	{0x034A, 0x09},	/* y_addr_end[15:8] */
	{0x034B, 0x9F},	/* y_addr_end[7:0] */
	{0x034C, 0x0C},	/* x_output_size[15:8] */
	{0x034D, 0xD0},	/* x_output_size[7:0] */
	{0x034E, 0x09},	/* y_output_size[15:8] */
	{0x034F, 0xA0},	/* y_output_size[7:0] */
	{0x0390, 0x00},
	{0x3020, 0x10},
	{0x302D, 0x03},
	{0x302F, 0x80},
	{0x3032, 0xA3},
	{0x3033, 0x20},
	{0x3034, 0x24},
	{0x3041, 0x15},
	{0x3042, 0x87},
	{0x3050, 0x35},
	{0x3056, 0x57},
	{0x305D, 0x41},
	{0x3097, 0x69},
	{0x3109, 0x41},
	{0x3148, 0x3F},
	{0x330F, 0x07},
	{0x3344, 0x57},
	{0x3345, 0x1F},
	{0x3364, 0x00},
	{0x3368, 0x18},
	{0x3369, 0x00},
	{0x3370, 0x77},
	{0x3371, 0x2F},
	{0x3372, 0x4F},
	{0x3373, 0x2F},
	{0x3374, 0x2F},
	{0x3375, 0x37},
	{0x3376, 0x9F},
	{0x3377, 0x37},
	{0x33C8, 0x00},
	{0x33D4, 0x0C},
	{0x33D5, 0xD0},
	{0x33D6, 0x09},
	{0x33D7, 0xA0},
	{0x4100, 0x0E},
	{0x4104, 0x32},
	{0x4105, 0x32},
	{0x4108, 0x01},
	{0x4109, 0x7C},
	{0x410A, 0x00},
	{0x410B, 0x00},

	{0x0100, 0x01},
	{IMX175_TABLE_WAIT_MS, 5},
	{IMX175_TABLE_END, 0x00}
};

/* 1640 x 1232 x 30fps, 4 lanes, MCLK = 24MHz, PLL freq 640MHz */
static struct imx175_reg mode_1640x1232_4lane[] = {
	{0x0100, 0x00}, /* mode_select */
	{0x0103, 0x01},
	{0x0202, 0x09},	/* coarse _integration_time[15:8] */
	{0x0203, 0xAD},	/* coarse _integration_time[7:0] */
	{0x0301, 0x05},	/* vt_pix_clk_div[7:0] */
	{0x0303, 0x01},	/* vt_sys_clk_div[7:0] */
	{0x0305, 0x06},	/* pre_pll_clk_div[7:0] */
	{0x0309, 0x05},	/* op_pix_clk_div[7:0] */
	{0x030B, 0x01},	/* op_sys_clk_div[7:0] */
	{0x030C, 0x00},
	{0x030D, 0xA0},
	{0x0340, 0x09},	/* frame_length_lines[15:8] */
	{0x0341, 0xB1},	/* frame_length_lines[7:0] */
	{0x0342, 0x0D},	/* line_length_pck[15:8] */
	{0x0343, 0x70},	/* line_length_pck[7:0] */
	{0x0344, 0x00},	/* x_addr_start[15:8] */
	{0x0345, 0x00},	/* x_addr_start[7:0] */
	{0x0346, 0x00},	/* y_addr_start[15:8] */
	{0x0347, 0x00},	/* y_addr_start[7:0] */
	{0x0348, 0x0C},	/* x_addr_end[15:8] */
	{0x0349, 0xCF},	/* x_addr_end[7:0] */
	{0x034A, 0x09},	/* y_addr_end[15:8] */
	{0x034B, 0x9F},	/* y_addr_end[7:0] */
	{0x034C, 0x06},	/* x_output_size[15:8] */
	{0x034D, 0x68},	/* x_output_size[7:0] */
	{0x034E, 0x04},	/* y_output_size[15:8] */
	{0x034F, 0xD0},	/* y_output_size[7:0] */
	{0x0390, 0x01},
	{0x3020, 0x10},
	{0x302D, 0x03},
	{0x302F, 0x80},
	{0x3032, 0xA3},
	{0x3033, 0x20},
	{0x3034, 0x24},
	{0x3041, 0x15},
	{0x3042, 0x87},
	{0x3050, 0x35},
	{0x3056, 0x57},
	{0x305D, 0x41},
	{0x3097, 0x69},
	{0x3109, 0x41},
	{0x3148, 0x3F},
	{0x330F, 0x07},
	{0x3344, 0x57},
	{0x3345, 0x1F},
	{0x3364, 0x00},
	{0x3368, 0x18},
	{0x3369, 0x00},
	{0x3370, 0x77},
	{0x3371, 0x2F},
	{0x3372, 0x4F},
	{0x3373, 0x2F},
	{0x3374, 0x2F},
	{0x3375, 0x37},
	{0x3376, 0x9F},
	{0x3377, 0x37},
	{0x33C8, 0x00},
	{0x33D4, 0x06},
	{0x33D5, 0x68},
	{0x33D6, 0x04},
	{0x33D7, 0xD0},
	{0x4100, 0x0E},
	{0x4104, 0x32},
	{0x4105, 0x32},
	{0x4108, 0x01},
	{0x4109, 0x7C},
	{0x410A, 0x00},
	{0x410B, 0x00},

	{0x0100, 0x01},
	{IMX175_TABLE_WAIT_MS, 5},
	{IMX175_TABLE_END, 0x00}
};

#else	/* 2 lane stream enabled */

#ifndef FULLRES_2LANE_SCRIPT_FROM_ASUS
/* 3280 x 2464 x 21fps, 2 lanes, MCLK = 24MHz, PLL freq 884MHz */
static struct imx175_reg mode_3280x2464_2lane[] = {
	{0x0100, 0x00}, /* mode_select */
	{0x0103, 0x01},
	{0x0202, 0x0A},	/* coarse _integration_time[15:8] */
	{0x0203, 0x06},	/* coarse _integration_time[7:0] */
	{0x0301, 0x0A},	/* vt_pix_clk_div[7:0] */
	{0x0303, 0x01},	/* vt_sys_clk_div[7:0] */
	{0x0305, 0x06},	/* pre_pll_clk_div[7:0] */
	{0x0309, 0x0A},	/* op_pix_clk_div[7:0] */
	{0x030B, 0x01},	/* op_sys_clk_div[7:0] */
	{0x030C, 0x00},
	{0x030D, 0xDD},
	{0x0340, 0x0A},	/* frame_length_lines[15:8] */
	{0x0341, 0x0A},	/* frame_length_lines[7:0] */
	{0x0342, 0x0D},	/* line_length_pck[15:8] */
	{0x0343, 0x70},	/* line_length_pck[7:0] */
	{0x0344, 0x00},	/* x_addr_start[15:8] */
	{0x0345, 0x00},	/* x_addr_start[7:0] */
	{0x0346, 0x00},	/* y_addr_start[15:8] */
	{0x0347, 0x00},	/* y_addr_start[7:0] */
	{0x0348, 0x0C},	/* x_addr_end[15:8] */
	{0x0349, 0xCF},	/* x_addr_end[7:0] */
	{0x034A, 0x09},	/* y_addr_end[15:8] */
	{0x034B, 0x9F},	/* y_addr_end[7:0] */
	{0x034C, 0x0C},	/* x_output_size[15:8] */
	{0x034D, 0xD0},	/* x_output_size[7:0] */
	{0x034E, 0x09},	/* y_output_size[15:8] */
	{0x034F, 0xA0},	/* y_output_size[7:0] */
	{0x0390, 0x00},
	{0x3020, 0x10},
	{0x302D, 0x03},
	{0x302F, 0x80},
	{0x3032, 0xA3},
	{0x3033, 0x20},
	{0x3034, 0x24},
	{0x3041, 0x15},
	{0x3042, 0x87},
	{0x3050, 0x35},
	{0x3056, 0x57},
	{0x305D, 0x41},
	{0x3097, 0x69},
	{0x3109, 0x41},
	{0x3148, 0x3F},
	{0x330F, 0x07},
	{0x3344, 0x67},
	{0x3345, 0x1F},
	{0x3364, 0x02},
	{0x3368, 0x18},
	{0x3369, 0x00},
	{0x3370, 0x7F},
	{0x3371, 0x37},
	{0x3372, 0x67},
	{0x3373, 0x3F},
	{0x3374, 0x3F},
	{0x3375, 0x47},
	{0x3376, 0xCF},
	{0x3377, 0x47},
	{0x33C8, 0x00},
	{0x33D4, 0x0C},
	{0x33D5, 0xD0},
	{0x33D6, 0x09},
	{0x33D7, 0xA0},
	{0x4100, 0x0E},
	{0x4104, 0x32},
	{0x4105, 0x32},
	{0x4108, 0x01},
	{0x4109, 0x7C},
	{0x410A, 0x00},
	{0x410B, 0x00},

	{0x0100, 0x01},
	{IMX175_TABLE_WAIT_MS, 5},
	{IMX175_TABLE_END, 0x00}
};

#else
/* IMX175_3280x2464_Bayer-10_2-lane_20fps */
static struct imx175_reg mode_3280x2464_2lane[] = {
	/* PLL settings */
	{0x0301, 0x0a},	/* VT_PIX_CLK_DIV */
	{0x0303, 0x01},	/* VT_SYS_CLK_DIV */
	{0x0305, 0x09},	/* PRE_PLL_CLK_DIV */
	{0x0309, 0x0a},	/* OP_PIX_CLK_DIV */
	{0x030b, 0x01},	/* OP_SYS_CLK_DIV */
	{0x030c, 0x01},	/* PLL_MULTIPLIER[10:8] */
	{0x030d, 0x4f},	/* PLL_MULTIPLIER[7:0] */
	{0x030e, 0x01},
	{IMX175_TABLE_WAIT_MS, 10},

	/* phase settings */
	/* full settings */
	
	{0x0100, 0x00}, /* original 0x03. 0x01 to meet asus mechanical */
	{0x0103, 0x01},
	{0x0202, 0x0a},	/* COARSE_INTEGRATION TIME */
	{0x0203, 0x06},
	{0x0301, 0x0a},
	{0x0303, 0x01},
	{0x0305, 0x06},
	{0x0309, 0x0a},
	{0x030B, 0x01},
	{0x030B, 0x00},
	{0x030D, 0xDD},
	{0x0340, 0x0a},	/* FRAME LENGTH */
	{0x0341, 0x0a},
	{0x0342, 0x0d},	/* LINE LENGTH */
	{0x0343, 0x70},
	{0x0344, 0x00}, /* X_ADDR_START */
	{0x0345, 0x00},
	{0x0346, 0x00}, /* Y_ADDR_START */
	{0x0347, 0x00},
	{0x0348, 0x0c}, /* X_ADDR_END */
	{0x0349, 0xcf},
	{0x034a, 0x09}, /* Y_ADDR_END */
	{0x034b, 0x9f},
	{0x034c, 0x0c}, /* X_OUTPUT_SIZE */
	{0x034d, 0xd0},
	{0x034e, 0x09}, /* Y_OUTPUT_SIZE */
	{0x034f, 0xa0},
	{0x0390, 0x00}, /* BINNING_MODE: 0, no binning; 1, 2x2; 2, 4x4 */
	{0x3020, 0x10},
	{0x302d, 0x03},
	{0x302f, 0x80},
	{0x3032, 0xa3},
	{0x3033, 0x20},
	{0x3034, 0x24},
	{0x3041, 0x15},
	{0x3042, 0x87},
	{0x3050, 0x35},
	{0x3056, 0x57},
	{0x305d, 0x41},
	{0x3097, 0x69},
	{0x3109, 0x41},
	{0x3148, 0x3f},
	{0x330f, 0x07},
	{0x3344, 0x6f},
	{0x3345, 0x1f},
	{0x3364, 0x02},
	{0x3368, 0x18},
	{0x3369, 0x00},
	{0x3370, 0x7f},
	{0x3371, 0x37},
	{0x3372, 0x67},
	{0x3373, 0x3f},
	{0x3374, 0x3f},
	{0x3375, 0x47},
	{0x3376, 0xcf},
	{0x3377, 0x47},
	{0x33C8, 0x00},
	{0x33d4, 0x0c},
	{0x33d5, 0xd0},
	{0x33d6, 0x09},
	{0x33d7, 0xa0},
	{0x4100, 0x0e},
	{0x4104, 0x32},
	{0x4105, 0x32},
	{0x4108, 0x01},
	{0x4109, 0x7c},
	{0x410a, 0x00},
	{0x410b, 0x00},
	
	{0x0100, 0x01}, /* 0: sw standby; 1: streaming on */
	{IMX175_TABLE_WAIT_MS, 5},
	{IMX175_TABLE_END, 0x00}
};
#endif

/* 1640 x 1232 x 30fps, 2 lanes, MCLK = 24MHz, PLL freq 648MHz */
static struct imx175_reg mode_1640x1232_2lane[] = {
	{0x0100, 0x00}, /* mode_select */
	{0x0103, 0x01}, /*SW Reset*/
	{0x0202, 0x04},	/* coarse _integration_time[15:8] */
	{0x0203, 0xE4},	/* coarse _integration_time[7:0] */
	{0x0301, 0x0A},	/* vt_pix_clk_div[7:0] */
	{0x0303, 0x01},	/* vt_sys_clk_div[7:0] */
	{0x0305, 0x06},	/* pre_pll_clk_div[7:0] */
	{0x0309, 0x0A},	/* op_pix_clk_div[7:0] */
	{0x030B, 0x01},	/* op_sys_clk_div[7:0] */
	{0x030C, 0x00},
	{0x030D, 0xA2},
	{0x0340, 0x04},	/* frame_length_lines[15:8] */
	{0x0341, 0xE8},	/* frame_length_lines[7:0] */
	{0x0342, 0x0D},	/* line_length_pck[15:8] */
	{0x0343, 0x70},	/* line_length_pck[7:0] */
	{0x0344, 0x00},	/* x_addr_start[15:8] */
	{0x0345, 0x00},	/* x_addr_start[7:0] */
	{0x0346, 0x00},	/* y_addr_start[15:8] */
	{0x0347, 0x00},	/* y_addr_start[7:0] */
	{0x0348, 0x0C},	/* x_addr_end[15:8] */
	{0x0349, 0xCF},	/* x_addr_end[7:0] */
	{0x034A, 0x09},	/* y_addr_end[15:8] */
	{0x034B, 0x9F},	/* y_addr_end[7:0] */
	{0x034C, 0x06},	/* x_output_size[15:8] */
	{0x034D, 0x68},	/* x_output_size[7:0] */
	{0x034E, 0x04},	/* y_output_size[15:8] */
	{0x034F, 0xD0},	/* y_output_size[7:0] */
	{0x0390, 0x01},
	{0x3020, 0x10},
	{0x302D, 0x03},
	{0x302F, 0x80},
	{0x3032, 0xA3},
	{0x3033, 0x20},
	{0x3034, 0x24},
	{0x3041, 0x15},
	{0x3042, 0x87},
	{0x3050, 0x35},
	{0x3056, 0x57},
	{0x305D, 0x41},
	{0x3097, 0x69},
	{0x3109, 0x41},
	{0x3148, 0x3F},
	{0x330F, 0x07},
	{0x3344, 0x57},
	{0x3345, 0x1F},
	{0x3364, 0x02},
	{0x3368, 0x18},
	{0x3369, 0x00},
	{0x3370, 0x77},
	{0x3371, 0x2F},
	{0x3372, 0x4F},
	{0x3373, 0x2F},
	{0x3374, 0x2F},
	{0x3375, 0x37},
	{0x3376, 0x9F},
	{0x3377, 0x37},
	{0x33C8, 0x00},
	{0x33D4, 0x06},
	{0x33D5, 0x68},
	{0x33D6, 0x04},
	{0x33D7, 0xD0},
	{0x4100, 0x0E},
	{0x4104, 0x32},
	{0x4105, 0x32},
	{0x4108, 0x01},
	{0x4109, 0x7C},
	{0x410A, 0x00},
	{0x410B, 0x00},

	{0x0100, 0x01},
	{IMX175_TABLE_WAIT_MS, 5},
	{IMX175_TABLE_END, 0x00}
};
#endif
enum {
	IMX175_MODE_3280x2464,
	IMX175_MODE_1640x1232,
};

static struct imx175_reg *mode_table[] = {
	
	[IMX175_MODE_3280x2464] = mode_3280x2464_2lane,
	[IMX175_MODE_1640x1232] = mode_1640x1232_2lane,

};

static inline void
msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base*1000, delay_base*1000 + 500);
}

/* 2 regs to program frame length */
static inline void imx175_get_frame_length_regs(struct imx175_reg *regs,
						u32 frame_length)
{
	regs->addr = IMX175_REG_GLOBAL_FRAME_LENGTH;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX175_REG_GLOBAL_FRAME_LENGTH + 1;
	(regs + 1)->val = (frame_length) & 0xff;
}

/* 2 regs to program coarse time */
static inline void imx175_get_coarse_time_regs(struct imx175_reg *regs,
						u32 coarse_time)
{
	regs->addr = IMX175_REG_GLOBAL_COARSE_TIME;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX175_REG_GLOBAL_COARSE_TIME + 1;
	(regs + 1)->val = (coarse_time) & 0xff;
}

/* 1 reg to program gain */
static inline void imx175_get_gain_reg(struct imx175_reg *regs, u16 gain)
{
	regs->addr = IMX175_REG_GLOBAL_GAIN;
	regs->val = gain;
}

static int imx175_read_reg_bulk(struct i2c_client *client,
	u16 addr, u8 *buf, u8 num)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = num;
	msg[1].buf = buf;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2) 
		return -EINVAL;
	
	return 0;
}

//BSP++ Jimmy add

static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

static int sensor_write_reg_word(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
    data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);
    //printk("jimmy: [sensor_write_reg_word] client->addr:%x\n",client->addr);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err(" , retrying %x %x\n",
		       addr, val);
		pr_err("Bayer_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
	} while (retry <= 3);

	return err;
}

static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("%s(%d) : i2c transfer failed, count 0x%x, err= 0x%x\n", __FUNCTION__, __LINE__, msg.addr, err);
//		msleep(3);
	} while (retry <= 3);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = 0xAAAA;
	}

	return err;
}
//BSP-- Jimmy add



static inline int imx175_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	return imx175_read_reg_bulk(client, addr, val, 1);
}

static int imx175_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
    //printk("%s\n",__func__);
	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8)(addr >> 8);
	data[1] = (u8)(addr & 0xff);
	data[2] = (u8)(val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	//pr_info("imx175: i2c transfer under transferring %x %x\n", addr, val); 
	if (err == 1)
		return 0;
	pr_err("imx175: i2c transfer failed, %x %x\n",
	       addr, val);

	return err;
}

static int imx175_write_table(struct i2c_client *client,
			      const struct imx175_reg table[],
			      const struct imx175_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct imx175_reg *next;
	int i;
	u16 val;

	pr_info("imx175: imx175_write_table Start\n"); 
	for (next = table; next->addr != IMX175_TABLE_END; next++) {
		if (next->addr == IMX175_TABLE_WAIT_MS) {
			 pr_info("imx175: imx175_write_table :"
			" IMX175_TABLE_WAIT_MS "); 
			msleep_range(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		err = imx175_write_reg(client, next->addr, val);
		if (err) {
			pr_err("imx175: imx175_write_table : err\n");
			return err;
		}
	}
	return 0;
}

static inline int get_mode_index(int xres, int yres)
{
	
	if (xres == 3264 && yres == 2448)
	{
        pr_err("%s:  resolution get_mode_index %d %d\n", __func__, xres, yres);
		return IMX175_MODE_3280x2464;
	}     
	else if (xres == 1640 && yres == 1216)
	{
		pr_err("%s: invalid resolution supplied to get mode %d %d\n", __func__, xres, yres);
		return IMX175_MODE_1640x1232;
	}
	return -1;
}

static int imx175_set_mode(struct imx175_info *info, struct imx175_mode *mode)
{
    u16 tmp1, tmp2, tmp3, tmp4, tmp5, tmp6;
	struct imx175_reg reg_override[5];
	int sensor_mode = get_mode_index(mode->xres, mode->yres);
	int err;

	//IMX175_REG_GLOBAL
	if (sensor_mode < 0)
		return -EINVAL;

	/* get a list of override regs for the asking frame length,
	  coarse integration time, and gain. */
	imx175_get_frame_length_regs(reg_override, mode->frame_length);
	imx175_get_coarse_time_regs(reg_override + 2, mode->coarse_time);
	imx175_get_gain_reg(reg_override + 4, mode->gain);
    printk("++++++imx175_get_gain_reg+++++++");
	err = imx175_write_table(info->i2c_client, mode_table[sensor_mode], reg_override, 5);
	
	if (err){
		pr_info("%s imx175_write_table err --: xres %u yres %u\n", __func__, mode->xres, mode->yres);
		return err;
	}
	info->mode = sensor_mode;
	pr_info("%s--: xres %u yres %u\n", __func__, mode->xres, mode->yres);
	
	return 0;
}

static int sensor_write_reg_dword(struct i2c_client *client, u16 addr, u32 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[6];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	data[2] = (u8) ((val >> 24) & 0xff);
	data[3] = (u8) ((val >> 16) & 0xff);
	data[4] = (u8) ((val >> 8) & 0xff);
	data[5] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 6;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("Bayer_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		pr_err("Bayer_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
	} while (retry <= 3);

	return err;
}

static int sensor_read_reg_word(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;
    msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;
    /* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;
        
	err = i2c_transfer(client->adapter, msg, 2);
    printk("Asus: after i2c_transfer\n");
	if (err != 2)
		return -EINVAL;

	swap(*(data+2),*(data+3)); //swap high and low byte to match table format
	memcpy(val, data+2, 2);

	return 0;
}

static int sensor_read_reg_dword(struct i2c_client *client, u16 addr, u32 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 4;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	/* swap high and low byte to match table format */
	swap(*(data+2),*(data+5));
	swap(*(data+3),*(data+4));
	memcpy(val, data+2, 4);

	return 0;
}

static int imx175_get_status(struct imx175_info *info, u8 *dev_status)
{
	*dev_status = 0;
	return 0;
}

static int imx175_set_frame_length(struct imx175_info *info, u32 frame_length)
{
	struct imx175_reg reg_list[2];
	int i = 0;
	int ret;
	/* pr_info("%s\n", __func__); */

	imx175_get_frame_length_regs(reg_list, frame_length);
	ret = imx175_write_reg(info->i2c_client,
			IMX175_REG_GLOBAL_GRPHOLD, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++)	{
		ret = imx175_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	ret = imx175_write_reg(info->i2c_client,
			IMX175_REG_GLOBAL_GRPHOLD, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int imx175_set_coarse_time(struct imx175_info *info, u32 coarse_time)
{
	int ret;

	struct imx175_reg reg_list[3];
	int i = 0;
	/* pr_info("%s %x\n", __func__, coarse_time); */

	imx175_get_coarse_time_regs(reg_list, coarse_time);

	ret = imx175_write_reg(info->i2c_client,
			IMX175_REG_GLOBAL_GRPHOLD, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++)	{
		ret = imx175_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	ret = imx175_write_reg(info->i2c_client,
			IMX175_REG_GLOBAL_GRPHOLD, 0x0);
	if (ret)
		return ret;

	return 0;
}

static int imx175_set_gain(struct imx175_info *info, u16 gain)
{
	int ret;
	struct imx175_reg reg_list;
	//pr_info("%s %x\n", __func__, gain);

	imx175_get_gain_reg(&reg_list, gain);

	ret = imx175_write_reg(info->i2c_client,
			IMX175_REG_GLOBAL_GRPHOLD, 0x1);
	if (ret) {
		//printk("(%d), ret = %d", __LINE__, ret);
		return ret;
	}
	ret = imx175_write_reg(info->i2c_client,
			reg_list.addr, reg_list.val);

	ret = imx175_write_reg(info->i2c_client,
			IMX175_REG_GLOBAL_GRPHOLD, 0x0);
	if (ret) {
		//printk("(%d), ret = %d", __LINE__, ret);
		return ret;
	}
	return 0;
}

static int imx175_get_setting(struct imx175_info *info,
			struct imx175_setting *setting)
{
	struct imx175_reg *reg_list;
	int xres = setting->xres;
	int yres = setting->yres;
	int sensor_mode = get_mode_index(xres, yres);

	pr_info("%s: xres %u yres %u sensor_mode = %d \n", __func__, xres, yres, sensor_mode);

	if (sensor_mode < 0)
		return -EINVAL;

	memset(setting, 0, sizeof(struct imx175_setting));
	setting->xres = xres;
	setting->yres = yres;

	for (reg_list = mode_table[sensor_mode];
		reg_list->addr != IMX175_TABLE_END; reg_list++) {
		switch (reg_list->addr) {
		/* vt_pix_clk_div 0x0301[3:0]*/
		/* vt_sys_clk_div 0x0303[1:0]*/
		/* pre_pll_clk_div 0x0305[3:0]*/
		/* op_pix_clk_div 0x0309[3:0]*/
		/* op_sys_clk_div 0x030b[1:0]*/
		/* pll_multiplier 0x030c/0x030d[10:0]*/
		/* frame_length_lines 0x0340[15:0] */
		/* coarse _integration_time 0x202[15:0] */
		/* line_length_pck 0x342[15:0] */
		/* x_output_size 0x34c[15:0] */
		/* y_output_size 0x34e[15:0] */
		case 0x0301:
			setting->vt_pix_div = (u32)(reg_list->val & 0x0f);
			break;
		case 0x0303:
			setting->vt_sys_div = (u32)(reg_list->val & 0x03);
			break;
		case 0x0305:
			setting->pre_div = (u32)(reg_list->val & 0x0f);
			break;
		case 0x0309:
			setting->op_pix_div = (u32)(reg_list->val & 0x0f);
			break;
		case 0x030b:
			setting->op_sys_div = (u32)(reg_list->val & 0x0f);
			break;
		case 0x030c:
			setting->multiplier = (u32)(reg_list->val & 0x07) << 8;
			break;
		case 0x030d:
			setting->multiplier |= ((u32)reg_list->val) & 0xff;
			break;
		case 0x0340:
			setting->frame_len = (u32)reg_list->val << 8;
			break;
		case 0x0341:
			setting->frame_len |= (u32)reg_list->val;
			break;
		case 0x0202:
			setting->coarse_tim = (u32)reg_list->val << 8;
			break;
		case 0x0203:
			setting->coarse_tim |= (u32)reg_list->val;
			break;
		case 0x0342:
			setting->line_len = (u32)reg_list->val << 8;
			break;
		case 0x0343:
			setting->line_len |= (u32)reg_list->val;
			break;
		case 0x034c:
			setting->out_width = (u32)reg_list->val << 8;
			break;
		case 0x034d:
			setting->out_width |= (u32)reg_list->val;
			break;
		case 0x034e:
			setting->out_height = (u32)reg_list->val << 8;
			break;
		case 0x034f:
			setting->out_height |= (u32)reg_list->val;
			break;
		default:
			break;
		}
	}

	return 0;
}

static long imx175_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct imx175_info *info = file->private_data;
    //info = file->private_data;
	int err;
	u16 dbg1, dbg2;
	pr_info("%s, %d, cmd=%x\n", __func__,__LINE__ ,cmd); 
	//printk("%s, %d, cmd=%x\n", __func__,__LINE__ ,cmd);
	switch (cmd) {
	case IMX175_IOCTL_SET_MODE:
	{
		printk("In IMX175_IOCTL_SET_MODE \n");
		struct imx175_mode mode;
		if (copy_from_user(&mode, (const void __user *)arg, sizeof(struct imx175_mode))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return imx175_set_mode(info, &mode);
	}
	case IMX175_IOCTL_SET_FRAME_LENGTH:
        return imx175_set_frame_length(info, (u32)arg);
	case IMX175_IOCTL_SET_COARSE_TIME:
        return imx175_set_coarse_time(info, (u32)arg);
	case IMX175_IOCTL_SET_GAIN:
        return imx175_set_gain(info, (u16)arg);
	case IMX175_IOCTL_GET_STATUS:
	{
		u8 status;

		err = imx175_get_status(info, &status);
        printk("%s  IMX175_IOCTL_GET_STATUS\n",__func__);
		if (err) {
            printk("%s IMX175_IOCTL_GET_STATUS error\n",__func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, &status,
				 sizeof(status))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case IMX175_IOCTL_GET_SETTING:
	{
		struct imx175_setting setting;
        
		if (copy_from_user(&setting,
				   (const void __user *)arg,
				   sizeof(setting))) {
			pr_info("%s %d\n", __func__, __LINE__);
			info = file->private_data;
            return -EFAULT;
		}
		err = imx175_get_setting(info, &setting);
		if (err) {
            printk("%s IMX175_IOCTL_GET_SETTING error\n",__func__);
			return err;
		}
		if (copy_to_user((void __user *)arg, &setting,
				 sizeof(setting))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
    case IMX175_IOCTL_GET_SENSORDATA:
	{
#if 0
		err = imx175_get_sensor_id(info);
		if (err) {
			pr_err("%s %d %d\n", __func__, __LINE__, err);
			return err;
		}
		if (copy_to_user((void __user *)arg,
				&info->sensor_data,
				sizeof(struct imx175_sensordata))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
#endif
		return 0;
	}
	default:
		pr_info("imx175_ioctl : unknown cmd.\n");
		return -EINVAL;
	}
	return 0;
}

static int imx175_power_on(struct imx175_info *info)
{
	//struct imx175_power_rail *pw = &info->power;

	pr_info("%s\n", __func__);

    if (info->pdata && info->pdata->power_on)
    info->pdata->power_on();//jimmy
    msleep_range(1);
	info->mode = -1;

	return 0;
}

static int imx175_power_off(struct imx175_info *info)
{
	//struct imx175_power_rail *pw = &info->power;

	pr_info("%s\n", __func__);

       
        info->pdata->power_off();
        //tegra_camera_mclk_on_off(0);
	info->mode = -1;

	return 0;
}

static int imx175_open(struct inode *inode, struct file *file)
{
	struct imx175_info *info;
    u16 tmp1, tmp2, tmp3;
	pr_info("%s\n", __func__);
	
	info = container_of(file->private_data,
			struct imx175_info, miscdev_info);
	/* check if device is in use */
	/*
	if (atomic_xchg(&info->in_use, 1)) {
		pr_info("BUSY.\n");
		return -EBUSY;
	}
	*/
    file->private_data = info;
    if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	

    //info->mode = -1;
    //printk("***********");
	msleep(1);
	//jimmy add for debug ++
    sensor_read_reg(info->i2c_client, 0x0000, &tmp1);
    printk("open get ID 1\n");
    printk("0x%x address 0x%x\n", tmp1, info->i2c_client->addr);
    sensor_read_reg(info->i2c_client, 0x0001, &tmp2);
    printk("open get ID 2\n");
    printk("0x%x\n", tmp2);
    printk("0x%x address 0x%x\n", tmp2, info->i2c_client->addr);
    sensor_write_reg(info->i2c_client, 0x0100, 0x01);
    sensor_read_reg(info->i2c_client, 0x0100, &tmp3);
    printk("open write read 0x0100\n");
    printk("0x%x\n", tmp3);

   //jimmy add for debug --
	
	return 0;
}
int imx175_release(struct inode *inode, struct file *file)
{
	struct imx175_info *info = file->private_data;

	pr_info("imx175: imx175_release");
	if (info->pdata && info->pdata->power_off) {
		info->pdata->power_off();
	}
	file->private_data = NULL;

	/* warn if device already released */
	//WARN_ON(!atomic_xchg(&info->in_use, 0));
	return 0;
}
#if 0
static ssize_t dbg_imx175_chip_id_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int imx175_sysfs_init(struct imx175_info *info);

static ssize_t dbg_imx175_chip_id_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	struct imx175_info *info = file->private_data;

	u16 chip_id = 0x0;
	int err = 0;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	else {
		len = snprintf(bp, dlen, "imx175 info isn't enough for power_on.\n");
		tot += len; bp += len; dlen -= len;
	}
	
	err = sensor_read_reg_word(info->i2c_client, 0x0, &chip_id);
	len = snprintf(bp, dlen, "chip_id= 0x%x, err= %d\n", chip_id, err);
	tot += len; bp += len; dlen -= len;
        
	if (info->pdata && info->pdata->power_off) {
		info->pdata->power_off();
	} else {
		len = snprintf(bp, dlen, "imx175 info isn't enough for power_off.\n");
		tot += len; bp += len; dlen -= len;
	}
	
	

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_imx175_chip_id_fops = {
	.open		= dbg_imx175_chip_id_open,
	.read		= dbg_imx175_chip_id_read,
};

/* --- get_mi1040_reg --- */
static ssize_t dbg_get_imx175_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_get_imx175_reg_write(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	char debug_buf[256];
	int cnt, byte_num = 0;
	char ofst_str[7];
	unsigned int ofst = 0;
	unsigned int val = 0;
	struct imx175_info *info = file->private_data;
	
	printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */
	cnt = sscanf(debug_buf, "%s %d", ofst_str, &byte_num);

	//if (sensor_opened == false) {
	//		printk("%s: Please open imx175 first.\n", __FUNCTION__);
	//} else {
		/* Str to Int*/
		if ((ofst_str[0] == '0') && (ofst_str[1] == 'x')) {
			/* Parse ofst */
			int i = 0;
			int err = 0;

			for (i = 2; i < 6; i++) {
				if ((ofst_str[i] >= '0') && (ofst_str[i] <= '9'))
					ofst = ofst * 16 + ofst_str[i] - '0';
				else if ((ofst_str[i] >= 'a') && (ofst_str[i] <= 'f'))
					ofst = ofst * 16 + ofst_str[i] - 'a' + 10;
				else if ((ofst_str[i] >= 'A') && (ofst_str[i] <= 'F'))
					ofst = ofst * 16 + ofst_str[i] - 'A' + 10;
				else {
					break;
				}
			}

			/* Write the Reg */
			printk("Offset= %d(0x%x); byte_num= %d\n", ofst, ofst, byte_num);
			if (byte_num == 1)
				err = sensor_read_reg(info->i2c_client, ofst, &val);
			else if (byte_num == 2)
				err = sensor_read_reg_word(info->i2c_client, ofst, &val);
			else if (byte_num == 4)
				err = sensor_read_reg_dword(info->i2c_client, ofst, &val);
			else {
				printk("%s: Byte Num should be 1, 2 or 4.\n", __FUNCTION__);
				err = -1;
			}

			if (err == 0) {
				g_under_the_table = val;
				printk("0x%x\n", val);
			} else {
				g_under_the_table = 0xabcdabcd;
				printk("%s: Read Reg Error: %d\n", __FUNCTION__, err);
			}
		} else {
			/* Wrong Usage */
			printk("%s: Wrong Format.\n", __FUNCTION__);
			printk("Usage: echo [Reg Ofst] [Byte Num] > /d/imx175/get_reg\n");
			printk("EX: echo 0x0 2 > /d/imx175/get_reg\n");
		}
	//}
	return count;
}

static const struct file_operations dbg_get_imx175_reg_fops = {
	.open		= dbg_get_imx175_reg_open,
	.write		= dbg_get_imx175_reg_write,
};


static ssize_t dbg_set_imx175_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_set_imx175_reg_write(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	char debug_buf[256];
	int cnt, byte_num;
	char ofst_str[7], reg_val_str[11];
	unsigned int ofst, reg_val= 0;
	struct imx175_info *info = file->private_data;
	
	printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */
	cnt = sscanf(debug_buf, "%s %s %d", ofst_str, reg_val_str, &byte_num);

	//printk("cnt= %d; ofst_str=\"%s\"; reg_val_str=\"%s\"; byte_num= %d\n", cnt, ofst_str, reg_val_str, byte_num);

	//if (sensor_opened == false) {
	//		printk("%s: Please open imx175 first.\n", __FUNCTION__);
	//} else {
		/* Str to Int*/
		if (((ofst_str[0] == '0') && (ofst_str[1] == 'x')) &&
			((reg_val_str[0]=='0')) && (reg_val_str[1]=='x')) {
			/* Parse ofst */
			int i = 0;
			int err = 0;

			/* Parse Reg Offset */
			for (i = 2; i < 6; i++) {
				if ((ofst_str[i] >= '0') && (ofst_str[i] <= '9'))
					ofst = ofst * 16 + ofst_str[i] - '0';
				else if ((ofst_str[i] >= 'a') && (ofst_str[i] <= 'f'))
					ofst = ofst * 16 + ofst_str[i] - 'a' + 10;
				else if ((ofst_str[i] >= 'A') && (ofst_str[i] <= 'F'))
					ofst = ofst * 16 + ofst_str[i] - 'A' + 10;
				else {
					break;
				}
			}
			ofst &= 0xFFFF;

			/* Parse Reg Value */
			for (i = 2; i < 11; i++) {
				// printk("i =%d\n", i);

				if ((reg_val_str[i] >= '0') && (reg_val_str[i] <= '9'))
					reg_val = reg_val * 16 + reg_val_str[i] - '0';
				else if ((reg_val_str[i] >= 'a') && (reg_val_str[i] <= 'f'))
					reg_val = reg_val * 16 + reg_val_str[i] - 'a' + 10;
				else if ((reg_val_str[i] >= 'A') && (reg_val_str[i] <= 'F'))
					reg_val = reg_val * 16 + reg_val_str[i] - 'A' + 10;
				else {
					break;
				}
			}

			/* Write the Reg */
			printk("Offset= %d(0x%x); Reg_Val= %d(0x%x)\n", ofst, ofst, reg_val, reg_val);
			if (byte_num == 2) {
				reg_val &= 0xFFFF;
				printk("%s: sensor_write_reg_word(0x%04x, 0x%04x)\n", __FUNCTION__, ofst, reg_val);
				err = sensor_write_reg_word(info->i2c_client, ofst, reg_val);
			}
			else if (byte_num == 1) {
				reg_val &= 0xFF;
				printk("%s: sensor_write_reg(0x%04x, 0x%02x)\n", __FUNCTION__, ofst, reg_val);
				err = sensor_write_reg(info->i2c_client, ofst, reg_val);
			} else if (byte_num == 4) {
				printk("%s: sensor_write_reg_dword(0x%04x, 0x%08x)\n", __FUNCTION__, ofst, reg_val);
				err = sensor_write_reg_dword(info->i2c_client, ofst, reg_val);
			} else {
				printk("%s: Byte Num should be 1, 2 or 4.\n", __FUNCTION__);
				err = -1;
			}
			if (err == 0) {
				printk("%s: Set Reg successfully\n", __FUNCTION__);
			} else {
				printk("%s: Read Reg Error: %d\n", __FUNCTION__, err);
			}
		} else {
			/* Wrong Usage */
			printk("%s: Wrong Format.\n", __FUNCTION__);
			printk("Usage: echo [Reg Ofst] [Reg Val] [Byte Num]> /d/mi1040/set_reg\n");
			printk("EX: echo 0x098e 0xc874 2 > /d/mi1040/set_reg\n");
		}
	//}

	return count;
}


static const struct file_operations dbg_set_imx175_reg_fops = {
	.open		= dbg_set_imx175_reg_open,
	.write		= dbg_set_imx175_reg_write,
};
#endif
static const struct file_operations imx175_fileops = {
	.owner = THIS_MODULE,
	.open = imx175_open,
	.unlocked_ioctl = imx175_ioctl,
	.release = imx175_release,
};

static struct miscdevice imx175_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx175",
	.fops = &imx175_fileops,
};

static int imx175_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct imx175_info *info;
        struct dentry *debugfs_dir;
	int err;
    //u16 tmp1, tmp2;
     
	pr_info("imx175: probing sensor.\n");
	info = kzalloc(sizeof(struct imx175_info), GFP_KERNEL);
	if (!info) {
		pr_err("imx175: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	memcpy(&(info->miscdev_info),
		&imx175_device,
		sizeof(struct miscdevice));

	err = misc_register(&(info->miscdev_info));
	if (err) {
		pr_err("imx175: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	atomic_set(&info->in_use, 0);
	info->mode = -1;

	i2c_set_clientdata(client, info);
	return 0;
}

static int imx175_remove(struct i2c_client *client)
{
	struct imx175_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&imx175_device);
	info->pdata->power_off();//Jimmy add
 	if (info->kobj_imx175)
		kobject_del(info->kobj_imx175);
	kfree(info);
	return 0;
}


#if 0
static struct imx175_info *k_info;
static ssize_t imx175_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = k_info->i2c_client;
	struct imx175_reg reg_override[5];
	ssize_t length;
	int err;

	pr_info("%s %p\n", __func__, dev);
	if (!dev)
		return 0;

	imx175_get_frame_length_regs(reg_override, 0);
	imx175_get_coarse_time_regs(reg_override + 2, 0);
	imx175_get_gain_reg(reg_override + 4, 0);

	err = imx175_read_reg_bulk(client, reg_override->addr,
		(u8 *)&(reg_override->val), 2);
	if (err)
		goto fail;

	err = imx175_read_reg_bulk(client, (reg_override + 2)->addr,
		(u8 *)&((reg_override + 2)->val), 2);
	if (err)
		goto fail;

	err = imx175_read_reg(client, (reg_override + 4)->addr,
		(u8 *)&((reg_override + 4)->val));
	if (err)
		goto fail;
	length = sprintf(buf, "IMX175 sensor status:\n"
				"    Gain        = 0x%02x\n"
				"    Framelength = 0x%04x\n"
				"    CoarseTime  = 0x%04x\n",
				(reg_override + 4)->val,
				(reg_override)->val,
				(reg_override + 2)->val
				);

	return length;

fail:
	pr_info("%s: get sensor status error.\n", __func__);

	return 0;
}

static ssize_t imx175_attr_set(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	char *ptr = buf + 1;
	int err;
	size_t idx = 1;
	u32 val = 0;
	u8 ch;

	pr_info("%s\n", __func__);

	if (!buf || count <= 1)
		return count;

	while (*ptr && idx < count) {
		ch = *ptr++;
		idx++;
		if (ch == 0x0a || ch == 0x0d)
			continue;

		val *= 16;
		if (ch >= '0' && ch <= '9')
			val += ch - '0';
		else if (ch >= 'a' && ch <= 'f')
			val += ch - 'a' + 10;
		else if (ch >= 'A' && ch <= 'F')
			val += ch - 'A' + 10;
		else {
			pr_info("ERROR, out of range %c(%x)\n", ch, ch);
			count = sprintf(buf, "ERROR, out of range\n");
			ptr = NULL;
			break;
		}
	}

	if (ptr) {
		switch (buf[0]) {
		case 'g':
			pr_info("new gain = %x\n", val);
			err = imx175_set_gain(k_info, (u16)val);
			if (err)
				count = sprintf(buf,
					"ERROR set gain %x\n", val);
			break;
		case 'c':
			pr_info("new coarse time = %x\n", val);
			err = imx175_set_coarse_time(k_info, val);
			if (err)
				count = sprintf(buf,
					"ERROR set coarse time%x\n", val);
			break;
		case 'f':
			pr_info("new frame length = %x\n", val);
			err = imx175_set_frame_length(k_info, val);
			if (err)
				count = sprintf(buf,
					"ERROR set framelength %x\n", val);
			break;
		}
	}

	return count;
}

static DEVICE_ATTR(d, 0777, imx175_status_show, imx175_attr_set);
static int imx175_sysfs_init(struct imx175_info *info)
{
	int ret ;

	info->kobj_imx175 = kobject_create_and_add("cam_imx175", NULL);
	if (info->kobj_imx175 == NULL) {
		pr_info("%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret ;
	}

	ret = sysfs_create_file(info->kobj_imx175, &dev_attr_d.attr);
	if (ret) {
		pr_info("%s: sysfs_create_file failed\n", __func__);
		kobject_del(info->kobj_imx175);
		info->kobj_imx175 = NULL;
	}

	k_info = info;
	return ret ;
}
#endif
static const struct i2c_device_id imx175_id[] = {
	{ "imx175", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, imx175_id);

static struct i2c_driver imx175_i2c_driver = {
	.driver = {
		.name = "imx175",
		.owner = THIS_MODULE,
	},
	.probe = imx175_probe,
	.remove = imx175_remove,
	.id_table = imx175_id,
};

static int __init imx175_init(void)
{
	pr_info("imx175 sensor driver loading\n");
	return i2c_add_driver(&imx175_i2c_driver);
}

static void __exit imx175_exit(void)
{
	i2c_del_driver(&imx175_i2c_driver);
}

module_init(imx175_init);
module_exit(imx175_exit);
