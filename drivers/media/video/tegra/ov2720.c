/*
 * ov2720.c - ov2720 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      erik lilliebjerg <elilliebjerg@nvidia.com>
 *
 * Leverage OV5650.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <ov2720.h>
#include <linux/debugfs.h>
#define SIZEOF_I2C_TRANSBUF 32

struct ov2720_reg {
	u16 addr;
	u16 val;
};

struct ov2720_info {
	int mode;
	struct i2c_client *i2c_client;
	struct ov2720_platform_data *pdata;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
};
static bool sensor_opened = false;
static struct ov2720_info *info;
#define OV2720_TABLE_WAIT_MS 0
#define OV2720_TABLE_END 1
#define OV2720_MAX_RETRIES 3

static struct ov2720_reg mode_1920x1080[] = {
	{0x0103, 0x01},
    {0x3718, 0x10},
    //{OV2720_TABLE_WAIT_MS, 5},
    {0x3702, 0x24},
    //{OV2720_TABLE_WAIT_MS, 5},
    {0x373a, 0x60},
    {0x3715, 0x2e},
    {0x3705, 0x10},
    {0x3730, 0x30},
    {0x3704, 0x62},
    {0x3f06, 0x3a},
    {0x371c, 0x00},
    {0x371d, 0xc4},
    {0x371e, 0x01},
    {0x371f, 0x0d},
    {0x3708, 0x61},
    {0x3709, 0x12},
    
    {0x3800, 0x00},
    {0x3801, 0x08},
    {0x3802, 0x00},
    {0x3803, 0x02},
    {0x3804, 0x07},
    {0x3805, 0x9b},
    {0x3806, 0x04},
    {0x3807, 0x45},
    {0x3808, 0x07},
    {0x3809, 0x80},
    {0x380a, 0x04},
    {0x380b, 0x38},
    {0x380c, 0x08},
    {0x380d, 0x5c},
    {0x380e, 0x04},
    {0x380f, 0x60},
    {0x3810, 0x00},
    {0x3811, 0x09},
    {0x3812, 0x00},
    {0x3813, 0x06},
    {0x3820, 0x80},
    {0x3821, 0x06},
    {0x3814, 0x11},
    {0x3815, 0x11},
    {0x3612, 0x0b},
    {0x3618, 0x04},
    {0x3a08, 0x01},
    {0x3a09, 0x50},
    {0x3a0a, 0x01},
    {0x3a0b, 0x18},
    {0x3a0d, 0x03},
    {0x3a0e, 0x03},
    {0x4520, 0x00},
    {0x4837, 0x1b},
    {0x3000, 0xff},
    {0x3001, 0xff},
    {0x3002, 0xf0},
    {0x3600, 0x08},
    {0x3621, 0xc0},
    {0x3632, 0xd2},
    {0x3633, 0x23},
    {0x3634, 0x54},
    {0x3f01, 0x0c},
    {0x5001, 0xc1},
    {0x3614, 0xf0},
    {0x3630, 0x2d},
    {0x370b, 0x62},
    {0x3706, 0x61},
    {0x4000, 0x02},
    {0x4002, 0xc5},
    {0x4005, 0x08},
    {0x404f, 0x84},
    {0x4051, 0x00},
    {0x5000, 0xff},
    {0x3a18, 0x00},
    {0x3a19, 0x80},
    {0x3503, 0x00},
    {0x4521, 0x00},
    {0x5183, 0xb0},
    {0x5184, 0xb0},
    {0x5185, 0xb0},
    {0x370c, 0x0c},
    {0x3035, 0x10},
    {0x3036, 0x1e},
    {0x3037, 0x21},
    {0x303e, 0x19},
    {0x3038, 0x06},
    {0x3018, 0x04},
    {0x3000, 0x00},
    {0x3001, 0x00},
    {0x3002, 0x00},
    {0x3a0f, 0x40},
    {0x3a10, 0x38},
    {0x3a1b, 0x48},
    {0x3a1e, 0x30},
    {0x3a11, 0x90},
    {0x3a1f, 0x10},
    {0x3011, 0x22},
    {0x3800, 0x00},
    {0x3801, 0x08},
    {0x3802, 0x00},
    {0x3803, 0x02},
    {0x3804, 0x07},
    {0x3805, 0x9b},
    {0x3806, 0x04},
    {0x3807, 0x45},
    {0x3808, 0x07},
    {0x3809, 0x88},
    {0x380a, 0x04},
    {0x380b, 0x40},
    {0x380c, 0x08},
    {0x380d, 0x5c},
    {0x380e, 0x04},
    {0x380f, 0x60},
              
    {0x3810, 0x00},
    {0x3811, 0x05},
    {0x3812, 0x00},
    {0x3813, 0x02},
    {0x5000, 0xcd},
    {0x3503, 0x17}, // vendor provide is 0x07
    {0x0100, 0x01},
    {OV2720_TABLE_WAIT_MS, 5},
	{OV2720_TABLE_END, 0x00}
};

#if 0
static struct ov2720_reg mode_1920x1080[] = {
	{0x0103, 0x01},
	{0x3718, 0x10},
	{OV2720_TABLE_WAIT_MS, 5},
	{0x3702, 0x24},
	{OV2720_TABLE_WAIT_MS, 5},
	{0x373a, 0x60},
	{0x3715, 0x01},
        {0x3703, 0x2e},
	{0x3705, 0x10},
	{0x3730, 0x30},
	{0x3704, 0x62},
	{0x3f06, 0x3a},

	{0x371c, 0x00},
	{0x371d, 0xc4},
	{0x371e, 0x01},
	{0x371f, 0x0d},
	{0x3708, 0x61},
	{0x3709, 0x12},
	{0x3800, 0x00},
	{0x3801, 0x00},

	{0x3703, 0x5c},
	{0x3704, 0x40},
	{0x370d, 0x0f},
	{0x3713, 0x9f},
	{0x3714, 0x4c},
	{0x3710, 0x9e},
	{0x3801, 0xc4},
	{0x3802, 0x00},
	{0x3803, 0x02},
	{0x3804, 0x07},
	{0x3805, 0x9b},
	{0x3806, 0x04},
	{0x3807, 0x45},
	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x08},
	{0x380d, 0x5c},
	{0x380e, 0x04},
	{0x380f, 0x60},
	{0x3810, 0x00},
	{0x3811, 0x09},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x11},

	{0x3815, 0x11},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0d, 0x03},
	{0x3a0e, 0x03},
	{0x4502, 0x00},
	{0x4837, 0x1b},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},

	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xff},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x00},
	{0x4521, 0x00},

	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},

	{0x3035, 0x10},
	{0x3036, 0x1e},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},

	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},

	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3011, 0x22},
	{0x0100, 0x01},
/*
	{0x380d, 0xec},
	{0x3703, 0x61},
	{0x3704, 0x44},
	{0x3801, 0xd2},

	{0x3503, 0x33},
	{0x3500, 0x00},
	{0x3501, 0x00},
	{0x3502, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x00},
	{0x5001, 0x4e},
	{0x5000, 0x5f},
	{0x3008, 0x02},
*/

	{OV2720_TABLE_END, 0x0000}
};
#endif
#if 0
static struct ov2720_reg mode_1280x720[] = {
	{0x3103, 0x93},
	{0x3008, 0x82},
	{OV2720_TABLE_WAIT_MS, 5},
	{0x3008, 0x42},
	{OV2720_TABLE_WAIT_MS, 5},
	{0x3017, 0x7f},
	{0x3018, 0xfc},

	{0x3706, 0x61},
	{0x3712, 0x0c},
	{0x3630, 0x6d},
	{0x3801, 0xb4},
	{0x3621, 0x04},
	{0x3604, 0x60},
	{0x3603, 0xa7},
	{0x3631, 0x26},
	{0x3600, 0x04},
	{0x3620, 0x37},
	{0x3623, 0x00},
	{0x3702, 0x9e},
	{0x3703, 0x5c},
	{0x3704, 0x40},
	{0x370d, 0x0f},
	{0x3713, 0x9f},
	{0x3714, 0x4c},
	{0x3710, 0x9e},
	{0x3801, 0xc4},
	{0x3605, 0x05},
	{0x3606, 0x3f},
	{0x302d, 0x90},
	{0x370b, 0x40},
	{0x3716, 0x31},
	{0x3707, 0x52},
	{0x380d, 0x74},
	{0x5181, 0x20},
	{0x518f, 0x00},
	{0x4301, 0xff},
	{0x4303, 0x00},
	{0x3a00, 0x78},
	{0x300f, 0x88},
	{0x3011, 0x28},
	{0x3a1a, 0x06},
	{0x3a18, 0x00},
	{0x3a19, 0x7a},
	{0x3a13, 0x54},
	{0x382e, 0x0f},
	{0x381a, 0x1a},
	{0x401d, 0x02},

	{0x381c, 0x10},
	{0x381d, 0xb0},
	{0x381e, 0x02},
	{0x381f, 0xec},
	{0x3800, 0x01},
	{0x3820, 0x0a},
	{0x3821, 0x2a},
	{0x3804, 0x05},
	{0x3805, 0x10},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3806, 0x02},
	{0x3807, 0xe0},
	{0x3808, 0x05},
	{0x3809, 0x10},
	{0x380a, 0x02},
	{0x380b, 0xe0},
	{0x380e, 0x02},
	{0x380f, 0xf0},
	{0x380c, 0x07},
	{0x380d, 0x00},
	{0x3810, 0x10},
	{0x3811, 0x06},

	{0x5688, 0x03},
	{0x5684, 0x05},
	{0x5685, 0x00},
	{0x5686, 0x02},
	{0x5687, 0xd0},

	{0x3a08, 0x1b},
	{0x3a09, 0xe6},
	{0x3a0a, 0x17},
	{0x3a0b, 0x40},
	{0x3a0e, 0x01},
	{0x3a0d, 0x02},
	{0x3011, 0x0a},
	{0x300f, 0x8a},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x4800, 0x24},
	{0x300e, 0x04},
	{0x4801, 0x0f},
	{0x300f, 0xc3},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},

	{0x3010, 0x10},
	{0x3a0e, 0x02},
	{0x3a0d, 0x03},
	{0x3a08, 0x0d},
	{0x3a09, 0xf3},
	{0x3a0a, 0x0b},
	{0x3a0b, 0xa0},

	{0x300f, 0xc3},
	{0x3011, 0x0e},
	{0x3012, 0x02},
	{0x380c, 0x07},
	{0x380d, 0x6a},
	{0x3703, 0x5c},
	{0x3704, 0x40},
	{0x3801, 0xbc},

	{0x3503, 0x33},
	{0x3500, 0x00},
	{0x3501, 0x00},
	{0x3502, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x00},
	{0x5001, 0x4e},
	{0x5000, 0x5f},
	{0x3008, 0x02},

	{OV2720_TABLE_END, 0x0000}
};
#endif
static struct ov2720_reg mode_1280x720[] = {
        {0x3800, 0x01},
        {0x3801, 0x4a},
        {0x3802, 0x00},
        {0x3803, 0xba},
        {0x3804, 0x06},
        {0x3805, 0x51+32},
        {0x3806, 0x03},
        {0x3807, 0x8d+24},
        {0x3810, 0x00},
        {0x3811, 0x05},
        {0x3812, 0x00},
        {0x3813, 0x02},
        {0x3820, 0x80},
        {0x3821, 0x06},
        {0x3814, 0x11},
        {0x3815, 0x11},
        {0x3612, 0x0b},
        {0x3618, 0x04},
        {0x3a08, 0x01},
        {0x3a09, 0x50},
        {0x3a0a, 0x01},
        {0x3a0b, 0x18},
        {0x3a0d, 0x03},
        {0x3a0e, 0x03},
        {0x4520, 0x00},
        {0x4837, 0x1b},
        {0x3000, 0xff},
        {0x3001, 0xff},
        {0x3002, 0xf0},
        {0x3600, 0x08},
        {0x3621, 0xc0},
        {0x3632, 0xd2},
        {0x3633, 0x23},
        {0x3634, 0x54},
        {0x3f01, 0x0c},
        {0x5001, 0xc1},
        {0x3614, 0xf0},
        {0x3630, 0x2d},
        {0x370b, 0x62},
        {0x3706, 0x61},
        {0x4000, 0x02},
        {0x4002, 0xc5},
        {0x4005, 0x08},
        {0x404f, 0x84},
        {0x4051, 0x00},
        {0x5000, 0xff},
        {0x3a18, 0x00},
        {0x3a19, 0x80},
        {0x3503, 0x13},
        {0x4521, 0x00},
        {0x5183, 0xb0},
        {0x5184, 0xb0},
        {0x5185, 0xb0},
        {0x370c, 0x0c},
        {0x3035, 0x10},
        {0x3036, 0x04},
        {0x3037, 0x61},
        {0x303e, 0x19},
        {0x3038, 0x06},
        {0x3018, 0x04},
        {0x3000, 0x00},
        {0x3001, 0x00},
        {0x3002, 0x00},
        {0x3a0f, 0x40},
        {0x3a10, 0x38},
        {0x3a1b, 0x48},
        {0x3a1e, 0x30},
        {0x3a11, 0x90},
        {0x3a1f, 0x10},
        {0x4800, 0x24},
	{OV2720_TABLE_END, 0x0000}
};

enum {
	OV2720_MODE_1920x1080,
	//OV2720_MODE_1280x720,
};


static struct ov2720_reg *mode_table[] = {
	[OV2720_MODE_1920x1080] = mode_1920x1080,
	//[OV2720_MODE_1280x720] = mode_1280x720,
};

static inline void ov2720_get_frame_length_regs(struct ov2720_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov2720_get_coarse_time_regs(struct ov2720_reg *regs,
					       u32 coarse_time)
{
	regs->addr = 0x3500;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = 0x3501;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = 0x3502;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

/* Freeze the High bit gain value (0x3508) is 0x00
 * let the gain value write to low bit (0x3509)
 * */
static inline void ov2720_get_gain_reg(struct ov2720_reg *regs, u16 gain)
{
	regs->addr = 0x3508;
	regs->val = 0x00;
	(regs + 1)->addr = 0x3509;
	(regs + 1)->val = gain;
	printk("Gain is: %x\n", gain);
}

static int ov2720_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

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

	*val = data[2];

	return 0;
}

static int ov2720_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
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
		pr_err("ov2720: i2c transfer failed, retrying %x %x\n",
		       addr, val);

		msleep(3);
	} while (retry <= OV2720_MAX_RETRIES);

	return err;
}

static int ov2720_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	
	err = i2c_transfer(client->adapter, &msg, 1);
    
	if (err == 1)
		return 0;

	pr_err("ov2720: i2c bulk transfer failed at %x\n",
	(int)data[0] << 8 | data[1]);

	return err;
}

static int ov2720_write_table(struct ov2720_info *info,
			      const struct ov2720_reg table[],
			      const struct ov2720_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct ov2720_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	unsigned int i;
	u16 val;
    
    pr_info("ov2720: ov2720_write_table Start\n");
	for (next = table; next->addr != OV2720_TABLE_END; next++) {
		if (next->addr == OV2720_TABLE_WAIT_MS) {
            pr_info("ov2720: ov2720_write_table :"
			" OV2720_TABLE_WAIT_MS "); 
			msleep(next->val);
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
/*
		if (!buf_filled) {
			b_ptr = info->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != OV2720_TABLE_END &&
			n_next->addr != OV2720_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

*/		
		err = ov2720_write_reg(info->i2c_client, next->addr, val);
		if (err)
        {
            printk("%s OV2720 i2c error\n",__func__);
			return err;
        }
		buf_filled = 0;
	}
	return 0;
}
#if 0
/* --- ov2720_chip_id --- */
int tegra_camera_mclk_on_off(int on);

static ssize_t dbg_ov2720_chip_id_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_ov2720_chip_id_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 chip_id1 = 0x00;
    u16 chip_id2 = 0x00;
	int err = 0;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	if (sensor_opened == false) {
		if (info->pdata && info->pdata->power_on)
			info->pdata->power_on();
		else {
			len = snprintf(bp, dlen, "ov2720 info isn't enough for power_on.\n");
			tot += len; bp += len; dlen -= len;
		}
		tegra_camera_mclk_on_off(1);
	}

	err = ov2720_read_reg(info->i2c_client, 0x300A, &chip_id1);
	len = snprintf(bp, dlen, "chip_id1= 0x%x, err= %d\n", chip_id1, err);
        printk("chip_id1:0x%x\n",chip_id1);
        
        err = ov2720_read_reg(info->i2c_client, 0x300B, &chip_id2);
	len = snprintf(bp, dlen, "chip_id2= 0x%x, err= %d\n", chip_id2, err);
        printk("chip_id2:0x%x\n",chip_id2);
	tot += len; bp += len; dlen -= len;

	if (sensor_opened == false) {
		tegra_camera_mclk_on_off(0);

		if (info->pdata && info->pdata->power_off) {
			info->pdata->power_off();
		} else {
			len = snprintf(bp, dlen, "ov2720 info isn't enough for power_off.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ov2720_chip_id_fops = {
	.open		= dbg_ov2720_chip_id_open,
	.read		= dbg_ov2720_chip_id_read,
};
#endif
/* --- get_ov2720_reg --- */



static int ov2720_set_mode(struct ov2720_info *info, struct ov2720_mode *mode)
{
        
	int sensor_mode;
	int err;
	struct ov2720_reg reg_list[6];
    
	pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
		__func__, mode->xres, mode->yres, mode->frame_length,
		mode->coarse_time, mode->gain);
	
	sensor_mode = OV2720_MODE_1920x1080;
	
	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	ov2720_get_frame_length_regs(reg_list, mode->frame_length);
	ov2720_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	ov2720_get_gain_reg(reg_list + 5, mode->gain);

	err = ov2720_write_table(info, mode_table[sensor_mode], reg_list, 6);
	if (err)
		return err;

	info->mode = sensor_mode;
    //ov2720_write_reg(info->i2c_client, 0x5040, 0x01);//jimmy write test pattern
	return 0;
}

static int ov2720_set_frame_length(struct ov2720_info *info, u32 frame_length)
{
    printk("%s++ %d\n", __func__,__LINE__);
	int ret;
	struct ov2720_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	ov2720_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = ov2720_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4); // 2 is ryant debug
	
	return ret;
}

static int ov2720_set_coarse_time(struct ov2720_info *info, u32 coarse_time)
{

    printk("%s++ %d\n", __func__,__LINE__);
	int ret;
	struct ov2720_reg reg_list[3];
	u8 *b_ptr = info->i2c_trans_buf;

	ov2720_get_coarse_time_regs(reg_list, coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	*b_ptr++ = reg_list[2].val & 0xff;
	ret = ov2720_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 5); // 88 is ryant debug

	return ret;
}

static int ov2720_set_gain(struct ov2720_info *info, u16 gain)
{
	int ret;
	struct ov2720_reg reg_list[2];
	int i = 0;

	ov2720_get_gain_reg(&reg_list, gain);

	for (i = 0; i < 2; i++)	{
		ret = ov2720_write_reg(info->i2c_client, reg_list[i].addr, reg_list[i].val);
		if (ret)
			return ret;
	}

	return ret;
}

static int ov2720_set_group_hold(struct ov2720_info *info, struct ov2720_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;
	pr_info("%s %d\n", __func__, __LINE__);
	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = ov2720_write_reg(info->i2c_client, 0x3208, 0x00);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		ov2720_set_gain(info, ae->gain);
	if (ae->coarse_time_enable)
		ov2720_set_coarse_time(info, ae->coarse_time);
	if (ae->frame_length_enable)
		ov2720_set_frame_length(info, ae->frame_length);

	if (groupHoldEnabled) {
		ret = ov2720_write_reg(info->i2c_client, 0x3208, 0x10);
		if (ret)
			return ret;

		ret = ov2720_write_reg(info->i2c_client, 0x3208, 0xa0);
		if (ret)
			return ret;
	}

	return 0;
}


static int ov2720_get_status(struct ov2720_info *info, u8 *status)
{
	int err;
    printk("%s++ %d\n", __func__,__LINE__);
	*status = 0;
	err = ov2720_read_reg(info->i2c_client, 0x002, status);
	return err;
}


static long ov2720_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct ov2720_info *info = file->private_data;

	switch (cmd) {
	case OV2720_IOCTL_SET_MODE:
	{
		struct ov2720_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov2720_mode))) {
			return -EFAULT;
		}
        printk("OV2720_IOCTL_SET_MODE\n");
		return ov2720_set_mode(info, &mode);
	}
	case OV2720_IOCTL_SET_FRAME_LENGTH:
        //printk("OV2720_IOCTL_SET_FRAME_LENGTH\n");
		return ov2720_set_frame_length(info, (u32)arg);
	case OV2720_IOCTL_SET_COARSE_TIME:
       //printk("OV2720_IOCTL_SET_COARSE_TIME\n");
		return ov2720_set_coarse_time(info, (u32)arg);
	case OV2720_IOCTL_SET_GAIN:
		return ov2720_set_gain(info, (u16)arg);
	case OV2720_IOCTL_SET_GROUP_HOLD:
	{
        //printk("OV2720_IOCTL_SET_GROUP_HOLD \n");
		struct ov2720_ae ae;
		if (copy_from_user(&ae,
				(const void __user *)arg,
				sizeof(struct ov2720_ae))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		int ret = ov2720_set_group_hold(info, &ae);
		//pr_info("In OV2720_IOCTL_SET_GROUP_HOLD the ret is %d\n", ret);
		return ret;
	}
	case OV2720_IOCTL_GET_STATUS:
	{
		u8 status;
        //printk("OV2720_IOCTL_GET_STATUS \n");
		err = ov2720_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, 2)) {
			return -EFAULT;
		}
		return 0;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static struct ov2720_info *info;

static int ov2720_open(struct inode *inode, struct file *file)
{
	u8 status;
    
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	ov2720_get_status(info, &status);
    return 0;
}

int ov2720_release(struct inode *inode, struct file *file)
{
    printk("%s++ %d\n", __func__,__LINE__);
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations ov2720_fileops = {
	.owner = THIS_MODULE,
	.open = ov2720_open,
	.unlocked_ioctl = ov2720_ioctl,
	.release = ov2720_release,
};

static struct miscdevice ov2720_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov2720",
	.fops = &ov2720_fileops,
};

static int ov2720_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
        struct dentry *debugfs_dir;
	pr_info("ov2720: probing sensor.\n");

	info = kzalloc(sizeof(struct ov2720_info), GFP_KERNEL);
	if (!info) {
		pr_err("ov2720: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ov2720_device);
	if (err) {
		pr_err("ov2720: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);
    //debugfs_dir = debugfs_create_dir("ov2720", NULL);
	//(void) debugfs_create_file("chip_id", S_IRUGO , debugfs_dir, NULL, &dbg_ov2720_chip_id_fops);
	return 0;
}

static int ov2720_remove(struct i2c_client *client)
{
	struct ov2720_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ov2720_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ov2720_id[] = {
	{ "ov2720", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov2720_id);

static struct i2c_driver ov2720_i2c_driver = {
	.driver = {
		.name = "ov2720",
		.owner = THIS_MODULE,
	},
	.probe = ov2720_probe,
	.remove = ov2720_remove,
	.id_table = ov2720_id,
};

static int __init ov2720_init(void)
{
	pr_info("ov2720 sensor driver loading\n");
	return i2c_add_driver(&ov2720_i2c_driver);
}

static void __exit ov2720_exit(void)
{
	i2c_del_driver(&ov2720_i2c_driver);
}

module_init(ov2720_init);
module_exit(ov2720_exit);
MODULE_LICENSE("GPL v2");
