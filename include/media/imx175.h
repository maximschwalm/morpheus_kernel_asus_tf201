/**
 * Copyright (c) 2012 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __IMX175_H__
#define __IMX175_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define IMX175_IOCTL_SET_MODE		_IOW('o', 1, struct imx175_mode)
#define IMX175_IOCTL_GET_STATUS		_IOR('o', 2, struct imx175_status)
#define IMX175_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define IMX175_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define IMX175_IOCTL_SET_GAIN		_IOW('o', 5, __u16)
#define IMX175_IOCTL_GET_SETTING	_IOR('o', 6, struct imx175_setting)
#define IMX175_IOCTL_GET_SENSORDATA	_IOR('o', 7, struct imx175_sensordata)
static int sensor_read_reg_word(struct i2c_client *client, u16 addr, u16 *val);
static int sensor_write_reg_dword(struct i2c_client *client, u16 addr, u32 val);
struct imx175_setting {
	int xres;
	int yres;
	__u32	multiplier;
	__u32	pre_div;
	__u32	op_pix_div;
	__u32	vt_pix_div;
	__u32	vt_sys_div;
	__u32	op_sys_div;
	__u32	frame_len;
	__u32	coarse_tim;
	__u32	line_len;
	__u32	out_width;
	__u32	out_height;
};

struct imx175_mode {
	int xres;
	int yres;
	__u32	frame_length;
	__u32	coarse_time;
	__u16	gain;
};

struct imx175_status {
	int data;
	int status;
};

#ifdef __KERNEL__
struct imx175_power_rail {
	struct regulator *vana;
	struct regulator *vdig;
	struct regulator *vddl;
	unsigned gpio_reset;
};

struct imx175_platform_data {
	int (*power_get)(struct imx175_power_rail *pw);
	int (*power_put)(struct imx175_power_rail *pw);
	int (*power_on)(void);
	int (*power_off)(void);
};

struct imx175_sensordata {
    __u32 fuse_id_size;
	__u8 fuse_id[16];
};
#endif /* __KERNEL__ */

#endif  /* __IMX175_H__ */
