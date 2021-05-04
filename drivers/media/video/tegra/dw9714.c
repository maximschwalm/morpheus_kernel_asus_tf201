/*
 * DW9714 focuser driver.
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation.
 *
 * Contributors:
 *      Sachin Nikam <snikam@nvidia.com>
 *
 * Based on ov5650.c.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/dw9714.h>
//Jimmy add for get VCM power
#include <mach/gpio.h>
#include "gpio-names.h"


#define POS_LOW (32)
#define POS_HIGH (850)
#define SETTLETIME_MS 50
#define DEFAULT_VCM_MODE 0x0c

#define FOCAL_LENGTH (3.2f)
#define FNUMBER (2.4f)

/* #define FPOS_COUNT 1024 */

struct dw9714_info {
	struct i2c_client *i2c_client;
	struct dw9714_config config;
      //  struct dw9714_platform_data	*pdata;
	u32 cur_pos;
};

static int dw9714_write(struct i2c_client *client, u16 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	data[0] = value >> 8;
	data[1] = value & 0xff;


	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (count == ARRAY_SIZE(msg))
		return 0;
	return -EIO;
}

static int dw9714_set_position(struct dw9714_info *info, u32 position)
{
	int err = 0;
        printk("[CAM] DW9714: %s  position:%d \n",__func__, position);
	if (position < info->config.pos_low ||
	    position > info->config.pos_high)
        {       
                printk("[CAM] DW9714: %s %d position < info->config.pos_low ||position > info->config.pos_high \n",__func__, __LINE__);	    
		return -EINVAL;
        }
	err = dw9714_write(info->i2c_client, 0xECA3);
	if (err)
        {
                printk("[CAM] DW9714: dw9714_write(info->i2c_client, 0xECA3) err %s %d \n",__func__, __LINE__);
		goto dw9714_set_position_fail;
        }
	err = dw9714_write(info->i2c_client, 0xF200|(0x0F<<3));
	if (err)
        {
                printk("[CAM] DW9714: dw9714_write(info->i2c_client, 0xF200|(0x0F<<3)) err %s %d \n",__func__, __LINE__);
		goto dw9714_set_position_fail;
	}
        err = dw9714_write(info->i2c_client, 0xDC51);
	if (err)
        {
                printk("[CAM] DW9714: dw9714_write(info->i2c_client, 0xDC51) err %s %d \n",__func__, __LINE__);
		goto dw9714_set_position_fail;
	}
        err = dw9714_write(info->i2c_client,
		((position<<4 |	info->config.vcm_mode)));
	if (err)
        {
                printk("[CAM] DW9714: dw9714_write(info->i2c_client,((position<<4 |	info->config.vcm_mode))) err %s %d \n",__func__, __LINE__);	
		goto dw9714_set_position_fail;
        }
	info->cur_pos = position;
	return 0;

dw9714_set_position_fail:
	pr_err("[CAM] DW9714: %s: set position failed\n", __func__);
	return err;
}

static long dw9714_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct dw9714_info *info = file->private_data;

	switch (cmd) {
	case DW9714_IOCTL_GET_CONFIG:
	{
                printk("[CAM] DW9714: DW9714_IOCTL_GET_CONFIG ++ %s: line:%d arg: %d\n", __func__, __LINE__, (u32)arg);
		if (copy_to_user((void __user *) arg,
				 &info->config,
				 sizeof(info->config))) {
			printk("[CAM] DW9714: %s: line:%d\ arg: %d\n", __func__, __LINE__, (u32)arg);
			return -EFAULT;
		}
                printk("[CAM] DW9714: DW9714_IOCTL_GET_CONFIG -- %s: line:%d\n", __func__, __LINE__);
		break;
	}
	case DW9714_IOCTL_SET_POSITION:
                printk("[CAM] DW9714: DW9714_IOCTL_SET_POSITION %s: line: %d arg: %d\n", __func__, __LINE__, (u32)arg);
		return dw9714_set_position(info, (u32) arg);
	default:
                printk("[CAM] DW9714: default  %s: %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

struct dw9714_info *info;

static int dw9714_open(struct inode *inode, struct file *file)
{
      //  struct dw9714_info *info;
	  file->private_data = info;
          gpio_direction_output(TEGRA_GPIO_PR6, 1);
          pr_info("gpio 2.85v %d set to %d\n",TEGRA_GPIO_PR6, gpio_get_value(TEGRA_GPIO_PR6));
     //   if (info->pdata && info->pdata->power_on)
      //  {
      //      info->pdata->power_on();//Jimmy add for DW9714 power on 
	//}
        return 0;
}

int dw9714_release(struct inode *inode, struct file *file)
{
       // struct dw9714_info *info = file->private_data;
      //  if (info->pdata && info->pdata->power_off)
     //   {
      //      info->pdata->power_off();//Jimmy add for DW9714 power off
     //   }
          gpio_direction_output(TEGRA_GPIO_PR6, 0);
          pr_info("gpio 2.85v %d set to %d\n",TEGRA_GPIO_PR6, gpio_get_value(TEGRA_GPIO_PR6));
	file->private_data = NULL;
	return 0;
}


static const struct file_operations dw9714_fileops = {
	.owner = THIS_MODULE,
	.open = dw9714_open,
	.unlocked_ioctl = dw9714_ioctl,
	.release = dw9714_release,
};

static struct miscdevice dw9714_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dw9714",
	.fops = &dw9714_fileops,
};

struct dw9714_info *k_info;
static ssize_t focuser_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dw9714_config *pcfg = &k_info->config;
	ssize_t length = 0;

	pr_info("%s\n", __func__);

	length = sprintf(buf, "focuser status:\n"
				"    Addr: 0x%02x, bus %d\n\n"
				"    Range       = (%04d - %04d)\n"
				"    Current Pos = %04d\n"
				"    Settle time = %04d\n"
				"    Act Range   = %04d\n",
				k_info->i2c_client->addr,
				k_info->i2c_client->adapter->nr,
				pcfg->pos_low, pcfg->pos_high,
				k_info->cur_pos,
				pcfg->settle_time,
				pcfg->actuator_range
				);

	return length;
}

static ssize_t focuser_attr_set(struct device *dev,
	struct device_attribute *attr, char *buf, size_t count)
{
	int err;
	char *ptr = buf + 1;
	u8 ch;
	size_t idx = 0;
	u32 val = 0;

	pr_info("%s %s(%d)\n", __func__, buf, count);

	if (count <= 1)
		return count;

	while (*ptr && idx < count) {
		ch = *ptr++;
		idx++;
		if (ch == 0x0a || ch == 0x0d)
			continue;

		val *= 10;
		if (ch >= '0' && ch <= '9')
			val += ch - '0';
		/*else if (ch >= 'a' && ch <= 'f')
			val += ch - 'a' + 10;
		else if (ch >= 'A' && ch <= 'F')
			val += ch - 'A' + 10;*/
		else {
			pr_info("ERROR, out of range %c(%x), %d\n",
				ch, ch, idx);
			count = sprintf(buf, "ERROR, out of range\n");
			ptr = NULL;
			break;
		}
	}

	if (ptr) {
		switch (buf[0]) {
		case 'p':
			pr_info("new pos = %d\n", val);
			err = dw9714_set_position(k_info, val);
			if (err)
				count = sprintf(buf,
					"ERROR set position %x\n", val);
			break;
		case 'h':
			if (val <= k_info->config.pos_low || val >= 1024) {
				pr_info("new pos_high(%d) out of range\n",
					val);
				break;
			}
			pr_info("new pos_high = %d\n", val);
			k_info->config.pos_high = val;
			break;
		case 'l':
			if (val >= k_info->config.pos_high) {
				pr_info("new pos_low(%d) out of range\n",
					val);
				break;
			}
			pr_info("new pos_low = %d\n", val);
			k_info->config.pos_low = val;
			break;
		case 'm':
			if (val >= 0x10) {
				pr_info("new vcm mode(%x) out of range\n",
					val);
				break;
			}
			pr_info("new vcm mode = %x\n", val);
			info->config.vcm_mode = val;
			break;
		}
	}

	return count;
}

static DEVICE_ATTR(d, 0775, focuser_status_show, focuser_attr_set);
struct kobject *kobj;

static int dw9714_focuser_sysfs_init(struct dw9714_info *info)
{
	kobj = kobject_create_and_add("foc", NULL);
	if (!kobj) {
		pr_info("%s subsystem register failed!\n", __func__);
		return -ENOMEM;
	}

	if (sysfs_create_file(kobj, &dev_attr_d.attr)) {
		pr_info("%s sysfs create file failed!\n", __func__);
		kobject_del(kobj);
		kobj = NULL;
		return -ENOMEM;
	}
	k_info = info;

	return 0;
}

static int dw9714_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("[CAM] dw9714: probing sensor.\n");

	info = kzalloc(sizeof(struct dw9714_info), GFP_KERNEL);
	if (!info) {
		pr_err("[CAM] dw9714: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&dw9714_device);
	if (err) {
		pr_err("[CAM] dw9714: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->i2c_client = client;
	info->config.settle_time = SETTLETIME_MS;
	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.pos_low = POS_LOW;
	info->config.pos_high = POS_HIGH;
	info->config.vcm_mode = DEFAULT_VCM_MODE;
	i2c_set_clientdata(client, info);
        printk("[CAM] dw9714: SETTLETIME_MS %d FOCAL_LENGTH %d FNUMBER %d POS_LOW %d POS_HIGH %d DEFAULT_VCM_MODE 0x%x\n",SETTLETIME_MS,FOCAL_LENGTH,FNUMBER,POS_LOW,POS_HIGH,DEFAULT_VCM_MODE); 
	dw9714_focuser_sysfs_init(info);
	return 0;
}

static int dw9714_remove(struct i2c_client *client)
{
	struct dw9714_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&dw9714_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id dw9714_id[] = {
	{ "dw9714", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, dw9714_id);

static struct i2c_driver dw9714_i2c_driver = {
	.driver = {
		.name = "dw9714",
		.owner = THIS_MODULE,
	},
	.probe = dw9714_probe,
	.remove = dw9714_remove,
	.id_table = dw9714_id,
};

static int __init dw9714_init(void)
{
	pr_info("[CAM] dw9714 sensor driver loading\n");
	return i2c_add_driver(&dw9714_i2c_driver);
}

static void __exit dw9714_exit(void)
{
	i2c_del_driver(&dw9714_i2c_driver);
}

module_init(dw9714_init);
module_exit(dw9714_exit);
