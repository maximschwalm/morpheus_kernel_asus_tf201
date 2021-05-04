/*
 * Copyright (C) 1998, 1999  Frodo Looijaard <frodol@dds.nl> and
 *                           Philip Edelbrock <phil@netroedge.com>
 * Copyright (C) 2003 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2003 IBM Corp.
 * Copyright (C) 2004 Jean Delvare <khali@linux-fr.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/mutex.h>

#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <mach/gpio.h>
#include "gpio-names.h"
#include <linux/debugfs.h>
#include <linux/proc_fs.h>

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x50, 0x51, 0x52, 0x53, 0x54,
					0x55, 0x56, 0x57, I2C_CLIENT_END };


/* Size of EEPROM in bytes */
#define EEPROM_SIZE		1024

//jimmy add ++
#define	factory_PROC_FILE	"/proc/factory"

static char  CALIBRATION_BIN_FILE_WITH_PATH[] = "/persist/factory.bin";

static struct proc_dir_entry *factory_proc_file;

static int yuv_front_sensor_power_on(void)
{
	printk("Bayer_front_sensor_power_on+\n");

	/* 1.8V VDDIO_CAM controlled by "EN_1V8_CAM(GPIO_PBB4)" */
        gpio_set_value(TEGRA_GPIO_PBB4, 1);
	gpio_direction_output(TEGRA_GPIO_PBB4, 1);
	
        msleep(1);

  	/* 2.85V VDD_CAM2 controlled by CAM2/3_LDO_EN(GPIO_PS0)*/
        gpio_set_value(TEGRA_GPIO_PS0, 1);
	gpio_direction_output(TEGRA_GPIO_PS0, 1);

  	msleep(5);

	/* cam_power_en, powdn*/
	pr_info("Ryant: %s(%d): gpio %d: %d",__func__, __LINE__,TEGRA_GPIO_PBB7, gpio_get_value(TEGRA_GPIO_PBB7));
	gpio_set_value(TEGRA_GPIO_PBB7, 0);
	gpio_direction_output(TEGRA_GPIO_PBB7, 0);
	//pr_info("--> %d\n", gpio_get_value(CAM3_POWER_DWN_GPIO));

	/* bayer_sensor_rst_lo*/
	pr_info("adogu: %s(%d): gpio %d: %d", __func__,__LINE__,TEGRA_GPIO_PO0, gpio_get_value(TEGRA_GPIO_PO0));
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	gpio_direction_output(TEGRA_GPIO_PO0, 1);
	pr_info("--> %d\n", gpio_get_value(TEGRA_GPIO_PO0));

	printk("bayer_front_sensor_power_on-\n");
	return 0;

}

static int yuv_front_sensor_power_off(void)
{
	printk("yuv_front_sensor_power_off+\n");

	gpio_set_value(TEGRA_GPIO_PO0, 0);
	gpio_direction_output(TEGRA_GPIO_PO0, 0);

	gpio_set_value(TEGRA_GPIO_PBB7, 1);
	gpio_direction_output(TEGRA_GPIO_PBB7, 1);

	gpio_set_value(TEGRA_GPIO_PS0, 0);
	gpio_direction_output(TEGRA_GPIO_PS0, 0);

	gpio_set_value(TEGRA_GPIO_PBB4, 0);
	gpio_direction_output(TEGRA_GPIO_PBB4, 0);

	printk("yuv_front_sensor_power_off-\n");
	return 0;
}

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

static ssize_t factory_proc_read(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{

        struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
	struct eeprom_data *data = i2c_get_clientdata(client);
        u16 address;
        u16 values = 0;
        off = (loff_t)0x0000;
        u8 str[897];
        int j,i =0;
        struct file *fp = NULL;

	mm_segment_t old_fs;
	loff_t offset = 0;
        //printk("jimmy %s %d off: %llx count: %d\n", __func__, __LINE__, off, count);
	if (off > EEPROM_SIZE)
		return 0;
	if (off + count > EEPROM_SIZE)
		count = EEPROM_SIZE - off;

      yuv_front_sensor_power_on();
      yuv_front_sensor_power_off();

    pr_info("jimmy gpio camera WP pin %d set to %d\n",TEGRA_GPIO_PS5, gpio_get_value(TEGRA_GPIO_PS5));
    gpio_direction_output(TEGRA_GPIO_PS5, 1);
	// GPIO_PBB4 EN_1V8_CAM
    // gpio_direction_output(TEGRA_GPIO_PR6, 1);
    // pr_info("gpio 2.85v %d set to %d\n",TEGRA_GPIO_PR6, gpio_get_value(TEGRA_GPIO_PR6));
    gpio_direction_output(TEGRA_GPIO_PH1, 1);
    pr_info("jimmy gpio 2.7v %d set to %d\n",TEGRA_GPIO_PH1, gpio_get_value(TEGRA_GPIO_PH1));
    msleep(5);
    gpio_direction_output(TEGRA_GPIO_PBB4, 1);
    printk("jimmy gpio 1.8v %d set to %d\n",TEGRA_GPIO_PBB4, gpio_get_value(TEGRA_GPIO_PBB4));
    gpio_direction_output(TEGRA_GPIO_PBB0, 1);
    pr_info("gpio camera reset pin %d set to %d\n",TEGRA_GPIO_PBB0, gpio_get_value(TEGRA_GPIO_PBB0));

    msleep(1000);

	/* Only refresh slices which contain requested bytes */
	//for (slice = off >> 5; slice <= (off + count - 1) >> 5; slice++)
		//eeprom_update_client(client, slice);

         fp = filp_open(CALIBRATION_BIN_FILE_WITH_PATH, O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
         if ( IS_ERR_OR_NULL(fp) ){
		printk("%s: open %s fail\n", __FUNCTION__, CALIBRATION_BIN_FILE_WITH_PATH);
                return 0;
	 }
         for(address=0x0000;address<897;address +=1)
         {
           
             sensor_read_reg(client,address,&values);
             str[i] = values;
             i +=1;
           
         }
            
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		offset = 0;
		if (fp->f_op != NULL && fp->f_op->write != NULL){
			fp->f_op->write(fp,
				str,
				sizeof(str),
				&offset);
		}else
		printk("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp, NULL);

    gpio_direction_output(TEGRA_GPIO_PBB4, 0);
    printk("jimmy gpio 1.8v %d set to %d\n",TEGRA_GPIO_PBB4, gpio_get_value(TEGRA_GPIO_PBB4));
    gpio_direction_output(TEGRA_GPIO_PH1, 0);
    pr_info("gpio 2.7v %d set to %d\n",TEGRA_GPIO_PH1, gpio_get_value(TEGRA_GPIO_PH1));
    gpio_direction_output(TEGRA_GPIO_PBB0, 0);
    pr_info("gpio camera reset pin %d set to %d\n",TEGRA_GPIO_PBB0, gpio_get_value(TEGRA_GPIO_PBB0));
    gpio_direction_output(TEGRA_GPIO_PS5, 0);

	return 0;

}
//jimmy add --
/* possible types of eeprom devices */
enum eeprom_nature {
	UNKNOWN,
	VAIO,
};

/* Each client has this additional data */
struct eeprom_data {
	struct mutex update_lock;
	u8 valid;			/* bitfield, bit!=0 if slice is valid */
	unsigned long last_updated[8];	/* In jiffies, 8 slices */
	u8 data[EEPROM_SIZE];		/* Register values */
	enum eeprom_nature nature;
};

//jimmy add ++
void create_factory_file(void)
{
    struct file *filp = filp_open("/sdcard/", O_RDONLY, 0);
    struct proc_dir_entry *parent = PDE(filp->f_dentry->d_inode);
    filp_close(filp, NULL);
//proc_entry = create_proc_entry( "004", 0666, parent );

    factory_proc_file = create_proc_entry(factory_PROC_FILE, 0666, parent);
    if (factory_proc_file) {
		printk("%s: create proc file.", __FUNCTION__);
		factory_proc_file->read_proc = factory_proc_read;
		//i7002a_proc_file->write_proc = i7002a_proc_write;
    } else{
        printk("proc file create failed!\n");
    }
}
//jimmy add --

static void eeprom_update_client(struct i2c_client *client, u8 slice)
{
	struct eeprom_data *data = i2c_get_clientdata(client);
	int i;

	mutex_lock(&data->update_lock);

	if (!(data->valid & (1 << slice)) ||
	    time_after(jiffies, data->last_updated[slice] + 300 * HZ)) {
		dev_dbg(&client->dev, "Starting eeprom update, slice %u\n", slice);
#if 1
		if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
			for (i = slice << 5; i < (slice + 1) << 5; i += 32)
                                printk("%d ",i);
                                printk("0x%x ",(char)data->data[i]);
				if (i2c_smbus_read_i2c_block_data(client, i,
							32, data->data + i)
							!= 32)
					goto exit;
		} else {
			for (i = slice << 5; i < (slice + 1) << 5; i += 2) {
				int word = i2c_smbus_read_word_data(client, i);
				if (word < 0)
					goto exit;
				data->data[i] = word & 0xff;
				data->data[i + 1] = word >> 8;
			}
		}
#endif

		data->last_updated[slice] = jiffies;
		data->valid |= (1 << slice);
	}
exit:
	mutex_unlock(&data->update_lock);
}

static ssize_t eeprom_read(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
	struct eeprom_data *data = i2c_get_clientdata(client);
	//u8 slice;
        u16 address;
        u16 values = 0;
        off = (loff_t)0x0000;
        u8 str[897];
        int j,i =0;
        struct file *fp = NULL;
	//struct inode *inode;
	mm_segment_t old_fs;
	loff_t offset = 0;
        //printk("jimmy %s %d off: %llx count: %d\n", __func__, __LINE__, off, count);
	if (off > EEPROM_SIZE)
		return 0;
	if (off + count > EEPROM_SIZE)
		count = EEPROM_SIZE - off;
#if 1
      yuv_front_sensor_power_on();
      yuv_front_sensor_power_off();

    pr_info("jimmy gpio camera WP pin %d set to %d\n",TEGRA_GPIO_PS5, gpio_get_value(TEGRA_GPIO_PS5));
    gpio_direction_output(TEGRA_GPIO_PS5, 1);
    gpio_direction_output(TEGRA_GPIO_PH1, 1);
    pr_info("jimmy gpio 2.7v %d set to %d\n",TEGRA_GPIO_PH1, gpio_get_value(TEGRA_GPIO_PH1));
    msleep(5);
    gpio_direction_output(TEGRA_GPIO_PBB4, 1);
    printk("jimmy gpio 1.8v %d set to %d\n",TEGRA_GPIO_PBB4, gpio_get_value(TEGRA_GPIO_PBB4));
    gpio_direction_output(TEGRA_GPIO_PBB0, 1);
    pr_info("gpio camera reset pin %d set to %d\n",TEGRA_GPIO_PBB0, gpio_get_value(TEGRA_GPIO_PBB0));
#endif
    //msleep(1000);

	/* Only refresh slices which contain requested bytes */
	//for (slice = off >> 5; slice <= (off + count - 1) >> 5; slice++)
		//eeprom_update_client(client, slice);

         fp = filp_open(CALIBRATION_BIN_FILE_WITH_PATH, O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
         if ( IS_ERR_OR_NULL(fp) ){
		printk("%s: open %s fail\n", __FUNCTION__, CALIBRATION_BIN_FILE_WITH_PATH);
                return 0;
	 }
         for(address=0x0000;address<897;address +=1)
         {
           
             sensor_read_reg(client,address,&values);
             str[i] = values;
             if(str[0]!=0xCA)
             {
                  printk("read data is wrong str[0]=0x%x\n",str[0]);
                  filp_close(fp, NULL);
                  return 0;
             }
             i +=1;
            // printk("0x%x address=0x%x\n", values,address); //add for debug
         }
#if 0
         for(j =0; j<897; j++)
         {
             printk("%x j= %d \n",str[j],j);
         }
#endif
             printk("Jimmy: In File Open success!\n"); //add for debug
		old_fs = get_fs();
             printk("Jimmy: In File Open success! %d\n",__LINE__); //add for debug
		set_fs(KERNEL_DS);
             printk("Jimmy: In File Open success! %d\n",__LINE__); //add for debug
		offset = 0;
		if (fp->f_op != NULL && fp->f_op->write != NULL){
			fp->f_op->write(fp,
				str,
				sizeof(str),
				&offset);
		}else
		printk("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp, NULL);

	/* Hide Vaio private settings to regular users:
	   - BIOS passwords: bytes 0x00 to 0x0f
	   - UUID: bytes 0x10 to 0x1f
	   - Serial number: 0xc0 to 0xdf */
	if (data->nature == VAIO && !capable(CAP_SYS_ADMIN)) {
		int i;

		for (i = 0; i < count; i++) {
			if ((off + i <= 0x1f) ||
			    (off + i >= 0xc0 && off + i <= 0xdf))
				buf[i] = 0;
			else
				buf[i] = data->data[off + i];
		}
	} else {
		memcpy(buf, &data->data[off], count);
	}

#if 1
    gpio_direction_output(TEGRA_GPIO_PBB4, 0);
    printk("jimmy gpio 1.8v %d set to %d\n",TEGRA_GPIO_PBB4, gpio_get_value(TEGRA_GPIO_PBB4));
    gpio_direction_output(TEGRA_GPIO_PH1, 0);
    pr_info("gpio 2.7v %d set to %d\n",TEGRA_GPIO_PH1, gpio_get_value(TEGRA_GPIO_PH1));
    gpio_direction_output(TEGRA_GPIO_PBB0, 0);
    pr_info("gpio camera reset pin %d set to %d\n",TEGRA_GPIO_PBB0, gpio_get_value(TEGRA_GPIO_PBB0));
    gpio_direction_output(TEGRA_GPIO_PS5, 0);
#endif
	return count;
}

static struct bin_attribute eeprom_attr = {
	.attr = {
		.name = "eeprom",
		.mode = S_IRUGO,
	},
	.size = EEPROM_SIZE,
	.read = eeprom_read,
};

/* Return 0 if detection is successful, -ENODEV otherwise */
static int eeprom_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	/* EDID EEPROMs are often 24C00 EEPROMs, which answer to all
	   addresses 0x50-0x57, but we only care about 0x50. So decline
	   attaching to addresses >= 0x51 on DDC buses */
	if (!(adapter->class & I2C_CLASS_SPD) && client->addr >= 0x55)
		return -ENODEV;

	/* There are four ways we can read the EEPROM data:
	   (1) I2C block reads (faster, but unsupported by most adapters)
	   (2) Word reads (128% overhead)
	   (3) Consecutive byte reads (88% overhead, unsafe)
	   (4) Regular byte data reads (265% overhead)
	   The third and fourth methods are not implemented by this driver
	   because all known adapters support one of the first two. */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)
	 && !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -ENODEV;

	strlcpy(info->type, "eeprom", I2C_NAME_SIZE);

	return 0;
}

static int eeprom_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct eeprom_data *data;
        struct kobject *kobj;
	int err;
        printk("Jimmy: %s probe start+++\n",__func__);
        gpio_request(TEGRA_GPIO_PS5, "gpio_ps5"); //Jimmy

	if (!(data = kzalloc(sizeof(struct eeprom_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}

	memset(data->data, 0xff, EEPROM_SIZE);
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	data->nature = UNKNOWN;

	/* Detect the Vaio nature of EEPROMs.
	   We use the "PCG-" or "VGN-" prefix as the signature. */
	if (client->addr == 0x54
	 && i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		char name[4];

		name[0] = i2c_smbus_read_byte_data(client, 0x80);
		name[1] = i2c_smbus_read_byte_data(client, 0x81);
		name[2] = i2c_smbus_read_byte_data(client, 0x82);
		name[3] = i2c_smbus_read_byte_data(client, 0x83);

		if (!memcmp(name, "PCG-", 4) || !memcmp(name, "VGN-", 4)) {
			dev_info(&client->dev, "Vaio EEPROM detected, "
				 "enabling privacy protection\n");
			data->nature = VAIO;
		}
	}

	/* create the sysfs eeprom file */
	err = sysfs_create_bin_file(&client->dev.kobj, &eeprom_attr);
    
       // create_factory_file();//jimmy add for test 
	if (err)
		goto exit_kfree;
        printk("Jimmy: %s probe start---\n",__func__);

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int eeprom_remove(struct i2c_client *client)
{
	sysfs_remove_bin_file(&client->dev.kobj, &eeprom_attr);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id eeprom_id[] = {
	{ "eeprom", 0 },
	{ }
};

static struct i2c_driver eeprom_driver = {
	.driver = {
		.name	= "eeprom",
	},
	.probe		= eeprom_probe,
	.remove		= eeprom_remove,
	.id_table	= eeprom_id,

	.class		= I2C_CLASS_DDC | I2C_CLASS_SPD,
	.detect		= eeprom_detect,
	.address_list	= normal_i2c,
};

static int __init eeprom_init(void)
{
	return i2c_add_driver(&eeprom_driver);
}

static void __exit eeprom_exit(void)
{
	i2c_del_driver(&eeprom_driver);
}


MODULE_AUTHOR("Frodo Looijaard <frodol@dds.nl> and "
		"Philip Edelbrock <phil@netroedge.com> and "
		"Greg Kroah-Hartman <greg@kroah.com>");
MODULE_DESCRIPTION("I2C EEPROM driver");
MODULE_LICENSE("GPL");

module_init(eeprom_init);
module_exit(eeprom_exit);
