#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include <linux/ktime.h>

#include <linux/miscdevice.h>
#include <linux/poll.h>

#define CM3218_DRIVER_NAME	"cm3218"
#define CM3218_DRIVER_VERSION "1.0"

#define CM3218_CONFIG_CMD 0
#define CM3218_DEFALUT_ALS_ON_CONFIG 0x0000
#define CM3218_DEFALUT_ALS_OFF_CONFIG 0x0001

#define CM3218_DATA_CMD 4
#define CM3218_INT_SLAVE_ADDR 0x0C

//add printk at prob,since now the adb is not work will
//#define DRIVER_DEBUG 0
//+++ [Calibration] temp not modify ini name to reduce HAL loading
#define CAL_ALS_PATH "/data/lightsensor/AL3010_Config.ini"
bool flagLoadCalibrationConfig = false;
//default K value 880 is average K value of PR devices
static int calibration_base_lux = 1000;
static int calibration_regs = 1170; //runtime used
static int default_calibration_regs = 1170; //default K value
static int cm3218_update_calibration(void);
//---

//+++ fine tune for ALS
//static bool is_poweron_after_resume = false;
//static struct timeval t_poweron_timestamp;
static int revise_lux_times = 2;
//---

//+++ driver polling event for HAL
static int cm3218_poll_time = 50; // ms , runtime use
static int min_poll_time = 50; // ms
//static int default_poll_time = 1000; //ms
struct delayed_work cm3218_report_poll_event_work;
static struct workqueue_struct *cm3218_poll_work_queue;
static struct miscdevice misc_dev;
static wait_queue_head_t poll_wait_queue_head_t;
static bool flag_pollin = true;
//---

//+++ driver status
static int current_power_onoff = 1;//1:on , 0:off
//---

//+++ i2c stress test
#define CM3218_IOC_MAGIC 0xF3
#define CM3218_IOC_MAXNR 2
#define CM3218_POLL_DATA _IOR(CM3218_IOC_MAGIC,2,int )

#define CM3218_IOCTL_START_HEAVY 2
#define CM3218_IOCTL_START_NORMAL 1
#define CM3218_IOCTL_END 0

#define START_NORMAL    (HZ)
#define START_HEAVY     (HZ)

static int stress_test_poll_mode=0;
struct delayed_work cm3218_stress_test_poll_work;
static struct workqueue_struct *cm3218_stress_test_work_queue;
//---

struct i2c_client *cm3218_client;

struct cm3218_data {
	struct i2c_client *client;
	struct mutex lock;
};


/**
 * read ALS INT data from 0x0C
 */
static int cm3218_read_interrupt_data(struct i2c_client *client){
	unsigned char buff = {0};
	int err = 0;
	struct i2c_msg msg[] = {
		{
			.addr = CM3218_INT_SLAVE_ADDR,
			.flags = I2C_M_RD, //read
			.len = 1,
			.buf = &buff,
		},
	};
	printk("light sensor cm3218 : read interrupt data\n");
	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (err != ARRAY_SIZE(msg))
		printk("light sensor cm3128 : read interrupt data fail , err code %d\n", err);
	return err;
}

/**
 * ALS power on by default
 */
static int cm3218_power_on_by_default(struct i2c_client *client){
	int err = i2c_smbus_write_word_data(client, 0x03, 0x0000);
	if(err<0){
		printk("light sensor cm3218 : set 0x03 fail \n");
	}

	err = i2c_smbus_write_word_data(client, CM3218_CONFIG_CMD, CM3218_DEFALUT_ALS_ON_CONFIG);
	if(err<0){
		printk("light sensor cm3218 : power on fail (default config) \n");
		cm3218_read_interrupt_data(client);
		err = i2c_smbus_write_word_data(client, CM3218_CONFIG_CMD, CM3218_DEFALUT_ALS_ON_CONFIG);
		if(err<0){
			printk("light sensor cm3218 : power on fail after read irq data(default config) \n");
		}else{
			current_power_onoff = 1;
		}
	}else{
		current_power_onoff = 1;
	}
	return err;
}

/**
 * ALS power off by default
 */
static int cm3218_power_off_by_default(struct i2c_client *client){
	int err = i2c_smbus_write_word_data(client, CM3218_CONFIG_CMD, CM3218_DEFALUT_ALS_ON_CONFIG);
	if(err<0){
		printk("light sensor cm3218 : power off fail ( default config ) \n");
		cm3218_read_interrupt_data(client);
		err = i2c_smbus_write_word_data(client, CM3218_CONFIG_CMD, CM3218_DEFALUT_ALS_ON_CONFIG);
		if(err<0){
			printk("light sensor cm3218 : power off fail after read irq data( default config ) \n");
		}else{
			current_power_onoff = 0;
		}
	}else{
		current_power_onoff = 0;
	}
	return err;
}

/**
 * ALS sensor init
 */
static int cm3218_sensor_init(struct i2c_client *client){
	int err = 0;
	cm3218_read_interrupt_data(client);
	cm3218_power_off_by_default(client);
	err = cm3218_power_on_by_default(client);
	if(err){
		printk("light sensor cm3218 : power on fail , do again\n");
		err = cm3218_power_on_by_default(client);
	}
	return err;
}

/**
 * get ambient light data reg value ( adc value , can not map to real lux value )
 */
static int cm3218_get_adc_value(struct i2c_client *client)
{
	int adc = 0;

	if(!flagLoadCalibrationConfig){
		if(cm3218_update_calibration()){
			printk("light sensor cm3218 : calibration file update fail ! \n");
		}
		flagLoadCalibrationConfig = true;
	}

	adc = i2c_smbus_read_word_data(client, CM3218_DATA_CMD);
	if(adc<0){
		printk("light sensor cm3218 : get adc fail! , adc = %d\n",adc);
		cm3218_sensor_init(client);
		adc = i2c_smbus_read_word_data(client, CM3218_DATA_CMD);
		printk("light sensor cm3218 : after sensor re-init , adc = %d\n",adc);
	}
	return adc;
}

/**
 * get lux value
 */
static int cm3218_get_lux_value(struct i2c_client *client){
	int adc = cm3218_get_adc_value(client);
	if(adc>=0){
		return (u32)( ( adc*calibration_base_lux )/calibration_regs );
	}else{
		return adc;
	}
}

/*
 * light sensor calibration
 */

static int cm3218_update_calibration()
{
	char buf[256];
	int calibration_value = 0;
	struct file *fp = NULL;
	mm_segment_t oldfs;
	oldfs=get_fs();
	set_fs(get_ds());
	memset(buf, 0, sizeof(u8)*256);
	fp=filp_open(CAL_ALS_PATH, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		int ret = 0;
		ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
		sscanf(buf,"%d\n", &calibration_value);
		if(calibration_value > 0){
			calibration_regs = calibration_value;
		}
		filp_close(fp, NULL);
		set_fs(oldfs);
		return 0;
	}else{
		return -1;
	}
}

/* lux */
static ssize_t cm3218_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", cm3218_get_lux_value(client));
}

/* reg */
static ssize_t cm3218_show_reg(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", cm3218_get_adc_value(client));
}

/* power state */
static ssize_t cm3218_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int power_state = 1;
	int adc = cm3218_get_adc_value(client);
	if(adc<0){
		power_state = 0;
	}
	return sprintf(buf, "%d\n", power_state);
}

/* refresh calibration */
static ssize_t cm3218_refresh_calibration(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	printk("light sensor cm3218 : refresh calibration ini \n");
	return sprintf(buf,"%d\n",cm3218_update_calibration());
}

/* revise lux */
static ssize_t cm3218_show_revise_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp = cm3218_get_lux_value(client)*revise_lux_times ;
	//printk("light sensor cm3218 : ----------> revise lux = %d\n",tmp);
	return sprintf(buf, "%d\n", tmp);
}

/* default lux */
static ssize_t cm3218_show_default_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    int show_lux_value = cm3218_get_lux_value(client);
    int show_default_lux_value = (show_lux_value*calibration_regs)/default_calibration_regs;
	return sprintf(buf, "%d\n", show_default_lux_value);
}

/* power onoff*/
static ssize_t cm3218_show_power_onoff(struct device *dev ,
                                 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", current_power_onoff);
}

static ssize_t cm3218_store_power_onoff(struct device *dev, struct device_attribute *attr,
		const char *buf , size_t count){
	struct i2c_client *client = to_i2c_client(dev);
	long onoff;
	printk("light sensor cm3218 : cm3218_store_power_onoff()\n");
	if (strict_strtol(buf, count, &onoff))
		return -EINVAL;
	if ((onoff != 1) && (onoff != 0))
		return -EINVAL;
	if(onoff == 1){
		printk("light sensor cm3218 : cm3218 power on\n");
		cm3218_power_on_by_default(client);
	}else{
		printk("light sensor cm3218 : cm3218 power off\n");
		cm3218_power_off_by_default(client);
	}
	return strnlen(buf, count);
}

/* sensor enable */
static ssize_t cm3218_enable_sensor(struct device *dev, struct device_attribute *attr,
		const char *buf , size_t count){
	long enable;
	printk("light sensor cm3218 : cm3218_enable_sensor()\n");
	if (strict_strtol(buf, 10, &enable))
		return -EINVAL;
	if ((enable != 1) && (enable != 0))
		return -EINVAL;

	if(enable == 1){
		printk("light sensor cm3218 : cm3218 poll enable\n");
		flag_pollin = true;
	    queue_delayed_work(cm3218_poll_work_queue, &cm3218_report_poll_event_work, cm3218_poll_time);
	}else{
		printk("light sensor cm3218 : cm3218 poll disable\n");
		cancel_delayed_work_sync(&cm3218_report_poll_event_work);
	}
	return strnlen(buf, count);
}

/* set delay time */
static ssize_t cm3218_set_delay(struct device *dev, struct device_attribute *attr,
		const char *buf , size_t count){
	long delay_time;
	if (strict_strtol(buf, count, &delay_time))
		return -EINVAL;

	if(delay_time<min_poll_time){
		cm3218_poll_time = min_poll_time;
	}else{
		cm3218_poll_time = delay_time;
	}
	printk("light sensor cm3218 : cm3218_poll_time = %d\n",cm3218_poll_time);
	return strnlen(buf, count);
}

/* power on*/
static ssize_t cm3218_get_delay(struct device *dev ,
                                 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cm3218_poll_time);
}

static SENSOR_DEVICE_ATTR(show_reg, 0644, cm3218_show_reg, NULL, 1);
static SENSOR_DEVICE_ATTR(show_lux, 0644, cm3218_show_lux, NULL, 2);
// lightsensor_status can not power off ALS , plz use power_onfff sysfs
static SENSOR_DEVICE_ATTR(lightsensor_status, 0664, cm3218_show_power_state, cm3218_enable_sensor, 3);
static SENSOR_DEVICE_ATTR(refresh_cal, 0644, cm3218_refresh_calibration, NULL, 4);
static SENSOR_DEVICE_ATTR(show_revise_lux, 0644, cm3218_show_revise_lux, NULL, 5);
static SENSOR_DEVICE_ATTR(show_default_lux, 0644, cm3218_show_default_lux, NULL, 6);
static SENSOR_DEVICE_ATTR(power_onoff,0644,cm3218_show_power_onoff,cm3218_store_power_onoff,7);
static SENSOR_DEVICE_ATTR(poll_time,0664,cm3218_get_delay,cm3218_set_delay,8);

static struct attribute *cm3218_attributes[] = {
	&sensor_dev_attr_show_reg.dev_attr.attr,
	&sensor_dev_attr_show_lux.dev_attr.attr,
	&sensor_dev_attr_lightsensor_status.dev_attr.attr,
	&sensor_dev_attr_refresh_cal.dev_attr.attr,
	&sensor_dev_attr_show_revise_lux.dev_attr.attr,
	&sensor_dev_attr_show_default_lux.dev_attr.attr,
	&sensor_dev_attr_power_onoff.dev_attr.attr,
	&sensor_dev_attr_poll_time.dev_attr.attr,
	NULL
};

static const struct attribute_group cm3218_attr_group = {
	.attrs = cm3218_attributes,
};

static void  cm3218_report_poll_event(struct work_struct * work)
{
	//wake up device poll wait queue
	flag_pollin = true;
	wake_up_interruptible(&poll_wait_queue_head_t);
	//printk("light sensor cm3218 : %s\n", __func__);
#ifdef DRIVER_DEBUG
	printk("light sensor cm3218 cm3218_get_lux_value : %d\n", cm3218_get_lux_value(cm3218_client));
#endif /* DRIVER_DEBUG */
	//add next work for polling
	queue_delayed_work(cm3218_poll_work_queue, &cm3218_report_poll_event_work, cm3218_poll_time);
}

static void cm3218_stress_test_poll(struct work_struct * work)
{
	cm3218_get_adc_value(cm3218_client);
	if(stress_test_poll_mode ==0)
		msleep(5);
	queue_delayed_work(cm3218_stress_test_work_queue, &cm3218_stress_test_poll_work, stress_test_poll_mode);
}

int cm3218_open(struct inode *inode, struct file *filp)
{
	printk("light sensor cm3218 : %s\n", __func__);
	return 0;
}

int cm3218_release(struct inode *inode, struct file *filp)
{
	printk("light sensor cm3218 : %s\n", __func__);
	return 0;
}

static unsigned int cm3218_poll(struct file *filp, poll_table *wait){
	unsigned int mask = 0;
	poll_wait(filp, &poll_wait_queue_head_t, wait);
	if(flag_pollin==true){
		mask |= POLLIN;
		flag_pollin=false;
	}
	//printk("light sensor cm3218 : %s\n", __func__);
	return mask;
}

long cm3218_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;
	if (_IOC_TYPE(cmd) != CM3218_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > CM3218_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case CM3218_POLL_DATA:
			if (arg == CM3218_IOCTL_START_HEAVY){
				printk("light sensor cm3218 : ioctl heavy\n");
				stress_test_poll_mode = START_HEAVY;
				queue_delayed_work(cm3218_stress_test_work_queue, &cm3218_stress_test_poll_work,
						stress_test_poll_mode);
			}
			else if (arg == CM3218_IOCTL_START_NORMAL){
				printk("light sensor cm3218 : ioctl normal\n");
				stress_test_poll_mode = START_NORMAL;
				queue_delayed_work(cm3218_stress_test_work_queue, &cm3218_stress_test_poll_work,
						stress_test_poll_mode);
			}
			else if  (arg == CM3218_IOCTL_END){
				printk("light sensor cm3218 : ioctl end\n");
				cancel_delayed_work_sync(&cm3218_stress_test_poll_work);
			}
			else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return 0;
}

struct file_operations cm3218_fops = {
	.owner = THIS_MODULE,
	.open = cm3218_open,
	.release = cm3218_release,
	.poll = cm3218_poll,
	.unlocked_ioctl = cm3218_ioctl,
};

static int cm3218_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct cm3218_data *data;
	int err = 0;

	data = kzalloc(sizeof(struct cm3218_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	/* cm3218 init */
	err = cm3218_power_on_by_default(client);
	if(err){
		cm3218_sensor_init(client);
	}

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &cm3218_attr_group);
	if (err){
		printk("light sensor cm3218 : init sysfs fail\n");
		goto exit_kfree;
	}
	cm3218_client = client;

	//poll event
	cm3218_poll_work_queue = create_singlethread_workqueue("lightsensor_poll_wq");
	if(!cm3218_poll_work_queue){
		printk("light sensor cm3218 : unable to create poll workqueue\n");
		goto remove_sysfs_group;
	}
	INIT_DELAYED_WORK(&cm3218_report_poll_event_work, cm3218_report_poll_event);

	//stress test
	cm3218_stress_test_work_queue = create_singlethread_workqueue("i2c_lightsensor_wq");
	if(!cm3218_stress_test_work_queue){
		printk("light sensor cm3218 : unable to create i2c stress test workqueue\n");
		goto destroy_wq;
	}
	INIT_DELAYED_WORK(&cm3218_stress_test_poll_work, cm3218_stress_test_poll);
	//misc device for HAL poll
	misc_dev.minor = MISC_DYNAMIC_MINOR;
	misc_dev.name = "lightsensor";
	misc_dev.fops  = &cm3218_fops;
    init_waitqueue_head(&poll_wait_queue_head_t);
    err = misc_register(&misc_dev);
    if(err){
    	printk("light sensor cm3218 : register misc dev fail\n");
    	goto destroy_stress_test_wq;
    }
    printk("light sensor cm3218 : probe success\n");

#ifdef DRIVER_DEBUG
    queue_delayed_work(cm3218_poll_work_queue, &cm3218_report_poll_event_work, 5000);
#endif /* DRIVER_DEBUG */
	return 0;

destroy_stress_test_wq:
	destroy_workqueue(cm3218_stress_test_work_queue);
destroy_wq:
	destroy_workqueue(cm3218_poll_work_queue);
remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &cm3218_attr_group);
exit_kfree:
	kfree(data);
	return err;
}



#ifdef CONFIG_PM
static int cm3218_suspend(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	printk("cm3218_suspend+\n");
	ret = cm3218_power_off_by_default(client);
	if(ret){
		printk("light sensor cm3218 : power off fail \n");
	}
	printk("cm3218_suspend-\n");
	return 0;// DO NOT return err , cause system fail
}

static int cm3218_resume(struct device *dev)
{
	int ret=0;
	struct i2c_client *client = to_i2c_client(dev);
	printk("cm3218_resume+\n");
	ret = cm3218_power_on_by_default(client);
	if(ret){
		printk("light sensor cm3218 : power on fail \n");
	}
	printk("cm3218_resume-\n");
	return 0;// DO NOT return err , cause system fail
}

static const struct dev_pm_ops cm3218_pm_ops = {
	.suspend = cm3218_suspend,
	.resume  = cm3218_resume,
};
#define CM3218_PM_OPS (&cm3218_pm_ops)

#else
#define CM3218_PM_OPS NULL
#endif /* CONFIG_PM */


static int cm3218_remove(struct i2c_client *client)
{
	misc_deregister(&misc_dev);
	sysfs_remove_group(&client->dev.kobj, &cm3218_attr_group);
	cm3218_power_off_by_default(client);
	kfree(i2c_get_clientdata(client));
	printk("light sensor cm3218 : cm3218 remove successed\n");
	return 0;
}

static const struct i2c_device_id cm3218_id[] = {
	{ "cm3218", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, cm3218_id);

static struct i2c_driver cm3218_driver = {
	.driver = {
		.name	= CM3218_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm = CM3218_PM_OPS,
	},
	/*
	//legacy pm callback function
	.suspend = cm3218_suspend,
	.resume	= cm3218_resume,
	*/
	.probe	= cm3218_probe,
	.remove	= cm3218_remove,
	.id_table = cm3218_id,
};

static int __init cm3218_init(void)
{
	int ret;
	printk(KERN_INFO "%s+ #####\n", __func__);
	printk("light sensor cm3218 : module init \n");
	ret = i2c_add_driver(&cm3218_driver);
	printk(KERN_INFO "%s- #####\n", __func__);
	return ret;
}

static void __exit cm3218_exit(void)
{
	printk("light sensor cm3218 : module exit \n");
	i2c_del_driver(&cm3218_driver);
}

MODULE_AUTHOR("asus");
MODULE_DESCRIPTION("cm3218 driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(CM3218_DRIVER_VERSION);

module_init(cm3218_init);
module_exit(cm3218_exit);
