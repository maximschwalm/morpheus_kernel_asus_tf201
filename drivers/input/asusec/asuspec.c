/*
 * ASUS EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
//Shouchung add for using touchpad with the absolute mode
#include <linux/input/mt.h>
//Shouchung end
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <../gpio-names.h>
#include <linux/statfs.h>
#include "asuspec.h"
#include <mach/board-cardhu-misc.h>
//[Brook- Docking charging porting]>>
#include "elan_i2c_asus.h"
//[Brook- Docking charging porting]<<
//[Brook- Bug 282345 + No charging icon when insert AC]>>
#define is_Present  0x01
#define is_10V_in   0x20
#define is_05A_in   0x40
extern void battery_callback(unsigned cable_status);
extern void low_battery_detect_isr(void);
//[Brook- Bug 282345 + No charging icon when insert AC]<<
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * functions declaration
 */
static int asuspec_dockram_write_data(int cmd, int length);
static int asuspec_dockram_read_data(int cmd);
static int asuspec_dockram_read_battery(int cmd);
static int asuspec_i2c_write_data(struct i2c_client *client, u16 data);
static int asuspec_i2c_read_data(struct i2c_client *client);
static int asuspec_chip_init(struct i2c_client *client);
static void asuspec_send_ec_req(void);
static void asuspec_enter_s3_timer(unsigned long data);
static void asuspec_enter_s3_work_function(struct work_struct *dat);
static void asuspec_fw_update_work_function(struct work_struct *dat);
static void asuspec_work_function(struct work_struct *dat);
//[Brook- Docking charging porting]>>
static void asusdec_lid_report_function(struct work_struct *dat);
static void asusdec_dock_init_work_function(struct work_struct *dat);
extern int docking_callback(unsigned usb_cable_state);
static ssize_t asusdec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t asusdec_switch_state(struct switch_dev *sdev, char *buf);
static ssize_t asuspec_dock_control_flag_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_dock_battery_show(struct device *class,
		struct device_attribute *attr,char *buf);
static int asusdec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
//Shouchung add for using touchpad with the absolute mode
static void asusdec_tp_command(struct i2c_client *client, u8 protocol,
		u8 command, int reg, u8 value);
static void asusdec_touchpad_processing(void);
//Shouchung end
static ssize_t asuspec_show_lid_status(struct device *class,
		struct device_attribute *attr,char *buf);
//[Brook- Docking charging porting]<<
static int __devinit asuspec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int __devexit asuspec_remove(struct i2c_client *client);
static ssize_t asuspec_status_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_info_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_version_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_battery_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_control_flag_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_send_ec_req_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_charging_led_store(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asuspec_led_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_enter_factory_mode_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_enter_normal_mode_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_switch_hdmi_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_win_shutdown_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_cmd_data_store(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asuspec_return_data_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asuspec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t asuspec_switch_state(struct switch_dev *sdev, char *buf);
static ssize_t apower_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t apower_switch_state(struct switch_dev *sdev, char *buf);
static int asuspec_suspend(struct i2c_client *client, pm_message_t mesg);
static int asuspec_resume(struct i2c_client *client);
static int asuspec_open(struct inode *inode, struct file *flip);
static int asuspec_release(struct inode *inode, struct file *flip);
static long asuspec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static void asuspec_switch_apower_state(int state);
static void asuspec_switch_hdmi(void);
static void asuspec_win_shutdown(void);
static void asuspec_storage_info_update(void);
static void asuspec_enter_factory_mode(void);
static void asuspec_enter_normal_mode(void);
static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static void BuffPush(char data);
//[Panda Porting Hall-sensor]>>
static int asusdec_lid_input_device_create(struct i2c_client *client);
static void asusdec_lid_set_input_params(struct input_dev *dev);
//[Panda Porting Hall-sensor]<<
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
static int asusdec_set_wakeup_cmd(void);
static void asuspec_reset_dock(void);
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
/*
* extern variable
*/
extern unsigned int factory_mode;

/*
 * global variable
 */
//[Brook- Docking charging porting]>>
char* switch_value[]={"0", "10", "11", "12"}; //0: no dock, 1:mobile dock, 2:audio dock, 3: audio st
static struct asusdec_chip *dec_chip;
static unsigned int asuspec_dock_in_gpio = TEGRA_GPIO_PU4;
//Panda Porting TF600T MP Hall Sensor:--->
//static unsigned int asusdec_hall_sensor_gpio = TEGRA_GPIO_PS6;
static unsigned int asusdec_hall_sensor_gpio = TEGRA_GPIO_PBB6;
//Panda Porting TF600T MP Hall Sensor:<---
/*Chris start*/
static unsigned int asuspec_kb_int_gpio = TEGRA_GPIO_PI6;
/*Chris end*/
//Shouchung add for touch pad
static unsigned int asusdec_ps2_int_gpio = TEGRA_GPIO_PK2;
//Shouchung end
//[Brook- Docking charging porting]<<
static unsigned int asuspec_apwake_gpio = TEGRA_GPIO_PS2;
static unsigned int asuspec_ecreq_gpio = TEGRA_GPIO_PQ1;
static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;	  // point to the next free place
static int buff_out_ptr;	  // points to the first data
int reg_addr = -1;

/*Chris start*/
static struct i2c_client kb_client;
/*Chris end*/

//Shouchung add for touch pad
static struct i2c_client tp_client;
//Shouchung end

struct i2c_client dockram_client;
static struct class *asuspec_class;
static struct device *asuspec_device ;
static struct asuspec_chip *ec_chip;

struct cdev *asuspec_cdev ;
static dev_t asuspec_dev ;
static int asuspec_major = 0 ;
static int asuspec_minor = 0 ;

static struct workqueue_struct *asuspec_wq;
struct delayed_work asuspec_stress_work;

static const struct i2c_device_id asuspec_id[] = {
	{"asuspec", 0},
	{}
};


MODULE_DEVICE_TABLE(i2c, asuspec_id);

struct file_operations asuspec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asuspec_ioctl,
	.open = asuspec_open,
	.write = ec_write,
	.read = ec_read,
	.release = asuspec_release,
};

static struct i2c_driver asuspec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "asuspec",
		.owner = THIS_MODULE,
	},
	.probe	 = asuspec_probe,
	.remove	 = __devexit_p(asuspec_remove),
	.suspend = asuspec_suspend,
	.resume = asuspec_resume,
	.id_table = asuspec_id,
};

static DEVICE_ATTR(ec_status, S_IWUSR | S_IRUGO, asuspec_status_show,NULL);
static DEVICE_ATTR(ec_info, S_IWUSR | S_IRUGO, asuspec_info_show,NULL);
static DEVICE_ATTR(ec_version, S_IWUSR | S_IRUGO, asuspec_version_show,NULL);
static DEVICE_ATTR(ec_battery, S_IWUSR | S_IRUGO, asuspec_battery_show,NULL);
static DEVICE_ATTR(ec_control_flag, S_IWUSR | S_IRUGO, asuspec_control_flag_show,NULL);
//[Brook- Docking charging porting]>>
static DEVICE_ATTR(ec_dock_control_flag, S_IWUSR | S_IRUGO, asuspec_dock_control_flag_show,NULL);
static DEVICE_ATTR(ec_dock_battery, S_IWUSR | S_IRUGO, asuspec_dock_battery_show,NULL);
static DEVICE_ATTR(ec_lid, S_IWUSR | S_IRUGO, asuspec_show_lid_status,NULL);
//[Brook- Docking charging porting]<<
static DEVICE_ATTR(ec_request, S_IWUSR | S_IRUGO, asuspec_send_ec_req_show,NULL);
static DEVICE_ATTR(ec_led, S_IWUSR | S_IRUGO, asuspec_led_show,NULL);
static DEVICE_ATTR(ec_charging_led, S_IWUSR | S_IRUGO, NULL, asuspec_charging_led_store);
static DEVICE_ATTR(ec_factory_mode, S_IWUSR | S_IRUGO, asuspec_enter_factory_mode_show,NULL);
static DEVICE_ATTR(ec_normal_mode, S_IWUSR | S_IRUGO, asuspec_enter_normal_mode_show,NULL);
static DEVICE_ATTR(ec_switch_hdmi, S_IWUSR | S_IRUGO, asuspec_switch_hdmi_show,NULL);
static DEVICE_ATTR(ec_win_shutdown, S_IWUSR | S_IRUGO, asuspec_win_shutdown_show,NULL);
static DEVICE_ATTR(ec_cmd_data_send, S_IWUSR | S_IRUGO, NULL, asuspec_cmd_data_store);
static DEVICE_ATTR(ec_data_read, S_IWUSR | S_IRUGO,  asuspec_return_data_show, NULL);

static struct attribute *asuspec_smbus_attributes[] = {
	&dev_attr_ec_status.attr,
	&dev_attr_ec_info.attr,
	&dev_attr_ec_version.attr,
	&dev_attr_ec_battery.attr,
	&dev_attr_ec_dock_battery.attr,
	&dev_attr_ec_control_flag.attr,
	//[Brook- Docking charging porting]>>
	&dev_attr_ec_dock_control_flag.attr,
	//[Brook- Docking charging porting]<<
	&dev_attr_ec_request.attr,
	&dev_attr_ec_led.attr,
	&dev_attr_ec_charging_led.attr,
	&dev_attr_ec_factory_mode.attr,
	&dev_attr_ec_normal_mode.attr,
	&dev_attr_ec_switch_hdmi.attr,
	&dev_attr_ec_win_shutdown.attr,
	&dev_attr_ec_cmd_data_send.attr,
	&dev_attr_ec_data_read.attr,
NULL
};

static const struct attribute_group asuspec_smbus_group = {
	.attrs = asuspec_smbus_attributes,
};

#if ASUSPEC_DEBUG
int dbg_counter = 0;
#endif
/*
 * functions definition
 */
/*Chris start*/
static void asuspec_kb_init(struct i2c_client *client){
         kb_client.adapter = client->adapter;
         kb_client.addr = 0x16;
         kb_client.detected = client->detected;
         kb_client.dev = client->dev;
         kb_client.driver = client->driver;
         kb_client.flags = client->flags;
         strcpy(kb_client.name,client->name);
}
/*Chris end*/
//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]>>
void asuspec_CapsLock_LED_Control(bool on)
{
	//kb hid power on
	memset(&ec_chip->i2c_kb_data, 0, 38);
	ec_chip->i2c_kb_data[0] = 0x00;
	ec_chip->i2c_kb_data[1] = 0x00;
	ec_chip->i2c_kb_data[2] = 0x08;
	i2c_smbus_write_i2c_block_data(&kb_client, 0x75, 3, ec_chip->i2c_kb_data);
	msleep(100);

	ec_chip->i2c_kb_data[0] = 0x00;
	ec_chip->i2c_kb_data[1] = 0x05;
	ec_chip->i2c_kb_data[2] = 0x00;
	ec_chip->i2c_kb_data[3] = 0x22;
	if (on){
	//		Keyboard Output Report
	//	BIT2		BIT1		BIT 0
	//	ScrollLock	CapsLock	NumLock
		ec_chip->i2c_kb_data[4] = 0x02;
		ASUSPEC_INFO("CapsLock LED on! \n");
	}
	else{
		ec_chip->i2c_kb_data[4] = 0x00;
		ASUSPEC_INFO("CapsLock LED off! \n");
	}
	ec_chip->i2c_kb_data[5] = 0xED;

	i2c_smbus_write_i2c_block_data(&kb_client, 0x74, 6, ec_chip->i2c_kb_data);
	msleep(150);
	i2c_smbus_read_i2c_block_data(&kb_client, 0x73, 38, ec_chip->i2c_tp_data);
	msleep(150);
}

static void asusdec_keypad_led_on(struct work_struct *dat)
{
       u8 ack;
	ec_chip->kbc_value = 1;
	ASUSPEC_INFO("send led cmd 1 \n");
       asuspec_CapsLock_LED_Control(true);
}
static void asusdec_keypad_led_off(struct work_struct *dat)
{
	ec_chip->kbc_value = 0;
	ASUSPEC_INFO("send led cmd 0 \n");
       asuspec_CapsLock_LED_Control(false);
}
//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]<<

//Shouchung add for touch pad
static void asusdec_tp_init(struct i2c_client *client){
	tp_client.adapter = client->adapter;
	tp_client.addr = 0x2C;
	tp_client.detected = client->detected;
	tp_client.dev = client->dev;
	tp_client.driver = client->driver;
	tp_client.flags = client->flags;
	strcpy(tp_client.name,client->name);
}
//Shouchung end
int asuspec_audio_recording(int record_enable){
	if (record_enable)
		asuspec_send_ec_req();
	ec_chip->audio_recording = record_enable;
	ASUSPEC_NOTICE("audio_recording = %d\n", ec_chip->audio_recording);
	return 0;
}
EXPORT_SYMBOL(asuspec_audio_recording);

int asuspec_is_usb_charger(int charger_enable){
	int ret = 0;

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}
	ret = asuspec_dockram_read_data(0x0A);
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}

	ec_chip->i2c_dm_data[0] = 8;
	if (charger_enable){
		ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[5] | 0x01;
	} else {
		ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[5] & 0xFE;
	}
	ret = asuspec_dockram_write_data(0x0A,9);
	mod_timer(&ec_chip->asuspec_timer,jiffies+(HZ * 1));
	return ret;
}
EXPORT_SYMBOL(asuspec_is_usb_charger);

//[Brook- Bug 282345 + No charging icon when insert AC]>>
unsigned int Check_Charging_Cable_Type(){
	int ret = 0;
 	if(!gpio_get_value(asuspec_dock_in_gpio)){
		ret = asuspec_dockram_read_data(0x23);
		ec_chip->dock_in = 1;
	}
	else{
		ret = asuspec_dockram_read_data(0x0A);
		ec_chip->dock_in = 0;
	}
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}
	ret = ec_chip->i2c_dm_data[1];
	ASUSPEC_NOTICE(" %s, ec_chip->i2c_dm_data[1] = 0x%x \n",ec_chip->dock_in?"DOCK_CONTROL_FLAGS:0x23":"HOST_CONTROL_FLAGS:0x0A", ret);
	if((ret & is_Present) && (ret & is_05A_in)) {
		ASUSPEC_NOTICE("%s charging USB 0.5A in! \n",ec_chip->dock_in?"Dock":"Pad");
		return 1;
	}else if((ret & is_Present) && (ret & is_10V_in)) {
		ASUSPEC_NOTICE("%s charging USB 10V in! \n",ec_chip->dock_in?"Dock":"Pad");
		return 3;
	}else if(ret & is_Present){
		ASUSPEC_NOTICE("%s USB in! \n",ec_chip->dock_in?"Dock":"Pad");
		return 1;
	}
	ASUSPEC_NOTICE("%s no cable in! \n",ec_chip->dock_in?"Dock":"Pad");
	return 0;
}
EXPORT_SYMBOL(Check_Charging_Cable_Type);
//[Brook- Bug 282345 + No charging icon when insert AC]<<

//[Brook- Bug 282345 + No charging icon when insert AC]>>
/*****************************************************************/
/* BitNo  DockStatusFlagsName  Description                       */
/* 2	    Charging            1 = Charging, 0 = Discharging    */
/*****************************************************************/
bool Check_Charging_status(bool pad){
	int ret = 0;
	if(pad){//pad battery
		ret = asuspec_dockram_read_battery(0x0A);
	}
	else {//dock battery
		ret = asuspec_dockram_read_battery(0x23);
	}
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}
	if (ec_chip->i2c_dm_data[1] & 0x4){
		return 1;
	}
    return 0;
}
EXPORT_SYMBOL(Check_Charging_status);
//[Brook- Bug 282345 + No charging icon when insert AC]<<

/*****************************************************************/
/* BitNo  DockStatusFlagsName  Description                       */
/* 1	    Charging            1 = BAT Present, 0 = N/A         */
/*****************************************************************/
bool Check_Battery_status(void){
	int ret = 0;
	ret = asuspec_dockram_read_data(0x0A);
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}
	if (ec_chip->i2c_dm_data[1] & 0x2){
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(Check_Battery_status);

/*****************************************************************/
/* BitNo  DockStatusFlagsName  Description                       */
/* 4	  BAT_LL Enabled       1 = BAT_LL Enabled, 0 = Disabled  */
/*****************************************************************/
bool Check_BAT_LL_Enabled(void){
	int ret = 0;
	ret = asuspec_dockram_read_data(0x0A);
	if (ret < 0){
		ASUSPEC_ERR("Fail to access control flag info.\n");
		return ret;
	}
	if (ec_chip->i2c_dm_data[1] & 0x10){
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(Check_BAT_LL_Enabled);
//[Brook- Bug 282345 + No charging icon when insert AC]>>
int asuspec_battery_monitor(char *cmd, bool pad){
//[Brook- Bug 282345 + No charging icon when insert AC]<<
	int ret_val = 0;

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}
//[Brook- Bug 282345 + No charging icon when insert AC]>>
	if(pad){//pad battery
		ret_val = asuspec_dockram_read_battery(0x14);
	}
	else {//dock battery
		ret_val = asuspec_dockram_read_battery(0x24);
	}
//[Brook- Bug 282345 + No charging icon when insert AC]<<
	if (ret_val == -1){
		ASUSPEC_ERR("Fail to access battery info.\n");
		return -1;
	}
	else {
#if FACTORY_MODE
                if((factory_mode != 2) && (ec_chip->audio_recording == 0)){
                        mod_timer(&ec_chip->asuspec_timer,jiffies+(HZ * 1));
                }
#else
                if(ec_chip->audio_recording == 0){
                        mod_timer(&ec_chip->asuspec_timer,jiffies+(HZ * 1));
                }
#endif
		if (!strcmp(cmd, "status"))
			ret_val = (ec_chip->i2c_dm_battery[2] << 8 ) | ec_chip->i2c_dm_battery[1];
		else if (!strcmp(cmd, "temperature"))
			ret_val = (ec_chip->i2c_dm_battery[8] << 8 ) | ec_chip->i2c_dm_battery[7];
		else if (!strcmp(cmd, "voltage"))
			ret_val = (ec_chip->i2c_dm_battery[10] << 8 ) | ec_chip->i2c_dm_battery[9];
		else if (!strcmp(cmd, "current"))
			ret_val = (ec_chip->i2c_dm_battery[12] << 8 ) | ec_chip->i2c_dm_battery[11];
		else if (!strcmp(cmd, "capacity"))
			ret_val = (ec_chip->i2c_dm_battery[14] << 8 ) | ec_chip->i2c_dm_battery[13];
		else if (!strcmp(cmd, "remaining_capacity"))
			ret_val = (ec_chip->i2c_dm_battery[16] << 8 ) | ec_chip->i2c_dm_battery[15];
		else if (!strcmp(cmd, "avg_time_to_empty"))
			ret_val = (ec_chip->i2c_dm_battery[18] << 8 ) | ec_chip->i2c_dm_battery[17];
		else if (!strcmp(cmd, "avg_time_to_full"))
			ret_val = (ec_chip->i2c_dm_battery[20] << 8 ) | ec_chip->i2c_dm_battery[19];
		else {
			ASUSPEC_ERR("Unknown command\n");
			ret_val = -2;
		}
		ASUSPEC_INFO("cmd %s, return %d\n", cmd, ret_val);
		return ret_val;
	}
}
EXPORT_SYMBOL(asuspec_battery_monitor);

/*Chris 0912 start*/
static void TP_LED_Control(int i)
{
        // 1: LED off, TP on 
        // 0: LED on,  TP off
        int ret = 0;
        ret = asuspec_dockram_read_data(0x0A);
        if(i == 1){
                ec_chip->i2c_dm_data[0] = 0x08;
                ec_chip->i2c_dm_data[1] = 0;
                ec_chip->i2c_dm_data[2] = 0;
                ec_chip->i2c_dm_data[3] = 0;
                ec_chip->i2c_dm_data[4] = 0;
                ec_chip->i2c_dm_data[5] = 0;
                ec_chip->i2c_dm_data[6] = 0x08;
        }
        else{
                ec_chip->i2c_dm_data[0] = 0x08;
                ec_chip->i2c_dm_data[1] = 0;
                ec_chip->i2c_dm_data[2] = 0x08;
                ec_chip->i2c_dm_data[3] = 0;
                ec_chip->i2c_dm_data[4] = 0;
                ec_chip->i2c_dm_data[5] = 0;
                ec_chip->i2c_dm_data[6] = 0;
        }
                ret = asuspec_dockram_write_data(0x0A,9);
}
/*Chris 0912 end*/

static void asuspec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x17;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name,client->name);
	ec_chip->ec_ram_init = ASUSPEC_MAGIC_NUM;
}

static int asuspec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_write_storageinfo(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_storage);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_read_storageinfo(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_storage);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_dockram_read_battery(int cmd)
{
	int ret = 0;

	if (ec_chip->ec_ram_init != ASUSPEC_MAGIC_NUM){
		ASUSPEC_ERR("DockRam is not ready.\n");
		return -1;
	}

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		return -2;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_battery);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read dockram battery, status %d\n", ret);
		ret = -1;
	} else {
		if (ec_chip->apwake_disabled){
			mutex_lock(&ec_chip->irq_lock);
			if (ec_chip->apwake_disabled){
				enable_irq(gpio_to_irq(asuspec_apwake_gpio));
				enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
				ec_chip->apwake_disabled = 0;
				ASUSPEC_ERR("Enable pad apwake\n");
			}
			mutex_unlock(&ec_chip->irq_lock);
		}
		ec_chip->i2c_err_count = 0;
	}
	ASUSPEC_I2C_DATA(ec_chip->i2c_dm_battery, dbg_counter);
	return ret;
}

static int asuspec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		return -3;
	}

	ret = i2c_smbus_write_word_data(client, 0x64, data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to write data, status %d\n", ret);
	} else {
		ec_chip->i2c_err_count = 0;
	}
	return ret;
}

static int asuspec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access ec under FW update mode.\n");
		return -1;
	}

	if (ec_chip->i2c_err_count > ASUSPEC_I2C_ERR_TOLERANCE){
		mutex_lock(&ec_chip->irq_lock);
		if(!ec_chip->apwake_disabled){
			disable_irq_nosync(gpio_to_irq(asuspec_apwake_gpio));
			disable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
			ec_chip->apwake_disabled = 1;
			ASUSPEC_ERR("Disable pad apwake\n");
		}
		mutex_unlock(&ec_chip->irq_lock);
		return -3;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read data, status %d\n", ret);
		ec_chip->i2c_err_count++;
	} else {
		ec_chip->i2c_err_count = 0;
	}
	ASUSPEC_I2C_DATA(ec_chip->i2c_data, dbg_counter);
	return ret;
}

static int asuspec_i2c_test(struct i2c_client *client){
	return asuspec_i2c_write_data(client, 0x0000);
}
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
static int asusdec_set_wakeup_cmd(void){
	int ret_val = 0;
	ASUSPEC_NOTICE("send command \n");
	if (ec_chip->dock_in){
		ret_val = asuspec_i2c_test(ec_chip->client);
		if(ret_val >= 0){
			asuspec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			if (ec_chip->dec_wakeup){
				ec_chip->i2c_dm_data[1] = 0x00;
				ec_chip->i2c_dm_data[2] = 0x00;
				ec_chip->i2c_dm_data[3] = 0x00;
				ec_chip->i2c_dm_data[4] = 0x00;
				ec_chip->i2c_dm_data[5] = 0x80;
			} else {
				ec_chip->i2c_dm_data[1] = 0x80;
			}
			asuspec_dockram_write_data(0x0A,9);
		}
	}
	return 0;
}
static void asuspec_reset_dock(void){
	ec_chip->dock_init = 0;
	ASUSPEC_NOTICE("send EC_Request \n");
	gpio_set_value(asuspec_ecreq_gpio, 0);
	msleep(20);
	gpio_set_value(asuspec_ecreq_gpio, 1);
}
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<

static int asuspec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i = 0;

	ec_chip->op_mode = 0;

	for ( i = 0; i < 10; i++){
		ret_val = asuspec_i2c_test(client);
		if (ret_val < 0)
			msleep(300);
		else
			break;
	}

	if(ret_val < 0){
		goto fail_to_access_ec;
	}

	for ( i=0; i<8; i++){
		asuspec_i2c_read_data(client);
	}

	if (asuspec_dockram_read_data(0x01) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("Model Name: %s\n", ec_chip->ec_model_name);

	if (asuspec_dockram_read_data(0x02) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);

	if (asuspec_dockram_read_data(0x03) < 0){
		goto fail_to_access_ec;
	}
	ASUSPEC_INFO("EC-Config Format: %s\n", &ec_chip->i2c_dm_data[1]);

	if (asuspec_dockram_read_data(0x04) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_pcba, &ec_chip->i2c_dm_data[1]);
	ASUSPEC_NOTICE("PCBA Version: %s\n", ec_chip->ec_pcba);

#if FACTORY_MODE
	if(factory_mode == 2)
		asuspec_enter_factory_mode();
	else
		asuspec_enter_normal_mode();
#else
		asuspec_enter_normal_mode();
#endif

	ec_chip->status = 1;
	switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
fail_to_access_ec:
	return 0;

}

static irqreturn_t asuspec_interrupt_handler(int irq, void *dev_id){
    int gpio = irq_to_gpio(irq);
	ASUSPEC_INFO("interrupt irq = %d, gpio = %d\n", irq, gpio);
    if (gpio == asuspec_apwake_gpio){
        disable_irq_nosync(irq);
            if (ec_chip->op_mode){
                queue_delayed_work(asuspec_wq, &ec_chip->asuspec_fw_update_work, 0);
            } else {
                queue_delayed_work(asuspec_wq, &ec_chip->asuspec_work, 0);
            }
    }
	/*Chris start*/
    else if (gpio == asuspec_kb_int_gpio){
        ASUSPEC_NOTICE("kb int = %d", irq);
        disable_irq_nosync(irq);
        queue_delayed_work(asuspec_wq, &ec_chip->asusdec_kb_report_work, 0);
    }
    /*Chris end*/
    //Shouchung add for touch pad
    else if (gpio == asusdec_ps2_int_gpio) {
        ASUSPEC_NOTICE("tp int = %d", irq);
        disable_irq_nosync(irq);
        queue_delayed_work(asuspec_wq, &ec_chip->asusdec_tp_report_work, 0);
    }
	//Shouchung end
    else if (irq == gpio_to_irq(asusdec_hall_sensor_gpio)){
        queue_delayed_work(asuspec_wq, &ec_chip->asusdec_hall_sensor_work, 0);
        ASUSPEC_NOTICE("lid irq = %d", irq);
    }
    return IRQ_HANDLED;
}

static int asusdec_irq_hall_sensor(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asusdec_hall_sensor_gpio;
	unsigned irq = gpio_to_irq(asusdec_hall_sensor_gpio);
	const char* label = "asusdec_hall_sensor" ;
	unsigned int pad_pid = tegra3_get_project_id();

	ASUSPEC_INFO("gpio = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asuspec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
//Panda Porting TF600T MP Hall Sensor:--->
  /*
	if ((pad_pid == TEGRA3_PROJECT_TF300T) || (pad_pid == TEGRA3_PROJECT_TF300TG)){
		ASUSPEC_NOTICE("Disable hall sensor wakeup function due to pid = %u\n", pad_pid);
	} else {
		enable_irq_wake(irq);
	}
	*/
	enable_irq_wake(irq);
//Panda Porting TF600T MP Hall Sensor:<---
	ASUSPEC_INFO("LID irq = %d, rc = %d\n", irq, rc);

	if (gpio_get_value(gpio)){
		ASUSPEC_NOTICE("LID open\n");
	} else{
		ASUSPEC_NOTICE("LID close\n");
	}

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}

static ssize_t asuspec_show_lid_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(asusdec_hall_sensor_gpio));
}

/*Chris start*/
static int asuspec_irq_kb_int(struct i2c_client *client)
{
        int rc = 0 ;
        unsigned gpio = asuspec_kb_int_gpio;
        /*Chris start*/
        gpio_free(gpio);
        /*Chris end*/
        int irq = gpio_to_irq(gpio);
        const char* label = "asuspec_kb_int" ;

        ASUSPEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
        ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

        rc = gpio_request(gpio, label);
        if (rc) {
               ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
               goto err_request_input_gpio_failed;
        }

        rc = gpio_direction_input(gpio) ;
        if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
                goto err_gpio_direction_input_failed;
        }
        ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

        rc = request_irq(irq, asuspec_interrupt_handler,/*IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
        if (rc < 0) {
                ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
                rc = -EIO;
                goto err_gpio_request_irq_fail ;
        }

        ASUSPEC_INFO("request irq = %d, rc = %d\n", irq, rc);

        return 0 ;

err_gpio_request_irq_fail:
        gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
        return rc;

        return 0 ;
}

static int asusdec_kp_key_mapping(int x)
{
	switch (x){
		case ASUSDEC_KEYPAD_ESC:
			return KEY_BACK;

		case ASUSDEC_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case ASUSDEC_KEYPAD_KEY_1:
			return KEY_1;

		case ASUSDEC_KEYPAD_KEY_2:
			return KEY_2;

		case ASUSDEC_KEYPAD_KEY_3:
			return KEY_3;

		case ASUSDEC_KEYPAD_KEY_4:
			return KEY_4;

		case ASUSDEC_KEYPAD_KEY_5:
			return KEY_5;

		case ASUSDEC_KEYPAD_KEY_6:
			return KEY_6;

		case ASUSDEC_KEYPAD_KEY_7:
			return KEY_7;

		case ASUSDEC_KEYPAD_KEY_8:
			return KEY_8;

		case ASUSDEC_KEYPAD_KEY_9:
			return KEY_9;

		case ASUSDEC_KEYPAD_KEY_0:
			return KEY_0;

		case ASUSDEC_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case ASUSDEC_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case ASUSDEC_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case ASUSDEC_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case ASUSDEC_KEYPAD_KEY_Q:
			return KEY_Q;

		case ASUSDEC_KEYPAD_KEY_W:
			return KEY_W;

		case ASUSDEC_KEYPAD_KEY_E:
			return KEY_E;

		case ASUSDEC_KEYPAD_KEY_R:
			return KEY_R;

		case ASUSDEC_KEYPAD_KEY_T:
			return KEY_T;

		case ASUSDEC_KEYPAD_KEY_Y:
			return KEY_Y;

		case ASUSDEC_KEYPAD_KEY_U:
			return KEY_U;

		case ASUSDEC_KEYPAD_KEY_I:
			return KEY_I;

		case ASUSDEC_KEYPAD_KEY_O:
			return KEY_O;

		case ASUSDEC_KEYPAD_KEY_P:
			return KEY_P;

		case ASUSDEC_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case ASUSDEC_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case ASUSDEC_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case ASUSDEC_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case ASUSDEC_KEYPAD_KEY_A:
			return KEY_A;

		case ASUSDEC_KEYPAD_KEY_S:
			return KEY_S;

		case ASUSDEC_KEYPAD_KEY_D:
			return KEY_D;

		case ASUSDEC_KEYPAD_KEY_F:
			return KEY_F;

		case ASUSDEC_KEYPAD_KEY_G:
			return KEY_G;

		case ASUSDEC_KEYPAD_KEY_H:
			return KEY_H;

		case ASUSDEC_KEYPAD_KEY_J:
			return KEY_J;

		case ASUSDEC_KEYPAD_KEY_K:
			return KEY_K;

		case ASUSDEC_KEYPAD_KEY_L:
			return KEY_L;

		case ASUSDEC_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case ASUSDEC_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case ASUSDEC_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case ASUSDEC_KEYPAD_KEY_Z:
			return KEY_Z;

		case ASUSDEC_KEYPAD_KEY_X:
			return KEY_X;

		case ASUSDEC_KEYPAD_KEY_C:
			return KEY_C;

		case ASUSDEC_KEYPAD_KEY_V:
			return KEY_V;

		case ASUSDEC_KEYPAD_KEY_B:
			return KEY_B;

		case ASUSDEC_KEYPAD_KEY_N:
			return KEY_N;

		case ASUSDEC_KEYPAD_KEY_M:
			return KEY_M;

		case ASUSDEC_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case ASUSDEC_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case ASUSDEC_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case ASUSDEC_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case ASUSDEC_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case ASUSDEC_KEYPAD_KEY_UP:
			return KEY_UP;

		case ASUSDEC_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case ASUSDEC_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case ASUSDEC_KEYPAD_WINAPP:
			return KEY_MENU;

		case ASUSDEC_KEYPAD_HOME:
			return KEY_HOME;

		case ASUSDEC_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case ASUSDEC_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case ASUSDEC_KEYPAD_END:
			return KEY_END;

		case ASUSDEC_KEYPAD_SCRLK:
			return KEY_SCROLLLOCK;

		case ASUSDEC_KEYPAD_NUMLK:
                        return KEY_NUMLOCK;	
	
		case ASUSDEC_KEYPAD_TPONOFF:
			return KEY_F2;

		case ASUSDEC_KEYPAD_MUTE:
                        return KEY_MUTE;
 
                case ASUSDEC_KEYPAD_VOLUMEDOWN:
                        return KEY_VOLUMEDOWN;
 
                case ASUSDEC_KEYPAD_VOLUMEUP:
                        return KEY_VOLUMEUP;

		case ASUSDEC_KEYPAD_DELETE:
			return KEY_DELETE;

		case ASUSDEC_KEYPAD_BRIGHTNESSDOWN:
			return KEY_BRIGHTNESSDOWN;

		case ASUSDEC_KEYPAD_BRIGHTNESSUP:
			return KEY_BRIGHTNESSUP;

		case ASUSDEC_KEYPAD_FLYMODE:
			return KEY_F22;

		case ASUSDEC_KEYPAD_SUSPEND:
			return KEY_SLEEP;

		case ASUSDEC_KEYPAD_PAUSE:
			return KEY_PAUSE;

		case ASUSDEC_KEYPAD_PRINTSCREEN:
			return KEY_PRINT;

		case ASUSDEC_KEYPAD_INSERT:
			return KEY_INSERT;
		/*Chris 0812 end*/
		/*Chris mark start
		//--- JP keys
		case ASUSDEC_YEN:
			return KEY_YEN;

		case ASUSDEC_RO:
			return KEY_RO;

		case ASUSDEC_MUHENKAN:
			return KEY_MUHENKAN;

		case ASUSDEC_HENKAN:
			return KEY_HENKAN;

		case ASUSDEC_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;

		//--- UK keys
		case ASUSDEC_EUROPE_2:
			return KEY_102ND;
		Chris mark end*/
		default:
			printk("No mapping string\n");
			return -1;
	}
}

static void asusdec_kb_report_work_function(struct work_struct *dat)
{
        int gpio = asuspec_kb_int_gpio;
        int irq = gpio_to_irq(gpio);
        int ret_val = 0;
        int i = 0;
	int j = 0;
	int the_same_key = 0;
        int scancode = 0;
        
        memset(&ec_chip->i2c_kb_data, 0, 32);
        
        ret_val = i2c_smbus_read_i2c_block_data(&kb_client, 0x73, 11, ec_chip->i2c_kb_data);
        enable_irq(irq);

	if(ec_chip->dock_status == 0){
                return;
        }

        if(ec_chip->i2c_kb_data[0] == 0 && ec_chip->i2c_kb_data[1] == 0){//not press key
                return;
        }

        ASUSPEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[0]);
        ASUSPEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[1]);
        ASUSPEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[2]);
        ASUSPEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[3]);
        ASUSPEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[4]);
        ASUSPEC_NOTICE("key code : 0x%x\n",ec_chip->i2c_kb_data[5]);
	
        ec_chip->keypad_data.extend = 0;
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
                input_report_key(ec_chip->indev, KEY_LEFTCTRL, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTCTRL){
                input_report_key(ec_chip->indev, KEY_LEFTCTRL, 0);
        }
	
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
                input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTCTRL){
                input_report_key(ec_chip->indev, KEY_RIGHTCTRL, 0);
        }

        if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
                input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_LEFTSHIFT){
                input_report_key(ec_chip->indev, KEY_LEFTSHIFT, 0);
        }

	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
                input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_KEY_RIGHTSHIFT){
                input_report_key(ec_chip->indev, KEY_RIGHTSHIFT, 0);
        }
       
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTALT){
                input_report_key(ec_chip->indev, KEY_LEFTALT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTALT){
                input_report_key(ec_chip->indev, KEY_LEFTALT, 0);
        }

	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
                input_report_key(ec_chip->indev, KEY_RIGHTALT, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_RIGHTALT){
                input_report_key(ec_chip->indev, KEY_RIGHTALT, 0);
        }
	if(ec_chip->i2c_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
                input_report_key(ec_chip->indev, KEY_HOMEPAGE, 1);
        }else if(ec_chip->i2c_old_kb_data[3] & ASUSDEC_KEYPAD_LEFTWIN){
                input_report_key(ec_chip->indev, KEY_HOMEPAGE, 0);
        }	

	for(i = 0;i < 6;i++)//normal keys
        {
                if(ec_chip->i2c_kb_data[i+5] > 0){//press key
                        ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_kb_data[i+5]);
                        ec_chip->keypad_data.value = 1;
                        ASUSPEC_INFO("keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
                        input_report_key(ec_chip->indev,
                               ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
                }else if(ec_chip->i2c_kb_data[i+5] == 0){
                       break;
                }else{
                       ASUSPEC_INFO("Unknown scancode = 0x%x\n", scancode);
                }
        }
	for(i = 0;i < 6;i++)
        {
                if(ec_chip->i2c_old_kb_data[i+5] > 0){
                        for(j = 0;j < 6;j++)//check key break
                        {
                                if(ec_chip->i2c_kb_data[j+5] == ec_chip->i2c_old_kb_data[i+5]){
                                       the_same_key = 1;
                                       break;
                                 }
                                 else
                                       the_same_key = 0;
                        }
                        if(the_same_key == 0){
                                ec_chip->keypad_data.input_keycode = asusdec_kp_key_mapping(ec_chip->i2c_old_kb_data[i+5]);
                                input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
                        }
                }else{
                        break;
                }
        }
	for(i = 0;i < 8;i++)
        {
               ec_chip->i2c_old_kb_data[i+3] = ec_chip->i2c_kb_data[i+3];
        }
        input_sync(ec_chip->indev);

}
/*Chris end*/

//Shouchung modify for using touchpad with the absolute mode
static void asusdec_tp_command(struct i2c_client *client, u8 protocol, 
		u8 command, int reg, u8 value)
{
	u8 tp_command[25];
	u8 reg_value;
	u8 reg_table;
	int i;

	memset(&tp_command, 0, 25);
	tp_command[0] = 0x00;
	tp_command[1] = 0x17;
	tp_command[2] = 0x00;
	tp_command[3] = protocol;
	tp_command[4] = command;
	reg_value = ((reg & 0xff00) >> 8);
	reg_table = reg & 0x00ff;
	tp_command[5] = reg_value;
	tp_command[6] = reg_table;
	tp_command[7] = value;
	i2c_smbus_write_i2c_block_data(client, 0x25, 25, tp_command);
	msleep(50);

	if (!command) {
		memset(&ec_chip->i2c_tp_data, 0, 38);
		i2c_smbus_read_i2c_block_data(client, 0x24, 32, ec_chip->i2c_tp_data);
		msleep(50);
		if (value > 0x10) {
			memset(&ec_chip->i2c_tp_data, 0, 38);
			i2c_smbus_read_i2c_block_data(client, 0x24, 32, ec_chip->i2c_tp_data);
			msleep(50);
		}
	}
}
//Shouchung end

//Shouchung add for using touchpad with the absolute mode
static void asusdec_tp_setup(struct i2c_client *client)
{
	memset(&ec_chip->i2c_tp_data, 0, 38);
	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x00;
	ec_chip->i2c_tp_data[2] = 0x08;
	i2c_smbus_write_i2c_block_data(client, 0x22, 3, ec_chip->i2c_tp_data);
	msleep(50);

	memset(&ec_chip->i2c_tp_data, 0, 38);
	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x00;
	ec_chip->i2c_tp_data[2] = 0x01;
	i2c_smbus_write_i2c_block_data(client, 0x22, 3, ec_chip->i2c_tp_data);
	msleep(50);

#if TF600T_TOUCHPAD_MODE
	memset(&ec_chip->i2c_tp_data, 0, 38);
	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x3F;
	ec_chip->i2c_tp_data[2] = 0x03;
	ec_chip->i2c_tp_data[3] = 0x0F;
	ec_chip->i2c_tp_data[4] = 0x23;
	ec_chip->i2c_tp_data[5] = 0x00;
	ec_chip->i2c_tp_data[6] = 0x04;
	ec_chip->i2c_tp_data[7] = 0x00;
	ec_chip->i2c_tp_data[8] = 0x0F;
	ec_chip->i2c_tp_data[9] = 0x01;
	i2c_smbus_write_i2c_block_data(client, 0x22, 10, ec_chip->i2c_tp_data);
	msleep(50);

	/* Check interrupt status */
	//asusdec_tp_command(client, 0x0A, 0x00, 0x1400, 0x01);
	//asusdec_tp_command(client, 0x0A, 0x00, 0x8800, 0x01);
	//asusdec_tp_command(client, 0x0A, 0x00, 0x8800, 0x01);
	/* Query Manufacturer ID(1), Product Properties(1), Customer Family(1),
	 * Firmware Revision(1), Device Serialization(7), Product ID(10)
	 */
	//asusdec_tp_command(client, 0x0A, 0x00, 0x8800, 0x15);
	/* Query Product Properties 2, Check DS4 */
	//asusdec_tp_command(client, 0x0A, 0x00, 0x9D00, 0x02);
	/* Query Per-device data(1), per-sensor data(15 in data sheet) */
	//asusdec_tp_command(client, 0x0A, 0x00, 0xA500, 0x1B);
	/* Query GPIO/LED */
	//asusdec_tp_command(client, 0x0A, 0x00, 0x0B03, 0x02);
	/* Get 2-D sensor control data */
	//asusdec_tp_command(client, 0x0A, 0x00, 0x4C00, 0x14);
	/* Set 2-D report mode to 0x40 */
	asusdec_tp_command(client, 0x09, 0x01, 0x4C00, 0x40);
	/* Set 2-D gesture enables to 0 */
	asusdec_tp_command(client, 0x09, 0x01, 0x5600, 0x00);
	/* Shouchung add, Modify the resolution to 1366x768 */
	asusdec_tp_command(client, 0x09, 0x01, 0x5200, 0x56);
	asusdec_tp_command(client, 0x09, 0x01, 0x5300, 0x05);
	asusdec_tp_command(client, 0x09, 0x01, 0x5400, 0x00);
	asusdec_tp_command(client, 0x09, 0x01, 0x5500, 0x03);
	/* Get GPIO direction */
	//asusdec_tp_command(client, 0x0A, 0x00, 0x0203, 0x01);
	/* Set interrupt enable 0 to 0x4E */
	asusdec_tp_command(client, 0x09, 0x01, 0x4800, 0x4E);
#endif
}
//Shouchung end

//Shouchung add for touch pad
static int asusdec_irq_tp_int(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asusdec_ps2_int_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asusdec_ps2_int" ;

	ASUSPEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d, state = %d\n", gpio, gpio_get_value(gpio));

	gpio_free(gpio);
	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asuspec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	ASUSPEC_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;

	return 0 ;
}

static void asusdec_tp_report_work_function(struct work_struct *dat) {
	int gpio = asusdec_ps2_int_gpio;
	int irq = gpio_to_irq(gpio);
    memset(&ec_chip->i2c_tp_data, 0, 38);
    //Shouchung modify for using touchpad with the absolute mode
	i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 32, ec_chip->i2c_tp_data);
	asusdec_touchpad_processing();
	//Shouchung end
	enable_irq(irq);
}

//Shouchung modify for using touchpad with the absolute mode
void asusdec_tp_enable(u8 cmd)
{
#if TF600T_TOUCHPAD_MODE
	if (cmd == 0xf4) {
		asusdec_tp_command(&tp_client, 0x09, 0x01, 0x4800, 0x4E);
		ASUSPEC_NOTICE("tp enable\n");
	} else {
		asusdec_tp_command(&tp_client, 0x09, 0x01, 0x4800, 0x00);
		ASUSPEC_NOTICE("tp disable\n");
	}
#else
	memset(&ec_chip->i2c_tp_data, 0, 38);
	ec_chip->i2c_tp_data[0] = 0x00;
	ec_chip->i2c_tp_data[1] = 0x05;
	ec_chip->i2c_tp_data[2] = 0x00;
	ec_chip->i2c_tp_data[3] = 0x22;
	ec_chip->i2c_tp_data[4] = 0xd4;
	ec_chip->i2c_tp_data[5] = cmd;
	i2c_smbus_write_i2c_block_data(&tp_client, 0x25, 6, ec_chip->i2c_tp_data);
	msleep(50);
	if(cmd == 0xf4)
		ASUSPEC_NOTICE("tp enable\n");
	else
		ASUSPEC_NOTICE("tp disable\n");
	i2c_smbus_read_i2c_block_data(&tp_client, 0x24, 38, ec_chip->i2c_tp_data);
	msleep(50);
#endif
}
//Shouchung end

//Shouchung add to solve the touch pad issue when dock in
static void asusdec_tp_enable_function(u8 cmd)
{
	mutex_lock(&ec_chip->tp_lock);
	ASUSPEC_NOTICE("Enter mutex lock \n");
	if ((cmd == 0xf4) && (ec_chip->kb_and_ps2_enable == 0) &&
		(ec_chip->tp_enable == 1)) {
		//Shouchung add for using touchpad with the absolute mode
		asusdec_tp_setup(&tp_client);
		//Shouchung end
		asusdec_tp_enable(cmd);
		ec_chip->touchpad_member = ELANTOUCHPAD;

		enable_irq(gpio_to_irq(asusdec_ps2_int_gpio));
		ec_chip->kb_and_ps2_enable = 1;
	} else if ((ec_chip->kb_and_ps2_enable == 1) && (ec_chip->tp_enable == 1)) {
		disable_irq_nosync(gpio_to_irq(asusdec_ps2_int_gpio));
		if (cmd == 0xf5)
			asusdec_tp_enable(cmd);
		ec_chip->kb_and_ps2_enable = 0;
	}
	ASUSPEC_NOTICE("Leave mutex lock \n");
	mutex_unlock(&ec_chip->tp_lock);
}
//Shouchung end

static int asusdec_tp_control(int arg){

	int ret_val = 0;

	if(arg == ASUSDEC_TP_ON){
		/*Chris 0912 start*/
		TP_LED_Control(0);
		/*Chris 0912 end*/
		if (ec_chip->tp_enable == 0){
			//Shouchung modify for touchpad enable/diable function
			ec_chip->tp_enable = 1;
			asusdec_tp_enable_function(0xf4);
			//Shouchung end
		}
		if (ec_chip->touchpad_member == -1){
			ec_chip->init_success = -1;
			queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
		}
		ret_val = 0;
	} else if (arg == ASUSDEC_TP_OFF){
		/*Chris 0912 start*/
		TP_LED_Control(1);
		/*Chris 0912 end*/
		//Shouchung modify for touchpad enable/diable function
		asusdec_tp_enable_function(0xf5);
		ec_chip->tp_enable = 0;
		//Shouchung end
		ret_val = 0;
	} else
		ret_val = -ENOTTY;

	return ret_val;

}

#if (!TF600T_TOUCHPAD_MODE)
static void asusdec_tp_rel(void){
	ec_chip->touchpad_data.x_sign = (ec_chip->i2c_tp_data[3] & X_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.y_sign = (ec_chip->i2c_tp_data[3] & Y_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.left_btn = (ec_chip->i2c_tp_data[3] & LEFT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.right_btn = (ec_chip->i2c_tp_data[3] & RIGHT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.delta_x =
		(ec_chip->i2c_tp_data[4] > 0x80) ? (ec_chip->i2c_tp_data[4] - 0xff):ec_chip->i2c_tp_data[4];
	ec_chip->touchpad_data.delta_y =
		(ec_chip->i2c_tp_data[5] > 0x80) ? (ec_chip->i2c_tp_data[5] - 0xff):ec_chip->i2c_tp_data[5];

	input_report_rel(ec_chip->indev, REL_X, ec_chip->touchpad_data.delta_x);
	input_report_rel(ec_chip->indev, REL_Y, ec_chip->touchpad_data.delta_y);
	input_report_key(ec_chip->indev, BTN_LEFT, ec_chip->touchpad_data.left_btn);
	input_report_key(ec_chip->indev, BTN_RIGHT, ec_chip->touchpad_data.right_btn);
	input_sync(ec_chip->indev);
}
#endif

//Shouchung modify for using touchpad with the absolute mode
#if TF600T_TOUCHPAD_MODE
static void asusdec_tp_abs(void) {
	int i;
	u8 finger_state;
	int tmp_x, tmp_y;
	int finger_num;

	finger_num = 0;
	for (i = 0; i < 5; i++) {
		if (i < 4)
			finger_state = (ec_chip->i2c_tp_data[4] >> 2 * i) & 0x03;
		else
			finger_state = ec_chip->i2c_tp_data[5] & 0x03;
		if (finger_state) {
			tmp_x = ec_chip->i2c_tp_data[6 + i * 5];
			ec_chip->t_abs.x_pos[i] = (tmp_x << 4) + (ec_chip->i2c_tp_data[8 + i * 5] & 0x0f);
			tmp_y = ec_chip->i2c_tp_data[7 + i * 5];
			ec_chip->t_abs.y_pos[i] = (tmp_y << 4) + ((ec_chip->i2c_tp_data[8 + i * 5] >> 4) & 0x0f);
			ec_chip->t_abs.w_val[i] = ec_chip->i2c_tp_data[9 + i * 5] & 0x0f;
			ec_chip->t_abs.z_val[i] = ec_chip->i2c_tp_data[10 + i * 5];
			finger_num++;
		} else {
			ec_chip->t_abs.x_pos[i] = 0;
			ec_chip->t_abs.y_pos[i] = 0;
			ec_chip->t_abs.w_val[i] = 0;
			ec_chip->t_abs.z_val[i] = 0;
		}
	}

	for (i = 0; i < 5; i++) {
		input_mt_slot(ec_chip->indev, i);
		if (i >= finger_num)
			input_mt_report_slot_state(ec_chip->indev, MT_TOOL_FINGER, false);
		else {
			input_mt_report_slot_state(ec_chip->indev, MT_TOOL_FINGER, true);
			input_report_abs(ec_chip->indev, ABS_MT_POSITION_X, ec_chip->t_abs.x_pos[i]);
			input_report_abs(ec_chip->indev, ABS_MT_POSITION_Y, 768 - ec_chip->t_abs.y_pos[i]);
			input_report_abs(ec_chip->indev, ABS_MT_PRESSURE, ec_chip->t_abs.z_val[i]);
			input_report_abs(ec_chip->indev, ABS_MT_TOUCH_MAJOR, ec_chip->t_abs.w_val[i]);
		}
	}

	input_report_key(ec_chip->indev, BTN_TOUCH, finger_num > 0);
	if (finger_num > 0) {
		input_report_abs(ec_chip->indev, ABS_X, ec_chip->t_abs.x_pos[0]);
		input_report_abs(ec_chip->indev, ABS_Y, 768 - ec_chip->t_abs.y_pos[0]);
	}
	input_report_abs(ec_chip->indev, ABS_PRESSURE, ec_chip->t_abs.z_val[0]);
	input_report_abs(ec_chip->indev, ABS_TOOL_WIDTH, ec_chip->t_abs.w_val[0]);
	input_report_key(ec_chip->indev, BTN_TOOL_FINGER, finger_num == 1);
	input_report_key(ec_chip->indev, BTN_TOOL_DOUBLETAP, finger_num == 2);
	input_report_key(ec_chip->indev, BTN_TOOL_TRIPLETAP, finger_num == 3);

	//Shouchung modify for right button
	if ((ec_chip->i2c_tp_data[3] & 0x40) == 0x40) {
		if (ec_chip->i2c_tp_data[31] == 0) {
			if (ec_chip->t_abs.x_pos[finger_num - 1] < 683) {
				input_report_key(ec_chip->indev, BTN_LEFT, true);
				ec_chip->t_abs.left = 1;
			} else {
				input_report_key(ec_chip->indev, BTN_RIGHT, true);
				ec_chip->t_abs.right = 1;
			}
		} else if (ec_chip->i2c_tp_data[31] == 4) {
			if (ec_chip->t_abs.left == 1) {
				input_report_key(ec_chip->indev, BTN_LEFT, false);
				ec_chip->t_abs.left = 0;
			} else if (ec_chip->t_abs.right == 1) {
				input_report_key(ec_chip->indev, BTN_RIGHT, false);
				ec_chip->t_abs.right = 0;
			}
		}
	}
	//Shouchung end

	input_sync(ec_chip->indev);
}
#endif
//Shouchung end

//Shouchung modify for using touchpad with the absolute mode
static void asusdec_touchpad_processing(void){
#if TF600T_TOUCHPAD_MODE
	asusdec_tp_abs();
#else
	asusdec_tp_rel();
#endif
}
//Shouchung end
//Shouchung end


static int asuspec_irq_ec_request(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_ecreq_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_request" ;

	ASUSPEC_INFO("gpio = %d, irq = %d\n", gpio,irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_exit;
	}

	rc = gpio_direction_output(gpio, 1) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_output failed for input %d\n", gpio);
		goto err_exit;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));
	return 0 ;

err_exit:
	return rc;
}


static int asuspec_irq_ec_apwake(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	const char* label = "asuspec_apwake" ;

	ASUSPEC_INFO("GPIO = %d, irq = %d\n", gpio, irq);
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		ASUSPEC_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		ASUSPEC_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	ASUSPEC_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, asuspec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		ASUSPEC_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(gpio_to_irq(asuspec_apwake_gpio));
	ASUSPEC_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail:
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;

	return 0 ;
}

static void asuspec_enter_s3_timer(unsigned long data){
	queue_delayed_work(asuspec_wq, &ec_chip->asuspec_enter_s3_work, 0);
}

static void asuspec_send_ec_req(void){
	ASUSPEC_NOTICE("send EC_Request\n");
	//Shouchung add to solve the touch pad issue when dock in
	asusdec_tp_enable_function(0xf5);
	//Shouchung end
	gpio_set_value(asuspec_ecreq_gpio, 0);
	msleep(DELAY_TIME_MS);
	gpio_set_value(asuspec_ecreq_gpio, 1);
	//Shouchung add to solve the touch pad issue when dock in
	msleep(100);
	asusdec_tp_enable_function(0xf4);
	//Shouchung end
}

static void asuspec_smi(void){
	if (ec_chip->i2c_data[2] == ASUSPEC_SMI_HANDSHAKING){
		ASUSPEC_NOTICE("ASUSPEC_SMI_HANDSHAKING\n");
		if(ec_chip->status == 0){
			asuspec_chip_init(ec_chip->client);
		}
		ec_chip->ec_in_s3 = 0;
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SMI_RESET){
		ASUSPEC_NOTICE("ASUSPEC_SMI_RESET\n");
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SMI_WAKE){
		ASUSPEC_NOTICE("ASUSPEC_SMI_WAKE\n");
	} else if (ec_chip->i2c_data[2] == APOWER_SMI_S5){
		ASUSPEC_NOTICE("APOWER_POWEROFF\n");
		asuspec_switch_apower_state(APOWER_POWEROFF);
	} else if (ec_chip->i2c_data[2] == APOWER_SMI_NOTIFY_SHUTDOWN){
		ASUSPEC_NOTICE("APOWER_NOTIFY_SHUTDOWN\n");
		asuspec_switch_apower_state(APOWER_NOTIFY_SHUTDOWN);
	} else if (ec_chip->i2c_data[2] == APOWER_SMI_RESUME){
		ASUSPEC_NOTICE("APOWER_SMI_RESUME\n");
		asuspec_switch_apower_state(APOWER_RESUME);
	}
	//[Brook- Bug 282345 + No charging icon when insert AC] >>
	else if (ec_chip->i2c_data[2] == ASUSPEC_SMI_AC_EVENT){
		ASUSPEC_NOTICE("ASUSPEC_SMI_AC_EVENT\n");
		//0x1: 0.5A connect 
		//0x3: 15V connect
		battery_callback(Check_Charging_Cable_Type());
		wake_lock_timeout(&ec_chip->wake_lock, 5* HZ);
	}
	//[Brook- Bug 282345 + No charging icon when insert AC]<<
       //[Brook- Docking charging porting]>>
	else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_Battery_Updated){
		ASUSPEC_NOTICE("ASUSPEC_SxI_Battery_Updated \n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_EC_WAKEUP){        
		ASUSPEC_NOTICE("ASUSPEC_SxI_EC_WAKEUP \n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_BOOTBLOCK_RESET){ 
		ASUSPEC_NOTICE("ASUSPEC_SxI_BOOTBLOCK_RESET \n");
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	}else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_WATCHDOG_RESET){
		ASUSPEC_NOTICE("ASUSPEC_SxI_WATCHDOG_RESET \n");
		queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	}else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_ADAPTER_CHANGE){
		ASUSPEC_NOTICE("ASUSPEC_SxI_ADAPTER_CHANGE \n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_INSERT){    
		ASUSPEC_NOTICE("ASUSPEC_SxI_DOCK_INSERT\n");
		queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
	}else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_DOCK_REMOVE){     
		ASUSPEC_NOTICE("ASUSPEC_SxI_DOCK_REMOVE\n");
		queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_PAD_BL_CHANGE){ 
		ASUSPEC_NOTICE("ASUSPEC_SxI_PAD_BL_CHANGE \n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_HID_Status_Changed){
		ASUSPEC_NOTICE("ASUSPEC_SxI_HID_Status_Changed \n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_HID_WakeUp){      
		ASUSPEC_NOTICE("ASUSPEC_SxI_HID_WakeUp \n");
	} else if (ec_chip->i2c_data[2] == ASUSPEC_DOCK_SxI_AC_Event){   
		ASUSPEC_NOTICE("ASUSPEC_DOCK_SxI_AC_Event \n");
		docking_callback(Check_Charging_Cable_Type());
		wake_lock_timeout(&ec_chip->wake_lock, 5* HZ);
	}else if (ec_chip->i2c_data[2] == ASUSPEC_SxI_Battery_FCCchange){
		ASUSPEC_NOTICE("Battery full charge capacity change!\n");
	}else if(ec_chip->i2c_data[2] == ASUSPEC_SxI_Battery_Low){
		ASUSPEC_NOTICE("ASUSPEC_SxI_Battery_Low!\n");
		low_battery_detect_isr();
	}
       //[Brook- Docking charging porting]>>
}
static void asuspec_enter_s3_work_function(struct work_struct *dat)
{
	int ret_val = 0;
	int i = 0;

	mutex_lock(&ec_chip->state_change_lock);

	if (ec_chip->op_mode){
		ASUSPEC_ERR("It's not allowed to access dockram under FW update mode.\n");
		mutex_unlock(&ec_chip->state_change_lock);
		return ;
	}

	ec_chip->ec_in_s3 = 1;
	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x02;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Send s3 command fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("EC in S3\n");
			break;
		}
	}
	mutex_unlock(&ec_chip->state_change_lock);
}

static void asuspec_init_work_function(struct work_struct *dat)
{
	asuspec_send_ec_req();
	msleep(200);
	asuspec_chip_init(ec_chip->client);
}

static void asuspec_stresstest_work_function(struct work_struct *dat)
{
	asuspec_i2c_read_data(ec_chip->client);
	queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
}

//[Brook- Docking charging porting]>>
static void asusdec_dock_status_report(void){
	ASUSPEC_NOTICE("dock_in = %d, ec_chip->dock_type = %d \n", ec_chip->dock_in, ec_chip->dock_type);
	switch_set_state(&ec_chip->dock_sdev, switch_value[ec_chip->dock_type]);
}

bool asuspec_check_dock_in_control_flag(void){
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asuspec_dockram_read_data(0x0A);
	if (ret_val < 0)
		ASUSPEC_NOTICE("fail to read dockram data\n");
	if((ec_chip->i2c_dm_data[1] & ASUSPEC_DOCK_PRESENT) && (!gpio_get_value(asuspec_dock_in_gpio)))
		return true;
	else
		return false;
}

static int asusdec_is_init_running(void){
	int ret_val;

	mutex_lock(&ec_chip->dock_init_lock);
	ret_val = ec_chip->dock_init;
	ec_chip->dock_init = 1;
	mutex_unlock(&ec_chip->dock_init_lock);
	return ret_val;
}

static int asusdec_dockram_read_data(int cmd)
{
	int ret = 0;
	int i = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}
	memset(&ec_chip->i2c_dm_data, 0, 32);
	ec_chip->i2c_dm_data[0] = 0x05;
	ec_chip->i2c_dm_data[1] = 0x0b;//i2c read block
	ec_chip->i2c_dm_data[2] = 0x00;//result :read only
	ec_chip->i2c_dm_data[3] = 0x36;//8bit dock i2c address
	ec_chip->i2c_dm_data[4] = (u8)cmd;
	ec_chip->i2c_dm_data[5] = (u8)24;//read byte number
	ret = i2c_smbus_write_i2c_block_data(&dockram_client, 0x11, 6, ec_chip->i2c_dm_data);
	if (ret < 0) {
	        ASUSPEC_ERR("Fail to write dockram data, status %d\n", ret);
	}
	msleep(20);
	ret = i2c_smbus_read_i2c_block_data(&dockram_client, 0x11, 32, ec_chip->i2c_dm_data);
	if (ret < 0) {
	        ASUSPEC_ERR("Fail to read dockram data, status %d\n", ret);
	}
	//FIXME:read status data
	for(i=9; i<32; i++)
	{
		ec_chip->i2c_dm_data[i-9] = ec_chip->i2c_dm_data[i];
	}
	return ret;
}

static int asusdec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	//FIXME:use 0x11 cmd  ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		ASUSPEC_ERR("Fail to read data, status %d\n", ret);
	}
	return ret;
}

static void asusdec_keypad_set_input_params(struct input_dev *dev)
{
	int i = 0;
	set_bit(EV_KEY, dev->evbit);
	for ( i = 0; i < 246; i++)
		set_bit(i,dev->keybit);
	//Shouchung add for touch pad
	//Shouchung modify for using touchpad with the absolute mode
#if TF600T_TOUCHPAD_MODE
	set_bit(INPUT_PROP_POINTER, dev->propbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);
	set_bit(BTN_TOOL_FINGER, dev->keybit);
	set_bit(BTN_TOOL_DOUBLETAP, dev->keybit);
	set_bit(BTN_TOOL_TRIPLETAP, dev->keybit);
	input_set_abs_params(dev, ABS_X, 0, 1366, 0, 0);
    input_set_abs_params(dev, ABS_Y, 0, 768, 0, 0);
    input_set_abs_params(dev, ABS_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
    input_abs_set_res(dev, ABS_X, 1366);
	input_abs_set_res(dev, ABS_Y, 768);

	input_mt_init_slots(dev, 5);
	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, 1366, 0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, 768, 0, 0);
	input_abs_set_res(dev, ABS_MT_POSITION_X, 1366);
	input_abs_set_res(dev, ABS_MT_POSITION_Y, 768);
	input_set_abs_params(dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
#else
    set_bit(EV_REL, dev->evbit);
	set_bit(REL_X, dev->relbit);
    set_bit(REL_Y, dev->relbit);
#endif
	//Shouchung end
	set_bit(BTN_LEFT, dev->keybit);
	set_bit(BTN_RIGHT, dev->keybit);
    set_bit(EV_SYN, dev->evbit);
	//Shouchung end
	input_set_capability(dev, EV_LED, LED_CAPSL);
}

static int asusdec_input_device_create(struct i2c_client *client){
	int err = 0;
	if (ec_chip->indev){
		return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		ASUSPEC_ERR("input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	ec_chip->indev->name = "asusdec";
	ec_chip->indev->phys = "/dev/input/asusdec";
	ec_chip->indev->dev.parent = &client->dev;
	ec_chip->indev->event = asusdec_event;
	asusdec_keypad_set_input_params(ec_chip->indev);
	err = input_register_device(ec_chip->indev);
	if (err) {
		ASUSPEC_ERR("input registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
exit:
	return err;
}

static int asusdec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value){
	ASUSPEC_INFO("type = 0x%x, code = 0x%x, value = 0x%x\n", type, code, value);
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if ((type == EV_LED) && (code == LED_CAPSL)){
			if(value == 0){
				//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]>>
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_led_off_work, 0);
				//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]<<
				return 0;
			} else {
				//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]>>
				queue_delayed_work(asuspec_wq, &ec_chip->asusdec_led_on_work, 0);
				//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]<<
				return 0;
			}
		}
	}
	return -ENOTTY;
}
//[Brook- Docking charging porting]<<


static void asusdec_dock_init_work_function(struct work_struct *dat)
{
	int i = 0;
	int d_counter = 0;
	int gpio_state = 0;
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
	wake_lock(&ec_chip->wake_lock_init);
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
	//Shouchung remove for solve dock in and out can't work problem
	/*if(asusdec_is_init_running()){
           return;
	}*/
	//Shouchung end
	if (!asuspec_check_dock_in_control_flag()){
           ASUSPEC_NOTICE("No dock detected\n");
           ec_chip->dock_in = 0;
           ec_chip->init_success = 0;
           ec_chip->dock_status = 0;
           ec_chip->dock_init = 0;
           ec_chip->dock_type = DOCK_UNKNOWN;
           ec_chip->touchpad_member = -1;
           memset(ec_chip->dec_model_name, 0, 32);
           memset(ec_chip->dec_version, 0, 32);
           /*Chris start*/
           if (ec_chip->indev){
               input_unregister_device(ec_chip->indev);
               ec_chip->indev = NULL;
           }
           /*Chris end*/
           if (ec_chip->private->abs_dev){
               input_unregister_device(ec_chip->private->abs_dev);
               ec_chip->private->abs_dev = NULL;
           }
           //Shouchung modify to solve the touch pad issue when dock in
           asusdec_tp_enable_function(0);
           //Shouchung end
           //Shouchung add for enable touchpad when dock-in
           ec_chip->tp_enable = 0;
           //Shouchung end
           asusdec_dock_status_report();
#if BATTERY_DRIVER
           docking_callback(Check_Charging_Cable_Type());
#endif
	}else{
           ASUSPEC_NOTICE("Dock-in detected\n");
           /*Chris start*/
           if (&kb_client != NULL){
               asusdec_input_device_create(&kb_client);
               printk("asusdec_input_device_create in\n");
           }
           printk("asusdec_input_device_create out\n");
           memset(&ec_chip->i2c_kb_data, 0, 32);
           ec_chip->i2c_kb_data[0] = 0x00;
           ec_chip->i2c_kb_data[1] = 0x00;
           ec_chip->i2c_kb_data[2] = 0x08;
           i2c_smbus_write_i2c_block_data(&kb_client, 0x75, 3, ec_chip->i2c_kb_data);
           msleep(50);//FIXME:will use retry
           i2c_smbus_read_i2c_block_data(&kb_client, 0x73, 11, ec_chip->i2c_kb_data);
           /*Chris end*/
           msleep(50);//FIXME:use retry
           ec_chip->dock_type = MOBILE_DOCK;
           ec_chip->dock_in = 1;
           for ( i=0; i<8; i++){
               asusdec_i2c_read_data(ec_chip->client);
             }
           if (asusdec_dockram_read_data(0x01) < 0){
               goto fail_to_access_ec;
             }
           strcpy(ec_chip->dec_model_name, &ec_chip->i2c_dm_data[0]);
           ASUSPEC_NOTICE("dock Model Name: %s\n", ec_chip->dec_model_name);
           if (asusdec_dockram_read_data(0x02) < 0){
               goto fail_to_access_ec;
             }
           strcpy(ec_chip->dec_version, &ec_chip->i2c_dm_data[0]);
               ASUSPEC_NOTICE("DEC-FW Version: %s\n", ec_chip->dec_version);
           if (asusdec_dockram_read_data(0x03) < 0){
               goto fail_to_access_ec;
             }
           ASUSPEC_INFO("DEC-Config Format: %s\n", &ec_chip->i2c_dm_data[0]);
           if (asusdec_dockram_read_data(0x04) < 0){
               goto fail_to_access_ec;
             }
           strcpy(ec_chip->dock_pid, &ec_chip->i2c_dm_data[0]);
           ASUSPEC_INFO("dock PCBA Version: %s\n", ec_chip->dock_pid);
           if (asusdec_dockram_read_data(0x0A) < 0){
               goto fail_to_access_ec;
             }
           ec_chip->dock_behavior = ec_chip->i2c_dm_data[2] & 0x02;
           ASUSPEC_NOTICE("DEC-FW Behavior: %s\n", ec_chip->dock_behavior ?
			"susb on when receive ec_req" : "susb on when system wakeup");
#if FACTORY_MODE
           if(factory_mode == 2)
               asusdec_enter_factory_mode();
#endif
           //Shouchung add for enable touchpad when dock-in
           ec_chip->tp_enable = 1;
           //Shouchung end
           //Shouchung add to solve the touch pad issue when dock in
           asusdec_tp_enable_function(0xf4);
           //Shouchung end

           ec_chip->init_success = 1;
           ec_chip->dock_status = 1;
           asusdec_dock_status_report();
#if BATTERY_DRIVER
           docking_callback(Check_Charging_Cable_Type());
#endif
	}
exit:
	ASUSPEC_NOTICE("exit! \n");
	wake_unlock(&ec_chip->wake_lock_init);
	return 0;

fail_to_access_ec:
	if (asusdec_dockram_read_data(0x00) < 0){
	    ASUSPEC_NOTICE("No EC detected\n");
           ec_chip->dock_in = 0;
	} else {
	    ASUSPEC_NOTICE("Need EC FW update\n");
	}
	goto exit;
}


static void asusdec_lid_report_function(struct work_struct *dat)
{
	int value = 0;

	if (ec_chip->lid_indev == NULL){
		ASUSPEC_ERR("LID input device doesn't exist\n");
		return;
	}
	msleep(CONVERSION_TIME_MS);
	value = gpio_get_value(asusdec_hall_sensor_gpio);
	input_report_switch(ec_chip->lid_indev, SW_LID, !value);
	input_sync(ec_chip->lid_indev);
	ASUSPEC_NOTICE("SW_LID report value = %d\n", !value);
}
//[Brook- Docking charging porting]<<

static void asuspec_fw_update_work_function(struct work_struct *dat)
{
	int smbus_data;
	int gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);

	mutex_lock(&ec_chip->lock);
	smbus_data = i2c_smbus_read_byte_data(&dockram_client, 0);
	enable_irq(irq);
	BuffPush(smbus_data);
	mutex_unlock(&ec_chip->lock);
}

static void asuspec_work_function(struct work_struct *dat)
{
	int gpio = asuspec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;

	ret_val = asuspec_i2c_read_data(ec_chip->client);
	enable_irq(irq);

	ASUSPEC_NOTICE("0x%x 0x%x 0x%x 0x%x\n", ec_chip->i2c_data[0],
		ec_chip->i2c_data[1], ec_chip->i2c_data[2], ec_chip->i2c_data[3]);

	if (ret_val < 0){
		return ;
	}

	if (ec_chip->i2c_data[1] & ASUSPEC_OBF_MASK){
		if (ec_chip->i2c_data[1] & ASUSPEC_SMI_MASK){
			asuspec_smi();
			return ;
		}
	}
}

static int __devinit asuspec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	ASUSPEC_INFO("asuspec probe\n");
	err = sysfs_create_group(&client->dev.kobj, &asuspec_smbus_group);
	if (err) {
		ASUSPEC_ERR("Unable to create the sysfs\n");
		goto exit;
	}

	ec_chip = kzalloc(sizeof (struct asuspec_chip), GFP_KERNEL);
	if (!ec_chip) {
		ASUSPEC_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	//[Brook- Docking charging porting]>>
	ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
	if (!ec_chip->private) {
		ASUSPEC_ERR("Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}
	//[Brook- Docking charging porting]<<
	dec_chip = kzalloc(sizeof (struct asusdec_chip), GFP_KERNEL);
	if (!dec_chip) {
		ASUSPEC_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	//[Brook- Docking charging porting]>>
	dec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
	if (!dec_chip->private) {
		ASUSPEC_ERR("Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}	
	//[Brook- Docking charging porting]<<
	ec_chip->pad_pid = tegra3_get_project_id();
	i2c_set_clientdata(client, ec_chip);
	i2c_set_clientdata(client, dec_chip);
	ec_chip->client = client;
	ec_chip->client->driver = &asuspec_driver;
	ec_chip->client->flags = 1;
	init_timer(&ec_chip->asuspec_timer);
	/*Chris start*/
	asuspec_kb_init(client);
	/*Chris end*/
	//Shouchung add for touch pad
	asusdec_tp_init(client);
	//Shouchung end
	ec_chip->asuspec_timer.function = asuspec_enter_s3_timer;
	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asuspec_wake");
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "asusdec_wake_init");
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
	mutex_init(&ec_chip->lock);
	mutex_init(&ec_chip->irq_lock);
	mutex_init(&ec_chip->state_change_lock);
	//Shouchung add to solve the touch pad issue when dock in
	mutex_init(&ec_chip->tp_lock);
	//Shouchung end
	//[Brook- Docking charging porting]>>
	mutex_init(&ec_chip->dock_init_lock);
	ec_chip->indev = NULL;
	ec_chip->lid_indev = NULL;
	ec_chip->private->abs_dev = NULL;
	dec_chip->private->abs_dev = NULL;
	ec_chip->dock_status = 0;
	ec_chip->dock_init = 0;	
	//[Brook- Docking charging porting]<<
	ec_chip->ec_ram_init = 0;
	ec_chip->audio_recording = 0;
	ec_chip->status = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->apwake_disabled = 0;
	ec_chip->dock_type = DOCK_UNKNOWN;
	//Shouchung add for touch pad
	ec_chip->kb_and_ps2_enable = 0;
	//Shouchung end
	//Shouchung modify for enable touchpad when dock-in
	ec_chip->tp_enable = 0;
	//Shouchung end
	asuspec_dockram_init(client);
	cdev_add(asuspec_cdev,asuspec_dev,1) ;

	ec_chip->pad_sdev.name = PAD_SDEV_NAME;
	ec_chip->pad_sdev.print_name = asuspec_switch_name;
	ec_chip->pad_sdev.print_state = asuspec_switch_state;
	if(switch_dev_register(&ec_chip->pad_sdev) < 0){
		ASUSPEC_ERR("switch_dev_register for pad failed!\n");
	}
	switch_set_state(&ec_chip->pad_sdev, 0);

	ec_chip->apower_sdev.name = APOWER_SDEV_NAME;
	ec_chip->apower_sdev.print_name = apower_switch_name;
	ec_chip->apower_sdev.print_state = apower_switch_state;
	ec_chip->apower_state = 0;
	if(switch_dev_register(&ec_chip->apower_sdev) < 0){
		ASUSPEC_ERR("switch_dev_register for apower failed!\n");
	}
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	//[Brook- Docking charging porting]>>
	ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
	ec_chip->dock_sdev.print_name = asusdec_switch_name;
	ec_chip->dock_sdev.print_state = asusdec_switch_state;
	if(switch_dev_register(&ec_chip->dock_sdev) < 0){
		ASUSPEC_ERR("switch_dev_register for dock failed!\n");
		goto exit;
	}
	switch_set_state(&ec_chip->dock_sdev, 0);
    //[Brook- Docking charging porting]<<
    //[Panda-Porting Hall Sensor]>>
    asusdec_lid_input_device_create(ec_chip->client);
    //[Panda-Porting Hall Sensor]<<
	asuspec_wq = create_singlethread_workqueue("asuspec_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_hall_sensor_work, asusdec_lid_report_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_work, asuspec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_init_work, asuspec_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_fw_update_work, asuspec_fw_update_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asuspec_enter_s3_work, asuspec_enter_s3_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&asuspec_stress_work, asuspec_stresstest_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_dock_init_work, asusdec_dock_init_work_function);
	/*Chris start*/
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_kb_report_work, asusdec_kb_report_work_function);
	/*Chris end*/
	//Shouchung add for touch pad
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_tp_report_work, asusdec_tp_report_work_function);
	//Shouchung end
	asuspec_irq_ec_request(client);
	asuspec_irq_ec_apwake(client);
	//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]>>
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_led_on_work, asusdec_keypad_led_on);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusdec_led_off_work, asusdec_keypad_led_off);
	//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]<<
	//[Brook- Docking charging porting]>>
	asusdec_irq_hall_sensor(client);
	//[Brook- Docking charging porting]<<
	queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
	queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 5*HZ);
	/*Chris start*/
	asuspec_irq_kb_int(client);
	/*Chris end*/
	//Shouchung add for touch pad
	asusdec_irq_tp_int(&tp_client);
	//Shouchung end
	queue_delayed_work(asuspec_wq, &ec_chip->asusdec_hall_sensor_work, 0);
	return 0;

exit:
	return err;
}

static int __devexit asuspec_remove(struct i2c_client *client)
{
	struct asuspec_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static ssize_t asuspec_status_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->status);
}

static ssize_t asuspec_info_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asuspec_version_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asuspec_battery_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int bat_status, bat_temp, bat_vol, bat_current, bat_capacity, remaining_cap;
	int ret_val;
	char temp_buf[64];
	//[Brook- Docking charging porting]>>
	bat_status = asuspec_battery_monitor("status",pad_battery_type);
	bat_temp = asuspec_battery_monitor("temperature",pad_battery_type);
	bat_vol = asuspec_battery_monitor("voltage",pad_battery_type);
	bat_current = asuspec_battery_monitor("current",pad_battery_type);
	bat_capacity = asuspec_battery_monitor("capacity",pad_battery_type);
	remaining_cap = asuspec_battery_monitor("remaining_capacity",pad_battery_type);
	//[Brook- Docking charging porting]<<
	if (ret_val < 0)
		return sprintf(buf, "fail to get battery info\n");
	else {
		sprintf(temp_buf, "status = 0x%x\n", bat_status);
		strcpy(buf, temp_buf);
		sprintf(temp_buf, "temperature = %d\n", bat_temp);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "voltage = %d\n", bat_vol);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "current = %d\n", bat_current);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "capacity = %d\n", bat_capacity);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "remaining capacity = %d\n", remaining_cap);
		strcat(buf, temp_buf);

		return strlen(buf);
	}
}
//[Brook- Docking charging porting]>>
static ssize_t asuspec_dock_battery_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int bat_status, bat_temp, bat_vol, bat_current, bat_capacity, remaining_cap;
	int avg_to_empty, avg_to_full, full_cap, design_cap, design_vol, cycle_cnt;
	int ret_val;
	char temp_buf[64];

	bat_status = asuspec_battery_monitor("status", dock_battery_type);
	bat_temp = asuspec_battery_monitor("temperature", dock_battery_type);
	bat_vol = asuspec_battery_monitor("voltage", dock_battery_type);
	bat_current = asuspec_battery_monitor("current", dock_battery_type);
	bat_capacity = asuspec_battery_monitor("capacity", dock_battery_type);
	remaining_cap = asuspec_battery_monitor("remaining_capacity", dock_battery_type);
	avg_to_empty = asuspec_battery_monitor("avg_time_to_empty", dock_battery_type);
	avg_to_full = asuspec_battery_monitor("avg_time_to_full", dock_battery_type);
	full_cap = asuspec_battery_monitor("full_capacity", dock_battery_type);
	design_cap = asuspec_battery_monitor("design_capacity", dock_battery_type);
	design_vol = asuspec_battery_monitor("design_voltage", dock_battery_type);
	cycle_cnt = asuspec_battery_monitor("cycle_count", dock_battery_type);

	if (ret_val < 0)
		return sprintf(buf, "fail to get battery info\n");
	else {
		sprintf(temp_buf, "status = 0x%x\n", bat_status);
		strcpy(buf, temp_buf);
		sprintf(temp_buf, "temperature = %d\n", bat_temp);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "voltage = %d\n", bat_vol);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "current = %d\n", bat_current);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "capacity = %d\n", bat_capacity);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "remaining capacity = %d\n", remaining_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "avg time to empty = %d\n", avg_to_empty);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "avg time to full = %d\n", avg_to_full);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "full capacity = %d\n", full_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "desing capacity = %d\n", design_cap);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "design voltage= %d\n", design_vol);
		strcat(buf, temp_buf);
		sprintf(temp_buf, "cycle count = %d\n", cycle_cnt);
		strcat(buf, temp_buf);
		return strlen(buf);
	}
}
//[Brook- Docking charging porting]<<
static ssize_t asuspec_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;
	int i = 0;
	char temp_buf[64];

	ret_val = asuspec_dockram_read_data(0x0A);
	if (ret_val < 0)
		return sprintf(buf, "fail to get pad ec control-flag info\n");
	else{
		sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
		strcpy(buf, temp_buf);
		for (i = 1; i < 9; i++){
			sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
			strcat(buf, temp_buf);
		}
		return strlen(buf);
	}
}
//[Brook- Docking charging porting]>>
static ssize_t asuspec_dock_control_flag_show(struct device *class,struct device_attribute *attr,char *buf)
{
        int i = 0;
        char temp_buf[64];
        int ret_val = 0;
        ret_val = asuspec_dockram_read_data(0x23);
        if (ret_val < 0)
            return sprintf(buf, "fail to get control-flag info\n");
        else{
            sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
            strcpy(buf, temp_buf);
            for (i = 1; i < 9; i++){
                sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
                strcat(buf, temp_buf);
              }
            return strlen(buf);
         }
        return sprintf(buf, "fail to get control-flag info\n");
}
//[Brook- Docking charging porting]<<
static ssize_t asuspec_send_ec_req_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_send_ec_req();
	return sprintf(buf, "EC_REQ is sent\n");
}


static ssize_t asuspec_charging_led_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret_val = 0;
	if (ec_chip->op_mode == 0){
		asuspec_dockram_read_data(0x0A);
		if (buf[0] == '0'){
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ret_val = asuspec_dockram_write_data(0x0A,9);
			if (ret_val < 0)
				ASUSPEC_NOTICE("Fail to diable led test\n");
			else
				ASUSPEC_NOTICE("Diable led test\n");
		} else if (buf[0] == '1'){
			asuspec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x02;
			ret_val = asuspec_dockram_write_data(0x0A,9);
			if (ret_val < 0)
				ASUSPEC_NOTICE("Fail to enable orange led test\n");
			else
				ASUSPEC_NOTICE("Enable orange led test\n");
		} else if (buf[0] == '2'){
			asuspec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x04;
			ret_val = asuspec_dockram_write_data(0x0A,9);
			if (ret_val < 0)
				ASUSPEC_NOTICE("Fail to enable green led test\n");
			else
				ASUSPEC_NOTICE("Enable green led test\n");
		}
	} else {
		ASUSPEC_NOTICE("Fail to enter led test\n");
	}
	return count;
}

static ssize_t asuspec_led_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;
	asuspec_dockram_read_data(0x0A);
	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x01;
	ret_val = asuspec_dockram_write_data(0x0A,9);
	if (ret_val < 0)
		return sprintf(buf, "Fail to EC LED Blink\n");
	else
		return sprintf(buf, "EC LED Blink\n");
}

static ssize_t asuspec_enter_factory_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_enter_factory_mode();
	return sprintf(buf, "Entering factory mode\n");
}

static ssize_t asuspec_enter_normal_mode_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_enter_normal_mode();
	return sprintf(buf, "Entering normal mode\n");
}

static ssize_t asuspec_switch_hdmi_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_switch_hdmi();
	return sprintf(buf, "Switch hdmi\n");
}

static ssize_t asuspec_win_shutdown_show(struct device *class,struct device_attribute *attr,char *buf)
{
	asuspec_win_shutdown();
	return sprintf(buf, "Win shutdown\n");
}
static ssize_t asuspec_cmd_data_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int buf_len = strlen(buf);
	int data_len = (buf_len -1)/2;
	char chr[2], data_num[data_len];
	int i=0, j=0, idx=0, ret, data_cnt, ret_val;
	chr[2] = '\0';
	u8 cmd;
	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}
	memset(&ec_chip->i2c_dm_data, 0, 32);
	printk("buf_len=%d, data_len=%d \n",buf_len, data_len);
	if(!(buf_len&0x01) || !data_len){
		return -1;
	} else if(buf_len==3){
		reg_addr = (u8) simple_strtoul (buf,NULL,16);
		return EAGAIN;
	}
	for(i=0;i<buf_len-1;i++){
		chr[j] = *(buf+i);
		if(j==1){
			if (i == 1) {
				cmd = (u8) simple_strtoul (chr,NULL,16);
			} else
				data_num[idx++] = (u8) simple_strtoul (chr,NULL,16);
		}
		j++;
		if(j>1){
			j=0;
		}
	}
	data_num[idx] = '\0';
	data_cnt = data_len - 1;

	if(data_cnt > 32) {
		printk("Input data count is over length\n");
		return -1;
	}
	memcpy(&ec_chip->i2c_dm_data[1], data_num, data_cnt);
	ec_chip->i2c_dm_data[0] = data_len - 1;

	for(i=0; i<data_len; i++){
		if (i ==0) printk("I2c cmd=0x%x\n", cmd);
		printk("I2c_dm_data[%d]=0x%x\n",i, ec_chip->i2c_dm_data[i]);
	}
	ret = asuspec_dockram_write_data(cmd, data_len);
	if(ret <0)
		ASUSPEC_NOTICE("Fail to write data\n");
	return count;
}

static ssize_t asuspec_return_data_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i, ret_val;
	char temp_buf[64];

	if (reg_addr != -1) {
		if (ec_chip->ec_in_s3){
			asuspec_send_ec_req();
			msleep(200);
		}
		printk("Smbus read EC command=0x%02x\n", reg_addr);
		ret_val = asuspec_dockram_read_data(reg_addr);
		reg_addr = -1;

		if (ret_val < 0)
			return sprintf(buf, "Fail to read ec data\n");
		else{
			if (ec_chip->i2c_dm_data[0]> 32)
				return sprintf(buf, "EC return data length error\n");
			for (i = 1; i <= ec_chip->i2c_dm_data[0] ; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}
	return 0;
}

static ssize_t asuspec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asuspec_switch_state(struct switch_dev *sdev, char *buf)
{
	if (201) {
		return sprintf(buf, "%s\n", "0");
	}
}

static ssize_t apower_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", APOWER_SDEV_NAME);
}

static ssize_t apower_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->apower_state);
}

static int asuspec_suspend(struct i2c_client *client, pm_message_t mesg){
	printk("asuspec_suspend+\n");
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
	int ret_val;
	if (ec_chip->dock_in && (ec_chip->ec_in_s3 == 0)){
		ret_val = asuspec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		asuspec_dockram_read_data(0x0A);
		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[1] = 0x20;   //Write-1-Clear Flag set i2c_dm_data[1] BIT5 to 1 i2c_dm_data[5] BIT5 will clear to 0.
		ec_chip->i2c_dm_data[5] = 0x22;   //Write-1-Set Flag set i2c_dm_data[5] BIT1 and BIT5 to 1
		if (ec_chip->dec_wakeup){
			ec_chip->i2c_dm_data[5] = 0x80;
		} else {
			ec_chip->i2c_dm_data[1] = 0x80;
		}
		asuspec_dockram_write_data(0x0A,9);
	}
fail_to_access_ec:
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
	//Shouchung add for solving after suspend/resume, the touchpad will
	//late to wake up
	asusdec_tp_enable_function(0);
	ec_chip->tp_enable = 0;
	//Shouchung end
	//[Brook- Docking charging porting]>>
	flush_workqueue(asuspec_wq);
	ec_chip->suspend_state = 1;
	ec_chip->dock_det = 0;
	ec_chip->dock_init = 0;
	ec_chip->init_success = 0;
	ec_chip->touchpad_member = -1;
	//[Brook- Docking charging porting]<<
	ec_chip->ec_in_s3 = 1;
	printk("asuspec_suspend-\n");
	return 0;
}

static int asuspec_resume(struct i2c_client *client){
	printk("asuspec_resume+\n");
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
	if (ec_chip->dock_in && gpio_get_value(asuspec_apwake_gpio)) {
		ec_chip->dock_type = DOCK_UNKNOWN;
		asuspec_reset_dock();
	}
	//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
	//[Brook- Docking charging porting]>>
	ec_chip->suspend_state = 0;
	ec_chip->dock_det = 0;
	ec_chip->init_success = 0;
	ec_chip->ec_in_s3 = 0;
	ec_chip->touchpad_member = -1;
	asusdec_lid_report_function(NULL);
	queue_delayed_work(asuspec_wq, &ec_chip->asusdec_dock_init_work, 0);
	//[Brook- Docking charging porting]<<
	ec_chip->i2c_err_count = 0;
	printk("asuspec_resume-\n");
	return 0;
}


static int asuspec_open(struct inode *inode, struct file *flip){
	ASUSPEC_NOTICE("\n");
	return 0;
}
static int asuspec_release(struct inode *inode, struct file *flip){
	ASUSPEC_NOTICE("\n");
	return 0;
}
static long asuspec_ioctl(struct file *flip,
					unsigned int cmd, unsigned long arg){
	int err = 1;

	if (_IOC_TYPE(cmd) != ASUSPEC_IOC_MAGIC)
	 return -ENOTTY;
	if (_IOC_NR(cmd) > ASUSPEC_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	switch (cmd) {
		case ASUSPEC_POLLING_DATA:
			if (arg == ASUSPEC_IOCTL_HEAVY){
				ASUSPEC_NOTICE("heavy polling\n");
				ec_chip->polling_rate = 80;
				queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if (arg == ASUSPEC_IOCTL_NORMAL){
				ASUSPEC_NOTICE("normal polling\n");
				ec_chip->polling_rate = 10;
				queue_delayed_work(asuspec_wq, &asuspec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if  (arg == ASUSPEC_IOCTL_END){
				ASUSPEC_NOTICE("polling end\n");
		    	cancel_delayed_work_sync(&asuspec_stress_work) ;
			}
			else
				return -ENOTTY;
			break;
		case ASUSPEC_FW_UPDATE:
			ASUSPEC_NOTICE("ASUSPEC_FW_UPDATE\n");
			mutex_lock(&ec_chip->state_change_lock);
			asuspec_send_ec_req();
			msleep(200);
			buff_in_ptr = 0;
			buff_out_ptr = 0;
			h2ec_count = 0;
			memset(host_to_ec_buffer, 0, EC_BUFF_LEN);
			memset(ec_to_host_buffer, 0, EC_BUFF_LEN);
			ec_chip->status = 0;
			ec_chip->op_mode = 1;
			wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			msleep(2400);
			i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
			msleep(1000);
			mutex_unlock(&ec_chip->state_change_lock);
			break;
		case ASUSPEC_INIT:
			ASUSPEC_NOTICE("ASUSPEC_INIT\n");
			msleep(500);
			ec_chip->status = 0;
			ec_chip->op_mode = 0;
			queue_delayed_work(asuspec_wq, &ec_chip->asuspec_init_work, 0);
			switch_set_state(&ec_chip->pad_sdev, !ec_chip->pad_sdev.state);
			msleep(2500);
			break;
		case ASUSDEC_TP_CONTROL:
		       ASUSPEC_NOTICE("ASUSDEC_TP_CONTROL\n");
                     if ((ec_chip->op_mode == 0) && ec_chip->dock_in){
						 //Shouchung modify for touchpad enable/disable function
                         err = asusdec_tp_control(arg);
                         //Shouchung end
                         return err;
                        }
		       else
			    return -ENOTTY;
		case ASUSPEC_FW_DUMMY:
			ASUSPEC_NOTICE("ASUSPEC_FW_DUMMY\n");
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
			break;
		case ASUSPEC_SWITCH_HDMI:
			ASUSPEC_NOTICE("ASUSPEC_SWITCH_HDMI\n", arg);
			asuspec_switch_hdmi();
			break;
		case ASUSPEC_WIN_SHUTDOWN:
			ASUSPEC_NOTICE("ASUSPEC_WIN_SHUTDOWN\n", arg);
			asuspec_win_shutdown();
			break;
		//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
		case ASUSDEC_EC_WAKEUP:
			msleep(500);
			ASUSPEC_NOTICE("ASUSPEC_EC_WAKEUP, arg = %s \n", arg?"ASUSDEC_EC_ON":"ASUSDEC_EC_OFF");
			if (arg == ASUSDEC_EC_OFF){
				ec_chip->dec_wakeup = 0;
				ASUSPEC_NOTICE("Set EC shutdown when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}
			else if (arg == ASUSDEC_EC_ON){
				ec_chip->dec_wakeup = 1;
				ASUSPEC_NOTICE("Keep EC active when PAD in LP0\n");
				return asusdec_set_wakeup_cmd();
			}
			else {
				ASUSPEC_ERR("Unknown argument");
				return -ENOTTY;
			}
		//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
		default: /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return 0;
}

static void asuspec_switch_apower_state(int state){
	ec_chip->apower_state = state;
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
	ec_chip->apower_state = APOWER_IDLE;
	switch_set_state(&ec_chip->apower_sdev, ec_chip->apower_state);
}

static void asuspec_win_shutdown(void){
	int ret_val = 0;
	int i = 0;

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[8] = ec_chip->i2c_dm_data[8] | 0x40;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Win shutdown command fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Win shutdown\n");
			break;
		}
	}
}

static void asuspec_switch_hdmi(void){
	int ret_val = 0;
	int i = 0;

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[8] = ec_chip->i2c_dm_data[8] | 0x01;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Switch hdmi command fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Switching hdmi\n");
			break;
		}
	}

}

static void asuspec_storage_info_update(void){
	int ret_val = 0;
	int i = 0;
	struct kstatfs st_fs;
	unsigned long long block_size;
	unsigned long long f_blocks;
	unsigned long long f_bavail;
	unsigned long long mb;

	ret_val = user_statfs("/data", &st_fs);
	if (ret_val < 0){
		ASUSPEC_ERR("fail to get data partition size\n");
		ec_chip->storage_total = 0;
		ec_chip->storage_avail = 0;
	} else {
		block_size = st_fs.f_bsize;
		f_blocks = st_fs.f_blocks;
		f_bavail = st_fs.f_bavail;
		mb = MB;
		ec_chip->storage_total = block_size * f_blocks / mb;
		ec_chip->storage_avail = block_size * f_bavail / mb;
		ASUSPEC_NOTICE("Storage total size = %ld, available size = %ld\n", ec_chip->storage_total, ec_chip->storage_avail);
	}

	if (ec_chip->ec_in_s3){
		asuspec_send_ec_req();
		msleep(200);
	}

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_storageinfo(0x28);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get PadInfo\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_storage[0] = 8;
	ec_chip->i2c_dm_storage[1] = ec_chip->storage_total & 0xFF;
	ec_chip->i2c_dm_storage[2] = (ec_chip->storage_total >> 8) & 0xFF;
	ec_chip->i2c_dm_storage[3] = ec_chip->storage_avail & 0xFF;;
	ec_chip->i2c_dm_storage[4] = (ec_chip->storage_avail >> 8) & 0xFF;;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_storageinfo(0x28,9);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to write PadInfo\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Write PadInof Total[H][L]: 0x%x, 0x%x\n", ec_chip->i2c_dm_storage[2], ec_chip->i2c_dm_storage[1]);
			ASUSPEC_NOTICE("Write PadInof Avail[H][L]: 0x%x, 0x%x\n", ec_chip->i2c_dm_storage[4], ec_chip->i2c_dm_storage[3]);
			break;
		}
	}
}

static void asuspec_enter_factory_mode(void){

	ASUSPEC_NOTICE("Entering factory mode\n");
	asuspec_dockram_read_data(0x0A);
	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x40;
#if CSC_IMAGE
        ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;
#endif
	asuspec_dockram_write_data(0x0A,9);
}

static void asuspec_enter_normal_mode(void){

	int ret_val = 0;
	int i = 0;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_read_data(0x0A);
		if (ret_val < 0){
			ASUSPEC_ERR("fail to get control flag\n");
			msleep(100);
		}
		else
			break;
	}

	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xBF;

	for ( i = 0; i < 3; i++ ){
		ret_val = asuspec_dockram_write_data(0x0A,9);
		if (ret_val < 0){
			ASUSPEC_ERR("Entering normal mode fail\n");
			msleep(100);
		}
		else {
			ASUSPEC_NOTICE("Entering normal mode\n");
			break;
		}
	}
}

static int BuffDataSize(void)
{
    int in = buff_in_ptr;
    int out = buff_out_ptr;

    if (in >= out){
        return (in - out);
    } else {
        return ((EC_BUFF_LEN - out) + in);
    }
}

static void BuffPush(char data)
{

    if (BuffDataSize() >= (EC_BUFF_LEN -1)){
        ASUSPEC_ERR("Error: EC work-buf overflow \n");
        return;
    }

    ec_to_host_buffer[buff_in_ptr] = data;
    buff_in_ptr++;
    if (buff_in_ptr >= EC_BUFF_LEN){
        buff_in_ptr = 0;
    }
}

static char BuffGet(void)
{
    char c = (char)0;

    if (BuffDataSize() != 0){
        c = (char) ec_to_host_buffer[buff_out_ptr];
        buff_out_ptr++;
         if (buff_out_ptr >= EC_BUFF_LEN){
             buff_out_ptr = 0;
         }
    }
    return c;
}

static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int i = 0;
    int ret;
    char tmp_buf[EC_BUFF_LEN];
	static int f_counter = 0;
	static int total_buf = 0;

	mutex_lock(&ec_chip->lock);
	mutex_unlock(&ec_chip->lock);

    while ((BuffDataSize() > 0) && count)
    {
        tmp_buf[i] = BuffGet();
		ASUSPEC_INFO("tmp_buf[%d] = 0x%x, total_buf = %d\n", i, tmp_buf[i], total_buf);
        count--;
        i++;
		f_counter = 0;
		total_buf++;
    }

    ret = copy_to_user(buf, tmp_buf, i);
    if (ret == 0)
    {
        ret = i;
    }

    return ret;
}

static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int err;
    int i;

    if (h2ec_count > 0)
    {                   /* There is still data in the buffer that */
        return -EBUSY;  /* was not sent to the EC */
    }
    if (count > EC_BUFF_LEN)
    {
        return -EINVAL; /* data size is too big */
    }

    err = copy_from_user(host_to_ec_buffer, buf, count);
    if (err)
    {
        ASUSPEC_ERR("ec_write copy error\n");
        return err;
    }

    h2ec_count = count;
    for (i = 0; i < count ; i++){
		i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
    }
    h2ec_count = 0;
    return count;

}

static int __init asuspec_init(void)
{
	int err_code = 0;

	printk(KERN_INFO "%s+ #####\n", __func__);

	if (asuspec_major) {
		asuspec_dev = MKDEV(asuspec_major, asuspec_minor);
		err_code = register_chrdev_region(asuspec_dev, 1, "asuspec");
	} else {
		err_code = alloc_chrdev_region(&asuspec_dev, asuspec_minor, 1,"asuspec");
		asuspec_major = MAJOR(asuspec_dev);
	}

	ASUSPEC_NOTICE("cdev_alloc\n") ;
	asuspec_cdev = cdev_alloc() ;
	asuspec_cdev->owner = THIS_MODULE ;
	asuspec_cdev->ops = &asuspec_fops ;

	err_code=i2c_add_driver(&asuspec_driver);
	if(err_code){
		ASUSPEC_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	asuspec_class = class_create(THIS_MODULE, "asuspec");
	if(asuspec_class <= 0){
		ASUSPEC_ERR("asuspec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	asuspec_device = device_create(asuspec_class, NULL, MKDEV(asuspec_major, asuspec_minor), NULL, "asuspec" );
	if(asuspec_device <= 0){
		ASUSPEC_ERR("asuspec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	ASUSPEC_INFO("return value %d\n", err_code) ;
	printk(KERN_INFO "%s- #####\n", __func__);

	return 0;

device_create_fail :
	class_destroy(asuspec_class) ;
class_create_fail :
	i2c_del_driver(&asuspec_driver);
i2c_add_driver_fail :
	printk(KERN_INFO "%s- #####\n", __func__);
	return err_code;

}

static void __exit asuspec_exit(void)
{
	device_destroy(asuspec_class,MKDEV(asuspec_major, asuspec_minor)) ;
	class_destroy(asuspec_class) ;
	i2c_del_driver(&asuspec_driver);
	unregister_chrdev_region(asuspec_dev, 1);
	switch_dev_unregister(&ec_chip->pad_sdev);
	switch_dev_unregister(&ec_chip->apower_sdev);
	switch_dev_unregister(&ec_chip->dock_sdev);		
}

//[Brook- Docking charging porting]>>
int asusAudiodec_i2c_write_data(char *data, int length)
{
	int ret;
	if ((ec_chip->dock_in == 0)||
		((ec_chip->dock_type != AUDIO_DOCK)&&(ec_chip->dock_type != AUDIO_STAND))){
		return -1;
	}

	ret = i2c_master_send(ec_chip->client, data, length);
	if(ret<0)
		ASUSPEC_ERR("Fail to write data, errno %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(asusAudiodec_i2c_write_data);

int asusAudiodec_i2c_read_data(char *data, int length)
{
	int ret;
	if ((ec_chip->dock_in == 0)||
		((ec_chip->dock_type != AUDIO_DOCK)&&(ec_chip->dock_type != AUDIO_STAND))){
		return -1;
	}

	ret = i2c_master_send(ec_chip->client, data, 1);
	if(ret<0){
		ASUSPEC_ERR("Fail to send data, errno %d\n", ret);
		return ret;
	}

	ret = i2c_master_recv(ec_chip->client, data, length);
	if(ret<0)
		ASUSPEC_ERR("Fail to receive data, errno %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(asusAudiodec_i2c_read_data);

static ssize_t asusdec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->dec_version);
}

static ssize_t asusdec_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", switch_value[ec_chip->dock_type]);
}
//[Brook- Docking charging porting]<<

//[Panda-Hall sensor]>>
static int asusdec_lid_input_device_create(struct i2c_client *client){
	int err = 0;

	ec_chip->lid_indev = input_allocate_device();
	if (!ec_chip->lid_indev) {
		ASUSPEC_ERR("lid_indev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->lid_indev->name = "lid_input";
	ec_chip->lid_indev->phys = "/dev/input/lid_indev";
	ec_chip->lid_indev->dev.parent = &client->dev;

	asusdec_lid_set_input_params(ec_chip->lid_indev);
	err = input_register_device(ec_chip->lid_indev);
	if (err) {
		ASUSPEC_ERR("lid_indev registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->lid_indev);
	ec_chip->lid_indev = NULL;
exit:
	return err;

}

static void asusdec_lid_set_input_params(struct input_dev *dev)
{
	set_bit(EV_SW, dev->evbit);
	set_bit(SW_LID, dev->swbit);
}

//[Panda-Hall sensor]<<

module_init(asuspec_init);
module_exit(asuspec_exit);
