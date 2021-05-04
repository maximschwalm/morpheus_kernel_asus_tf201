/*
 * drivers/power/pad_battery.c
 *
 * Gas Gauge driver for TI's BQ20Z45
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"
#include "../../arch/arm/mach-tegra/wakeups-t3.h"
#include <mach/board-cardhu-misc.h>
#include <linux/delay.h>
#define SMBUS_RETRY                                     (3)
#define dock_in_gpio	                 TEGRA_GPIO_PU4
#define BATTERY_POLLING_RATE                    (120)
#define DELAY_FOR_CORRECT_CHARGER_STATUS	(4)
#define DELAY_FOR_CORRECT_CHARGER_STATUS_P1801	(6)
#define TEMP_KELVIN_TO_CELCIUS                             (2731)
#define MAXIMAL_VALID_BATTERY_TEMP                             (200)
#define USB_NO_Cable 0
#define USB_DETECT_CABLE 1 
#define USB_SHIFT 0
#define AC_SHIFT 1 
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)
unsigned battery_cable_status=0;
unsigned battery_docking_status=0;
unsigned battery_driver_ready=0;
static int ac_on ;
static int usb_on ;
extern int asuspec_battery_monitor(char *cmd, bool pad);
static unsigned int 	battery_current;
static unsigned int  battery_remaining_capacity;
module_param(battery_current, uint, 0644);
module_param(battery_remaining_capacity, uint, 0644);
//[Brook- Docking charging porting]>>
static int dock_battery_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val);
extern unsigned int Check_Charging_Cable_Type();
static int dock_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val, int battery_type);
static int dock_get_capacity(union power_supply_propval *val);
static int dock_get_status(union power_supply_propval *val);
extern bool Check_Charging_status(bool pad);
extern bool Check_Battery_status(void);
extern bool Check_BAT_LL_Enabled(void);
extern bool asuspec_check_dock_in_control_flag(void);
typedef enum {
	dock_battery_type = 0,
	pad_battery_type,
}battery_type;
//[Brook- Docking charging porting]<<
enum {
	REG_MANUFACTURER_DATA,  	
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};    
typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_Docking_AC,
	Charger_Type_Docking_Battery,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS	0x0006
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_CHARGING 		    0x40
#define BATTERY_FULL_CHARGED		0x20
#define BATTERY_FULL_DISCHARGED 	0x10
#define PAD_BAT_DATA(_psp, _addr, _min_value, _max_value)	\
	{							\
		.psp = POWER_SUPPLY_PROP_##_psp,		\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
	}

static struct pad_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} pad_data[] = {
	[REG_MANUFACTURER_DATA] = PAD_BAT_DATA(PRESENT, 0, 0, 65535),
	[REG_STATE_OF_HEALTH]   = PAD_BAT_DATA(HEALTH, 0, 0, 65535),
	[REG_TEMPERATURE]       = PAD_BAT_DATA(TEMP, 0x08, 0, 65535),
	[REG_VOLTAGE]           = PAD_BAT_DATA(VOLTAGE_NOW, 0x09, 0, 20000),
	[REG_CURRENT]           = PAD_BAT_DATA(CURRENT_NOW, 0x0A, -32768, 32767),
	[REG_TIME_TO_EMPTY]     = PAD_BAT_DATA(TIME_TO_EMPTY_AVG, 0x12, 0, 65535),
	[REG_TIME_TO_FULL]      = PAD_BAT_DATA(TIME_TO_FULL_AVG, 0x13, 0, 65535),
	[REG_STATUS]            = PAD_BAT_DATA(STATUS, 0x16, 0, 65535),
	[REG_CAPACITY]          = PAD_BAT_DATA(CAPACITY, 0x0d, 0, 100),//battery HW request
	[REG_SERIAL_NUMBER]     = PAD_BAT_DATA(SERIAL_NUMBER, 0x1C, 0, 65535),
};

static struct pad_device_info {
	struct i2c_client	*client;
        struct delayed_work battery_stress_test;
	struct delayed_work thermal_stress_test;
	struct delayed_work pmu_stress_test;
	struct delayed_work status_poll_work;
	struct delayed_work dock_status_poll_work;
	struct delayed_work low_low_bat_work;
	int smbus_status;
	int battery_present;
	int low_battery_present;
	unsigned int old_capacity;
	unsigned int cap_err;
	unsigned int old_temperature;
	unsigned int temp_err;
	int gpio_battery_detect;
	int gpio_low_battery_detect;
	int irq_low_battery_detect;
	int irq_battery_detect;
	bool dock_charger_pad_interrupt_enabled;
	spinlock_t		lock;
	struct miscdevice battery_misc;
	unsigned int prj_id;
	struct wake_lock low_battery_wake_lock;
	struct wake_lock cable_event_wake_lock;
	int bat_status;
	int bat_temp;
	int bat_vol;
	int bat_current;
	int bat_capacity;
//[Brook- Docking charging porting]>>
	int gpio_dock_detect;
	int dock_bat_status;
	bool dock_in;
	int dock_bat_capacity;
	unsigned int dock_old_capacity;
	unsigned int dock_cap_err;
//[Brook- Docking charging porting]<<
} *pad_device;

static enum power_supply_property pad_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,

};

unsigned (*get_usb_cable_status_cb) (void);

void check_cabe_type(void)
{
	if(battery_cable_status == USB_AC_Adapter){
		ac_on = 1 ;
		usb_on = 0;
	}else if(battery_cable_status  == USB_Cable){
		usb_on = 1;
		ac_on = 0;
	}else if(battery_cable_status  == USB_NO_Cable){
		ac_on = 0 ;
		usb_on = 0;
	}
}
static int pad_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int pad_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val, int battery_type);

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret=0;
	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
		   if(psy->type == POWER_SUPPLY_TYPE_MAINS && ac_on){
		   	val->intval =  1;
		   }
		   else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on){
			val->intval =  1;
		   }
		   else if (psy->type == POWER_SUPPLY_TYPE_DOCK_AC && ac_on && pad_device->dock_in){
			val->intval =  1;
		   }
		   else{ 
		   	val->intval = 0;
		   }
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

static char *supply_list[] = {
	"battery",
	"ac",
	"usb",
};

//[Brook- Docking charging porting]>>
static enum power_supply_property dock_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};
//[Brook- Docking charging porting]<<

static struct power_supply pad_supply[] = {
	{
		.name             = "battery",
		.type             = POWER_SUPPLY_TYPE_BATTERY,
		.properties       = pad_properties,
		.num_properties   = ARRAY_SIZE(pad_properties),
		.get_property     = pad_get_property,
       },
	{
		.name             = "ac",
		.type             = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to      = supply_list,
		.num_supplicants  = ARRAY_SIZE(supply_list),
		.properties       = power_properties,
		.num_properties   = ARRAY_SIZE(power_properties),
		.get_property     = power_get_property,
	},
	{
		.name             = "usb",
		.type             = POWER_SUPPLY_TYPE_USB,
		.supplied_to      = supply_list,
		.num_supplicants  = ARRAY_SIZE(supply_list),
		.properties       = power_properties,
		.num_properties   = ARRAY_SIZE(power_properties),
		.get_property     = power_get_property,
	},
	{
		.name             = "docking_ac",
		.type             = POWER_SUPPLY_TYPE_DOCK_AC,
		.properties       = power_properties,
		.num_properties   = ARRAY_SIZE(power_properties),
		.get_property     = power_get_property,
	},
	{
		.name             = "dock_battery",
		.type             = POWER_SUPPLY_TYPE_DOCK_BATTERY,
		.properties       = dock_battery_properties,
		.num_properties   = ARRAY_SIZE(dock_battery_properties),
		.get_property	  = dock_battery_get_property,
	},
};

void  register_usb_cable_status_cb(unsigned  (*fn) (void))
{
	if (!get_usb_cable_status_cb)
		get_usb_cable_status_cb = fn;
}

unsigned  get_usb_cable_status(void)
{
	if (!get_usb_cable_status_cb) {
		printk(KERN_ERR "Battery: get_usb_cable_status_cb is NULL");
		return 0;
	}
	return get_usb_cable_status_cb();
}

static ssize_t show_battery_charger_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	if(pad_device->smbus_status < 0)
	{
		return sprintf(buf, "%d\n", 0);
	}
	else
	{
		return sprintf(buf, "%d\n", 1);
	}
}

static DEVICE_ATTR(battery_charger, S_IWUSR | S_IRUGO, show_battery_charger_status,NULL);

static struct attribute *battery_charger_attributes[] = {

	&dev_attr_battery_charger.attr,
	NULL
};

static const struct attribute_group battery_charger_group = {
	.attrs = battery_charger_attributes,
};

int pad_smbus_read_data(int reg_offset,int byte)
{
     s32 ret=-EINVAL;
     int count=0; 
     do{
	    if(byte)
              ret=i2c_smbus_read_byte_data(pad_device->client,pad_data[reg_offset].addr);
	    else
	    ret=i2c_smbus_read_word_data(pad_device->client,pad_data[reg_offset].addr);

     	}while((ret<0)&& (++count<=SMBUS_RETRY));
      return ret;
}

int pad_smbus_write_data(int reg_offset,int byte, unsigned int value)
{
     s32 ret=-EINVAL;
     int count=0; 

     do{
	    if(byte){
                  ret=i2c_smbus_write_byte_data(pad_device->client,pad_data[reg_offset].addr,value&0xFF);
	    	}
	    else{
	          ret=i2c_smbus_write_word_data(pad_device->client,pad_data[reg_offset].addr,value&0xFFFF);
	    	}

     	}while((ret<0)&& (++count<=SMBUS_RETRY));
      return ret;
}
struct workqueue_struct *battery_work_queue=NULL;
struct workqueue_struct *dock_battery_work_queue=NULL;
static ssize_t show_battery_smbus_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int status=!pad_device->smbus_status;
	return sprintf(buf, "%d\n", status);
}
static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO, show_battery_smbus_status,NULL);

static struct attribute *battery_smbus_attributes[] = {
	&dev_attr_battery_smbus.attr,
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};
static void battery_status_poll(struct work_struct *work)
{
       struct pad_device_info *battery_device =container_of(work, struct pad_device_info,status_poll_work.work);
	//printk("battery_status_poll\n");
	//kobject_uevent(&pad_supply.dev->kobj, KOBJ_CHANGE);
	if(!battery_driver_ready)
		printk("battery_status_poll:driver not ready\n");
	power_supply_changed(&pad_supply[Charger_Type_Battery]);
	/* Schedule next poll */
	pad_device->battery_present = Check_Battery_status();
	printk("battery_status_poll %u %u \n",BATTERY_POLLING_RATE,pad_device->battery_present);
	if(pad_device->battery_present)
		queue_delayed_work(battery_work_queue, &battery_device->status_poll_work,BATTERY_POLLING_RATE*HZ);
}

#if 0
static irqreturn_t battery_detect_isr(int irq, void *dev_id)
{
	pad_device->battery_present =Check_Battery_status();
	printk("battery_detect_isr battery %s \n",pad_device->battery_present?"instered":"removed" );
	if( pad_device->battery_present)
		queue_delayed_work(battery_work_queue, &pad_device->status_poll_work, BATTERY_POLLING_RATE*HZ);
	return IRQ_HANDLED;
}
#endif
static void low_low_battery_check(struct work_struct *work)
{
	cancel_delayed_work(&pad_device->status_poll_work);
	queue_delayed_work(battery_work_queue,&pad_device->status_poll_work,0.1*HZ);
	msleep(2000);
	//enable_irq(pad_device->irq_low_battery_detect );
}

void low_battery_detect_isr()
{
	//disable_irq_nosync(pad_device->irq_low_battery_detect );
	pad_device->low_battery_present=Check_BAT_LL_Enabled();
	printk("low_battery_detect_isr is %x\n",pad_device->low_battery_present?"enable":"disable");
	wake_lock_timeout(&pad_device->low_battery_wake_lock, 10*HZ);
	queue_delayed_work(battery_work_queue,&pad_device->low_low_bat_work,0.1*HZ);
	return 0;
}
#if 0
void setup_detect_irq(void )
{
       s32 ret=0;
	
	pad_device->battery_present=0;
	pad_device->low_battery_present=0;
	ret = gpio_request(pad_device->gpio_battery_detect, "battery_detect");
	if (ret < 0) {
		printk("request battery_detect gpio failed\n");
		pad_device->gpio_battery_detect = -1;
		goto setup_low_bat_irq;
	}
	pad_device->irq_battery_detect = gpio_to_irq(pad_device->gpio_battery_detect);
	if (pad_device->irq_battery_detect < 0) {
		printk("invalid battery_detect GPIO\n");
		pad_device->gpio_battery_detect = -1;
		pad_device->irq_battery_detect = -1;
		goto setup_low_bat_irq;
	}
	ret = gpio_direction_input(pad_device->gpio_battery_detect);
	if (ret < 0) {
			printk("failed to configure GPIO\n");
			gpio_free(pad_device->gpio_battery_detect);
			pad_device->gpio_battery_detect= -1;
			goto setup_low_bat_irq;
	}
	ret = request_irq(pad_device->irq_battery_detect , battery_detect_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"pad-battery (detect)", NULL);
	if (ret < 0) {
		printk("failed to request  battery_detect irq\n");
	}
	pad_device->battery_present=!(gpio_get_value(pad_device->gpio_battery_detect));
	printk("setup_irq: battery_present =%x   \n",pad_device->battery_present);
setup_low_bat_irq:
 	
	return;
}
void setup_low_battery_irq(void )
{
       s32 ret=0;

	pad_device->gpio_low_battery_detect=GPIOPIN_LOW_BATTERY_DETECT;
       ret = gpio_request( pad_device->gpio_low_battery_detect, "low_battery_detect");
	if (ret < 0) {
		printk("request low_battery_detect gpio failed\n");
		 pad_device->gpio_low_battery_detect = -1;
		goto exit;
	}

	 pad_device->irq_low_battery_detect = gpio_to_irq(pad_device->gpio_low_battery_detect);
	if (pad_device->irq_low_battery_detect< 0) {
		printk("invalid low_battery_detect gpio\n");
		pad_device->gpio_low_battery_detect = -1;
		pad_device->irq_low_battery_detect = -1;
		goto exit;
	}

	ret = gpio_direction_input(pad_device->gpio_low_battery_detect);
	if (ret < 0) {
			printk("failed to configure low_battery_detect  gpio\n");
			gpio_free(pad_device->gpio_battery_detect);
			pad_device->gpio_low_battery_detect= -1;
			goto exit;
	}
	ret = request_irq(pad_device->irq_low_battery_detect , low_battery_detect_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"bq20z45-battery (low battery)", NULL);
       pad_device->low_battery_present=gpio_get_value(pad_device->gpio_low_battery_detect);
	if (ret < 0) {
            printk("failed to request  low_battery_detect irq\n");
	}
	enable_irq_wake(pad_device->irq_low_battery_detect );
	printk("setup_low_battery_irq:low_battery_present=%x \n",pad_device->low_battery_present);
exit:
	return;
}
#endif
void cable_status_callback(unsigned usb_cable_state){
	pad_device->dock_in =!(gpio_get_value(pad_device->gpio_dock_detect));
	battery_cable_status = usb_cable_state;
	check_cabe_type();
	if (battery_cable_status == USB_Cable){
		power_supply_changed(&pad_supply[Charger_Type_USB]);
	}else if (battery_cable_status == USB_AC_Adapter){
		power_supply_changed(&pad_supply[Charger_Type_AC]);
		if(pad_device->dock_in)
			power_supply_changed(&pad_supply[Charger_Type_Docking_AC]);
	}else if(battery_cable_status == USB_NO_Cable){
		power_supply_changed(&pad_supply[Charger_Type_Battery]);	
	}
	return 0;
}


void battery_callback(unsigned usb_cable_state)
{
	printk("[battery_callback] usb_cable_state =%x\n",usb_cable_state ) ;
	if(!battery_driver_ready){
		printk("battery_callback battery driver not ready\n") ;
		return 0;
	}
	cable_status_callback(usb_cable_state);
	cancel_delayed_work(&pad_device->status_poll_work);
	queue_delayed_work(battery_work_queue, &pad_device->status_poll_work,0.3*HZ);
}
//[Brook- Docking charging porting]>>
static void dock_battery_status_poll(struct work_struct *work)
{
	struct pad_device_info *battery_device =container_of(work, struct pad_device_info,dock_status_poll_work.work);
	if(!battery_driver_ready)
		printk("[dock_battery_status_poll] driver not ready\n");
	printk("[dock_battery_status_poll] %u \n",BATTERY_POLLING_RATE);
	power_supply_changed(&pad_supply[Charger_Type_Docking_Battery]);
	/* Schedule next poll */
	queue_delayed_work(dock_battery_work_queue, &battery_device->dock_status_poll_work,BATTERY_POLLING_RATE*HZ);
}

int docking_callback(unsigned usb_cable_state)
{
	printk("[docking_callback] dock %s , usb_cable_state = %d \n", pad_device->dock_in?"inserted":"removed", usb_cable_state);
	if(!battery_driver_ready){
		printk("[docking_callback] battery driver not ready\n") ;
		return 0;
	}
	cable_status_callback(usb_cable_state);
	cancel_delayed_work(&pad_device->dock_status_poll_work);
	queue_delayed_work(dock_battery_work_queue, &pad_device->dock_status_poll_work,0.3*HZ);
	return 0;
}
EXPORT_SYMBOL(docking_callback);
//[Brook- Docking charging porting]<<
static int pad_get_health(enum power_supply_property psp,
	union power_supply_propval *val)
{	
	if (psp == POWER_SUPPLY_PROP_PRESENT) {
		val->intval = pad_device->battery_present;
	}		
	else if (psp == POWER_SUPPLY_PROP_HEALTH) {
		if( pad_device->battery_present)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
	}
	return 0;
}
static int pad_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val, int battery_type)
{
	s32 ret;
	int smb_retry=0;
	if(pad_device->battery_present){
		do{
			if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW){
				pad_device->smbus_status=pad_device->bat_vol=asuspec_battery_monitor("voltage",pad_battery_type);
				battery_current=asuspec_battery_monitor("current",pad_battery_type);
				battery_remaining_capacity=asuspec_battery_monitor("remaining_capacity",pad_battery_type);
			}
			else if (psp == POWER_SUPPLY_PROP_STATUS)
				pad_device->smbus_status=pad_device->bat_status=asuspec_battery_monitor("status",pad_battery_type);
			else  if (psp == POWER_SUPPLY_PROP_TEMP)
				pad_device->smbus_status=pad_device->bat_temp=asuspec_battery_monitor("temperature",pad_battery_type);
		}while((pad_device->smbus_status < 0) && ( ++smb_retry <= SMBUS_RETRY));
	}else{
		pad_device->smbus_status=-1;
		printk("pad_get_psp: pad_device->battery_present=%u\n",pad_device->battery_present);
	}
	if (pad_device->smbus_status < 0) {
		dev_err(&pad_device->client->dev,"%s: i2c read for %d failed\n", __func__, reg_offset);
		if(psp == POWER_SUPPLY_PROP_TEMP && (++pad_device->temp_err<=3) &&(pad_device->old_temperature!=0xFF)){
			val->intval=pad_device->old_temperature;
			printk("read battery's tempurate fail use old temperature=%u pad_device->temp_err=%u\n",val->intval,pad_device->temp_err);
			return 0;
		}
		else
			return -EINVAL;
	}
	if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
		val->intval=pad_device->bat_vol;
		printk("pad_get_psp voltage_now =%u\n",val->intval );//4
	}
	if (psp == POWER_SUPPLY_PROP_STATUS) {
		ret=pad_device->bat_status;
		static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};
		/* mask the upper byte and then find the actual status */
		if (Check_Charging_status(pad_battery_type) && (ac_on||pad_device->dock_in) ){/*DSG*/
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			if (pad_device->old_capacity==100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		else if (pad_device->old_capacity==100 && ac_on)//fc
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else 
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
	    	printk("pad_get_psp,  val->intval = %s, ret = %x, ac_on = %x, Check_Charging_status = %d \n" ,status_text[val->intval],ret,ac_on,Check_Charging_status(pad_battery_type));//4
	}else if (psp == POWER_SUPPLY_PROP_TEMP) {
		ret=pad_device->bat_temp;
		ret -=TEMP_KELVIN_TO_CELCIUS;

		if ( (ret/10) >= MAXIMAL_VALID_BATTERY_TEMP && (pad_device->temp_err ==0)){
			ret=300;
			printk("[Warning] pad_get_psp  batttery temp=%u, set to 30.0\n",ret);
			WARN_ON(1);
			pad_device->temp_err++;
		}
		else
			pad_device->temp_err=0;

		pad_device->old_temperature=val->intval = ret;
		printk("pad_get_psp  batttery temperature=%u\n",val->intval );
	}
	return 0;
}
static int pad_get_capacity(union power_supply_propval *val, int battery_type)
{
	s32 ret;
	s32 temp_capacity;
	int smb_retry=0;
	if(pad_device->battery_present){
		do{
			pad_device->smbus_status=pad_device->bat_capacity=asuspec_battery_monitor("capacity",pad_battery_type);
		}while((pad_device->smbus_status < 0) && ( ++smb_retry <= SMBUS_RETRY));

	}else{
			pad_device->smbus_status=-1;
			printk("pad_get_capacity: pad_device->battery_present=%u\n",pad_device->battery_present);
	}
	if (pad_device->smbus_status < 0) {
		dev_err(&pad_device->client->dev, "%s: i2c read for %d "
			"failed pad_device->cap_err=%u\n", __func__, REG_CAPACITY,pad_device->cap_err);
		if(pad_device->cap_err>5 || (pad_device->old_capacity==0xFF))
		return -EINVAL;
		else{
			val->intval=pad_device->old_capacity;
			pad_device->cap_err++;
			printk("read capacity fail, use old capacity=%u pad_device->cap_err=%u\n",val->intval,pad_device->cap_err);
			return 0;
		}
	}
	ret=pad_device->bat_capacity;
        /* pad spec says that this can be >100 %
         * even if max value is 100 % */

	temp_capacity = ((ret >= 100) ? 100 : ret);

	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);
	val->intval=temp_capacity;


	pad_device->old_capacity=val->intval;
	pad_device->cap_err=0;
	printk("pad_get_capacity val->intval=%u ret=%u\n",val->intval,ret);
	return 0;
}
static int pad_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	u8 count;
	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (pad_get_health(psp, val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (pad_get_capacity(val, pad_battery_type))
				goto error;
			break;

		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_TEMP:
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		case POWER_SUPPLY_PROP_SERIAL_NUMBER:
			for (count = 0; count < REG_MAX; count++) {
				if (psp == pad_data[count].psp)
					break;
			}

			if (pad_get_psp(count, psp, val, pad_battery_type))
				return -EINVAL;//return -EINVAL;
			break;

		default:
			dev_err(&pad_device->client->dev,
				"%s: INVALID property psp=%u\n", __func__,psp);
			return -EINVAL;
	}
  
	return 0;

	error:
		
	return -EINVAL;	
}
#include "stress_test.c"
void config_thermal_power(void)
{
	int ret;

	ret = gpio_request(TEGRA_GPIO_PU3, "thermal_power_u3");
	if (ret < 0)
		 pr_err("%s: gpio_request failed for gpio %s\n",__func__, "TEGRA_GPIO_PU3");
	gpio_direction_output(TEGRA_GPIO_PU3, 1);
	pr_info("gpio %d set to %d\n",TEGRA_GPIO_PU3, gpio_get_value(TEGRA_GPIO_PU3));
}
static int pad_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc;
	int i=0;
	printk("pad_probe: client->addr=%x \n",client->addr);
	pad_device = kzalloc(sizeof(*pad_device), GFP_KERNEL);
	if (!pad_device) {
		return -ENOMEM;
	}
	memset(pad_device, 0, sizeof(*pad_device));
	pad_device->client = client;
	i2c_set_clientdata(client, pad_device);
	pad_device->smbus_status=0;
	pad_device->cap_err=0;
	pad_device->temp_err=0;
	pad_device->old_capacity=0xFF;
	pad_device->old_temperature=0xFF;
	//[Brook- Docking charging porting]>>
	pad_device->dock_cap_err=0;
	pad_device->dock_old_capacity=0xFF;
	//[Brook- Docking charging porting]<<
	pad_device->low_battery_present=Check_BAT_LL_Enabled();
	pad_device->battery_present = Check_Battery_status();
	pad_device->gpio_dock_detect= dock_in_gpio;
	for (i = 0; i < ARRAY_SIZE(pad_supply); i++) {
		rc = power_supply_register(&client->dev, &pad_supply[i]);
		if (rc) {
			printk(KERN_ERR "Failed to register power supply for pad \n");
			while (i--)
				power_supply_unregister(&pad_supply[i]);
			kfree(pad_device);
			return rc;
		}
	}
	dev_info(&pad_device->client->dev,"%s: battery driver registered\n", client->name);
	spin_lock_init(&pad_device->lock);
	INIT_DELAYED_WORK(&pad_device->status_poll_work, battery_status_poll) ;
	INIT_DELAYED_WORK(&pad_device->dock_status_poll_work, dock_battery_status_poll) ;
	INIT_DELAYED_WORK(&pad_device->battery_stress_test,  battery_strees_test) ;
	INIT_DELAYED_WORK(&pad_device->low_low_bat_work , low_low_battery_check) ;
	battery_work_queue = create_singlethread_workqueue("battery_workqueue");
	dock_battery_work_queue = create_singlethread_workqueue("dock_battery_workqueue");
	/* Register sysfs hooks */
	if (sysfs_create_group(&client->dev.kobj, &battery_smbus_group )) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
	}

	/* Register sysfs */
	if(sysfs_create_group(&client->dev.kobj, &battery_charger_group))
	{
		dev_err(&client->dev, "pad_battery_probe: unable to create battery_group sysfs\n");
	}
	battery_cable_status = Check_Charging_Cable_Type();
	cancel_delayed_work(&pad_device->status_poll_work);
	cancel_delayed_work(&pad_device->dock_status_poll_work);
	//setup_detect_irq();
	//setup_low_battery_irq();
	pad_device->battery_misc.minor	= MISC_DYNAMIC_MINOR;
	pad_device->battery_misc.name	= "battery";
	pad_device->battery_misc.fops  	= &battery_fops;
	rc=misc_register(&pad_device->battery_misc);
	printk(KERN_INFO "battery register misc device for I2C stress test rc=%x\n", rc);

	wake_lock_init(&pad_device->low_battery_wake_lock, WAKE_LOCK_SUSPEND, "low_battery_detection");
	wake_lock_init(&pad_device->cable_event_wake_lock, WAKE_LOCK_SUSPEND, "battery_cable_event");
	battery_driver_ready=1;
	if(pad_device->battery_present)
		queue_delayed_work(battery_work_queue, &pad_device->status_poll_work,15*HZ);
	if(pad_device->dock_in)
		queue_delayed_work(dock_battery_work_queue, &pad_device->dock_status_poll_work,15*HZ);
	return 0;
}

//[Brook- Docking charging porting]>>
static int dock_get_capacity(union power_supply_propval *val){
	s32 ret;
	s32 temp_capacity;
	int smb_retry=0;
	if(!(gpio_get_value(pad_device->gpio_dock_detect))){
		do{
			pad_device->smbus_status=pad_device->dock_bat_capacity=asuspec_battery_monitor("capacity",dock_battery_type);
		}while((pad_device->smbus_status < 0) && ( ++smb_retry <= SMBUS_RETRY));

	}else{
		return 0;
	}
	if (pad_device->smbus_status < 0) {
		dev_err(&pad_device->client->dev, "%s: i2c read for %d "
			"failed pad_device->dock_cap_err=%u\n", __func__, REG_CAPACITY,pad_device->dock_cap_err);
		if(pad_device->dock_cap_err>5 || (pad_device->dock_old_capacity==0xFF))
			return -EINVAL;
		else{
			val->intval=pad_device->dock_old_capacity;
			pad_device->dock_cap_err++;
			printk("read capacity fail, use old capacity=%u pad_device->dock_cap_err=%u\n",val->intval,pad_device->dock_cap_err);
			return 0;
		}
	}
	ret=pad_device->dock_bat_capacity;
	/* pad spec says that this can be >100 %
	* even if max value is 100 % */

	temp_capacity = ((ret >= 100) ? 100 : ret);

	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	/* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);
	val->intval=temp_capacity;


	pad_device->dock_old_capacity=val->intval;
	pad_device->dock_cap_err=0;
	printk("dock_get_capacity val->intval=%u ret=%u\n",val->intval,ret);
	return 0;
}

static int dock_get_status(union power_supply_propval *val){
	int ret_val = 0;
	val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
	if (ret_val < 0){
		return -1;
	}
	else {
		if(gpio_get_value(pad_device->gpio_dock_detect)){
			pr_debug("dock_get_status: gpio_dock_detect Pin = %u, dock not exit! \n",gpio_get_value(pad_device->gpio_dock_detect));
			return 0;
		}
		//[Brook- Fix dock battery show charging icon when full]>>
		pad_device->dock_bat_status=asuspec_battery_monitor("status",dock_battery_type);
		static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};
		if (Check_Charging_status(dock_battery_type) && ac_on){
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			if (pad_device->dock_old_capacity==100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		else if (pad_device->dock_old_capacity==100 && ac_on)
	        	val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else 
	        	val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		printk("[dock_get_status] val->intval = %s, pad_device->dock_bat_status = %x, ac_on = %x, Check_Charging_status= %x \n" ,status_text[val->intval],pad_device->dock_bat_status,ac_on,Check_Charging_status(dock_battery_type));
		//[Brook- Fix dock battery show charging icon when full]<<
		return 0;
	}
	return -1;

}
static int dock_battery_get_property(struct power_supply *psy,enum power_supply_property psp,union power_supply_propval *val)
{
	u8 count;
	switch (psp) {
		case POWER_SUPPLY_PROP_CAPACITY:
			if(dock_get_capacity(val) < 0)
				goto error;
			break;
		case POWER_SUPPLY_PROP_STATUS:
			if(dock_get_status(val) < 0)
				goto error;
			break;
		default:
			return -EINVAL;
	}
	return 0;

error:
	return -EINVAL;
}

//[Brook- Docking charging porting]<<

static int pad_remove(struct i2c_client *client)
{
	struct pad_device_info *pad_device;
       int i=0;
	pad_device = i2c_get_clientdata(client);
	for (i = 0; i < ARRAY_SIZE(pad_supply); i++) {
		power_supply_unregister(&pad_supply[i]);
	}
	if (pad_device) {
		wake_lock_destroy(&pad_device->low_battery_wake_lock);
		kfree(pad_device);
		pad_device = NULL;
	}

	return 0;
}

#if defined (CONFIG_PM)
static int pad_suspend(struct i2c_client *client,
	pm_message_t state)
{

	cancel_delayed_work_sync(&pad_device->status_poll_work);
	flush_workqueue(battery_work_queue);
	pad_device->dock_in =!(gpio_get_value(pad_device->gpio_dock_detect));
	if(pad_device->dock_in){
		cancel_delayed_work_sync(&pad_device->dock_status_poll_work);
		flush_workqueue(dock_battery_work_queue);
	}
	return 0;
}

/* any smbus transaction will wake up pad */

static int pad_resume(struct i2c_client *client)
{
	pad_device->battery_present =Check_Battery_status();
	pad_device->dock_in =!(gpio_get_value(pad_device->gpio_dock_detect));
	cancel_delayed_work(&pad_device->status_poll_work);
	queue_delayed_work(battery_work_queue,&pad_device->status_poll_work,5*HZ);
	if(pad_device->dock_in){
		cancel_delayed_work(&pad_device->dock_status_poll_work);
		queue_delayed_work(dock_battery_work_queue,&pad_device->dock_status_poll_work,5*HZ);
	}
	return 0;
}
#endif

static const struct i2c_device_id pad_id[] = {
	{ "pad-battery", 0 },
	{},
};

static struct i2c_driver pad_battery_driver = {
	.probe		= pad_probe,
	.remove 	= pad_remove,
#if defined (CONFIG_PM)
	.suspend	= pad_suspend,
	.resume 	= pad_resume,
#endif
	.id_table	= pad_id,
	.driver = {
		.name	= "pad-battery",
	},
};
static int __init pad_battery_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&pad_battery_driver);
	if (ret)
	    dev_err(&pad_device->client->dev,"%s: i2c_add_driver failed\n", __func__);
	return ret;
}
module_init(pad_battery_init);

static void __exit pad_battery_exit(void)
{
	i2c_del_driver(&pad_battery_driver);
}
module_exit(pad_battery_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("PAD battery monitor driver");
MODULE_LICENSE("GPL");
