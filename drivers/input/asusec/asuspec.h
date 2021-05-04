#ifndef _ASUSPEC_H
#define _ASUSPEC_H
#include <linux/wakelock.h>
#include <linux/switch.h>

/*
 * compiler option
 */
#define ASUSPEC_DEBUG			0
//[Brook- Docking charging porting]>>
//Shouchung modify for touch pad working with absolute mode
//and modify the name to avoid build error
#define TF600T_TOUCHPAD_MODE	1	// 0: relative mode, 1: absolute mode
//Shouchung end
#define TOUCHPAD_ELAN			1	// 0: not elan, 1:elantech
#define BATTERY_DRIVER			1
//[Brook- Docking charging porting]<<

/*
 * Debug Utility
 */
#if ASUSPEC_DEBUG
#define ASUSPEC_INFO(format, arg...)	\
	printk(KERN_INFO "asuspec: [%s] " format , __FUNCTION__ , ## arg)
#define ASUSPEC_I2C_DATA(array, i)	\
					do {		\
						for (i = 0; i < array[0]+1; i++) \
							ASUSPEC_INFO("pad_ec_data[%d] = 0x%x\n", i, array[i]);	\
					} while(0)
#else
#define ASUSPEC_INFO(format, arg...)
#define ASUSPEC_I2C_DATA(array, i)
#endif

#define ASUSPEC_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "asuspec: [%s] " format , __FUNCTION__ , ## arg)

#define ASUSPEC_ERR(format, arg...)	\
	printk(KERN_ERR "asuspec: [%s] " format , __FUNCTION__ , ## arg)
//-----------------------------------------

#define DRIVER_DESC     				"ASUS PAD EC driver"
#define PAD_SDEV_NAME					"pad"
//[Brook- Docking charging porting]>>
#define CONVERSION_TIME_MS				50
#define DOCK_SDEV_NAME					"dock"
//[Brook- Docking charging porting]<<
#define APOWER_SDEV_NAME				"apower"

#define DELAY_TIME_MS					50
#define ASUSPEC_RETRY_COUNT				3
#define ASUSPEC_I2C_ERR_TOLERANCE		32
//[Brook- Docking charging porting]>>
#define ASUSDEC_RELATIVE_MODE           0
#define ASUSDEC_ABSOLUTE_MODE           1

#define NUM_RELATIVE_MODE               3       // The number of bytes per packet in relative mode
#define NUM_ABSOLUTE_MODE               6       // The number of bytes per packet in absolute mode

/* relative mode packet formate */
#define Y_OVERFLOW_MASK                 0x80
#define X_OVERFLOW_MASK                 0x40
#define Y_SIGN_MASK                     0x20
#define X_SIGN_MASK                     0x10
#define RIGHT_BTN_MASK                  0x2
#define LEFT_BTN_MASK                   0x1

/* absolute mode packet formate */
#define TOUCHPAD_SAMPLE_RATE            20
#define ABSOLUTE_BIT_MASK               0x80
#define Z_SNESITIVITY                   30
#define X_LIMIT                         5600
#define Y_LIMIT                         1300
#define X_MAX                           5855
#define X_MIN                           1198
#define Y_MAX                           4942
#define Y_MIN                           946

/*************scan 2 make mapping***************/
/*Chris 0812 start*/
#define ASUSDEC_KEYPAD_DELETE		0x4c	//DELETE
#define ASUSDEC_KEYPAD_TPONOFF		0x42	//F9
#define ASUSDEC_KEYPAD_MUTE		0x43	//F10
#define ASUSDEC_KEYPAD_VOLUMEDOWN	0x44	//F11
#define ASUSDEC_KEYPAD_VOLUMEUP		0x45	//F12
#define ASUSDEC_KEYPAD_BRIGHTNESSDOWN	0x3e	//F5
#define ASUSDEC_KEYPAD_BRIGHTNESSUP	0x3f	//F6
#define ASUSDEC_KEYPAD_SWITCHVIDEOMODE	0x41	//F8
#define ASUSDEC_KEYPAD_FLYMODE		0x3d	//F4
#define ASUSDEC_KEYPAD_SUSPEND		0x3a	//F1
#define ASUSDEC_KEYPAD_PAUSE		0x48    //PAUSE
#define ASUSDEC_KEYPAD_PRINTSCREEN	0x46	//PRT SC
#define ASUSDEC_KEYPAD_INSERT		0x49	//INSERT
#define ASUSDEC_KEYPAD_LEFTWIN		0x08	//0xE01F
#define ASUSDEC_KEYPAD_SCRLK		0x47	//SCR LK
#define ASUSDEC_KEYPAD_NUMLK		0x53    //NUM LK

/*Chris 0812 end*/
/*Chris modify key mapping start*/
#define ASUSDEC_KEYPAD_ESC				0x29	//0x76
#define ASUSDEC_KEYPAD_KEY_WAVE			0x35	//0x0E
#define ASUSDEC_KEYPAD_KEY_1			0x1e	//0x16
#define ASUSDEC_KEYPAD_KEY_2			0X1f	//0X1E
#define ASUSDEC_KEYPAD_KEY_3			0x20	//0x26
#define ASUSDEC_KEYPAD_KEY_4			0x21	//0x25
#define ASUSDEC_KEYPAD_KEY_5			0x22	//0x2E
#define ASUSDEC_KEYPAD_KEY_6        	0x23	//0x36
#define ASUSDEC_KEYPAD_KEY_7        	0x24	//0x3D
#define ASUSDEC_KEYPAD_KEY_8        	0x25	//0x3E
#define ASUSDEC_KEYPAD_KEY_9        	0x26	//0x46
#define ASUSDEC_KEYPAD_KEY_0        	0x27	//0x45
#define ASUSDEC_KEYPAD_KEY_MINUS    	0x2d	//0x4E
#define ASUSDEC_KEYPAD_KEY_EQUAL		0x2e	//0x55
#define ASUSDEC_KEYPAD_KEY_BACKSPACE	0x2a	//0x66
#define ASUSDEC_KEYPAD_KEY_TAB      	0x2b	//0x0D
#define ASUSDEC_KEYPAD_KEY_Q        	0x14	//0x15
#define ASUSDEC_KEYPAD_KEY_W        	0x1a	//0x1D
#define ASUSDEC_KEYPAD_KEY_E        	0x08	//0x24
#define ASUSDEC_KEYPAD_KEY_R        	0x15	//0x2D
#define ASUSDEC_KEYPAD_KEY_T        	0x17	//0x2C
#define ASUSDEC_KEYPAD_KEY_Y        	0x1c	//0x35
#define ASUSDEC_KEYPAD_KEY_U        	0x18	//0x3C
#define ASUSDEC_KEYPAD_KEY_I        	0x0c	//0x43
#define ASUSDEC_KEYPAD_KEY_O        	0x12	//0x44
#define ASUSDEC_KEYPAD_KEY_P        	0x13	//0x4D
#define ASUSDEC_KEYPAD_KEY_LEFTBRACE	0x2f	//0x54
#define ASUSDEC_KEYPAD_KEY_RIGHTBRACE 	0x30	//0x5B
#define ASUSDEC_KEYPAD_KEY_BACKSLASH	0x31	//0x5D
#define ASUSDEC_KEYPAD_KEY_CAPSLOCK 	0x39	//0x58
#define ASUSDEC_KEYPAD_KEY_A        	0x04	//0x1C
#define ASUSDEC_KEYPAD_KEY_S        	0x16	//0x1B
#define ASUSDEC_KEYPAD_KEY_D        	0x07	//0x23
#define ASUSDEC_KEYPAD_KEY_F        	0x09	//0x2B
#define ASUSDEC_KEYPAD_KEY_G        	0x0a	//0x34
#define ASUSDEC_KEYPAD_KEY_H        	0x0b	//0x33
#define ASUSDEC_KEYPAD_KEY_J        	0x0d	//0x3B
#define ASUSDEC_KEYPAD_KEY_K        	0x0e	//0x42
#define ASUSDEC_KEYPAD_KEY_L        	0x0f	//0x4B
#define ASUSDEC_KEYPAD_KEY_SEMICOLON	0x33	//0x4C
#define ASUSDEC_KEYPAD_KEY_APOSTROPHE	0x34	//0x52
#define ASUSDEC_KEYPAD_KEY_ENTER    	0x28	//0x5A
#define ASUSDEC_KEYPAD_KEY_LEFTSHIFT 	0x02	//0x12
#define ASUSDEC_KEYPAD_KEY_Z        	0x1d	//0x1A
#define ASUSDEC_KEYPAD_KEY_X        	0x1b	//0x22
#define ASUSDEC_KEYPAD_KEY_C        	0x06	//0x21
#define ASUSDEC_KEYPAD_KEY_V        	0x19	//0x2A
#define ASUSDEC_KEYPAD_KEY_B        	0x05	//0x32
#define ASUSDEC_KEYPAD_KEY_N        	0x11	//0x31
#define ASUSDEC_KEYPAD_KEY_M        	0x10	//0x3A
#define ASUSDEC_KEYPAD_KEY_COMMA    	0x36	//0x41
#define ASUSDEC_KEYPAD_KEY_DOT   		0x37	//0x49
#define ASUSDEC_KEYPAD_KEY_SLASH    	0x38	//0x4A
#define ASUSDEC_KEYPAD_KEY_RIGHTSHIFT   0x20//0xe5	//0x59

#define ASUSDEC_KEYPAD_KEY_LEFT   		0x50	//0xE06B
#define ASUSDEC_KEYPAD_KEY_RIGHT   		0x4f	//0xE074
#define ASUSDEC_KEYPAD_KEY_UP			0x52	//0xE075
#define ASUSDEC_KEYPAD_KEY_DOWN			0x51	//0xE072

//#define ASUSDEC_KEYPAD_RIGHTWIN		0x04	//0xE027
#define ASUSDEC_KEYPAD_LEFTCTRL			0x01	//0x14
#define ASUSDEC_KEYPAD_LEFTALT			0x04	//0x11
#define ASUSDEC_KEYPAD_KEY_SPACE		0x2c	//0x29
#define ASUSDEC_KEYPAD_RIGHTALT			0x40	//0xE011
#define ASUSDEC_KEYPAD_WINAPP			0x65	//0xE02F
#define ASUSDEC_KEYPAD_RIGHTCTRL		0x10	//0xE014
#define ASUSDEC_KEYPAD_HOME				0x4a	//0xE06C
#define ASUSDEC_KEYPAD_PAGEUP			0x4b	//0xE07D
#define ASUSDEC_KEYPAD_PAGEDOWN			0x4e	//0xE07A
#define ASUSDEC_KEYPAD_END				0x4d	//0xE069
/************  JP keys *************/
#define ASUSDEC_HANKAKU_ZENKAKU			0x5F
#define ASUSDEC_YEN						0x6A
#define ASUSDEC_MUHENKAN				0x67
#define ASUSDEC_HENKAN					0x64
#define ASUSDEC_HIRAGANA_KATAKANA		0x13
#define ASUSDEC_RO						0x51
/********************************/
/************  UK keys *************/
#define ASUSDEC_EUROPE_2				0x61
/********************************/


#define ASUSDEC_KEYPAD_LOCK				0x4c	//0xE071

/*Chris unused key start*/
#define ASUSDEC_KEYPAD_KEY_BREAK   		0xF0
#define ASUSDEC_KEYPAD_KEY_EXTEND   	0xF0
/*Chris unused key end*/

/*************scan 2 make code mapping***************/
#define ASUSPEC_OBF_MASK				0x1
#define ASUSDEC_OBF_MASK				0x1
#define ASUSPEC_KEY_MASK				0x4
#define ASUSPEC_KBC_MASK				0x8
#define ASUSPEC_AUX_MASK				0x20
#define ASUSDEC_AUX_MASK				0x20
#define ASUSPEC_SCI_MASK				0x40
#define ASUSPEC_SMI_MASK				0x80
/*Chris modify key mapping end*/

//[Brook- Docking charging porting]<<
#define ASUSPEC_OBF_MASK				0x1
#define ASUSPEC_KEY_MASK				0x4
#define ASUSPEC_KBC_MASK				0x8
#define ASUSPEC_AUX_MASK				0x20
#define ASUSPEC_SCI_MASK				0x40
#define ASUSPEC_SMI_MASK				0x80
//[Brook- Docking charging porting]>>
/************* control flag *****************/
#define ASUSPEC_DOCK_PRESENT			0x80
//[Brook- Docking charging porting]<<
/************* SMI event ********************/
//[Brook- Bug 282345 + No charging icon when insert AC]>>
#define ASUSPEC_SMI_AC_EVENT			0x31
//[Brook- Bug 282345 + No charging icon when insert AC]<<
#define ASUSPEC_SMI_HANDSHAKING			0x50
#define ASUSPEC_SMI_WAKE				0x53
#define ASUSPEC_SMI_RESET				0x5F
//[Brook- Docking charging porting]>>
#define ASUSPEC_SxI_Battery_FCCchange   0x35
#define ASUSPEC_SxI_Battery_Low				0x36 
#define ASUSPEC_SxI_Battery_Updated     0x37
#define ASUSPEC_SxI_EC_WAKEUP           0x53
#define ASUSPEC_SxI_BOOTBLOCK_RESET     0x5E
#define ASUSPEC_SxI_WATCHDOG_RESET      0x5F
#define ASUSPEC_SxI_ADAPTER_CHANGE		0x60
#define ASUSPEC_SxI_DOCK_INSERT		    0x61
#define ASUSPEC_SxI_DOCK_REMOVE		    0x62
#define ASUSPEC_SxI_PAD_BL_CHANGE		0x63
#define ASUSPEC_SxI_HID_Status_Changed	0x64
#define ASUSPEC_SxI_HID_WakeUp		    0x65
#define ASUSPEC_DOCK_SxI_AC_Event       0x71
//[Brook- Docking charging porting]<<
#define APOWER_SMI_S3			        0x83
#define APOWER_SMI_S5			        0x85
#define APOWER_SMI_NOTIFY_SHUTDOWN		0x90
#define APOWER_SMI_RESUME			    0x91
/*************APOWER switch state***************/
#define APOWER_IDLE			    		0
#define APOWER_RESUME					1
#define APOWER_SUSPEND					2
#define APOWER_POWEROFF					3
#define APOWER_NOTIFY_SHUTDOWN			4
/*************IO control setting***************/
#define ASUSPEC_IOCTL_HEAVY				2
#define ASUSPEC_IOCTL_NORMAL			1
#define ASUSPEC_IOCTL_END				0
//[Brook- Docking charging porting]>>
#define ASUSDEC_CPAS_LED_ON				1
#define ASUSDEC_CPAS_LED_OFF			0
#define ASUSDEC_TP_ON					1
#define ASUSDEC_TP_OFF					0
#define ASUSDEC_TP_CONTROL				_IOR(ASUSPEC_IOC_MAGIC,	5,	int)
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]>>
#define ASUSDEC_EC_ON					1
#define ASUSDEC_EC_OFF					0
#define ASUSDEC_EC_WAKEUP				_IOR(ASUSPEC_IOC_MAGIC,	6,	int)
//[Brook- Fix bug 295323 + System can't wakeup with dcoking when insert/removed AC]<<
//[Brook- Docking charging porting]<<
#define ASUSPEC_IOC_MAGIC				0xf4
#define ASUSPEC_IOC_MAXNR				11
#define ASUSPEC_POLLING_DATA 			_IOR(ASUSPEC_IOC_MAGIC,	1,	int)
#define ASUSPEC_FW_UPDATE 				_IOR(ASUSPEC_IOC_MAGIC,	2,	int)
#define ASUSPEC_INIT 					_IOR(ASUSPEC_IOC_MAGIC,	3,	int)
#define ASUSPEC_FW_DUMMY				_IOR(ASUSPEC_IOC_MAGIC,	7,	int)
#define ASUSPEC_SWITCH_HDMI				_IOR(ASUSPEC_IOC_MAGIC,	10,	int)
#define ASUSPEC_WIN_SHUTDOWN			_IOR(ASUSPEC_IOC_MAGIC,	11,	int)
/*****************************************/
#define ASUSPEC_MAGIC_NUM				0x19850604

/************* EC FW update ***********/
#define EC_BUFF_LEN  					256
/********************** ***********/

/************* Dock Defifition ***********/
#define DOCK_UNKNOWN					0
#define MOBILE_DOCK						1
#define AUDIO_DOCK						2
#define AUDIO_STAND						3

/************* Dock State ***********/
#define DOCK_OUT						0
#define DOCK_IN							1

/************* Cable Type ***********/
#define BAT_CABLE_OUT					0
#define BAT_CABLE_USB					1
#define BAT_CABLE_AC					3
#define BAT_CABLE_UNKNOWN				-1
#define CABLE_0V						0x00
#define CABLE_5V						0x05
#define CABLE_12V						0x12
#define CABLE_15V						0x15
#define MB 								1024*1024

/*
 * data struct
 */
//[Brook- Docking charging porting]>>
struct asusdec_touchpad_relative{
        int y_overflow;
        int x_overflow;
        int y_sign;
        int x_sign;
        int left_btn;
        int right_btn;
        int delta_x;
        int delta_y;
};

//Shouchung modify for using touchpad with the absolute mode
struct asusdec_touchpad_absolute{
        int w_val[5];
        int x_pos[5];
        int y_pos[5];
        int z_val[5];
        int left;
        int right; //Shouchung add for right button
};
//Shouchung end

struct asusdec_keypad{
        int value;
        int input_keycode;
        int extend;
};

typedef enum {
	dock_battery_type = 0,
	pad_battery_type,
}battery_type;
//[Brook- Docking charging porting]<<
struct asuspec_chip {
	struct input_dev	*indev;
	struct input_dev	*lid_indev;
	struct switch_dev 	pad_sdev;
	struct switch_dev	dock_sdev;
	struct switch_dev 	apower_sdev;
	struct i2c_client	*client;
	struct elantech_data	*private;
	struct i2c_client       *tp_client;
	struct mutex		lock;
	struct mutex		irq_lock;
	struct mutex		dock_init_lock;
	struct mutex		state_change_lock;
	//Shouchung add to solve the touch pad issue when dock in
	struct mutex		tp_lock;
	//Shouchung end
	struct delayed_work asuspec_fw_update_work;
	struct delayed_work asuspec_init_work;
	struct delayed_work asuspec_work;
	struct delayed_work asuspec_enter_s3_work;
	//[Brook- Docking charging porting]>>
	struct delayed_work asusdec_dock_init_work;
	struct delayed_work asusdec_hall_sensor_work;
	/*Chris start*/
	struct delayed_work asusdec_kb_report_work;
	/*Chris end*/
	//Shouchung add for touch pad
	struct delayed_work asusdec_tp_report_work;
	//Shouchung end
	struct asusdec_keypad keypad_data;
	//[Brook- Docking charging porting]<<
	//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]>>
	struct delayed_work asusdec_led_on_work;
	struct delayed_work asusdec_led_off_work;
	//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]<<
	struct wake_lock 	wake_lock;
       struct wake_lock        wake_lock_init;
	struct timer_list	asuspec_timer;
	int polling_rate;
	int status;
       int dock_status;
//Shouchung modify for avoid build error
#if TF600T_TOUCHPAD_MODE
//Shouchung end
        struct asusdec_touchpad_absolute t_abs;
#else
        struct asusdec_touchpad_relative touchpad_data;
#endif
	int ret_val;
	/*Chris start*/
	u8 i2c_kb_data[38];
	u8 i2c_old_kb_data[38];
	/*Chris end*/
	//Shouchung add for touch pad
	u8 i2c_tp_data[38];
	//Shouchung end
	u8 ec_data[32];
	u8 i2c_data[32];
	u8 i2c_dm_data[32];
	u8 i2c_dm_battery[32];
	u8 i2c_dm_storage[32];
	char ec_model_name[32];
	char ec_version[32];
	char ec_pcba[32];
	int op_mode;					// 0: normal mode, 1: fw update mode
	int ec_ram_init;				// 0: not init, MAGIC_NUM: init successfully
	int ec_in_s3;					// 0: normal mode, 1: ec in deep sleep mode
	int i2c_err_count;
	int apwake_disabled;			// 0: normal mode, 1: apwake gets disabled
	int audio_recording;			// 0: not recording, 1: audio recording
	unsigned long storage_total;
	unsigned long storage_avail;
	unsigned int pad_pid;
	int apower_state;
	char dec_model_name[32];
	char dec_version[32];
	char dec_pcba[32];
	int touchpad_member;
	int kb_and_ps2_enable;
	int tp_wait_ack;				// 0 : normal mode, 1: waiting for an ACK
	int tp_enable;					// 0 : touchpad has not enabled, 1: touchpad has enabled
	int suspend_state;				// 0: normal, 1: suspend
	int init_success;				// 0: ps/2 not ready. 1: init OK, -1: tp not ready
	int dock_in;					// 0: no dock, 1: dock in
	int dock_det;					// dock-in interrupt count
	int dock_init;					// 0: dock not init, 1: dock init successfully
	int dock_type;					// 0: unknown, 1: mobile_dock, 2: audio_dock, 3: audio_stand
	int dock_behavior;				// 0: susb_on follows wakeup event, 1: susb_on follows ec_req
	char dock_pid[32];
	int d_index;					// touchpad byte counter
	//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]>>
	int kbc_value;					// capslock_led 0: led off, 1: led on
	//[Brook- Fix bug 295162 + Caps Lock led on/off on Docking]<<
	//[Brook- Fix bub 295323 + System can't wakeup with dcoking when insert/removed AC]>>
	int dec_wakeup;				// 0 : dec shutdown when PAD in LP0, 1 : keep dec active when PAD in LP0,
	int cable_in;
	//[Brook- Fix bub 295323 + System can't wakeup with dcoking when insert/removed AC]<<
};

struct asusdec_chip {
	struct elantech_data *private;
	struct i2c_client       *client;
	u8 ec_data[32];
#if TOUCHPAD_MODE
        struct asusdec_touchpad_absolute t_abs;
#else
        struct asusdec_touchpad_relative touchpad_data;
#endif
};
#endif
