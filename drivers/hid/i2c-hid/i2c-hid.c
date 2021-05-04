/*
 * HID over I2C protocol implementation
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * This code is partly based on "USB HID support for Linux":
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/acpi.h>

#include <linux/i2c/i2c-hid.h>
#include <linux/usb.h>
#include "../hid-ids.h"
#include <linux/gpio.h>
#include <../arch/arm/mach-tegra/gpio-names.h>
#include <linux/input/mt.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/switch.h>

/* flags */
#define I2C_HID_STARTED		(1 << 0)
#define I2C_HID_RESET_PENDING	(1 << 1)
#define I2C_HID_READ_PENDING	(1 << 2)

#define I2C_HID_PWR_ON		0x00
#define I2C_HID_PWR_SLEEP	0x01

#define ATML_MXT_HID_PACKET 1
#ifdef ATML_MXT_HID_PACKET
#define HID_MXT_BL_ADDRESS 0x27
#endif
/* debug option */
static bool debug = true;
module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

#define i2c_hid_dbg(ihid, fmt, arg...)					  \
do {									  \
	if (debug)							  \
		dev_printk(KERN_INFO, &(ihid)->client->dev, fmt, ##arg); \
} while (0)

struct i2c_hid_desc {
	__le16 wHIDDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wReportDescRegister;
	__le16 wInputRegister;
	__le16 wMaxInputLength;
	__le16 wOutputRegister;
	__le16 wMaxOutputLength;
	__le16 wCommandRegister;
	__le16 wDataRegister;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le32 reserved;
} __packed;

struct i2c_hid_cmd {
	unsigned int registerIndex;
	__u8 opcode;
	unsigned int length;
	bool wait;
};

union command {
	u8 data[0];
	struct cmd {
		__le16 reg;
		__u8 reportTypeID;
		__u8 opcode;
	} __packed c;
};

#ifdef ATML_MXT_HID_PACKET
#define UINT16 __le16 
#define UINT8 u8
typedef struct _MxtHIDOutput {
    //HID over I2C spec.
    UINT16 OutputReg;
    UINT16 OutputLen;
    UINT8  ReportID;
    //Mxt format
    UINT8  CommandID;
    UINT8  NumWx;
    UINT8  NumRx;
    UINT16 Addr;
    UINT8  Data;   
}MxtHIDOutput; 

typedef struct _MxtHIDInput {
    //HID over I2C spec.
    UINT16  InputLen;
    UINT8  ReportID;
    //Mxt format
    UINT8   Status;
    UINT8      NumRx;
    UINT8   data[15];
}MxtHIDInput;

/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
	u32 cfg_checksum;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u16 size;
	u16 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 min_reportid;
	u8 max_reportid;
};

#include "hid_fw.h"
extern int mxt_update_fw(struct i2c_client *client, unsigned short bl_address, unsigned char *firmware_data, unsigned int firmware_size);
#endif
#define I2C_HID_CMD(opcode_) \
	.opcode = opcode_, .length = 4, \
	.registerIndex = offsetof(struct i2c_hid_desc, wCommandRegister)

/* fetch HID descriptor */
static const struct i2c_hid_cmd hid_descr_cmd = { .length = 2 };
/* fetch report descriptors */
static const struct i2c_hid_cmd hid_report_descr_cmd = {
		.registerIndex = offsetof(struct i2c_hid_desc,
			wReportDescRegister),
		.opcode = 0x00,
		.length = 2 };
/* commands */
static const struct i2c_hid_cmd hid_reset_cmd =		{ I2C_HID_CMD(0x01),
							  .wait = true };
static const struct i2c_hid_cmd hid_get_report_cmd =	{ I2C_HID_CMD(0x02) };
static const struct i2c_hid_cmd hid_set_report_cmd =	{ I2C_HID_CMD(0x03) };
static const struct i2c_hid_cmd hid_set_power_cmd =	{ I2C_HID_CMD(0x08) };

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct i2c_hid_cmd hid_get_idle_cmd = { I2C_HID_CMD(0x04) };
 * static const struct i2c_hid_cmd hid_set_idle_cmd = { I2C_HID_CMD(0x05) };
 * static const struct i2c_hid_cmd hid_get_protocol_cmd = { I2C_HID_CMD(0x06) };
 * static const struct i2c_hid_cmd hid_set_protocol_cmd = { I2C_HID_CMD(0x07) };
 */

static DEFINE_MUTEX(i2c_hid_open_mut);

/* The main device structure */
struct i2c_hid {
	struct i2c_client	*client;	/* i2c client */
	struct hid_device	*hid;	/* pointer to corresponding HID dev */
	union {
		__u8 hdesc_buffer[sizeof(struct i2c_hid_desc)];
		struct i2c_hid_desc hdesc;	/* the HID Descriptor */
	};
	__le16			wHIDDescRegister; /* location of the i2c
						   * register of the HID
						   * descriptor. */
	unsigned int		bufsize;	/* i2c buffer size */
	char			*inbuf;		/* Input buffer */
	char			*cmdbuf;	/* Command buffer */
	char			*argsbuf;	/* Command arguments buffer */

	unsigned long		flags;		/* device flags */

	wait_queue_head_t	wait;		/* For waiting the interrupt */

	struct i2c_hid_platform_data pdata;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#ifdef ATML_MXT_HID_PACKET
     	struct mxt_object *object_table;
	struct mxt_info info;
	struct mutex access_mutex;
	u16 msg_address;
	u16 last_address;
	struct switch_dev touch_sdev;
#endif
};

static int __i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command, u8 reportID,
		u8 reportType, u8 *args, int args_len,
		unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	union command *cmd = (union command *)ihid->cmdbuf;
	int ret;
	struct i2c_msg msg[2];
	int msg_num = 1;

	int length = command->length;
	bool wait = command->wait;
	unsigned int registerIndex = command->registerIndex;

	/* special case for hid_descr_cmd */
	if (command == &hid_descr_cmd) {
		cmd->c.reg = ihid->wHIDDescRegister;
	} else {
		cmd->data[0] = ihid->hdesc_buffer[registerIndex];
		cmd->data[1] = ihid->hdesc_buffer[registerIndex + 1];
	}

	if (length > 2) {
		cmd->c.opcode = command->opcode;
		cmd->c.reportTypeID = reportID | reportType << 4;
	}

	memcpy(cmd->data + length, args, args_len);
	length += args_len;

	i2c_hid_dbg(ihid, "%s: cmd=%*ph\n", __func__, length, cmd->data);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = cmd->data;
	if (data_len > 0) {
		msg[1].addr = client->addr;
		msg[1].flags = client->flags & I2C_M_TEN;
		msg[1].flags |= I2C_M_RD;
		msg[1].len = data_len;
		msg[1].buf = buf_recv;
		msg_num = 2;
		set_bit(I2C_HID_READ_PENDING, &ihid->flags);
	}

	if (wait)
		set_bit(I2C_HID_RESET_PENDING, &ihid->flags);

	ret = i2c_transfer(client->adapter, msg, msg_num);

	if (data_len > 0)
		clear_bit(I2C_HID_READ_PENDING, &ihid->flags);

	if (ret != msg_num)
		return ret < 0 ? ret : -EIO;

	ret = 0;

	if (wait) {
		i2c_hid_dbg(ihid, "%s: waiting...\n", __func__);
		if (!wait_event_timeout(ihid->wait,
				!test_bit(I2C_HID_RESET_PENDING, &ihid->flags),
				msecs_to_jiffies(5000)))
			ret = -ENODATA;
		i2c_hid_dbg(ihid, "%s: finished.\n", __func__);
	}

	return ret;
}

static int i2c_hid_command(struct i2c_client *client,
		const struct i2c_hid_cmd *command,
		unsigned char *buf_recv, int data_len)
{
	return __i2c_hid_command(client, command, 0, 0, NULL, 0,
				buf_recv, data_len);
}

static int i2c_hid_get_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf_recv, int data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 args[3];
	int ret;
	int args_len = 0;
	u16 readRegister = le16_to_cpu(ihid->hdesc.wDataRegister);

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (reportID >= 0x0F) {
		args[args_len++] = reportID;
		reportID = 0x0F;
	}

	args[args_len++] = readRegister & 0xFF;
	args[args_len++] = readRegister >> 8;

	ret = __i2c_hid_command(client, &hid_get_report_cmd, reportID,
		reportType, args, args_len, buf_recv, data_len);
	if (ret) {
		dev_err(&client->dev,
			"failed to retrieve report from device.\n");
		return ret;
	}

	return 0;
}

static int i2c_hid_set_report(struct i2c_client *client, u8 reportType,
		u8 reportID, unsigned char *buf, size_t data_len)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 *args = ihid->argsbuf;
	int ret;
	u16 dataRegister = le16_to_cpu(ihid->hdesc.wDataRegister);

	/* hidraw already checked that data_len < HID_MAX_BUFFER_SIZE */
	u16 size =	2			/* size */ +
			(reportID ? 1 : 0)	/* reportID */ +
			data_len		/* buf */;
	int args_len =	(reportID >= 0x0F ? 1 : 0) /* optional third byte */ +
			2			/* dataRegister */ +
			size			/* args */;
	int index = 0;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	if (reportID >= 0x0F) {
		args[index++] = reportID;
		reportID = 0x0F;
	}

	args[index++] = dataRegister & 0xFF;
	args[index++] = dataRegister >> 8;

	args[index++] = size & 0xFF;
	args[index++] = size >> 8;

	if (reportID)
		args[index++] = reportID;

	memcpy(&args[index], buf, data_len);

	ret = __i2c_hid_command(client, &hid_set_report_cmd, reportID,
		reportType, args, args_len, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to set a report to device.\n");
		return ret;
	}

	return data_len;
}

static int i2c_hid_set_power(struct i2c_client *client, int power_state)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	ret = __i2c_hid_command(client, &hid_set_power_cmd, power_state,
		0, NULL, 0, NULL, 0);
	if (ret)
		dev_err(&client->dev, "failed to change power setting.\n");

	return ret;
}

static int i2c_hid_hwreset(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	i2c_hid_dbg(ihid, "%s\n", __func__);

	ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);
	if (ret)
		return ret;

	i2c_hid_dbg(ihid, "resetting...\n");

	ret = i2c_hid_command(client, &hid_reset_cmd, NULL, 0);
	if (ret) {
		dev_err(&client->dev, "failed to reset device.\n");
		i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
		return ret;
	}

	return 0;
}

static void i2c_hid_get_input(struct i2c_hid *ihid)
{
	int ret, ret_size, i;
	int size = le16_to_cpu(ihid->hdesc.wMaxInputLength);

	ret = i2c_master_recv(ihid->client, ihid->inbuf, size);
	if (ret != size) {
		if (ret < 0)
			return;

		dev_err(&ihid->client->dev, "%s: got %d data instead of %d\n",
			__func__, ret, size);
		return;
	}

	ret_size = ihid->inbuf[0] | ihid->inbuf[1] << 8;

	if (!ret_size) {
		/* host or device initiated RESET completed */
		if (test_and_clear_bit(I2C_HID_RESET_PENDING, &ihid->flags))
			wake_up(&ihid->wait);
		return;
	}

	if (ret_size > size) {
		dev_err(&ihid->client->dev, "%s: incomplete report (%d/%d)\n",
			__func__, size, ret_size);
		return;
	}
/*
	i2c_hid_dbg(ihid, "input: %*ph\n", ret_size, ihid->inbuf);

      printk("%s: ", __func__);
	for(i = 0; i < ret_size; i++){
	    printk(" %02X", ihid->inbuf[i]);
	}
	printk("\n");
*/
	if (test_bit(I2C_HID_STARTED, &ihid->flags))
		hid_input_report(ihid->hid, HID_INPUT_REPORT, ihid->inbuf + 2,
				ret_size - 2, 1);

	return;
}

static irqreturn_t i2c_hid_irq(int irq, void *dev_id)
{
	struct i2c_hid *ihid = dev_id;

	if (test_bit(I2C_HID_READ_PENDING, &ihid->flags))
		return IRQ_HANDLED;

	i2c_hid_get_input(ihid);

	return IRQ_HANDLED;
}

static int i2c_hid_get_report_length(struct hid_report *report)
{
	return ((report->size - 1) >> 3) + 1 +
		report->device->report_enum[report->type].numbered + 2;
}

static void i2c_hid_init_report(struct hid_report *report, u8 *buffer,
	size_t bufsize)
{
	struct hid_device *hid = report->device;
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	unsigned int size, ret_size;

	size = i2c_hid_get_report_length(report);
	if (i2c_hid_get_report(client,
			report->type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report->id, buffer, size))
		return;

	i2c_hid_dbg(ihid, "report (len=%d): %*ph\n", size, size, ihid->inbuf);

	ret_size = buffer[0] | (buffer[1] << 8);

	if (ret_size != size) {
		dev_err(&client->dev, "error in %s size:%d / ret_size:%d\n",
			__func__, size, ret_size);
		return;
	}

	/* hid->driver_lock is held as we are in probe function,
	 * we just need to setup the input fields, so using
	 * hid_report_raw_event is safe. */
	hid_report_raw_event(hid, report->type, buffer + 2, size - 2, 1);
}

/*
 * Initialize all reports
 */
static void i2c_hid_init_reports(struct hid_device *hid)
{
	struct hid_report *report;
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	u8 *inbuf = kzalloc(ihid->bufsize, GFP_KERNEL);

	if (!inbuf) {
		dev_err(&client->dev, "can not retrieve initial reports\n");
		return;
	}

	list_for_each_entry(report,
		&hid->report_enum[HID_INPUT_REPORT].report_list, list)
		i2c_hid_init_report(report, inbuf, ihid->bufsize);

	list_for_each_entry(report,
		&hid->report_enum[HID_FEATURE_REPORT].report_list, list)
		i2c_hid_init_report(report, inbuf, ihid->bufsize);

	kfree(inbuf);
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void i2c_hid_find_max_report(struct hid_device *hid, unsigned int type,
		unsigned int *max)
{
	struct hid_report *report;
	unsigned int size;

	/* We should not rely on wMaxInputLength, as some devices may set it to
	 * a wrong length. */
	list_for_each_entry(report, &hid->report_enum[type].report_list, list) {
		size = i2c_hid_get_report_length(report);
		if (*max < size)
			*max = size;
	}
}

static void i2c_hid_free_buffers(struct i2c_hid *ihid)
{
	kfree(ihid->inbuf);
	kfree(ihid->argsbuf);
	kfree(ihid->cmdbuf);
	ihid->inbuf = NULL;
	ihid->cmdbuf = NULL;
	ihid->argsbuf = NULL;
	ihid->bufsize = 0;
}

static int i2c_hid_alloc_buffers(struct i2c_hid *ihid, size_t report_size)
{
	/* the worst case is computed from the set_report command with a
	 * reportID > 15 and the maximum report length */
	int args_len = sizeof(__u8) + /* optional ReportID byte */
		       sizeof(__u16) + /* data register */
		       sizeof(__u16) + /* size of the report */
		       report_size; /* report */

	ihid->inbuf = kzalloc(report_size, GFP_KERNEL);
	ihid->argsbuf = kzalloc(args_len, GFP_KERNEL);
	ihid->cmdbuf = kzalloc(sizeof(union command) + args_len, GFP_KERNEL);

	if (!ihid->inbuf || !ihid->argsbuf || !ihid->cmdbuf) {
		i2c_hid_free_buffers(ihid);
		return -ENOMEM;
	}

	ihid->bufsize = report_size;

	return 0;
}

static int i2c_hid_get_raw_report(struct hid_device *hid,
		unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	size_t ret_count, ask_count;
	int ret;

	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;

	/* +2 bytes to include the size of the reply in the query buffer */
	ask_count = min(count + 2, (size_t)ihid->bufsize);

	ret = i2c_hid_get_report(client,
			report_type == HID_FEATURE_REPORT ? 0x03 : 0x01,
			report_number, ihid->inbuf, ask_count);

	if (ret < 0)
		return ret;

	ret_count = ihid->inbuf[0] | (ihid->inbuf[1] << 8);

	if (ret_count <= 2)
		return 0;

	ret_count = min(ret_count, ask_count);

	/* The query buffer contains the size, dropping it in the reply */
	count = min(count, ret_count - 2);
	memcpy(buf, ihid->inbuf + 2, count);

	return count;
}

static int i2c_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
		size_t count, unsigned char report_type)
{
	struct i2c_client *client = hid->driver_data;
	int report_id = buf[0];
	int ret;

	if (report_type == HID_INPUT_REPORT)
		return -EINVAL;

	if (report_id) {
		buf++;
		count--;
	}

	ret = i2c_hid_set_report(client,
				report_type == HID_FEATURE_REPORT ? 0x03 : 0x02,
				report_id, buf, count);

	if (report_id && ret >= 0)
		ret++; /* add report_id to the number of transfered bytes */

	return ret;
}

static void i2c_hid_request(struct hid_device *hid, struct hid_report *rep,
		int reqtype)
{
	struct i2c_client *client = hid->driver_data;
	char *buf;
	int ret;
	int len = i2c_hid_get_report_length(rep) - 2;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return;

	switch (reqtype) {
	case HID_REQ_GET_REPORT:
		ret = i2c_hid_get_raw_report(hid, rep->id, buf, len, rep->type);
		if (ret < 0)
			dev_err(&client->dev, "%s: unable to get report: %d\n",
				__func__, ret);
		else
			hid_input_report(hid, rep->type, buf, ret, 0);
		break;
	case HID_REQ_SET_REPORT:
		hid_output_report(rep, buf);
		i2c_hid_output_raw_report(hid, buf, len, rep->type);
		break;
	}

	kfree(buf);
}

static int i2c_hid_parse(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int rsize;
	char *rdesc;
	int ret;
	int tries = 3, i;

	i2c_hid_dbg(ihid, "entering %s\n", __func__);

	rsize = le16_to_cpu(hdesc->wReportDescLength);
	if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
		dbg_hid("weird size of report descriptor (%u)\n", rsize);
		return -EINVAL;
	}

	do {
		ret = i2c_hid_hwreset(client);
		if (ret)
			msleep(1000);
	} while (tries-- > 0 && ret);

	if (ret)
		return ret;

	rdesc = kzalloc(rsize, GFP_KERNEL);

	if (!rdesc) {
		dbg_hid("couldn't allocate rdesc memory\n");
		return -ENOMEM;
	}

	i2c_hid_dbg(ihid, "asking HID report descriptor\n");

	ret = i2c_hid_command(client, &hid_report_descr_cmd, rdesc, rsize);
	if (ret) {
		hid_err(hid, "reading report descriptor failed\n");
		kfree(rdesc);
		return -EIO;
	}

	i2c_hid_dbg(ihid, "Report Descriptor: %*ph\n", rsize, rdesc);

      for(i = 0; i < rsize; i++)
	  	i2c_hid_dbg(ihid, "Input Discriptor[%d]:%02X\n", i, rdesc[i]);
	
	ret = hid_parse_report(hid, rdesc, rsize);
	kfree(rdesc);
	if (ret) {
		dbg_hid("parsing report descriptor failed\n");
		return ret;
	}

	return 0;
}

static int i2c_hid_start(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;
	unsigned int bufsize = HID_MIN_BUFFER_SIZE;

	i2c_hid_find_max_report(hid, HID_INPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_OUTPUT_REPORT, &bufsize);
	i2c_hid_find_max_report(hid, HID_FEATURE_REPORT, &bufsize);

	if (bufsize > ihid->bufsize) {
		i2c_hid_free_buffers(ihid);

		ret = i2c_hid_alloc_buffers(ihid, bufsize);

		if (ret)
			return ret;
	}

	if (!(hid->quirks & HID_QUIRK_NO_INIT_REPORTS))
		i2c_hid_init_reports(hid);

	return 0;
}

static void i2c_hid_stop(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	hid->claimed = 0;

	i2c_hid_free_buffers(ihid);
}

static int i2c_hid_open(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&i2c_hid_open_mut);
	if (!hid->open++) {
		ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);
		if (ret) {
			hid->open--;
			goto done;
		}
		set_bit(I2C_HID_STARTED, &ihid->flags);
	}
done:
	mutex_unlock(&i2c_hid_open_mut);
	return ret;
}

static void i2c_hid_close(struct hid_device *hid)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);

	/* protecting hid->open to make sure we don't restart
	 * data acquistion due to a resumption we no longer
	 * care about
	 */
	mutex_lock(&i2c_hid_open_mut);
	if (!--hid->open) {
		clear_bit(I2C_HID_STARTED, &ihid->flags);

		/* Save some power */
		i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
	}
	mutex_unlock(&i2c_hid_open_mut);
}

static int i2c_hid_power(struct hid_device *hid, int lvl)
{
	struct i2c_client *client = hid->driver_data;
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret = 0;

	i2c_hid_dbg(ihid, "%s lvl:%d\n", __func__, lvl);

	switch (lvl) {
	case PM_HINT_FULLON:
		ret = i2c_hid_set_power(client, I2C_HID_PWR_ON);
		break;
	case PM_HINT_NORMAL:
		ret = i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);
		break;
	}
	return ret;
}

static int i2c_hid_hidinput_input_event(struct input_dev *dev,
		unsigned int type, unsigned int code, int value)
{
	struct hid_device *hid = input_get_drvdata(dev);
	struct hid_field *field;
	int offset;

	if (type == EV_FF)
		return input_ff_event(dev, type, code, value);

	if (type != EV_LED)
		return -1;

	offset = hidinput_find_field(hid, type, code, &field);

	if (offset == -1) {
		hid_warn(dev, "event field not found\n");
		return -1;
	}

	return hid_set_field(field, offset, value);
}

static struct hid_ll_driver i2c_hid_ll_driver = {
	.parse = i2c_hid_parse,
	.start = i2c_hid_start,
	.stop = i2c_hid_stop,
	.open = i2c_hid_open,
	.close = i2c_hid_close,
	.power = i2c_hid_power,
	//.request = i2c_hid_request,
	.hidinput_input_event = i2c_hid_hidinput_input_event,
};

static int i2c_hid_init_irq(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	int ret;

	dev_dbg(&client->dev, "Requesting IRQ: %d\n", client->irq);

	ret = request_threaded_irq(client->irq, NULL, i2c_hid_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			client->name, ihid);
	if (ret < 0) {
		dev_warn(&client->dev,
			"Could not register for %s interrupt, irq = %d,"
			" ret = %d\n",
			client->name, client->irq, ret);

		return ret;
	}

	return 0;
}

static int i2c_hid_fetch_hid_descriptor(struct i2c_hid *ihid)
{
	struct i2c_client *client = ihid->client;
	struct i2c_hid_desc *hdesc = &ihid->hdesc;
	unsigned int dsize;
	int ret, i;

      for(i = 0; i < 3; i++){
          /* Fetch the length of HID description, retrieve the 4 first bytes:
	    * bytes 0-1 -> length
	    * bytes 2-3 -> bcdVersion (has to be 1.00) */
	   ret = i2c_hid_command(client, &hid_descr_cmd, ihid->hdesc_buffer, 2);

	   i2c_hid_dbg(ihid, "%s, ihid->hdesc_buffer: %*ph\n",
			__func__, 4, ihid->hdesc_buffer);

	   if (ret) {
		  dev_err(&client->dev,
			"unable to fetch the size of HID descriptor (ret=%d)\n",
			ret);
		  msleep(30);
	   }else 
	       break;
	}
	

	dsize = le16_to_cpu(hdesc->wHIDDescLength);
	
	/*
	 * the size of the HID descriptor should at least contain
	 * its size and the bcdVersion (4 bytes), and should not be greater
	 * than sizeof(struct i2c_hid_desc) as we directly fill this struct
	 * through i2c_hid_command.
	 */
	if (dsize < 4 || dsize > sizeof(struct i2c_hid_desc)) {
		dev_err(&client->dev, "weird size of HID descriptor (%u)\n",
			dsize);
		return -ENODEV;
	}

      i2c_hid_dbg(ihid, "Fetching the HID descriptor\n");

	ret = i2c_hid_command(client, &hid_descr_cmd, ihid->hdesc_buffer,
				dsize);
	if (ret) {
		dev_err(&client->dev, "hid_descr_cmd Fail\n");
		return -ENODEV;
	}

	/* check bcdVersion == 1.0 */
	if (le16_to_cpu(hdesc->bcdVersion) != 0x0100) {
		dev_err(&client->dev,
			"unexpected HID descriptor bcdVersion (0x%04hx)\n",
			le16_to_cpu(hdesc->bcdVersion));
		return -ENODEV;
	}

	
	i2c_hid_dbg(ihid, "HID Descriptor: %*ph\n", dsize, ihid->hdesc_buffer);
	for(i = 0; i < dsize; i++){
	     i2c_hid_dbg(ihid, "HID Discriptor [%d]:%02X\n", i, ihid->hdesc_buffer[i]);
	}

	return 0;
}

#ifdef CONFIG_ACPI
static int i2c_hid_acpi_pdata(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	static u8 i2c_hid_guid[] = {
		0xF7, 0xF6, 0xDF, 0x3C, 0x67, 0x42, 0x55, 0x45,
		0xAD, 0x05, 0xB3, 0x0A, 0x3D, 0x89, 0x38, 0xDE,
	};
	struct acpi_buffer buf = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object params[4], *obj;
	struct acpi_object_list input;
	struct acpi_device *adev;
	acpi_handle handle;

	handle = ACPI_HANDLE(&client->dev);
	if (!handle || acpi_bus_get_device(handle, &adev))
		return -ENODEV;

	input.count = ARRAY_SIZE(params);
	input.pointer = params;

	params[0].type = ACPI_TYPE_BUFFER;
	params[0].buffer.length = sizeof(i2c_hid_guid);
	params[0].buffer.pointer = i2c_hid_guid;
	params[1].type = ACPI_TYPE_INTEGER;
	params[1].integer.value = 1;
	params[2].type = ACPI_TYPE_INTEGER;
	params[2].integer.value = 1; /* HID function */
	params[3].type = ACPI_TYPE_INTEGER;
	params[3].integer.value = 0;

	if (ACPI_FAILURE(acpi_evaluate_object(handle, "_DSM", &input, &buf))) {
		dev_err(&client->dev, "device _DSM execution failed\n");
		return -ENODEV;
	}

	obj = (union acpi_object *)buf.pointer;
	if (obj->type != ACPI_TYPE_INTEGER) {
		dev_err(&client->dev, "device _DSM returned invalid type: %d\n",
			obj->type);
		kfree(buf.pointer);
		return -EINVAL;
	}

	pdata->hid_descriptor_address = obj->integer.value;

	kfree(buf.pointer);
	return 0;
}

static const struct acpi_device_id i2c_hid_acpi_match[] = {
	{"ACPI0C50", 0 },
	{"PNP0C50", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, i2c_hid_acpi_match);
#else
static inline int i2c_hid_acpi_pdata(struct i2c_client *client,
		struct i2c_hid_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void i2c_hid_early_suspend(struct early_suspend *es);
static void i2c_hid_early_resume(struct early_suspend *es);
#endif

#ifdef ATML_MXT_HID_PACKET
#define MXT_HID_OUT_REPOART_ID  0x06
#define MXT_HID_OUT_COMMAND_ID 0x51

#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_GEN_MESSAGE_T5		5

#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_TOUCH_MULTI_T9		9

static int __mxt_hid_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	u8 buf[2] = {0};
	int retval = 0;
	struct i2c_hid *data = i2c_get_clientdata(client);
	MxtHIDOutput out_message = {0};
	MxtHIDInput  in_message = {0};

	//buf[0] = reg & 0xff;
	//buf[1] = (reg >> 8) & 0xff;

	if(len > 0x0F){
	    dev_err(&client->dev, "The length should be not larger than 0xF!\n");
	    return -1;
	}
	
     	out_message.OutputReg = data->hdesc.wOutputRegister;
	out_message.OutputLen = cpu_to_le16(8); 
	out_message.ReportID = MXT_HID_OUT_REPOART_ID;
	out_message.CommandID = MXT_HID_OUT_COMMAND_ID;
	out_message.NumWx = 2; 
	out_message.NumRx = len;
	out_message.Addr = cpu_to_le16(reg);
	out_message.Data = 0;
	mutex_lock(&data->access_mutex);

	if ((data->last_address != reg) || (reg != data->msg_address)) {
		if (i2c_master_send(client, (u8 *)&out_message, sizeof(MxtHIDOutput) -1) != sizeof(MxtHIDOutput) -1) {
			dev_dbg(&client->dev, "i2c retry\n");
			msleep(MXT_WAKEUP_TIME);

			if (i2c_master_send(client, (u8 *)&out_message, sizeof(MxtHIDOutput) -1) != sizeof(MxtHIDOutput) -1) {
				dev_err(&client->dev, "%s: i2c send failed\n",
					__func__);
				retval = -EIO;
				goto mxt_read_exit;
			}
		}
	}

	if (i2c_master_recv(client, (u8 *)&in_message, sizeof(MxtHIDInput)) != sizeof(MxtHIDInput)) {
		dev_dbg(&client->dev, "i2c retry\n");
		msleep(MXT_WAKEUP_TIME);

		if (i2c_master_recv(client, (u8 *)&in_message, sizeof(MxtHIDInput)) != sizeof(MxtHIDInput)) {
			dev_err(&client->dev, "%s: i2c recv failed\n",
				__func__);
			retval = -EIO;
			goto mxt_read_exit;
		}
	}

      if(in_message.Status != 0x00){
	    dev_err(&client->dev, "Error to read register from touch chip!\n");
          goto mxt_read_exit;
      }
      memcpy(val, in_message.data, len);
	data->last_address = reg;

mxt_read_exit:
	mutex_unlock(&data->access_mutex);
	return retval;
}

static int mxt_hid_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_hid_read_reg(client, reg, 1, val);
}

static int mxt_hid_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	int retval = 0;
	struct i2c_hid *data = i2c_get_clientdata(client);
	MxtHIDOutput out_message = {0};
	MxtHIDInput  in_message = {0};
	
     	out_message.OutputReg = data->hdesc.wOutputRegister;
	out_message.OutputLen = cpu_to_le16(9); 
	out_message.ReportID = MXT_HID_OUT_REPOART_ID;
	out_message.CommandID = MXT_HID_OUT_COMMAND_ID;
	out_message.NumWx = 3; 
	out_message.NumRx = 0;
	out_message.Addr = cpu_to_le16(reg);
	out_message.Data = val;
	mutex_lock(&data->access_mutex);

	if ((data->last_address != reg) || (reg != data->msg_address)) {
		if (i2c_master_send(client, (u8 *)&out_message, sizeof(MxtHIDOutput)) != sizeof(MxtHIDOutput)) {
			dev_dbg(&client->dev, "i2c retry\n");
			msleep(MXT_WAKEUP_TIME);

			if (i2c_master_send(client, (u8 *)&out_message, sizeof(MxtHIDOutput)) != sizeof(MxtHIDOutput)) {
				dev_err(&client->dev, "%s: i2c send failed\n",
					__func__);
				retval = -EIO;
				goto mxt_read_exit;
			}
		}
	}

	if (i2c_master_recv(client, (u8 *)&in_message, sizeof(MxtHIDInput)) != sizeof(MxtHIDInput)) {
		dev_dbg(&client->dev, "i2c retry\n");
		msleep(MXT_WAKEUP_TIME);

		if (i2c_master_recv(client, (u8 *)&in_message, sizeof(MxtHIDInput)) != sizeof(MxtHIDInput)) {
			dev_err(&client->dev, "%s: i2c recv failed\n",
				__func__);
			retval = -EIO;
			goto mxt_read_exit;
		}
	}

      if(in_message.Status != 0x04){
	    dev_err(&client->dev, "Error to write register to touch chip!\n");
          goto mxt_read_exit;
      }

	data->last_address = reg + 1;
mxt_read_exit:
	mutex_unlock(&data->access_mutex);
	return retval;}


static int mxt_hid_get_info(struct i2c_hid *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	/* force send of address pointer on first read during probe */
	data->last_address = -1;

	error = mxt_hid_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt_hid_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_hid_read_reg(client, MXT_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt_hid_read_reg(client, MXT_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = mxt_hid_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt_hid_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_hid_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static int mxt_hid_get_object_table(struct i2c_hid *data)
{
	struct device *dev = &data->client->dev;
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_hid_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3] + 1;
		object->instances = buf[4] + 1;
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids * object->instances;
			object->max_reportid = reportid;
			object->min_reportid = object->max_reportid -
				object->instances * object->num_report_ids + 1;
		}

		/* Store message window address so we don't have to
		   search the object table every time we read message */
		if (object->type == MXT_GEN_MESSAGE_T5)
			data->msg_address = object->start_address;

		dev_info(dev, "T%d, start:%d size:%d instances:%d "
			"min_reportid:%d max_reportid:%d\n",
			object->type, object->start_address, object->size,
			object->instances,
			object->min_reportid, object->max_reportid);
	}

	return 0;
}

static struct mxt_object * mxt_hid_get_object(struct i2c_hid *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type T%d\n", type);
	return NULL;
}

static int mxt_hid_read_object(struct i2c_hid *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_hid_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_hid_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_hid_write_object(struct i2c_hid *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_hid_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_hid_write_reg(data->client, reg + offset, val);
}


static void mxt_hid_dump_object(struct i2c_hid *data, u8 type){
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	struct mxt_object *object;
	int instance, size;
	u8 val;

	object = mxt_hid_get_object(data, type);
	if(object){
	    for(instance = 0; instance < object->instances; instance++){
	        for(size = 0; size < object->size; size++){
		      mxt_hid_read_object(data, type, (instance * object->size) + size, &val);
			//if(size%8 == 0)
		         printk("MXT Oject[%d][%d][%d]: 0x%02X(%d)\n", type, instance, size, val, val);
			//else
			//   printk(", 0x%02X", val);
		  }
	    }
	}
}

static void mxt_hid_config_object(struct i2c_hid *data, struct tp_config_values *tp_config){
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	struct mxt_object *object;
	u16 reg;
	int offset;
	u8 *values;

	object = mxt_hid_get_object(data, tp_config->type);
	if(object){
	   reg = object->start_address;
	   values = tp_config->values;
	   for(offset = 0; offset < tp_config->size; offset++){
	       mxt_hid_write_reg(data->client, reg + offset, (tp_config->zero ? 0x00 : values[offset]));
	       //printk("Write MXT Oject[%d][%d]: 0x%02X\n", tp_config->type, offset, (tp_config->zero ? 0x00 : values[offset]));
	   }
	}
}

static bool mxt_wait_irq_low(struct i2c_hid *data, unsigned int ms_time){
       struct i2c_client *client = data->client;
       int wating_time = 0;

	 while(wating_time <= ms_time && gpio_get_value(data->pdata.irq_gpio) != 0){
	     usleep_range(1000, 1500);
	     wating_time++;
	 }

	 if(wating_time > ms_time)
	     return false;
	 else 
	     return true;
}

static int mxt_hid_initialize(struct i2c_hid *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error, i;
	u8 buff[6] = {0};

	error = mxt_hid_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_hid_get_object_table(data);
	if (error) {
		dev_err(&client->dev, "Failed to read object table\n");
		return error;
	}

      /* Get the Touch config 24-bit checksum*/
	mxt_hid_write_object(data, 6, 0x03, 0x01);
	msleep(5);
	/* Read the T6 message*/
	__mxt_hid_read_reg(data->client, data->msg_address, sizeof(buff), &buff);
	info->cfg_checksum =  buff[2] | (buff[3] << 8) | (buff[4] << 16);

	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d, cfg_checksum=0x%06X\n",
			info->family_id, info->variant_id, info->version,
			info->build, info->cfg_checksum);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);
	
	return 0;
}

static int mxt_hid_config_tp(struct i2c_hid *data){
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error, i, tp_vendor = 0;
	u8 buff[6] = {0};
	u32 checksum;
	struct tp_config_values *tp_configs; 

	tp_vendor = gpio_get_value(data->pdata.tp_id_gpio[1]) > 0 ? 1 : 0;
	checksum = tp_vendor ? TOUCH_WINTEK_CONFIG_CHECKSUM : TOUCH_CANDO_CONFIG_CHECKSUM;
	tp_configs = tp_vendor ? wintek_configs : cando_configs;
       if(info->cfg_checksum == checksum){
	   	dev_info(&client->dev, "tp_vendor=%d, tp_vendor_gpio=%d, tp_vendor_gpio_value[0]=%d, tp_vendor_gpio_value[1]=%d\n", tp_vendor, 
			data->pdata.tp_id_gpio[1], gpio_get_value(data->pdata.tp_id_gpio[0]), gpio_get_value(data->pdata.tp_id_gpio[1]));
	   	return 0;
	 }
	   	
	/*Dump the main configuration registers value*/
	for(i = 0; i < sizeof(wintek_configs)/sizeof(struct tp_config_values); i++){
	    mxt_hid_config_object(data, &tp_configs[i]);
	    mxt_hid_dump_object(data, tp_configs[i].type);
	}

       /* Write the Touch backup message*/
	mxt_hid_write_object(data, 6, 0x01, 0x55);
	mxt_wait_irq_low(data, 20);
	/* Read the T6 message*/
	memset(buff, 0, sizeof(buff));
	__mxt_hid_read_reg(data->client, data->msg_address, sizeof(buff), &buff);
	return 0;
}

static bool isInBootLoaderMode(struct i2c_client *client){
    u8 buf[2];
	int ret;
	int identified;
	int retry = 2;
	int times;
	struct i2c_msg rmsg;

	rmsg.addr= HID_MXT_BL_ADDRESS;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 2;
	rmsg.buf = buf;
	
    /* Read 2 byte from boot loader I2C address to make sure touch chip is in bootloader mode */   
	for(times = 0; times < retry; times++ ){
	     ret = i2c_transfer(client->adapter, &rmsg, 1); 
	     if(ret >= 0)
		 	break;
		 	  	 
	     mdelay(25);
	}
	dev_err(&client->dev, "The touch is %s in bootloader mode.\n", (ret < 0 ? "not" : "indeed"));
	return ret >= 0;
}


static void mxt_touch_reset(struct i2c_hid *ihid, bool force){
     int irq_value;
     struct i2c_client *client = ihid->client;

     irq_value = gpio_get_value(ihid->pdata.irq_gpio);
     if(!force && irq_value == 0){
         dev_info(&client->dev, "%s bypass touch reset for irq_value=%d\n", __func__, irq_value);
         return;
     }

     dev_info(&client->dev, "%s start touch reset\n", __func__);
     gpio_direction_output(ihid->pdata.rst_gpio, 0);
     msleep(10);
     gpio_direction_output(ihid->pdata.rst_gpio, 1);
     msleep(200);
}

static int recovery_from_bootMode(struct i2c_client *client){
    unsigned char data[] = {0x01, 0x01};
    struct i2c_msg wmsg;
    struct i2c_hid_platform_data *pdata = client->dev.platform_data;
    
    wmsg.addr = HID_MXT_BL_ADDRESS;
    wmsg.flags = 0;
    wmsg.len = 2;
    wmsg.buf = data;
    dev_err(&client->dev, "---------Touch: Try to leave the bootloader mode!\n");
	/*Write two nosense bytes to I2C address HID_MXT_BL_ADDRESS in order to force touch to leave the bootloader mode.*/
    i2c_transfer(client->adapter, &wmsg, 1);
    mdelay(10);
	
    /* The touch had failed in fw update if touch was still in bootloader mode*/
    if(isInBootLoaderMode(client)){
        mxt_update_fw(client, HID_MXT_BL_ADDRESS, fw_V13_AC_20, sizeof(fw_V13_AC_20));
        dev_info(&client->dev, "%s start touch reset\n", __func__);
        gpio_direction_output(pdata->rst_gpio, 0);
        msleep(10);
        gpio_direction_output(pdata->rst_gpio, 1);
        msleep(200);
    }
    return 0;
}

static ssize_t hid_touch_switch_name(struct switch_dev *sdev, char *buf)
{
      struct i2c_hid *hid;
	struct mxt_info *info;
	hid = container_of(sdev, struct i2c_hid, touch_sdev);
	info = &hid->info;
	return sprintf(buf, "MXT-0x%02X-0x%02X-0x%06X\n",  info->version, info->build, info->cfg_checksum);
}

static ssize_t hid_touch_switch_state(struct switch_dev *sdev, char *buf)
{ 
      	return sprintf(buf, "%s\n", "0");
}

#endif

static int i2c_hid_probe(struct i2c_client *client,
			 const struct i2c_device_id *dev_id)
{
	int ret;
	struct i2c_hid *ihid;
	struct hid_device *hid;
	__u16 hidRegister;
	struct i2c_hid_platform_data *platform_data = client->dev.platform_data;

	dbg_hid("HID probe called for i2c 0x%02x\n", client->addr);

	if (!client->irq) {
		dev_err(&client->dev,
			"HID over i2c has not been provided an Int IRQ\n");
		return -EINVAL;
	}

	ihid = kzalloc(sizeof(struct i2c_hid), GFP_KERNEL);
	if (!ihid)
		return -ENOMEM;

	if (!platform_data) {
		ret = i2c_hid_acpi_pdata(client, &ihid->pdata);
		if (ret) {
			dev_err(&client->dev,
				"HID register address not provided\n");
			goto err;
		}
	} else {
		ihid->pdata = *platform_data;
	}

	i2c_set_clientdata(client, ihid);

	ihid->client = client;

	hidRegister = ihid->pdata.hid_descriptor_address;
	ihid->wHIDDescRegister = cpu_to_le16(hidRegister);

	init_waitqueue_head(&ihid->wait);

	/* we need to allocate the command buffer without knowing the maximum
	 * size of the reports. Let's use HID_MIN_BUFFER_SIZE, then we do the
	 * real computation later. */
	ret = i2c_hid_alloc_buffers(ihid, HID_MIN_BUFFER_SIZE);
	if (ret < 0)
		goto err;

#ifdef ATML_MXT_HID_PACKET
      /*Start to reset*/
      if(platform_data && platform_data->rst_gpio && platform_data->irq_gpio){
	   dev_info(&client->dev, "Start device reset with pin:%d", platform_data->rst_gpio);
         mxt_touch_reset(ihid, false);
	}

	if(isInBootLoaderMode(client))
	    recovery_from_bootMode(client);
#endif      
	ret = i2c_hid_fetch_hid_descriptor(ihid);
	if (ret < 0)
		goto err;

#ifdef ATML_MXT_HID_PACKET
      dev_info(&client->dev, "Start get object configuration!\n");
      mutex_init(&ihid->access_mutex);
      ret = mxt_hid_initialize(ihid);
      if(ihid->info.version != HID_MXT_FW_VERSION){
          dev_info(&client->dev, " Start getting into MXT boot loader mode\n");
          mxt_hid_write_object(ihid, 6, 0x00, 0xA5);
          msleep(250);
	    isInBootLoaderMode(client);
          mxt_update_fw(client, HID_MXT_BL_ADDRESS, fw_V13_AC_20, sizeof(fw_V13_AC_20));
          mxt_touch_reset(ihid, true);
          kfree(ihid->object_table);
          ret = i2c_hid_fetch_hid_descriptor(ihid);
	    if (ret < 0)
              goto err;
          ret = mxt_hid_initialize(ihid);
      }
	mxt_hid_config_tp(ihid);
	/* Register Switch file */
      ihid->touch_sdev.name = "touch";
      ihid->touch_sdev.print_name = hid_touch_switch_name;
      ihid->touch_sdev.print_state = hid_touch_switch_state;
	if(switch_dev_register(&ihid->touch_sdev) < 0){
		dev_info(&client->dev, "switch_dev_register for dock failed!\n");
	}
	switch_set_state(&ihid->touch_sdev, 0);
#endif


	ret = i2c_hid_init_irq(client);
	if (ret < 0)
		goto err;

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		ret = PTR_ERR(hid);
		goto err_irq;
	}

	ihid->hid = hid;
	hid->driver_data = client;
	hid->ll_driver = &i2c_hid_ll_driver;
	hid->hid_get_raw_report = i2c_hid_get_raw_report;
	hid->hid_output_raw_report = i2c_hid_output_raw_report;
	hid->dev.parent = &client->dev;
	//ACPI_HANDLE_SET(&hid->dev, ACPI_HANDLE(&client->dev));
	hid->bus = BUS_I2C;
	hid->version = le16_to_cpu(ihid->hdesc.bcdVersion);
	hid->vendor = le16_to_cpu(ihid->hdesc.wVendorID);
	hid->product = le16_to_cpu(ihid->hdesc.wProductID);

	snprintf(hid->name, sizeof(hid->name), "atmel-maxtouch");
		// client->name, hid->vendor, hid->product);

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			hid_err(client, "can't add hid device: %d\n", ret);
		goto err_mem_free;
	}
#if defined(CONFIG_HAS_EARLYSUSPEND)
	ihid->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ihid->early_suspend.suspend = i2c_hid_early_suspend;
	ihid->early_suspend.resume = i2c_hid_early_resume;
	register_early_suspend(&ihid->early_suspend);
#endif

	return 0;

err_mem_free:
	hid_destroy_device(hid);

err_irq:
	free_irq(client->irq, ihid);

err:
	i2c_hid_free_buffers(ihid);
	kfree(ihid);
	return ret;
}

static int i2c_hid_remove(struct i2c_client *client)
{
	struct i2c_hid *ihid = i2c_get_clientdata(client);
	struct hid_device *hid;

	hid = ihid->hid;
	hid_destroy_device(hid);

	free_irq(client->irq, ihid);

	if (ihid->bufsize)
		i2c_hid_free_buffers(ihid);

	kfree(ihid);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void i2c_hid_early_suspend(struct early_suspend *es)
{
	struct i2c_hid *hid;
	struct device *dev;
	hid = container_of(es, struct i2c_hid, early_suspend);
	dev = &hid->client->dev;
	dev_info(dev, "MXT Early Suspend entered\n");

      disable_irq(hid->client->irq);

	/* Save some power */
	i2c_hid_set_power(hid->client, I2C_HID_PWR_SLEEP);
	
	dev_info(dev, "MXT Early Suspended\n");
}

static void i2c_hid_early_resume(struct early_suspend *es)
{
	struct i2c_hid *hid;
	struct device *dev;
	int ret;
	
	hid = container_of(es, struct i2c_hid, early_suspend);
	dev = &hid->client->dev;
	dev_info(dev, "MXT Early Resume entered\n");

      enable_irq(hid->client->irq);
      ret = i2c_hid_hwreset(hid->client);
	if (ret)
	    dev_err(&hid->client->dev, "%s: failed\n", __func__);
	dev_info(dev, "MXT Early Resumed\n");
}
#else
#ifdef CONFIG_PM_SLEEP
static int i2c_hid_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	/* Save some power */
	i2c_hid_set_power(client, I2C_HID_PWR_SLEEP);

	return 0;
}

static int i2c_hid_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = i2c_hid_hwreset(client);
	if (ret)
		return ret;

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#endif
#endif

static struct hid_device *hid_dev_backup = NULL;  //backup address
static struct urb *backup_urb = NULL;
static int idx=-1;
static int pkg_num = 0;
#define MAX_POINT		15
#define HID_DG_SCANTIME		0x000d0056	//new usage not defined in hid.h
#define REPORTID_TYPE1		0x30
#define REPORTID_01		0x01
#define REPORTID_10		0x10

static struct Point {
	u16 x, y, id, pressure, width, height;
};

static struct mxt_data {
	int id, total, ReportID, scantime;
	struct Point pt[MAX_POINT];
};

static int mxt_hid_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct mxt_data *nd;
   	u8 *rep_data = NULL;
	//hid_dev_backup = hdev;

	printk(KERN_INFO "%s\n", __func__);
	rep_data = kmalloc(3,GFP_KERNEL);	//return value will be 0xabcd
	nd = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!nd) {
		dev_err(&hdev->dev, "cannot allocate SiS 9200 data\n");
		kfree(rep_data);
		rep_data = NULL;
		return -ENOMEM;
	}
	
	hid_set_drvdata(hdev, nd);

	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		dev_err(&hdev->dev, "hw start failed\n");
		goto err_free;
	}

err_free:
	kfree(rep_data);
	rep_data = NULL;	
	return ret;
}

static void mxt_hid_remove(struct hid_device *hdev)
{
	dev_t dev;
	printk(KERN_INFO "%s\n", __func__);
}


static int mxt_raw_event (struct hid_device *hid, struct hid_report *report,
		                        u8 *raw_data, int size)
{
	struct mxt_data *nd = hid_get_drvdata(hid);
	nd->ReportID = raw_data[0];
	//printk(KERN_INFO "raw_event : ReportID = %d\n", nd->ReportID);
	hid_set_drvdata(hid, nd);
	return 0;
}

static int mxt_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	if (usage->type == EV_KEY || usage->type == EV_ABS)
		set_bit(usage->type, hi->input->evbit);

	return -1;
}

static void set_abs(struct input_dev *input, unsigned int code,
		struct hid_field *field, int snratio)
{
	int fmin = field->logical_minimum;
	int fmax = field->logical_maximum;
	int fuzz = snratio ? (fmax - fmin) / snratio : 0;
	input_set_abs_params(input, code, fmin, fmax, fuzz, 0);
}

static int mxt_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
      printk (KERN_INFO "%s : usage->hid:%x, l_min:%d, l_max:%d, count:%d\n", __func__, usage->hid,
	field->logical_minimum, field->logical_maximum, field->report_count);

	/* Only map fields from TouchScreen or TouchPad collections.
         * We need to ignore fields that belong to other collections
         * such as Mouse that might have the same GenericDesktop usages. */
	if (field->application == HID_DG_TOUCHSCREEN)
		set_bit(INPUT_PROP_DIRECT, hi->input->propbit);
	else if (field->application == HID_DG_TOUCHPAD)
		set_bit(INPUT_PROP_POINTER, hi->input->propbit);
	else
		return 0;

	switch (usage->hid & HID_USAGE_PAGE) {

	case HID_UP_GENDESK:
		switch (usage->hid) {
		case HID_GD_X:
              
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_POSITION_X);
			set_abs(hi->input, ABS_MT_POSITION_X, field,
				0);
			return 1;
		case HID_GD_Y:
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_POSITION_Y);
			set_abs(hi->input, ABS_MT_POSITION_Y, field,
				0);
			return 1;
		}
		return 0;

	case HID_UP_DIGITIZER:
		switch (usage->hid) {
		case HID_DG_INRANGE:
			
			return 1;
		case HID_DG_CONFIDENCE:
			
			return 1;
		case HID_DG_TIPSWITCH:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_PRESSURE);
			input_set_abs_params(hi->input, ABS_MT_PRESSURE, 0, 1, 0, 0);
			return 1;
		case HID_DG_CONTACTID:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TRACKING_ID);
			input_set_abs_params(hi->input, ABS_MT_TRACKING_ID, 0, 127, 0, 0);
			return 1;
		case HID_DG_WIDTH:
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_TOUCH_MAJOR);
			set_abs(hi->input, ABS_MT_TOUCH_MAJOR, field,
				0);
			return 1;
		case HID_DG_HEIGHT:
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_TOUCH_MINOR);
			set_abs(hi->input, ABS_MT_TOUCH_MINOR, field,
				0);
			input_set_abs_params(hi->input,
					ABS_MT_ORIENTATION, 0, 1, 0, 0);
			return 1;
		case HID_DG_TIPPRESSURE:
			hid_map_usage(hi, usage, bit, max,
					EV_ABS, ABS_MT_TOUCH_MAJOR);
			set_abs(hi->input, ABS_MT_TOUCH_MAJOR, field,
				0);
			return 1;
		case HID_DG_CONTACTCOUNT:
			return 1;
		case HID_DG_CONTACTMAX:
			return -1;
		}
		/* let hid-input decide for the others */
		return 0;

	case 0xff000000:
		/* we do not want to map these: no input-oriented meaning */
		return -1;
	}

	return 0;
}

static void mxt_event_emission(struct mxt_data *nd, struct input_dev *input)
{
	int i;
	bool all_touch_up = true;
	for(i=0; i< nd->total; i++)
	{
		//printk(KERN_INFO "MT_event: finger(s)=%d, id=%d, x=%d, y=%d, pressure=%d, width=%d\n", nd->total, nd->pt[i].id, nd->pt[i].x, nd->pt[i].y, nd->pt[i].pressure, nd->pt[i].width);
		//printk(KERN_INFO "MT_event: pressure=%d, width=%d, height=%d, \n", nd->pt[i].pressure, nd->pt[i].width, nd->pt[i].height);
		if(nd->pt[i].pressure)
		{
			input_report_abs(input, ABS_MT_PRESSURE, nd->pt[i].pressure);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, nd->pt[i].width);
			input_report_abs(input, ABS_MT_POSITION_X, nd->pt[i].x);
			input_report_abs(input, ABS_MT_POSITION_Y, nd->pt[i].y);
			input_report_abs(input, ABS_MT_TRACKING_ID, nd->pt[i].id);
			input_mt_sync(input);
			all_touch_up = false;
		}

		if(i == (nd->total - 1) && all_touch_up == true)
			input_mt_sync(input);
	}
	input_report_key(input, BTN_TOUCH, !all_touch_up);
	input_sync(input);
}

static void mxt_event_clear(struct mxt_data *nd, int max)
{
	int i;
	for(i=0; i<max; i++)
	{
		nd->pt[i].id = 0;
		nd->pt[i].x = 0;
		nd->pt[i].y = 0;
		nd->pt[i].pressure = 0;
		nd->pt[i].width = 0;
		nd->pt[i].height = 0;
	}
	nd->scantime = 0;
	idx = -1;
	pkg_num = 0;
}

static void mxt_event_lastdata(struct hid_device *hid, struct mxt_data *nd, struct input_dev *input)
{
     mxt_event_emission(nd, input);
     mxt_event_clear(nd, MAX_POINT);
}

static int mxt_event (struct hid_device *hid, struct hid_field *field,
		                        struct hid_usage *usage, __s32 value)
{
	struct mxt_data *nd = hid_get_drvdata(hid);

        if (hid->claimed & HID_CLAIMED_INPUT) {
		//printk (KERN_INFO "%s : usage->hid = %x value=%x\n", __func__, usage->hid, value);
		struct input_dev *input = field->hidinput->input;
		switch (usage->hid) {
		case HID_DG_INRANGE:			
			break;

		case HID_DG_TIPSWITCH:
			idx++;
			if (value > 1 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : TipSwitch value error : %d, idx = %d", value, idx);
				value = 1;
			}
			nd->pt[idx].pressure = !!value;
			break;

		case HID_DG_CONTACTID:
			if (value > 30 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : Contact ID value error : %d, idx = %d", value, idx);
				value = 30;
			}
			nd->pt[idx].id = value;
			break;

		case HID_GD_X:
			if (value > 4095 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : X-axis value error : %d, idx = %d", value, idx);
				value = 4095;
			}
			nd->pt[idx].x = value;
			break;

		case HID_GD_Y:
			if (value > 4095 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : Y-axis value error : %d, idx = %d", value, idx);
				value = 4095;
			}
			nd->pt[idx].y = value;
			break;

		//new usage for SiS817 Extend Class Device
		case HID_DG_SCANTIME:
			if (value > 65535 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : scantime value error : %d, idx = %d", value, idx);
				value = 65535;
			}
			nd->scantime = value;
			if ( (nd->ReportID & 0xf0) > REPORTID_TYPE1 )
				mxt_event_lastdata(hid, nd, input);
			break;

		case HID_DG_WIDTH:
			nd->pt[idx].width = value;
			break;

		case HID_DG_HEIGHT:
			nd->pt[idx].height = value;
			break;
		
		case HID_DG_TIPPRESSURE:
			nd->pt[idx].width = value;
			break;
		//end of new usage for SiS817 Extend Class Device

		case HID_DG_CONTACTCOUNT:
			if (value > 10 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : ContactCount value error : %d, idx = %d", value, idx);
			}
			if(value) nd->total = value;
			//if ( (nd->ReportID & 0xf0) <= REPORTID_TYPE1 )
			if(idx == nd->total -1)
			    mxt_event_lastdata(hid, nd, input);
			break;			
		default:
			//fallback to the generic hidinput handling
			return 0;
		}
	}

	/* we have handled the hidinput part, now remains hiddev */
	if (hid->claimed & HID_CLAIMED_HIDDEV && hid->hiddev_hid_event)
	{
		//printk (KERN_INFO "!!!!!!!	sis_event : hid->hiddev_hid_event : idx = %d", idx);
		hid->hiddev_hid_event(hid, field, usage, value);
	}

	return 1;
}

static const struct hid_device_id mxt_devices[] = {
	{ HID_DEVICE(BUS_I2C, USB_VENDOR_ID_ATMEL,
			HID_ANY_ID)},     // 0x03EB, 0x212C
	{ }
};
MODULE_DEVICE_TABLE(hid, mxt_devices);



static struct hid_driver mxt_hid_driver = {
	.name = "mxt_hid",
	.id_table = mxt_devices,
	.probe = mxt_hid_probe,
	.remove = mxt_hid_remove,
	.raw_event = mxt_raw_event,
	.input_mapped = mxt_input_mapped,
	.input_mapping = mxt_input_mapping,
	.event = mxt_event,
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
#else
static SIMPLE_DEV_PM_OPS(i2c_hid_pm, i2c_hid_suspend, i2c_hid_resume);
#endif

static const struct i2c_device_id i2c_hid_id_table[] = {
	{ "i2c_hid", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, i2c_hid_id_table);



static struct i2c_driver i2c_hid_driver = {
	.driver = {
		.name	= "i2c_hid",
		.owner	= THIS_MODULE,
#if defined(CONFIG_HAS_EARLYSUSPEND)
#else
		.pm	= &i2c_hid_pm,
#endif
		//.acpi_match_table = ACPI_PTR(i2c_hid_acpi_match),
	},

	.probe		= i2c_hid_probe,
	.remove		= i2c_hid_remove,

	.id_table	= i2c_hid_id_table,
};

module_i2c_driver(i2c_hid_driver);


static int __init i2c_hid_init(void)
{
	return hid_register_driver(&mxt_hid_driver);
	//return 0;
}

static void __exit i2c_hid_exit(void)
{
	hid_unregister_driver(&mxt_hid_driver);
}

module_init(i2c_hid_init);
module_exit(i2c_hid_exit);

MODULE_DESCRIPTION("HID over I2C core driver");
MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_LICENSE("GPL");
