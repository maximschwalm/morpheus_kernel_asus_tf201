/*
 * arch/arm/mach-tegra/board-cardhu-sensors.c
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_I2C_MUX_PCA954x
#include <linux/i2c/pca954x.h>
#endif
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <mach/fb.h>
#include <mach/gpio.h>
#ifdef CONFIG_VIDEO_OV5650
#include <media/ov5650.h>
#include <media/ov5640.h>
#include <media/ov14810.h>
#endif
#include <media/imx175.h>
#include <media/dw9714.h>
#ifdef CONFIG_VIDEO_OV2710
#include <media/ov2710.h>
#endif
#ifdef CONFIG_VIDEO_YUV
#include <media/yuv_sensor.h>
#endif /* CONFIG_VIDEO_YUV */
#include <media/tps61050.h>
#include <generated/mach-types.h>
#include "gpio-names.h"
#include "board.h"
#include <linux/mpu.h>

/* KIONIX KXT_9 Digital Tri-axis Accelerometer */
//#include <plat/mux.h>
#include <linux/kxt_9.h>

#ifdef CONFIG_VIDEO_SH532U
#include <media/sh532u.h>
#endif
#include <linux/bq27x00.h>
#include <mach/gpio.h>
#include <mach/edp.h>
#include <mach/thermal.h>
#include <linux/therm_est.h>

#include "gpio-names.h"
#include "board-cardhu.h"
#include "cpu-tegra.h"

#include <mach/board-cardhu-misc.h>
#include <linux/smb347-charger.h>
//leon add for cm3218
#include <linux/cm3218.h>

#if 0 //WK: Disable NV's camera code
static struct regulator *cardhu_1v8_cam1 = NULL;
static struct regulator *cardhu_1v8_cam2 = NULL;
static struct regulator *cardhu_1v8_cam3 = NULL;
static struct regulator *cardhu_vdd_2v8_cam1 = NULL;
static struct regulator *cardhu_vdd_2v8_cam2 = NULL;
static struct regulator *cardhu_vdd_cam3 = NULL;
#endif

static struct board_info board_info;
static struct regulator *reg_cardhu_cam;	/* LDO6 */
static struct regulator *reg_cardhu_1v8_cam;	/* VDDIO_CAM 1.8V PBB4 */
static struct regulator *reg_cardhu_2v85_cam;	/* Front Camera 2.85V power */
static struct regulator *reg_cardhu_1v2_cam;	/* VDDIO_CAM 1.2V PS0 */
static struct regulator *reg_cardhu_af_pwr_en;	/* ICATCH7002A_AF_PWR_EN_GPIO PS0 */
static struct regulator *reg_cardhu_vdda_en;	/* ICATCH7002A_VDDA_EN_GPIO GPIO_PR6*/
static struct regulator *reg_cardhu_vddio_cam;	/* LDO5 It's only for ME301T */
static bool camera_busy = false;

#ifdef CONFIG_I2C_MUX_PCA954x
static struct pca954x_platform_mode cardhu_pca954x_modes[] = {
	{ .adap_id = PCA954x_I2C_BUS0, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS1, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS2, .deselect_on_exit = true, },
	{ .adap_id = PCA954x_I2C_BUS3, .deselect_on_exit = true, },
};

static struct pca954x_platform_data cardhu_pca954x_data = {
	.modes    = cardhu_pca954x_modes,
	.num_modes      = ARRAY_SIZE(cardhu_pca954x_modes),
};
#endif

static int IsTF300(void)
{
    u32 project_info = tegra3_get_project_id();

    if (project_info == TEGRA3_PROJECT_TF300T)
        return 1;
    else if (project_info == TEGRA3_PROJECT_TF300TG)
        return 1;
    else if (project_info == TEGRA3_PROJECT_TF300TL)
        return 1;
    else
        return 0;
}

static int cardhu_camera_init(void)
{

    pr_info("cardhu_camera_init \n");
    
    gpio_request(TEGRA_GPIO_PS1, "LED_PIN");
    gpio_request(TEGRA_GPIO_PH1, "gpio_ph1"); //Jimmy
    gpio_request(ISP_POWER_1V2_EN_GPIO, "isp_power_1v2_en");
    gpio_request(ISP_POWER_RESET_GPIO, "isp_power_rstx");
    gpio_request(CAM3_POWER_DWN_GPIO, "cam3_power_dwn");
    gpio_request(FRONT_YUV_SENSOR_RST_GPIO, "yuv_sensor_rst_lo");

    return 0;
}

#ifdef CONFIG_VIDEO_YUV
#if 0
#ifdef CONFIG_VIDEO_IMX175
static int cardhu_imx175_power_put(struct imx175_power_rail *pw)
{
	if (pw->vana) {
		regulator_put(pw->vana);
		pw->vana = NULL;
	}
	if (pw->vdig) {
		regulator_put(pw->vdig);
		pw->vdig = NULL;
	}
	if (pw->vddl) {
		regulator_put(pw->vddl);
		pw->vddl = NULL;
	}
	tegra_gpio_disable(pw->gpio_reset);
	return 0;
}
static int cardhu_imx175_power_get(struct imx175_power_rail *pw)
{
	int ret;
	pw->gpio_reset = TEGRA_GPIO_PBB0;
	tegra_gpio_enable(pw->gpio_reset);
	ret = gpio_request(pw->gpio_reset, "imx175_reset");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "IMX175_RESETN_GPIO");
		pw->gpio_reset = ARCH_NR_GPIOS;
		return -ENODEV;
	}
	if (pw->vana == NULL) {
		pw->vana = regulator_get(NULL, "vdd_2v8_cam1");
		if (WARN_ON(IS_ERR(pw->vana))) {
			pr_err("%s: couldn't get regulator vana: %ld\n",
				__func__, PTR_ERR(pw->vana));
			pw->vana = NULL;
			goto reg_alloc_fail;
		}
	}
	if (pw->vdig == NULL) {
		pw->vdig = regulator_get(NULL, "vdd_1v8_cam1");
		if (WARN_ON(IS_ERR(pw->vdig))) {
			pr_err("%s: couldn't get regulator vdig: %ld\n",
				__func__, PTR_ERR(pw->vdig));
			pw->vdig = NULL;
			goto reg_alloc_fail;
		}
	}
	if (pw->vddl == NULL) {
		pw->vddl = regulator_get(NULL, "vcore_cam1");
		if (WARN_ON(IS_ERR(pw->vddl))) {
			pr_err("%s: couldn't get regulator vddl: %ld\n",
				__func__, PTR_ERR(pw->vddl));
			pw->vddl = NULL;
			goto reg_alloc_fail;
		}
	}
	return 0;
reg_alloc_fail:
	cardhu_imx175_power_put(pw);
	return -ENODEV;
}
static int cardhu_imx175_power_on(void)
{
	return 0;
}
static int cardhu_imx175_power_off(void)
{
	return 0;
}
struct imx175_platform_data cardhu_imx175_data = {
	.power_get = cardhu_imx175_power_get,
	.power_put = cardhu_imx175_power_put,
	.power_on = cardhu_imx175_power_on,
	.power_off = cardhu_imx175_power_off,
};
static const struct i2c_board_info cardhu_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("imx175", 0x10),
		.platform_data = &cardhu_imx175_data,
	},
};
#endif
#endif //#if 0--
#if 0
//Jimmy add for dw9714 power on/off ++++++
static int vcm_sensor_power_on(void)
{
     gpio_direction_output(TEGRA_GPIO_PR6, 1);
     pr_info("gpio 2.85v %d set to %d\n",TEGRA_GPIO_PR6, gpio_get_value(TEGRA_GPIO_PR6));
     return 0;
}
static int vcm_sensor_power_off(void)
{
     pr_info("gpio 2.85v %d set to %d\n",TEGRA_GPIO_PR6, gpio_get_value(TEGRA_GPIO_PR6));
     gpio_direction_output(TEGRA_GPIO_PH1, 0);
     return 0;
}
//Jimmy add for dw9714 power on/off ------
#endif
static int yuv_front_sensor_power_on(void)
{
	printk("Bayer_front_sensor_power_on+\n");

	/* 1.8V VDDIO_CAM controlled by "EN_1V8_CAM(GPIO_PBB4)" */
	if (!reg_cardhu_1v8_cam) {
		reg_cardhu_1v8_cam = regulator_get(NULL, "vdd_1v8_cam1"); /*cam2/3?*/
		if (IS_ERR_OR_NULL(reg_cardhu_1v8_cam)) {
			reg_cardhu_1v8_cam = NULL;
			pr_err("Can't get reg_cardhu_1v8_cam.\n");
			goto fail_to_get_reg;
		}
		regulator_set_voltage(reg_cardhu_1v8_cam, 1800000, 1800000);
		regulator_enable(reg_cardhu_1v8_cam);
	}
    pr_info("reg_cardhu_1v8_cam 1.8v set to %d\n",regulator_get_voltage(reg_cardhu_1v8_cam)); //jimmy add
    msleep(1);

  	/* 2.85V VDD_CAM2 controlled by CAM2/3_LDO_EN(GPIO_PS0)*/
  	if (!reg_cardhu_2v85_cam) {
  		reg_cardhu_2v85_cam = regulator_get(NULL, "vdd_cam3");
  		if (IS_ERR_OR_NULL(reg_cardhu_2v85_cam)) {
  			reg_cardhu_2v85_cam = NULL;
  			pr_err("Can't get reg_cardhu_2v85_cam.\n");
  			goto fail_to_get_reg;
  		}
  		regulator_set_voltage(reg_cardhu_2v85_cam, 2850000, 2850000);
  		regulator_enable(reg_cardhu_2v85_cam);
  	}
  	msleep(5);

	/* cam_power_en, powdn*/
	pr_info("Ryant: %s(%d): gpio %d: %d",__func__, __LINE__,TEGRA_GPIO_PBB7, gpio_get_value(TEGRA_GPIO_PBB7));
	gpio_set_value(TEGRA_GPIO_PBB7, 0);
	gpio_direction_output(TEGRA_GPIO_PBB7, 0);
	pr_info("--> %d\n", gpio_get_value(CAM3_POWER_DWN_GPIO));

	/* bayer_sensor_rst_lo*/
	pr_info("adogu: %s(%d): gpio %d: %d", __func__,__LINE__,TEGRA_GPIO_PO0, gpio_get_value(TEGRA_GPIO_PO0));
	gpio_set_value(TEGRA_GPIO_PO0, 1);
	gpio_direction_output(TEGRA_GPIO_PO0, 1);
	pr_info("--> %d\n", gpio_get_value(TEGRA_GPIO_PO0));

	printk("bayer_front_sensor_power_on-\n");
	return 0;

fail_to_get_reg:
	if (reg_cardhu_2v85_cam) {
		regulator_put(reg_cardhu_2v85_cam);
		reg_cardhu_2v85_cam = NULL;
	}
	if (reg_cardhu_1v8_cam) {
		regulator_put(reg_cardhu_1v8_cam);
		reg_cardhu_1v8_cam = NULL;
	}

	camera_busy = false;
	printk("bayer_front_sensor_power_on- : -ENODEV\n");
	return -ENODEV;
}

static int yuv_front_sensor_power_off(void)
{
	printk("yuv_front_sensor_power_off+\n");

	gpio_set_value(TEGRA_GPIO_PO0, 0);
	gpio_direction_output(TEGRA_GPIO_PO0, 0);

	gpio_set_value(TEGRA_GPIO_PBB7, 1);
	gpio_direction_output(TEGRA_GPIO_PBB7, 1);

	if (reg_cardhu_2v85_cam) {
		regulator_disable(reg_cardhu_2v85_cam);
		regulator_put(reg_cardhu_2v85_cam);
		reg_cardhu_2v85_cam = NULL;
	}
	if (reg_cardhu_1v8_cam) {
		regulator_disable(reg_cardhu_1v8_cam);
		regulator_put(reg_cardhu_1v8_cam);
		reg_cardhu_1v8_cam = NULL;
	}

	camera_busy = false;
	printk("yuv_front_sensor_power_off-\n");
	return 0;
}
static int yuv_sensor_power_on(void)
{
	/*Ryant work around can't open camera issue*/
	yuv_front_sensor_power_on();
	yuv_front_sensor_power_off();
	
	printk("yuv_sensor_power_on++\n");
    pr_info("gpio LED pin 145 set to 0\n");
    gpio_direction_output(TEGRA_GPIO_PS1, 1);

    pr_info("gpio camera WP pin %d set to %d\n",TEGRA_GPIO_PS5, gpio_get_value(TEGRA_GPIO_PS5));
    gpio_direction_output(TEGRA_GPIO_PS5, 1);
	// GPIO_PBB4 EN_1V8_CAM
    // gpio_direction_output(TEGRA_GPIO_PR6, 1);
    // pr_info("gpio 2.85v %d set to %d\n",TEGRA_GPIO_PR6, gpio_get_value(TEGRA_GPIO_PR6));
    gpio_direction_output(TEGRA_GPIO_PH1, 1);
    pr_info("gpio 2.7v %d set to %d\n",TEGRA_GPIO_PH1, gpio_get_value(TEGRA_GPIO_PH1));
    msleep(5);
    if (!reg_cardhu_1v8_cam) {
		reg_cardhu_1v8_cam = regulator_get(NULL, "vdd_1v8_cam1");
        if (IS_ERR_OR_NULL(reg_cardhu_1v8_cam)) {
			pr_err("TF201_m6mo_power_on PBB4: vdd_1v8_cam1 failed\n");
            reg_cardhu_1v8_cam = NULL;
            return PTR_ERR(reg_cardhu_1v8_cam);
        }
        regulator_set_voltage(reg_cardhu_1v8_cam, 1800000, 1800000);
        regulator_enable(reg_cardhu_1v8_cam);
        pr_info("gpio 1.8v set to %d\n",regulator_get_voltage(reg_cardhu_1v8_cam));
    }
    gpio_direction_output(TEGRA_GPIO_PBB0, 1);
    pr_info("gpio camera reset pin %d set to %d\n",TEGRA_GPIO_PBB0, gpio_get_value(TEGRA_GPIO_PBB0));
    return 0;
}

static int yuv_sensor_power_off(void)
{
	pr_info("gpio LED pin 145 set to 0\n");
    gpio_direction_output(TEGRA_GPIO_PS1, 0);

    if(reg_cardhu_1v8_cam){
		regulator_disable(reg_cardhu_1v8_cam);
        regulator_put(reg_cardhu_1v8_cam);
        reg_cardhu_1v8_cam = NULL;
    }
    //pr_info("gpio 2.85v %d set to %d\n",TEGRA_GPIO_PR6, gpio_get_value(TEGRA_GPIO_PR6));
    gpio_direction_output(TEGRA_GPIO_PH1, 0);
    pr_info("gpio 2.7v %d set to %d\n",TEGRA_GPIO_PH1, gpio_get_value(TEGRA_GPIO_PH1));
    gpio_direction_output(TEGRA_GPIO_PBB0, 0);
    pr_info("gpio camera reset pin %d set to %d\n",TEGRA_GPIO_PBB0, gpio_get_value(TEGRA_GPIO_PBB0));
    gpio_direction_output(TEGRA_GPIO_PS5, 0);
    printk("Bayer_sensor_power_off--\n");

    return 0;
}


struct yuv_sensor_platform_data yuv_front_sensor_data = {
	.power_on = yuv_front_sensor_power_on,
	.power_off = yuv_front_sensor_power_off,
};


#endif  /* CONFIG_VIDEO_YUV */

#ifdef CONFIG_VIDEO_OV5650
static int cardhu_left_ov5650_power_on(void)
{
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {

		if (cardhu_vdd_2v8_cam1 == NULL) {
			cardhu_vdd_2v8_cam1 = regulator_get(NULL, "vdd_2v8_cam1");
			if (WARN_ON(IS_ERR(cardhu_vdd_2v8_cam1))) {
				pr_err("%s: couldn't get regulator vdd_2v8_cam1: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_2v8_cam1));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_2v8_cam1);
		mdelay(5);
	}

	/* Enable VDD_1V8_Cam1 */
	if (cardhu_1v8_cam1 == NULL) {
		cardhu_1v8_cam1 = regulator_get(NULL, "vdd_1v8_cam1");
		if (WARN_ON(IS_ERR(cardhu_1v8_cam1))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam1: %ld\n",
				__func__, PTR_ERR(cardhu_1v8_cam1));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(cardhu_1v8_cam1);

	mdelay(5);
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		mdelay(20);
		gpio_direction_output(OV5650_RESETN_GPIO, 0);
		mdelay(100);
		gpio_direction_output(OV5650_RESETN_GPIO, 1);
	}

	if (board_info.board_id == BOARD_PM269) {
		gpio_direction_output(CAM1_RST_L_GPIO, 0);
		mdelay(100);
		gpio_direction_output(CAM1_RST_L_GPIO, 1);
	}

	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam1) {
		regulator_put(cardhu_1v8_cam1);
		cardhu_1v8_cam1 = NULL;
	}
	if (cardhu_vdd_2v8_cam1) {
		regulator_put(cardhu_vdd_2v8_cam1);
		cardhu_vdd_2v8_cam1 = NULL;
	}

	return -ENODEV;

}

static int cardhu_left_ov5650_power_off(void)
{
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}
	if (cardhu_1v8_cam1)
		regulator_disable(cardhu_1v8_cam1);
	if (cardhu_vdd_2v8_cam1)
		regulator_disable(cardhu_vdd_2v8_cam1);

	return 0;
}

struct ov5650_platform_data cardhu_left_ov5650_data = {
	.power_on = cardhu_left_ov5650_power_on,
	.power_off = cardhu_left_ov5650_power_off,
};

#ifdef CONFIG_VIDEO_OV14810
static int cardhu_ov14810_power_on(void)
{
	if (board_info.board_id == BOARD_E1198) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		mdelay(20);
		gpio_direction_output(OV14810_RESETN_GPIO, 0);
		mdelay(100);
		gpio_direction_output(OV14810_RESETN_GPIO, 1);
	}

	return 0;
}

static int cardhu_ov14810_power_off(void)
{
	if (board_info.board_id == BOARD_E1198) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}

	return 0;
}

struct ov14810_platform_data cardhu_ov14810_data = {
	.power_on = cardhu_ov14810_power_on,
	.power_off = cardhu_ov14810_power_off,
};

struct ov14810_platform_data cardhu_ov14810uC_data = {
	.power_on = NULL,
	.power_off = NULL,
};
/*
struct ov14810_platform_data cardhu_ov14810SlaveDev_data = {
	.power_on = NULL,
	.power_off = NULL,
};

static struct i2c_board_info cardhu_i2c_board_info_e1214[] = {
	{
		I2C_BOARD_INFO("ov14810", 0x36),
		.platform_data = &cardhu_ov14810_data,
	},
	{
		I2C_BOARD_INFO("ov14810uC", 0x67),
		.platform_data = &cardhu_ov14810uC_data,
	},
	{
		I2C_BOARD_INFO("ov14810SlaveDev", 0x69),
		.platform_data = &cardhu_ov14810SlaveDev_data,
	}
};
*/
#endif

static int cardhu_right_ov5650_power_on(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {

		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
		mdelay(10);

		if (cardhu_vdd_2v8_cam2 == NULL) {
			cardhu_vdd_2v8_cam2 = regulator_get(NULL, "vdd_2v8_cam2");
			if (WARN_ON(IS_ERR(cardhu_vdd_2v8_cam2))) {
				pr_err("%s: couldn't get regulator vdd_2v8_cam2: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_2v8_cam2));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_2v8_cam2);
		mdelay(5);
	}

	/* Enable VDD_1V8_Cam2 */
	if (cardhu_1v8_cam2 == NULL) {
		cardhu_1v8_cam2 = regulator_get(NULL, "vdd_1v8_cam2");
		if (WARN_ON(IS_ERR(cardhu_1v8_cam2))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam2: %ld\n",
				__func__, PTR_ERR(cardhu_1v8_cam2));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(cardhu_1v8_cam2);

	mdelay(5);

	if (board_info.board_id == BOARD_PM269) {
		gpio_direction_output(CAM2_RST_L_GPIO, 0);
		mdelay(100);
		gpio_direction_output(CAM2_RST_L_GPIO, 1);
	}

	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam2) {
		regulator_put(cardhu_1v8_cam2);
		cardhu_1v8_cam2 = NULL;
	}
	if (cardhu_vdd_2v8_cam2) {
		regulator_put(cardhu_vdd_2v8_cam2);
		cardhu_vdd_2v8_cam2 = NULL;
	}

	return -ENODEV;

}

static int cardhu_right_ov5650_power_off(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 0);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and do not have TCA6416 for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}

	if (cardhu_1v8_cam2)
		regulator_disable(cardhu_1v8_cam2);
	if (cardhu_vdd_2v8_cam2)
		regulator_disable(cardhu_vdd_2v8_cam2);

	return 0;
}

static void cardhu_ov5650_synchronize_sensors(void)
{
	if (board_info.board_id == BOARD_E1198) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		mdelay(50);
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		mdelay(50);
	}
	else if (board_info.board_id == BOARD_E1291) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		mdelay(50);
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
		mdelay(50);
	}
	else
		pr_err("%s: UnSupported BoardId\n", __func__);
}

struct ov5650_platform_data cardhu_right_ov5650_data = {
	.power_on = cardhu_right_ov5650_power_on,
	.power_off = cardhu_right_ov5650_power_off,
	.synchronize_sensors = cardhu_ov5650_synchronize_sensors,
};
#endif

#ifdef CONFIG_VIDEO_OV2710

static int cardhu_ov2710_power_on(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);

	/* Enable VDD_1V8_Cam3 */
	if (cardhu_1v8_cam3 == NULL) {
		cardhu_1v8_cam3 = regulator_get(NULL, "vdd_1v8_cam3");
		if (WARN_ON(IS_ERR(cardhu_1v8_cam3))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam3: %ld\n",
				__func__, PTR_ERR(cardhu_1v8_cam3));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(cardhu_1v8_cam3);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and do not have TCA6416 for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		if (cardhu_vdd_cam3 == NULL) {
			cardhu_vdd_cam3 = regulator_get(NULL, "vdd_cam3");
			if (WARN_ON(IS_ERR(cardhu_vdd_cam3))) {
				pr_err("%s: couldn't get regulator vdd_cam3: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_cam3));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_cam3);

		mdelay(5);

		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 0);
		mdelay(10);

	}

	mdelay(20);

	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam3) {
		regulator_put(cardhu_1v8_cam3);
		cardhu_1v8_cam3 = NULL;
	}
	if (cardhu_vdd_cam3) {
		regulator_put(cardhu_vdd_cam3);
		cardhu_vdd_cam3 = NULL;
	}

	return -ENODEV;
}

static int cardhu_ov2710_power_off(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
		if (cardhu_vdd_cam3)
			regulator_disable(cardhu_vdd_cam3);
	}

	if (cardhu_1v8_cam3)
		regulator_disable(cardhu_1v8_cam3);

	return 0;
}

struct ov2710_platform_data cardhu_ov2710_data = {
	.power_on = cardhu_ov2710_power_on,
	.power_off = cardhu_ov2710_power_off,
};

static int cardhu_ov5640_power_on(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {

		gpio_direction_output(CAM1_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 0);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 0);
		mdelay(10);

		if (cardhu_vdd_cam3 == NULL) {
			cardhu_vdd_cam3 = regulator_get(NULL, "vdd_cam3");
			if (WARN_ON(IS_ERR(cardhu_vdd_cam3))) {
				pr_err("%s: couldn't get regulator vdd_cam3: %ld\n",
					__func__, PTR_ERR(cardhu_vdd_cam3));
				goto reg_alloc_fail;
			}
		}
		regulator_enable(cardhu_vdd_cam3);
	}

	/* Enable VDD_1V8_Cam3 */
	if (cardhu_1v8_cam3 == NULL) {
		cardhu_1v8_cam3 = regulator_get(NULL, "vdd_1v8_cam3");
		if (WARN_ON(IS_ERR(cardhu_1v8_cam3))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam3: %ld\n",
				__func__, PTR_ERR(cardhu_1v8_cam3));
			goto reg_alloc_fail;
		}
	}
	regulator_enable(cardhu_1v8_cam3);
	mdelay(5);

	return 0;

reg_alloc_fail:
	if (cardhu_1v8_cam3) {
		regulator_put(cardhu_1v8_cam3);
		cardhu_1v8_cam3 = NULL;
	}
	if (cardhu_vdd_cam3) {
		regulator_put(cardhu_vdd_cam3);
		cardhu_vdd_cam3 = NULL;
	}

	return -ENODEV;
}

static int cardhu_ov5640_power_off(void)
{
	/* CSI-B and front sensor are muxed on cardhu */
	gpio_direction_output(CAMERA_CSI_MUX_SEL_GPIO, 1);

	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291)) {
		gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM2_POWER_DWN_GPIO, 1);
		gpio_direction_output(CAM3_POWER_DWN_GPIO, 1);
	}

	if (cardhu_1v8_cam3)
		regulator_disable(cardhu_1v8_cam3);
	if (cardhu_vdd_cam3)
		regulator_disable(cardhu_vdd_cam3);

	return 0;
}

struct ov5640_platform_data cardhu_ov5640_data = {
	.power_on = cardhu_ov5640_power_on,
	.power_off = cardhu_ov5640_power_off,
};

static const struct i2c_board_info cardhu_i2c3_board_info[] = {
	{
		I2C_BOARD_INFO("pca9546", 0x70),
		.platform_data = &cardhu_pca954x_data,
	},
};


static struct nvc_gpio_pdata sh532u_gpio_pdata[] = {
	{ SH532U_GPIO_RESET, TEGRA_GPIO_PBB0, false, 0, },
};

static struct sh532u_platform_data sh532u_left_pdata = {
	.cfg		= NVC_CFG_NODEV,
	.num		= 1,
	.sync		= 2,
	.dev_name	= "focuser",
	.gpio_count	= ARRAY_SIZE(sh532u_gpio_pdata),
	.gpio		= sh532u_gpio_pdata,
};

static struct sh532u_platform_data sh532u_right_pdata = {
	.cfg		= NVC_CFG_NODEV,
	.num		= 2,
	.sync		= 1,
	.dev_name	= "focuser",
	.gpio_count	= ARRAY_SIZE(sh532u_gpio_pdata),
	.gpio		= sh532u_gpio_pdata,
};

static struct nvc_gpio_pdata pm269_sh532u_left_gpio_pdata[] = {
	{ SH532U_GPIO_RESET, CAM1_RST_L_GPIO, false, 0, },
};

static struct sh532u_platform_data pm269_sh532u_left_pdata = {
	.cfg		= 0,
	.num		= 1,
	.sync		= 2,
	.dev_name	= "focuser",
	.gpio_count	= ARRAY_SIZE(pm269_sh532u_left_gpio_pdata),
	.gpio		= pm269_sh532u_left_gpio_pdata,
};

static struct nvc_gpio_pdata pm269_sh532u_right_gpio_pdata[] = {
	{ SH532U_GPIO_RESET, CAM2_RST_L_GPIO, false, 0, },
};

static struct sh532u_platform_data pm269_sh532u_right_pdata = {
	.cfg		= 0,
	.num		= 2,
	.sync		= 1,
	.dev_name	= "focuser",
	.gpio_count	= ARRAY_SIZE(pm269_sh532u_right_gpio_pdata),
	.gpio		= pm269_sh532u_right_gpio_pdata,
};
#endif

static struct nvc_torch_pin_state cardhu_tps61050_pinstate = {
	.mask		= 0x0008, /*VGP3*/
	.values		= 0x0008,
};
static struct tps61050_platform_data cardhu_tps61050_pdata = {
	.dev_name	= "torch",
	.pinstate	= &cardhu_tps61050_pinstate,
};

static const struct i2c_board_info cardhu_i2c_board_info_tps61050[] = {
	{
		I2C_BOARD_INFO("tps61050", 0x33),
		.platform_data = &cardhu_tps61050_pdata,
	},
};

#ifdef CONFIG_VIDEO_OV5650

static struct i2c_board_info cardhu_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650L", 0x36),
		.platform_data = &cardhu_left_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_left_pdata,
	},
};

static struct i2c_board_info cardhu_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650R", 0x36),
		.platform_data = &cardhu_right_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &sh532u_right_pdata,
	},
};

static struct i2c_board_info pm269_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650L", 0x36),
		.platform_data = &cardhu_left_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &pm269_sh532u_left_pdata,
	},
};

static struct i2c_board_info pm269_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("ov5650R", 0x36),
		.platform_data = &cardhu_right_ov5650_data,
	},
	{
		I2C_BOARD_INFO("sh532u", 0x72),
		.platform_data = &pm269_sh532u_right_pdata,
	},
};
#endif

#ifdef CONFIG_VIDEO_OV2710
static struct i2c_board_info cardhu_i2c8_board_info[] = {
	{
		I2C_BOARD_INFO("ov2710", 0x36),
		.platform_data = &cardhu_ov2710_data,
	},
	{
		I2C_BOARD_INFO("ov5640", 0x3C),
		.platform_data = &cardhu_ov5640_data,
	},
};
#endif

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int nct_get_itemp(void *dev_data, long *temp)
{
	struct nct1008_data *data = dev_data;
	return nct1008_thermal_get_temps(data, NULL, temp);
}
#endif

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *ext_nct;

	ext_nct = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!ext_nct) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	ext_nct->name = "nct_ext";
	ext_nct->id = THERMAL_DEVICE_ID_NCT_EXT;
	ext_nct->data = data;
	ext_nct->offset = TDIODE_OFFSET;
	ext_nct->get_temp = nct_get_temp;
	ext_nct->get_temp_low = nct_get_temp_low;
	ext_nct->set_limits = nct_set_limits;
	ext_nct->set_alert = nct_set_alert;
	ext_nct->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(ext_nct);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		struct tegra_thermal_device *int_nct;
		int_nct = kzalloc(sizeof(struct tegra_thermal_device),
						GFP_KERNEL);
		if (!int_nct) {
			kfree(int_nct);
			pr_err("unable to allocate thermal device\n");
			return;
		}

		int_nct->name = "nct_int";
		int_nct->id = THERMAL_DEVICE_ID_NCT_INT;
		int_nct->data = data;
		int_nct->get_temp = nct_get_itemp;

		tegra_thermal_device_register(int_nct);
	}
#endif
}

static struct nct1008_platform_data cardhu_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
	.probe_callback = nct1008_probe_callback,
};

static struct i2c_board_info cardhu_i2c4_bq27510_board_info[] = {
	{
		I2C_BOARD_INFO("bq27510", 0x55),
	},
};

static struct i2c_board_info cardhu_i2c4_bq27541_board_info[] = {
	{
		I2C_BOARD_INFO("bq27541-battery", 0x55),
	},
};

static struct i2c_board_info grouper_i2c4_smb347_board_info[] = {
	{
		I2C_BOARD_INFO("smb347", 0x6a),
	},
};

static struct i2c_board_info cardhu_i2c4_pad_bat_board_info[] = {
	{
		I2C_BOARD_INFO("pad-battery", 0xb),
	},
};

static struct i2c_board_info cardhu_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &cardhu_nct1008_pdata,
		.irq = -1,
	}
};

struct imx175_platform_data cardhu_imx175_data = {
	.power_on = yuv_sensor_power_on,
	.power_off = yuv_sensor_power_off,
};

#ifdef CONFIG_VIDEO_YUV

static struct i2c_board_info rear_sensor_i2c2_board_info[] = {  //ddebug
    {
         //I2C_BOARD_INFO("fjm6mo", 0x10),
         I2C_BOARD_INFO("imx175", 0x10),//Jimmy amend
        .platform_data = &cardhu_imx175_data,
    },

    {
		I2C_BOARD_INFO("dw9714", 0x0c),
        //.platform_data = &cardhu_dw9714_data, //Jimmy add for dw9714 power on/off
    },
    
    {
		I2C_BOARD_INFO("eeprom", 0x54),
        //Jimmy add for eeprom driver register
    },
};

static struct i2c_board_info front_sensor_i2c2_board_info[] = {  //ddebug
	{
		I2C_BOARD_INFO("ov2720", 0x36), //Jimmy amend
		.platform_data = &yuv_front_sensor_data,
	},
};
#endif /* CONFIG_VIDEO_YUV */
static int cardhu_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;
	u32 project_info = tegra3_get_project_id();

	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291) ||
		(board_info.board_id == BOARD_E1257) ||
		(board_info.board_id == BOARD_PM269) ||
		(board_info.board_id == BOARD_PM305) ||
		(board_info.board_id == BOARD_PM311)) {
		if(project_info == TEGRA3_PROJECT_ME301T || project_info == TEGRA3_PROJECT_ME301TL)
		{
			nct1008_port = TEGRA_GPIO_PS3;
		}
		else
		{
			nct1008_port = TEGRA_GPIO_PCC2;
		}
	} else if ((board_info.board_id == BOARD_E1186) ||
		(board_info.board_id == BOARD_E1187) ||
		(board_info.board_id == BOARD_E1256)) {
		/* FIXME: seems to be conflicting with usb3 vbus on E1186 */
		/* nct1008_port = TEGRA_GPIO_PH7; */
	}

	if (nct1008_port >= 0) {
		/* FIXME: enable irq when throttling is supported */
		cardhu_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(nct1008_port);

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0)
			gpio_free(nct1008_port);
	}

	return ret;
}

#if defined(CONFIG_GPIO_PCA953X)
static struct pca953x_platform_data cardhu_pmu_tca6416_data = {
	.gpio_base      = PMU_TCA6416_GPIO_BASE,
};

static const struct i2c_board_info cardhu_i2c4_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &cardhu_pmu_tca6416_data,
	},
};

static struct pca953x_platform_data cardhu_cam_tca6416_data = {
	.gpio_base      = CAM_TCA6416_GPIO_BASE,
};

static const struct i2c_board_info cardhu_i2c2_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &cardhu_cam_tca6416_data,
	},
};

static int __init pmu_tca6416_init(void)
{
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291))
			return 0;

	pr_info("Registering pmu pca6416\n");
	i2c_register_board_info(4, cardhu_i2c4_board_info_tca6416,
		ARRAY_SIZE(cardhu_i2c4_board_info_tca6416));
	return 0;
}

static int __init cam_tca6416_init(void)
{
	/* Boards E1198 and E1291 are of Cardhu personality
	 * and donot have TCA6416 exp for camera */
	if ((board_info.board_id == BOARD_E1198) ||
		(board_info.board_id == BOARD_E1291))
		return 0;

	pr_info("Registering cam pca6416\n");
	i2c_register_board_info(2, cardhu_i2c2_board_info_tca6416,
		ARRAY_SIZE(cardhu_i2c2_board_info_tca6416));
	return 0;
}
#else
static int __init pmu_tca6416_init(void)
{
	return 0;
}

static int __init cam_tca6416_init(void)
{
	return 0;
}
#endif


//Panda:--->
static struct mpu_platform_data mpu6050_data = {
        .int_config  = 0x10,
        .level_shifter = 0,
        .orientation = { 0, -1, 0,-1, 0,0,0, 0, -1 },
        .sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
        .sec_slave_id = COMPASS_ID_AK8975,
        .secondary_i2c_addr = 0x0D,
        .secondary_orientation = { 0, -1, 0,-1, 0,0,0, 0, -1 },  
};
static struct i2c_board_info __initdata inv_mpu6050_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU6050_GYRO_NAME, 0x69),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu6050_data,
	},
};
static int mpu6050(void)
{
	pr_info("*** MPU6050 START *** cardhu_mpuirq_init...\n");
	i2c_register_board_info(2, inv_mpu6050_i2c2_board_info,
	ARRAY_SIZE(inv_mpu6050_i2c2_board_info));
	return 0;

}
//Panda:<----

//leon add for cm3218++

static struct cm3218_platform_data cardhu_cm3218_pdata = {
//	.intr = CM3218_INT_N,
	.levels = { 0x0A, 0xA0, 0xE1, 0x140, 0x280,	0x500,0xA28, 0x16A8, 0x1F40, 0x2800},
	.power = NULL,
	.ALS_slave_address = CM3218_ALS_cmd,
	.check_interrupt_add = CM3218_check_INI,
	.is_cmd = CM3218_ALS_SM_2 | CM3218_ALS_IT_250ms | CM3218_ALS_PERS_1 | CM3218_ALS_RES_1,
};

static struct i2c_board_info cardhu_i2c0_cm3218_board_info[] = {
	{
		I2C_BOARD_INFO(CM3218_I2C_NAME, 0x48),
		.platform_data = &cardhu_cm3218_pdata,
//		.irq = OMAP_GPIO_IRQ(CM3218_INT_N),
	},
};

//leon add for cm3218--

/*Sensors orientation definition*/
struct mpu_orientation_def{
	__s8 gyro_orientation[9];
	__s8 accel_orientation[9];
	__s8 compass_orientation[9];
};

static const struct i2c_board_info cardhu_i2c1_board_info_al3010[] = {
    {
        I2C_BOARD_INFO("al3010",0x1C),
        .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
    },
};
//Panda

int __init cardhu_sensors_init(void)
{
	int err;
	int ret = 0;
	u32 project_info = tegra3_get_project_id();

	tegra_get_board_info(&board_info);
	
	
	cardhu_camera_init();
	cam_tca6416_init();
	/*Disable Cam LED*/
      //  gpio_direction_output(TEGRA_GPIO_PR6, 1);//Jimmy add for test VCM power pull high
	gpio_direction_output(TEGRA_GPIO_PS1, 0);
#ifdef CONFIG_I2C_MUX_PCA954x
	i2c_register_board_info(2, cardhu_i2c3_board_info,
		ARRAY_SIZE(cardhu_i2c3_board_info));


	if (board_info.board_id != BOARD_PM269) {
		i2c_register_board_info(PCA954x_I2C_BUS0,
					cardhu_i2c6_board_info,
					ARRAY_SIZE(cardhu_i2c6_board_info));

		i2c_register_board_info(PCA954x_I2C_BUS1,
					cardhu_i2c7_board_info,
					ARRAY_SIZE(cardhu_i2c7_board_info));
	} else {
		i2c_register_board_info(PCA954x_I2C_BUS0,
					pm269_i2c6_board_info,
					ARRAY_SIZE(pm269_i2c6_board_info));

		i2c_register_board_info(PCA954x_I2C_BUS1,
					pm269_i2c7_board_info,
					ARRAY_SIZE(pm269_i2c7_board_info));
	}
	i2c_register_board_info(PCA954x_I2C_BUS2, cardhu_i2c8_board_info,
	ARRAY_SIZE(cardhu_i2c8_board_info));
#endif

#ifdef CONFIG_VIDEO_YUV

	/*Flash LED*/
	pr_info("Flash LED i2c_register_board_info");
	i2c_register_board_info(2, cardhu_i2c_board_info_tps61050,
		ARRAY_SIZE(cardhu_i2c_board_info_tps61050));

	pr_info("imx175 i2c_register_board_info");
    i2c_register_board_info(2, rear_sensor_i2c2_board_info,
        ARRAY_SIZE(rear_sensor_i2c2_board_info));
/* Front Camera ov2720 + */
	pr_info("ov2720 i2c_register_board_info");
	i2c_register_board_info(2, front_sensor_i2c2_board_info,
		ARRAY_SIZE(front_sensor_i2c2_board_info));
/* Front Camera ov2720 - */

#endif /* CONFIG_VIDEO_YUV */
	pmu_tca6416_init();

	if (board_info.board_id == BOARD_E1291)
		i2c_register_board_info(4, cardhu_i2c4_bq27510_board_info,
			ARRAY_SIZE(cardhu_i2c4_bq27510_board_info));

	if(TEGRA3_PROJECT_ME301T == project_info || TEGRA3_PROJECT_ME301TL == project_info || TEGRA3_PROJECT_ME570T == project_info)
	{
		ret = i2c_register_board_info(4, grouper_i2c4_smb347_board_info,
		ARRAY_SIZE(grouper_i2c4_smb347_board_info));
		printk("smb347 i2c_register_board_info, ret = %d\n", ret);

		ret = i2c_register_board_info(4, cardhu_i2c4_bq27541_board_info,
		ARRAY_SIZE(cardhu_i2c4_bq27541_board_info));
		printk("bq27510 i2c_register_board_info, ret = %d\n", ret);
	}
	else
	{
		i2c_register_board_info(4, cardhu_i2c4_pad_bat_board_info,
		ARRAY_SIZE(cardhu_i2c4_pad_bat_board_info));
	}

	err = cardhu_nct1008_init();
	if (err)
		return err;

	i2c_register_board_info(4, cardhu_i2c4_nct1008_board_info,
		ARRAY_SIZE(cardhu_i2c4_nct1008_board_info));

	//leon add for cm3218++
	i2c_register_board_info(4, cardhu_i2c0_cm3218_board_info,
		ARRAY_SIZE(cardhu_i2c0_cm3218_board_info));
	//leon add for cm3218--
    
    mpu6050(); 
	return 0;

}


#ifdef CONFIG_VIDEO_OV5650
struct ov5650_gpios {
	const char *name;
	int gpio;
	int enabled;
};

#define OV5650_GPIO(_name, _gpio, _enabled)		\
	{						\
		.name = _name,				\
		.gpio = _gpio,				\
		.enabled = _enabled,			\
	}

static struct ov5650_gpios ov5650_gpio_keys[] = {
	[0] = OV5650_GPIO("cam1_pwdn", CAM1_PWR_DN_GPIO, 0),
	[1] = OV5650_GPIO("cam1_rst_lo", CAM1_RST_L_GPIO, 1),
	[2] = OV5650_GPIO("cam1_af_pwdn_lo", CAM1_AF_PWR_DN_L_GPIO, 0),
	[3] = OV5650_GPIO("cam1_ldo_shdn_lo", CAM1_LDO_SHUTDN_L_GPIO, 1),
	[4] = OV5650_GPIO("cam2_pwdn", CAM2_PWR_DN_GPIO, 0),
	[5] = OV5650_GPIO("cam2_rst_lo", CAM2_RST_L_GPIO, 1),
	[6] = OV5650_GPIO("cam2_af_pwdn_lo", CAM2_AF_PWR_DN_L_GPIO, 0),
	[7] = OV5650_GPIO("cam2_ldo_shdn_lo", CAM2_LDO_SHUTDN_L_GPIO, 1),
	[8] = OV5650_GPIO("cam3_pwdn", CAM_FRONT_PWR_DN_GPIO, 0),
	[9] = OV5650_GPIO("cam3_rst_lo", CAM_FRONT_RST_L_GPIO, 1),
	[10] = OV5650_GPIO("cam3_af_pwdn_lo", CAM_FRONT_AF_PWR_DN_L_GPIO, 0),
	[11] = OV5650_GPIO("cam3_ldo_shdn_lo", CAM_FRONT_LDO_SHUTDN_L_GPIO, 1),
	[12] = OV5650_GPIO("cam_led_exp", CAM_FRONT_LED_EXP, 1),
	[13] = OV5650_GPIO("cam_led_rear_exp", CAM_SNN_LED_REAR_EXP, 1),
	[14] = OV5650_GPIO("cam_i2c_mux_rst", CAM_I2C_MUX_RST_EXP, 1),
};

int __init cardhu_ov5650_late_init(void)
{
	int ret;
	int i;

	if (!machine_is_cardhu())
		return 0;

	if ((board_info.board_id == BOARD_E1198) || (board_info.board_id == BOARD_E1291))
		return 0;

	printk("%s: \n", __func__);
	for (i = 0; i < ARRAY_SIZE(ov5650_gpio_keys); i++) {
		ret = gpio_request(ov5650_gpio_keys[i].gpio,
			ov5650_gpio_keys[i].name);
		if (ret < 0) {
			printk("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
		printk("%s: enable - %d\n", __func__, i);
		gpio_direction_output(ov5650_gpio_keys[i].gpio,
			ov5650_gpio_keys[i].enabled);
		gpio_export(ov5650_gpio_keys[i].gpio, false);
	}

	return 0;

fail:
	while (i--)
		gpio_free(ov5650_gpio_keys[i].gpio);
	return ret;
}

late_initcall(cardhu_ov5650_late_init);
#endif
