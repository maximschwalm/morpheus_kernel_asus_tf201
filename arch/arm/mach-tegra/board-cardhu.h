/*
 * arch/arm/mach-tegra/board-cardhu.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef _MACH_TEGRA_BOARD_CARDHU_H
#define _MACH_TEGRA_BOARD_CARDHU_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/ricoh583.h>

/* Processor Board  ID */
#define BOARD_PM267   0x0243
#define BOARD_PM269   0x0245

/* SKU Information */
#define BOARD_SKU_B11	0xb11

#define SKU_DCDC_TPS62361_SUPPORT	0x1
#define SKU_SLT_ULPI_SUPPORT		0x2
#define SKU_T30S_SUPPORT		0x4
#define SKU_TOUCHSCREEN_MECH_FIX	0x0100

#define SKU_TOUCH_MASK			0xFF00
#define SKU_TOUCH_2000			0x0B00

#define SKU_MEMORY_TYPE_BIT		0x3
#define SKU_MEMORY_TYPE_MASK		0x7
/* If BOARD_PM269 */
#define SKU_MEMORY_SAMSUNG_EC		0x0
#define SKU_MEMORY_ELPIDA		0x2
#define SKU_MEMORY_SAMSUNG_EB		0x4
/* If BOARD_PM272 */
#define SKU_MEMORY_1GB_1R_HYNIX		0x0
#define SKU_MEMORY_2GB_2R_HYH9		0x2
/* If other BOARD_ variants */
#define SKU_MEMORY_CARDHU_1GB_1R	0x0
#define SKU_MEMORY_CARDHU_2GB_2R	0x2
#define SKU_MEMORY_CARDHU_2GB_1R_HYK0	0x4
#define SKU_MEMORY_CARDHU_2GB_1R_HYH9	0x6
#define SKU_MEMORY_CARDHU_2GB_1R_HYNIX	0x1
#define MEMORY_TYPE(sku) (((sku) >> SKU_MEMORY_TYPE_BIT) & SKU_MEMORY_TYPE_MASK)

/* Board Fab version */
#define BOARD_FAB_A00			0x0
#define BOARD_FAB_A01			0x1
#define BOARD_FAB_A02			0x2
#define BOARD_FAB_A03			0x3
#define BOARD_FAB_A04			0x4
#define BOARD_FAB_A05			0x5
#define BOARD_FAB_A06			0x6
#define BOARD_FAB_A07			0x7

/* Display Board ID */
#define BOARD_DISPLAY_PM313		0x030D
#define BOARD_DISPLAY_E1213		0x0C0D
#define BOARD_DISPLAY_E1247		0x0C2F
#define BOARD_DISPLAY_E1253		0x0C35
#define BOARD_DISPLAY_E1506		0x0F06

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS6591X_GPIO_0		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP0)
#define TPS6591X_GPIO_1		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP1)
#define TPS6591X_GPIO_2		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP2)
#define TPS6591X_GPIO_3		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP3)
#define TPS6591X_GPIO_4		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP4)
#define TPS6591X_GPIO_5		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP5)
#define TPS6591X_GPIO_6		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP6)
#define TPS6591X_GPIO_7		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP7)
#define TPS6591X_GPIO_8		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP8)
#define TPS6591X_GPIO_END	(TPS6591X_GPIO_BASE + TPS6591X_GPIO_NR)

/* RICOH583 GPIO */
#define RICOH583_GPIO_BASE	TEGRA_NR_GPIOS
#define RICOH583_GPIO_END	(RICOH583_GPIO_BASE + 8)

/* MAX77663 GPIO */
#define MAX77663_GPIO_BASE	TEGRA_NR_GPIOS
#define MAX77663_GPIO_END	(MAX77663_GPIO_BASE + MAX77663_GPIO_NR)

/* PMU_TCA6416 GPIOs */
#define PMU_TCA6416_GPIO_BASE	(TPS6591X_GPIO_END)
#define PMU_TCA6416_GPIO_PORT00	(PMU_TCA6416_GPIO_BASE + 0)
#define PMU_TCA6416_GPIO_PORT01	(PMU_TCA6416_GPIO_BASE + 1)
#define PMU_TCA6416_GPIO_PORT02	(PMU_TCA6416_GPIO_BASE + 2)
#define PMU_TCA6416_GPIO_PORT03	(PMU_TCA6416_GPIO_BASE + 3)
#define PMU_TCA6416_GPIO_PORT04	(PMU_TCA6416_GPIO_BASE + 4)
#define PMU_TCA6416_GPIO_PORT05	(PMU_TCA6416_GPIO_BASE + 5)
#define PMU_TCA6416_GPIO_PORT06	(PMU_TCA6416_GPIO_BASE + 6)
#define PMU_TCA6416_GPIO_PORT07	(PMU_TCA6416_GPIO_BASE + 7)
#define PMU_TCA6416_GPIO_PORT10	(PMU_TCA6416_GPIO_BASE + 8)
#define PMU_TCA6416_GPIO_PORT11	(PMU_TCA6416_GPIO_BASE + 9)
#define PMU_TCA6416_GPIO_PORT12	(PMU_TCA6416_GPIO_BASE + 10)
#define PMU_TCA6416_GPIO_PORT13	(PMU_TCA6416_GPIO_BASE + 11)
#define PMU_TCA6416_GPIO_PORT14	(PMU_TCA6416_GPIO_BASE + 12)
#define PMU_TCA6416_GPIO_PORT15	(PMU_TCA6416_GPIO_BASE + 13)
#define PMU_TCA6416_GPIO_PORT16	(PMU_TCA6416_GPIO_BASE + 14)
#define PMU_TCA6416_GPIO_PORT17	(PMU_TCA6416_GPIO_BASE + 15)
#define PMU_TCA6416_GPIO_END	(PMU_TCA6416_GPIO_BASE + 16)

/* PCA954x I2C bus expander bus addresses */
#define PCA954x_I2C_BUS_BASE	6
#define PCA954x_I2C_BUS0	(PCA954x_I2C_BUS_BASE + 0)
#define PCA954x_I2C_BUS1	(PCA954x_I2C_BUS_BASE + 1)
#define PCA954x_I2C_BUS2	(PCA954x_I2C_BUS_BASE + 2)
#define PCA954x_I2C_BUS3	(PCA954x_I2C_BUS_BASE + 3)

#define AC_PRESENT_GPIO		TPS6591X_GPIO_4

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)
#define DOCK_DETECT_GPIO TEGRA_GPIO_PU4

/* RICOH583 IRQs */
#define RICOH583_IRQ_BASE	TEGRA_NR_IRQS
#define RICOH583_IRQ_END	(RICOH583_IRQ_BASE + RICOH583_NR_IRQS)

/* MAX77663 IRQs */
#define MAX77663_IRQ_BASE	TEGRA_NR_IRQS
#define MAX77663_IRQ_END	(MAX77663_IRQ_BASE + MAX77663_IRQ_NR)

#define EN_HSIC_GPIO		TEGRA_GPIO_PR7  /* PMU_GPIO25 */

int cardhu_regulator_init(void);
int cardhu_suspend_init(void);
int cardhu_sdhci_init(void);
int cardhu_pinmux_init(void);
int cardhu_panel_init(void);
int cardhu_sensors_init(void);
int cardhu_keys_init(void);
int cardhu_pins_state_init(void);
int cardhu_emc_init(void);
int cardhu_edp_init(void);

#define TDIODE_OFFSET	(10000)	/* in millicelsius */

#endif
