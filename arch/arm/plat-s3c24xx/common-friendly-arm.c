/* linux/arch/arm/plat-s3c24xx/common-smdk.c
 *
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Common code for SMDK2410 and SMDK2440 boards
 *
 * http://www.fluff.org/ben/smdk2440/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <mach/regs-gpio.h>
#include <mach/leds-gpio.h>

#include <plat/nand.h>

#include <plat/common-friendly-arm.h>
#include <plat/devs.h>
#include <plat/pm.h>

/* NAND parititon from 2.4.18-swl5 */

static struct mtd_partition friendly_arm_default_nand_part[] = {
	[0] = {
		.name	= "supervivi",
		.size	= 0x00060000,
		.offset	= 0,
	},
	[1] = {
		.name	= "Kernel",
		.offset = 0x00060000,
		.size	= 0x00200000,
	},
	[2] = {
		.name	= "root",
		.offset = 0x00260000,
		.size	= 1024 * 1024 * 1024, //64U * 1024 * 1024 - 0x00260000,
	},
	[3] = {
		.name	= "nand",
		.offset = 0x00000000,
		.size	= 1024 * 1024 * 1024, //64U * 1024 * 1024 - 0x00260000,
	}
};

static struct s3c2410_nand_set friendly_arm_nand_sets[] = {
	[0] = {
		.name		= "NAND",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(friendly_arm_default_nand_part),
		.partitions	= friendly_arm_default_nand_part,
	},
};

/* choose a set of timings which should suit most 512Mbit
 * chips and beyond.
*/

static struct s3c2410_platform_nand friendly_arm_nand_info = {
	.tacls		= 20,
	.twrph0		= 60,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(friendly_arm_nand_sets),
	.sets		= friendly_arm_nand_sets,
};

/* devices we initialise */

static struct platform_device __initdata *friendly_arm_devs[] = {
	&s3c_device_nand,
	&s3c_device_sdi,
	&s3c_device_usbgadget,
};

void __init friendly_arm_machine_init(void)
{
	/* Configure the LEDs (even if we have no LED support)*/

	s3c_device_nand.dev.platform_data = &friendly_arm_nand_info;

	platform_add_devices(friendly_arm_devs, ARRAY_SIZE(friendly_arm_devs));

	s3c2410_pm_init();
}
