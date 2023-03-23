// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "apu_top_entry.h"

int g_apupw_drv_ver = 3; // On IoT, default use power v3.0

static int __init apu_top_entry_init(void)
{
	int ret = 0;

	if (g_apupw_drv_ver != 3)
		ret |= apu_power_init();	// 2.5
	else
		ret |= apu_top_3_init();	// 3.0

	return ret;
}

static void __exit apu_top_entry_exit(void)
{
	if (g_apupw_drv_ver != 3)
		apu_power_exit();	// 2.5
	else
		apu_top_3_exit();	// 3.0
}

// caller is middleware
int apu_power_drv_init(struct apusys_core_info *info)
{
	pr_info("%s ++\n", __func__);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (g_apupw_drv_ver != 3)
		return apupw_dbg_init(info);	// 2.5
	else
		return aputop_dbg_init(info);	// 3.0
#endif

	return 0;
}
EXPORT_SYMBOL(apu_power_drv_init);

// caller is middleware
void apu_power_drv_exit(void)
{
#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (g_apupw_drv_ver != 3)
		apupw_dbg_exit();	// 2.5
	else
		aputop_dbg_exit();	// 3.0
#endif
}
EXPORT_SYMBOL(apu_power_drv_exit);

/* caller is middleware */
int apu_power_top_on(void)
{
	pr_info("%s ++\n", __func__);
	if (g_apupw_drv_ver == 3)
		apu_top_3_on();

	return 0;
}
EXPORT_SYMBOL(apu_power_top_on);

/* caller is middleware */
void apu_power_top_off(void)
{
	pr_info("%s ++\n", __func__);
	if (g_apupw_drv_ver == 3)
		apu_top_3_off();
}
EXPORT_SYMBOL(apu_power_top_off);

module_init(apu_top_entry_init);
module_exit(apu_top_entry_exit);
MODULE_LICENSE("GPL");
