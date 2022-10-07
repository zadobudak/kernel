// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "apu_log.h"
#include "apu_top_entry.h"

int g_pwr_log_level = APUSYS_PWR_LOG_ERR;
int g_apupw_drv_ver;

int apu_top_entry_init(struct apusys_core_info *info)
{
	int ret = 0;

	ret |= apu_power_init();
	ret |= apu_top_3_init();

	return ret;
}
EXPORT_SYMBOL(apu_top_entry_init);

void apu_top_entry_exit(void)
{
	apu_top_3_exit();
	apu_power_exit();
}
EXPORT_SYMBOL(apu_top_entry_exit);

/* caller is middleware */
int apu_power_drv_init(struct apusys_core_info *info)
{
	pr_info("%s ++\n", __func__);
	if (g_apupw_drv_ver == 3)
		return aputop_dbg_init(info);

	return 0;
}
EXPORT_SYMBOL(apu_power_drv_init);

/* caller is middleware */
void apu_power_drv_exit(void)
{
	if (g_apupw_drv_ver == 3)
		aputop_dbg_exit();
}
EXPORT_SYMBOL(apu_power_drv_exit);

