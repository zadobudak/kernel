// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include <linux/module.h>
#include <linux/of_device.h>

#include <utilities/mdla_profile.h>
#include <utilities/mdla_debug.h>

#include "mdla_plat_internal.h"

static struct mdla_plat_drv rv_drv = {
	.init           = mdla_rv_init,
	.deinit         = mdla_rv_deinit,
	.sw_cfg         = BIT(CFG_MICRO_P_SUPPORT) | BIT(CFG_DUMMY_PWR),
	.profile_ver    = PROF_NONE,
};

static const struct of_device_id mdla_of_match[] = {
	{ .compatible = "mediatek, mdla-rv", .data = &rv_drv},
	{ /* end of list */},
};
MODULE_DEVICE_TABLE(of, mdla_of_match);

const struct of_device_id *mdla_plat_get_device(void)
{
	return mdla_of_match;
}
