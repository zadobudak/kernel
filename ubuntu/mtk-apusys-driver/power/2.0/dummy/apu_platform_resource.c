// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>

#include "hal_config_power.h"
#include "apu_log.h"
struct device *apu_dev;


int init_platform_resource(struct platform_device *pdev,
struct hal_param_init_power *init_power_data)
{
	apu_dev = &pdev->dev;
	return 0;
}
