// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
//#if IS_ENABLED(CONFIG_PM_SLEEP)
//#include <linux/pm_wakeup.h>
//#endif
#include <linux/uaccess.h>

#include "apu_top.h"
#include "aputop_rpmsg.h"
#include <apu_top_entry.h>

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
static int apu_devfreq_probe(struct platform_device *pdev)
{
	if (pwr_data->plat_apu_devfreq_cooling_register)
		pwr_data->plat_apu_devfreq_cooling_register(pdev);

	return 0;
}

static int apu_devfreq_remove(struct platform_device *pdev)
{
	if (pwr_data->plat_apu_devfreq_cooling_stop_monitor)
		pwr_data->plat_apu_devfreq_cooling_stop_monitor(pdev);

	if (pwr_data->plat_apu_devfreq_cooling_unregister)
		pwr_data->plat_apu_devfreq_cooling_unregister(pdev);

	return 0;
}

static const struct of_device_id of_match_mdla_devfreq[] = {
	{ .compatible = "mediatek,mt8188-mdla-devfreq"},
	{ .compatible = "mediatek,mt8195-mdla-devfreq"},
	{ /* end of list */},
};

static const struct of_device_id of_match_vpu_devfreq[] = {
	{ .compatible = "mediatek,mt8188-vpu-devfreq"},
	{ .compatible = "mediatek,mt8195-vpu-devfreq"},
	{ /* end of list */},
};

static struct platform_driver mdla_devfreq_drv = {
	.probe = apu_devfreq_probe,
	.remove = apu_devfreq_remove,
	.driver = {
		.name = "mt8188-mdla-devfreq",
		.of_match_table = of_match_mdla_devfreq,
	},
};

static struct platform_driver vpu_devfreq_drv = {
	.probe = apu_devfreq_probe,
	.remove = apu_devfreq_remove,
	.driver = {
		.name = "mt8188-vpu-devfreq",
		.of_match_table = of_match_vpu_devfreq,
	},
};

int apu_devfreq_init(void)
{
	int ret = 0;

	pr_info("%s register mdla_devfreq platform driver...\n", __func__);
	ret = platform_driver_register(&mdla_devfreq_drv);
	if (ret) {
		pr_info("failed to register mdla_devfreq platform driver\n");
		goto register_mdla_fail;
	}

	pr_info("%s register vpu_devfreq platform driver...\n", __func__);
	ret = platform_driver_register(&vpu_devfreq_drv);
	if (ret) {
		pr_info("failed to register mdla_devfreq platform driver\n");
		goto register_vpu_fail;
	}

	return ret;

register_vpu_fail:
	platform_driver_unregister(&mdla_devfreq_drv);

register_mdla_fail:
	return -1;
}

void apu_devfreq_exit(void)
{
	platform_driver_unregister(&vpu_devfreq_drv);
	platform_driver_unregister(&mdla_devfreq_drv);
}
#endif

