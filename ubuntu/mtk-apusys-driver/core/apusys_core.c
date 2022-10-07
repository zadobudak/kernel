// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/module.h>  /* Needed by all modules */
#include <linux/kernel.h>  /* Needed for KERN_ALERT */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

#include "apusys_core.h"

/* define */
#define APUSYS_DEV_NAME "apusys"

/* global variable */
static struct apusys_core_info g_core_info;

struct apusys_core_func {
	int enable;
	int (*init)(struct apusys_core_info *info);
	void (*exit)(void);
};

struct apusys_core_func core_pipe[] = {
	{1, apu_top_entry_init, apu_top_entry_exit},
	{1, hw_logger_init, hw_logger_exit},
	{1, sw_logger_init, sw_logger_exit},
	//{1, power_wrapper_init, power_wrapper_uninit},
	{1, apu_power_drv_init, apu_power_drv_exit},
	{1, apusys_devapc_init, apusys_devapc_exit},
	{1, reviser_init, reviser_exit},
	{1, apumem_init, apumem_exit},
	{1, mnoc_init, mnoc_exit},
	{1, mdw_init, mdw_exit},
	//{1, mvpu_debugfs_init, mvpu_debugfs_exit},
	{1, edma_init, edma_exit},
#if IS_ENABLED(CONFIG_MTK_APUSYS_MDLA_SUPPORT)
	{1, mdla_init, mdla_exit},
#endif
#if IS_ENABLED(CONFIG_MTK_APUSYS_VPU)
	{1, vpu_init, vpu_exit},
#endif
#if IS_ENABLED(CONFIG_MTK_APUSYS_DEBUG)
	{1, debug_init, debug_exit},
#endif
	{1, apu_rproc_init, apu_rproc_exit},

};

static void create_dbg_root(void)
{
	g_core_info.dbg_root = debugfs_create_dir(APUSYS_DEV_NAME, NULL);

	/* check dbg root create status */
	if (IS_ERR_OR_NULL(g_core_info.dbg_root))
		pr_info("failed to create debug dir.\n");
}

static void destroy_dbg_root(void)
{
	debugfs_remove_recursive(g_core_info.dbg_root);
}

static int __init apusys_init(void)
{
	int i = 0, j = 0, ret = 0;
	int func_num = sizeof(core_pipe) / sizeof(struct apusys_core_func);

	/* init apusys_dev */
	create_dbg_root();

	/* call init func */
	for (i = 0; i < func_num; i++) {
		if (!core_pipe[i].enable || (core_pipe[i].init == NULL))
			continue;

		ret = core_pipe[i].init(&g_core_info);
		if (ret) {
			pr_info("%s: init function(%d) fail(%d)", __func__,
			       i, ret);

			/* exit device */
			for (j = i-1; j >= 0; j--) {
				if (core_pipe[j].enable && core_pipe[j].exit)
					core_pipe[j].exit();
			}
			destroy_dbg_root();
			break;
		}
	}

	return ret;
}

static void __exit apusys_exit(void)
{
	int i = 0;
	int func_num = sizeof(core_pipe) / sizeof(struct apusys_core_func);

	/* call release func */
	for (i = func_num - 1; i >= 0; i--) {
		if (!core_pipe[i].enable || (core_pipe[i].exit == NULL))
			continue;

		core_pipe[i].exit();
	}

	destroy_dbg_root();
}

module_init(apusys_init);
module_exit(apusys_exit);
MODULE_DESCRIPTION("MTK APUSys Driver");
MODULE_AUTHOR("SPT1");
MODULE_LICENSE("GPL");
