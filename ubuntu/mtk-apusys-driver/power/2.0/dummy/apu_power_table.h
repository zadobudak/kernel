/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _APU_POWER_TABLE_H_
#define _APU_POWER_TABLE_H_

#include "apusys_power_cust.h"
#include "apusys_power_user.h"


/******************************************************
 * for other cooperation module API, e.g. EARA, thermal
 ******************************************************/

struct apu_opp_info {
	enum APU_OPP_INDEX opp_index;   /* vpu or mdla opp */
	int power;                      /* power consumption (mW) */
};

static int init_devfreq_cooling(struct device *dev)
{
	return 0;
}

static int register_devfreq_cooling(struct platform_device *pdev,
						enum DVFS_USER user)
{
	return 0;
}

static void unregister_devfreq_cooling(enum DVFS_USER user)
{

}

static void start_monitor_devfreq_cooling(enum DVFS_USER user)
{

}

static void stop_monitor_devfreq_cooling(enum DVFS_USER user)
{

}

#endif
