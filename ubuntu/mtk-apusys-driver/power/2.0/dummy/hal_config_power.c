// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/sched/clock.h>

#include "hal_config_power.h"
#include "apu_power_api.h"
#include "apusys_power_ctl.h"
#include "apusys_power_cust.h"
#include "apusys_power_reg.h"
#include "apu_log.h"
#include "apusys_power_rule_check.h"

#define CREATE_TRACE_POINTS
#include "apu_power_events.h"
#ifdef APUPWR_TAG_TP
#include "apu_power_tag.h"
#include "apupwr_events.h"
#endif

//#include "mtk_devinfo.h"

#if SUPPORT_VCORE_TO_IPUIF
#define NUM_OF_IPUIF_OPP VCORE_OPP_NUM
#endif

int conn_mtcmos_on;

void *g_APU_RPCTOP_BASE;
void *g_APU_PCUTOP_BASE;
void *g_APU_VCORE_BASE;
void *g_APU_INFRACFG_AO_BASE;
void *g_APU_INFRA_BCRM_BASE;
void *g_APU_CONN_BASE;
void *g_APU_CONN1_BASE;
void *g_APU_VPU0_BASE;
void *g_APU_VPU1_BASE;
void *g_APU_MDLA0_BASE;
void *g_APU_MDLA1_BASE;
void *g_APU_SPM_BASE;
void *g_APU_APMIXED_BASE;
void *g_APU_PLL_BASE;
void *g_APU_ACC_BASE;

/************************************
 * platform related power APIs
 ************************************/

/************************************
 * common power hal command
 ************************************/

int hal_config_power(enum HAL_POWER_CMD cmd, enum DVFS_USER user, void *param)
{
	return 0;
}



