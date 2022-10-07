// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>

#include "apusys_power_ctl.h"
#include "hal_config_power.h"
#include "apu_log.h"
#include "apu_power_api.h"
#include "apusys_power.h"
#include "apusys_power_reg.h"
#include "apu_platform_debug.h"
#ifdef APUPWR_TAG_TP
#include "apu_power_tag.h"
#endif

void apu_power_dump_opp_table(struct seq_file *s)
{

}

void _apu_power_dump_acc_status(struct seq_file *s,
				struct apu_power_info *info, int index)
{

}

int apu_power_dump_curr_status(struct seq_file *s, int oneline_str)
{
	return 0;
}

int apusys_power_fail_show(struct seq_file *s, void *unused)
{
	return 0;
}
