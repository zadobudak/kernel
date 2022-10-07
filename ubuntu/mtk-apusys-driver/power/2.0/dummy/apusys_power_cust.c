// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "apusys_power_cust.h"


//	| VPU0 |		| VPU1 |		| MDLA0 |  --> DVFS_USER
//	(VPU0_VPU)    (VPU1_VPU)     (VMDLA0_MDLA)
//	   |              |               |
//	   v	          v               v
//   |APU_CONN|     |APU_CONN|     |APU_CONN |
//   (VPU0_APU_CONN)(VPU1_APU_CONN)(VMDLA0_APU_CONN)

char *user_str[APUSYS_DVFS_USER_NUM] = {
};


char *buck_domain_str[APUSYS_BUCK_DOMAIN_NUM] = {
};

char *buck_str[APUSYS_BUCK_NUM] = {
};


bool apusys_dvfs_user_support[APUSYS_DVFS_USER_NUM] = {
};

#if VCORE_DVFS_SUPPORT
bool apusys_dvfs_buck_domain_support[APUSYS_BUCK_DOMAIN_NUM] = {
	true, true, true, true, true, true
};
#else
bool apusys_dvfs_buck_domain_support[APUSYS_BUCK_DOMAIN_NUM] = {
};
#endif

enum DVFS_VOLTAGE_DOMAIN apusys_user_to_buck_domain[APUSYS_DVFS_USER_NUM] = {
};


enum DVFS_BUCK apusys_user_to_buck[APUSYS_DVFS_USER_NUM] = {
};


enum DVFS_USER apusys_buck_domain_to_user[APUSYS_BUCK_DOMAIN_NUM] = {
};


enum DVFS_BUCK apusys_buck_domain_to_buck[APUSYS_BUCK_DOMAIN_NUM] = {
};


enum DVFS_VOLTAGE_DOMAIN apusys_buck_to_buck_domain[APUSYS_BUCK_NUM] = {
};


// voltage for clk path
uint8_t dvfs_clk_path[APUSYS_DVFS_USER_NUM][APUSYS_PATH_USER_NUM] = {
};

enum DVFS_VOLTAGE
	dvfs_clk_path_max_vol[APUSYS_DVFS_USER_NUM][APUSYS_PATH_USER_NUM] = {
};

// relation for dvfs_clk_path,
bool buck_shared[APUSYS_BUCK_NUM]
			[APUSYS_DVFS_USER_NUM][APUSYS_PATH_USER_NUM] = {
};

// confirmed by Gary
// small voltage first [Fix me]
struct apusys_dvfs_constraint
	dvfs_constraint_table[APUSYS_DVFS_CONSTRAINT_NUM] = {
};

// confirmed by Arvin
enum DVFS_VOLTAGE vcore_opp_mapping[] = {
};

#if SUPPORT_VCORE_TO_IPUIF
/**************************************************
 * IPUIF OPP table definition
 **************************************************/
/*
 *#define NUM_OF_IPUIF_OPP (sizeof(g_ipuif_opp_table) / \
 *				sizeof(g_ipuif_opp_table[0]))
 */

#define IPUIFOP(khz, apu_vcore)	\
{							\
	.ipuif_khz = khz,				\
	.ipuif_vcore = apu_vcore,				\
}

struct ipuif_opp_table g_ipuif_opp_table[] = {
};
#endif

struct apusys_dvfs_steps
	dvfs_table[APUSYS_MAX_NUM_OPPS][APUSYS_BUCK_DOMAIN_NUM] = {
};

#ifdef AGING_MARGIN
/* Below array define relation between different freq <--> aging voltage */
struct apusys_aging_steps aging_tbl[APUSYS_MAX_NUM_OPPS][V_APU_CONN] = {
};
#endif /* AGING_MARGIN */

