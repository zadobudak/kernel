/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __MT8188_APUPWR_PROT_H__
#define __MT8188_APUPWR_PROT_H__

#include "apu_top.h"
#include "mt8188_apupwr.h"

/* mbox offset define (for data exchange with remote) */
#define SPARE_DBG_REG0		0x400	/* mbox4_dummy0 */
#define SPARE_DBG_REG1		0x404	/* mbox4_dummy1 */
#define SPARE_DBG_REG4		0x410	/* mbox4_dummy4 */
#define SPARE_DBG_REG5		0x414	/* mbox4_dummy5 */
#define SPARE_DBG_REG10		0x428	/* mbox4_dummy10 */
#define SPARE_DBG_REG11		0x42C	/* mbox4_dummy11 */
#define SPARE_DBG_REG12		0x430	/* mbox4_dummy12 */
#define SPARE_DBG_REG13		0x434	/* mbox4_dummy13 */
#define SPARE_DBG_REG14		0x438	/* mbox4_dummy14 */
#define SPARE_DBG_REG15		0x43C	/* mbox4_dummy15 */
#define SPARE_DBG_REG16		0x440	/* mbox4_dummy16 */
#define SPARE_DBG_REG17		0x444	/* mbox4_dummy17 */

#define OPP_SYNC_REG0		SPARE_DBG_REG0
#define OPP_SYNC_REG1		SPARE_DBG_REG1
#define OPP_SYNC_REG2		SPARE_DBG_REG4
#define THERMAL_SYNC_REG	SPARE_DBG_REG5
#define ACX0_LIMIT_OPP_REG      SPARE_DBG_REG10
#define ACX1_LIMIT_OPP_REG      SPARE_DBG_REG11
#define DEV_OPP_SYNC_REG        SPARE_DBG_REG12
#define HW_RES_SYNC_REG         SPARE_DBG_REG13
#define PLAT_CFG_SYNC_REG	SPARE_DBG_REG14
#define DRV_CFG_SYNC_REG	SPARE_DBG_REG15
#define PWR_FLOW_SYNC_REG	SPARE_DBG_REG16
#define THERMAL_SYNC_TEMP_REG	SPARE_DBG_REG17

enum {
	APUPWR_DBG_DEV_CTL = 0,
	APUPWR_DBG_DEV_SET_OPP,
	APUPWR_DBG_DVFS_DEBUG,
	APUPWR_DBG_DUMP_OPP_TBL,
	APUPWR_DBG_CURR_STATUS,
	APUPWR_DBG_PROFILING,
	APUPWR_DBG_DPIDLE_SKIP,
};

enum apu_opp_limit_type {
	OPP_LIMIT_THERMAL = 0,	/* limit by power API */
	OPP_LIMIT_HAL,		/* limit by i/o ctl */
	OPP_LIMIT_DEBUG,	/* limit by i/o ctl */
};

struct drv_cfg_data {
	int8_t log_level;
	int8_t dvfs_debounce;	/* debounce unit : ms */
	int8_t disable_hw_meter;/* 1: disable hw meter, bypass to read volt/freq */
};

struct plat_cfg_data {
	int8_t aging_flag:4,
	       hw_id:4;
	uint16_t freq[4];
	uint8_t  fix_volt;
};

struct device_opp_limit {
	int32_t vpu_max:6,
		vpu_min:6,
		dla_max:6,
		dla_min:6,
		lmt_type:8; /* limit reason */
};

struct cluster_dev_opp_info {
	uint32_t opp_lmt_reg;
	struct device_opp_limit dev_opp_lmt;
};

/*
 * due to this struct will be used to do data exchange through rpmsg
 * so the struct size can't over than 256 bytes
 * 4 bytes * 14 struct members = 56 bytes
 */
struct apu_pwr_curr_info {
	int buck_volt[BUCK_NUM];
	int buck_opp[BUCK_NUM];
	int pll_freq[PLL_NUM];
	int pll_opp[PLL_NUM];
};

/*
 * for satisfy size limitation of rpmsg data exchange is 256 bytes
 * we only put necessary information for opp table here
 * opp entries : 4 bytes * 5 struct members * 10 opp entries = 200 bytes
 * tbl_size : 4 bytes
 * total : 200 + 4 = 204 bytes
 */
struct tiny_dvfs_opp_entry {
	int vapu;       /* = volt_bin - volt_age + volt_avs */
	int pll_freq[PLL_NUM];
};

struct tiny_dvfs_opp_tbl {
	int tbl_size;   /* entry number */
	struct tiny_dvfs_opp_entry opp[];   /* entry data */
};

void mt8188_aputop_opp_limit(struct aputop_func_param *aputop,
		enum apu_opp_limit_type type);

#if IS_ENABLED(CONFIG_DEBUG_FS)
int mt8188_apu_top_dbg_open(struct inode *inode, struct file *file);
ssize_t mt8188_apu_top_dbg_write(
		struct file *flip, const char __user *buffer,
		size_t count, loff_t *f_pos);
#endif

int mt8188_init_remote_data_sync(void __iomem *reg_base);
int mt8188_drv_cfg_remote_sync(struct aputop_func_param *aputop);
int mt8188_chip_data_remote_sync(struct plat_cfg_data *plat_cfg);
int mt8188_apu_top_rpmsg_cb(int cmd, void *data, int len,
				void *priv, u32 src);
int mt8188_read_pwr_flow_sync_reg(void);
int mt8188_pwr_flow_remote_sync(uint32_t cfg);
void mt8188_apu_temperature_sync(const char *tz_name);

extern struct tiny_dvfs_opp_tbl *opp_tbl;
extern int g_pwr_log_level;

#endif /* __MT8188_APUPWR_PROT_H__ */
