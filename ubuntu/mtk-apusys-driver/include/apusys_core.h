/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __APUSYS_CORE_H__
#define __APUSYS_CORE_H__
struct apusys_core_info {
	struct dentry *dbg_root;
};

int apusys_devapc_init(struct apusys_core_info *info);
void apusys_devapc_exit(void);

int mdw_init(struct apusys_core_info *info);
void mdw_exit(void);

int reviser_init(struct apusys_core_info *info);
void reviser_exit(void);

int mdla_init(struct apusys_core_info *info);
void mdla_exit(void);

#if IS_ENABLED(CONFIG_MTK_APUSYS_VPU)
int vpu_init(struct apusys_core_info *info);
void vpu_exit(void);
#endif

int apu_top_entry_init(struct apusys_core_info *info);
void apu_top_entry_exit(void);

int apu_power_drv_init(struct apusys_core_info *info);
void apu_power_drv_exit(void);

#if IS_ENABLED(CONFIG_MTK_APUSYS_DEBUG)
int debug_init(struct apusys_core_info *info);
void debug_exit(void);
#endif

int apumem_init(struct apusys_core_info *info);
void apumem_exit(void);

int mnoc_init(struct apusys_core_info *info);
void mnoc_exit(void);

int edma_init(struct apusys_core_info *info);
void edma_exit(void);

int sw_logger_init(struct apusys_core_info *info);
void sw_logger_exit(void);

int hw_logger_init(struct apusys_core_info *info);
void hw_logger_exit(void);

int apu_rproc_init(struct apusys_core_info *info);
void apu_rproc_exit(void);
#endif /* __APUSYS_CORE_H__ */

