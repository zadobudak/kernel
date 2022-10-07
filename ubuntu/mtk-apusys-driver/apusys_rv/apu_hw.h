/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef APU_HW_H_
#define APU_HW_H_

struct mtk_apu_reg_ofs {
	/* reviser offset */
	uint32_t normal_dns;
	uint32_t pri_dns;
	uint32_t iommu_ctrl;
	uint32_t core0_vabase0;
	uint32_t core0_mvabase0;
	uint32_t core0_vabase1;
	uint32_t core0_mvabase1;
	uint32_t userfw_ctxt;
	uint32_t secfw_ctxt;
	/* md32 offset */
	uint32_t md32_sys_ctrl;
	uint32_t dbg_bus_sel;
	uint32_t md32_clk_en;
	uint32_t up_wake_host_mask0;
	/* apu ao offset */
	uint32_t md32_pre_def;
	uint32_t md32_boot_ctrl;
	uint32_t md32_runstall;
	uint32_t md32_secfw_domain;
	uint32_t md32_userfw_domain;
	/* mbox offset */
	uint32_t mbox_host_cfg;
};
#endif /* APU_HW_H_ */
