/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __MT8195_APUPWR_H__
#define __MT8195_APUPWR_H__

#include <linux/io.h>
#include <linux/clk.h>

#define APU_POWER_INIT		(1) /* 1: init in kernel ; 0: init in lk2 */
#define APU_POWER_BRING_UP	(0) /* pwr all on in kernel */
#define ENABLE_SW_BUCK_CTL	(1) /* 1: enable regulator in rpm resume */
#define ENABLE_SOC_CLK_MUX	(1) /* 1: enable soc clk in rpm resume */
#define DEBUG_DUMP_REG		(0) /* dump overall apu registers for debug */
#define APMCU_REQ_RPC_SLEEP	(0) /* rpm suspend trigger sleep req to rpc */

#define VAPU_DEF_VOLT		(mt8195_opp_tbl->opp[0].vapu)
#define VMDLA_DEF_VOLT		(mt8195_opp_tbl->opp[0].vmdla)
#define VSRAM_DEF_VOLT		((VAPU_DEF_VOLT > VSRAM_CON_SAFE_VOLT) ?	\
					VSRAM_CON_SAFE_VOLT_H		\
						: VSRAM_CON_SAFE_VOLT)
#define USER_MAX_OPP_VAL	(0) /* fastest speed user can specify */
#define TURBO_BOOST_OPP		USER_MAX_OPP_VAL
#define TURBO_BOOST_VAL		(100)
#define MTK_POLL_DELAY_US	(10)
#define MTK_POLL_TIMEOUT	USEC_PER_SEC

#define VSRAM_CON_SAFE_VOLT	(750000)
#define VSRAM_CON_SAFE_VOLT_H	(850000)

/* MHZ */
#define KHZ			(1000)
#define MHZ			(KHZ * KHZ)
#define MDLA_DEFAULT_FREQ	(mt8195_opp_tbl->opp[0].pll_freq[PLL_DLA] / 1000)
#define MVPU_DEFAULT_FREQ	(mt8195_opp_tbl->opp[0].pll_freq[PLL_VPU] / 1000)
#define MNOC_DEFAULT_FREQ	(mt8195_opp_tbl->opp[0].pll_freq[PLL_CONN] / 1000)

enum t_acx_id {
	ACX0 = 0,
	CLUSTER_NUM,
	RCX,
};

enum t_dev_id {
	VPU0 = 0,
	VPU1,
	DLA0,
	DLA1,
	DEVICE_NUM,
};

enum apu_clksrc_id {
	PLL_CONN = 0, /* MNOC */
	PLL_IOMMU,
	PLL_VPU,
	PLL_DLA,
	PLL_NUM,
};

enum apu_buck_id {
	BUCK_VAPU = 0,
	BUCK_VMDLA,
	BUCK_VSRAM,
	BUCK_NUM,
};

enum dvfs_device_id {
	DVFS_DEV_VPU = 0,
	DVFS_DEV_MDLA,
	DVFS_DEV_MAX,
};

enum apupw_reg {
	sys_spm,
	apu_conn,
	apu_conn1,
	apu_vcore,
	apu_md32_mbox,
	apu_rpc,
	apu_pcu,
	apu_ao_ctl,
	apu_pll,
	apu_acc,
	apu_vpu0,
	apu_vpu1,
	apu_mdla0,
	apu_mdla1,
	APUPW_MAX_REGS,
};

struct apu_power {
	void __iomem *regs[APUPW_MAX_REGS];
	unsigned int phy_addr[APUPW_MAX_REGS];
};

struct rpc_status_dump {
	uint32_t rpc_reg_status;
	uint32_t conn_reg_status;
	uint32_t conn1_reg_status;
	uint32_t vcore_reg_status;
};

void mt8195_apu_dump_pwr_status(struct rpc_status_dump *dump);

/* RPC offset define */
#define APU_RPC_TOP_CON           0x0000
#define APU_RPC_TOP_SEL           0x0004
#define APU_RPC_SW_FIFO_WE        0x0008
#define APU_RPC_IO_DEBUG          0x000C
#define APU_RPC_INTF_PWR_RDY_REG  0x0040
#define APU_RPC_INTF_PWR_RDY      0x0044

/* PLL offset define */
#define PLL4H_PLL1_CON1		0x00C
#define PLL4H_PLL2_CON1		0x01C
#define PLL4H_PLL3_CON1		0x02C
#define PLL4H_PLL4_CON1		0x03C
#define PLL4HPLL_FHCTL_HP_EN	0xE00
#define PLL4HPLL_FHCTL_CLK_CON	0xE08
#define PLL4HPLL_FHCTL_RST_CON	0xE0C
#define PLL4HPLL_FHCTL0_CFG	0xE3C
#define PLL4HPLL_FHCTL0_DDS	0xE44
#define PLL4HPLL_FHCTL1_CFG	0xE50
#define PLL4HPLL_FHCTL1_DDS	0xE58
#define PLL4HPLL_FHCTL2_CFG	0xE64
#define PLL4HPLL_FHCTL2_DDS	0xE6C
#define PLL4HPLL_FHCTL3_CFG	0xE78
#define PLL4HPLL_FHCTL3_DDS	0xE80

/* ACC offset define */
#define APU_ACC_CONFG_SET0      0x0000
#define APU_ACC_CONFG_SET1      0x0004
#define APU_ACC_CONFG_SET2      0x0008
#define APU_ACC_CONFG_SET4      0x0010
#define APU_ACC_CONFG_SET5      0x0014
#define APU_ACC_CONFG_SET7      0x001C
#define APU_ACC_CONFG_CLR0      0x0040
#define APU_ACC_CONFG_CLR1      0x0044
#define APU_ACC_CONFG_CLR2      0x0048
#define APU_ACC_CONFG_CLR4      0x0050
#define APU_ACC_CONFG_CLR5      0x0054
#define APU_ACC_CONFG_CLR7      0x005C
#define APU_ACC_FM_CONFG_SET    0x00C0
#define APU_ACC_FM_CONFG_CLR    0x00C4
#define APU_ACC_FM_SEL          0x00C8
#define APU_ACC_FM_CNT          0x00CC

#define BIT_SEL_APU		0x0800
#define BIT_CGEN_SOC		0x0004
#define BIT_CGEN_APU		0x0008
#define BIT_SEL_APU_DIV2	0x0400
#define BIT_INVEN_OUT		0x8000

/* vcore offset define */
#define APUSYS_VCORE_CG_CON     0x0000
#define APUSYS_VCORE_CG_SET     0x0004
#define APUSYS_VCORE_CG_CLR     0x0008
#define APUSYS_VCORE_SW_RST     0x000C

/* apu conn define */
#define APUSYS_CONN_CG_CON	0x000
#define APUSYS_CONN_CG_SET	0x004
#define APUSYS_CONN_CG_CLR	0x008
#define APUSYS_CONN1_CG_CON	0x000
#define APUSYS_CONN1_CG_SET	0x004
#define APUSYS_CONN1_CG_CLR	0x008

/* apu device define */
#define APU_VPU_CG_CON		0x100
#define APU_VPU_CG_CLR		0x108
#define APU_MDLA_CG_CON		0x000
#define APU_MDLA_CG_CLR		0x008

/* spm offset define */
#define APUSYS_OTHER_PWR_STATUS		(0x198)
#define APUSYS_BUCK_ISOLATION		(0x3EC)
#define APUSYS_SPM_CROSS_WAKE_M01_REQ	(0x670)

/* apu_rcx_ao_ctrl  */
#define CSR_DUMMY_0_ADDR		(0x0024)

#endif /* __MT8195_APUPWR_H__ */
