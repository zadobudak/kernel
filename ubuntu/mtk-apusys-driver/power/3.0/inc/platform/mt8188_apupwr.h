/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __MT8188_APUPWR_H__
#define __MT8188_APUPWR_H__

#include <linux/io.h>
#include <linux/clk.h>

#define APU_POWER_INIT		(0) /* 1: init in kernel ; 0: init in lk2 */
#define APU_POWER_BRING_UP	(0) /* pwr all on in kernel */
#define APU_PWR_SOC_PATH	(0) /* 1: do not run apu pll/acc init */
#define ENABLE_SW_BUCK_CTL	(1) /* 1: enable regulator in rpm resume */
#define ENABLE_SOC_CLK_MUX	(0) /* 1: enable soc clk in rpm resume */
#define DEBUG_DUMP_REG		(0) /* dump overall apu registers for debug */
#define APMCU_REQ_RPC_SLEEP	(0) /* rpm suspend trigger sleep req to rpc */
#define THERMAL_TEMP_UPDATE	(1) /* 1: update apu temperature periodically */
#define APUPW_DUMP_FROM_APMCU	(0) /* 1: dump reg from APMCU, 0: from ATF */

#define VAPU_DEF_VOLT		(opp_tbl->opp[0].vapu)
#define VSRAM_DEF_VOLT		((VAPU_DEF_VOLT > VSRAM_CON_SAFE_VOLT) ?	\
					VSRAM_CON_SAFE_VOLT_H		\
						: VSRAM_CON_SAFE_VOLT)
#define USER_MAX_OPP_VAL	(0) /* fastest speed user can specify */
#define TURBO_BOOST_OPP		USER_MAX_OPP_VAL
#define TURBO_BOOST_VAL		(100)
#define MTK_POLL_DELAY_US	(10)
#define MTK_POLL_TIMEOUT	USEC_PER_SEC

#define VSRAM_CON_SAFE_VOLT	(750000)
#define VSRAM_CON_SAFE_VOLT_H	(800000)

#define VCORE_CON_DEF_VOLT	(550000)
#define VCORE_CON_SAFE_VOLT	(600000)

/* MHZ */
#define MDLA_DEFAULT_FREQ	(opp_tbl->opp[0].pll_freq[PLL_DLA] / 1000)
#define MVPU_DEFAULT_FREQ	(opp_tbl->opp[0].pll_freq[PLL_VPU] / 1000)
#define MNOC_DEFAULT_FREQ	(opp_tbl->opp[0].pll_freq[PLL_CONN] / 1000)

#define TEMP_UPDATE_TIME	(3000) /* update apu temperature time */

/* log lvl */
enum {
	APUSYS_PWR_LOG_ERR,
	APUSYS_PWR_LOG_WARN,
	APUSYS_PWR_LOG_INFO,
	APUSYS_PWR_LOG_DEBUG,
	APUSYS_PWR_LOG_VERBOSE,
};

#define LOG_DBG(format, args...) \
	do { \
		if (g_pwr_log_level >= APUSYS_PWR_LOG_DEBUG) \
			pr_info("[apu_top_3] " format, ##args); \
	} while (0)

enum smc_rcx_pwr_op {
	SMC_RCX_PWR_AFC_EN = 0,
	SMC_RCX_PWR_WAKEUP_RPC,
	SMC_RCX_PWR_CG_EN,
};

enum smc_pwr_dump {
	SMC_PWR_DUMP_RPC = 0,
	SMC_PWR_DUMP_PCU,
	SMC_PWR_DUMP_ARE,
	SMC_PWR_DUMP_ALL,
};

enum t_acx_id {
	ACX0 = 0,
	CLUSTER_NUM,
	RCX,
};

enum t_dev_id {
	VPU0 = 0,
	DLA0,
	DLA1,
	DEVICE_NUM,
};

enum apu_clksrc_id {
	PLL_CONN = 0, /* MNOC */
	PLL_UP,
	PLL_VPU,
	PLL_DLA,
	PLL_NUM,
};

enum apu_buck_id {
	BUCK_VAPU = 0,
	BUCK_VSRAM,
	BUCK_VCORE,
	BUCK_NUM,
};

enum dvfs_device_id {
	DVFS_DEV_MVPU = 0,
	DVFS_DEV_MDLA,
	DVFS_DEV_MAX,
};

enum apupw_reg {
	apu_rpc,
	sys_spm,
	bcrm_fmem_pdn,
	apu_rcx,
	apu_vcore,
	apu_md32_mbox,
	apu_pcu,
	apu_ao_ctl,
	apu_pll,
	apu_acc,
	apu_are0,
	apu_are1,
	apu_are2,
	apu_acx0,
	apu_acx0_rpc_lite,
	APUPW_MAX_REGS,
};

struct apu_power {
	void __iomem *regs[APUPW_MAX_REGS];
	unsigned int phy_addr[APUPW_MAX_REGS];
};

struct rpc_status_dump {
	uint32_t rpc_reg_status;
	uint32_t conn_reg_status;
	uint32_t vcore_reg_status;	/* rpc_lite bypss this */
};

void mt8188_apu_dump_rpc_status(enum t_acx_id id, struct rpc_status_dump *dump);

/* RPC offset define */
#define APU_RPC_TOP_CON           0x0000
#define APU_RPC_TOP_SEL           0x0004
#define APU_RPC_SW_FIFO_WE        0x0008
#define APU_RPC_IO_DEBUG          0x000C
#define APU_RPC_STATUS            0x0014
#define APU_RPC_TOP_SEL_1         0x0018
#define APU_RPC_HW_CON            0x001C
#define APU_RPC_LITE_CON          0x0020
#define APU_RPC_INTF_PWR_RDY_REG  0x0040
#define APU_RPC_INTF_PWR_RDY      0x0044
#define RPC_TOP_SEL_HW_DEF        (0x012b0000) // cfg in hw default
#define RPC_TOP_SEL_SW_CFG1       (0x1800531e) // cfg in cold boot
#define RPC_TOP_SEL_SW_CFG2       (0x192b531e) // cfg in warm boot

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
#define APU_ACC_CONFG_SET3      0x000C
#define APU_ACC_CONFG_CLR0      0x0040
#define APU_ACC_CONFG_CLR1      0x0044
#define APU_ACC_CONFG_CLR2      0x0048
#define APU_ACC_CONFG_CLR3      0x004C
#define APU_ACC_FM_CONFG_SET    0x00C0
#define APU_ACC_FM_CONFG_CLR    0x00C4
#define APU_ACC_FM_SEL          0x00C8
#define APU_ACC_FM_CNT          0x00CC
#define APU_ACC_CLK_EN_SET      0x00E0
#define APU_ACC_CLK_EN_CLR      0x00E4
#define APU_ACC_CLK_INV_EN_SET  0x00E8
#define APU_ACC_CLK_INV_EN_CLR  0x00EC
#define APU_ACC_AUTO_CONFG0     0x0100
#define APU_ACC_AUTO_CONFG1     0x0104
#define APU_ACC_AUTO_CONFG2     0x0108
#define APU_ACC_AUTO_CONFG3     0x010C
#define APU_ACC_AUTO_CTRL_SET0  0x0120
#define APU_ACC_AUTO_CTRL_SET1  0x0124
#define APU_ACC_AUTO_CTRL_SET2  0x0128
#define APU_ACC_AUTO_CTRL_SET3  0x012C
#define APU_ACC_AUTO_CTRL_CLR0  0x0140
#define APU_ACC_AUTO_CTRL_CLR1  0x0144
#define APU_ACC_AUTO_CTRL_CLR2  0x0148
#define APU_ACC_AUTO_CTRL_CLR3  0x014C
#define APU_ACC_AUTO_STATUS0    0x0160
#define APU_ACC_AUTO_STATUS1    0x0164
#define APU_ACC_AUTO_STATUS2    0x0168
#define APU_ACC_AUTO_STATUS3    0x016C

/* ARE offset define */
#define APU_ARE_INI_CTRL        0x0000
#define APU_ARE_SRAM_CON        0x0004
#define APU_ARE_CONFG0          0x0040
#define APU_ARE_CONFG1          0x0044
#define APU_ARE_GLO_FSM         0x0048
#define APU_ARE_APB_FSM         0x004C
#define APU_ARE_ETRY0_SRAM_H    0x0C00
#define APU_ARE_ETRY0_SRAM_L    0x0800
#define APU_ARE_ETRY1_SRAM_H    0x0C04
#define APU_ARE_ETRY1_SRAM_L    0x0804
#define APU_ARE_ETRY2_SRAM_H    0x0C08
#define APU_ARE_ETRY2_SRAM_L    0x0808
#define APU_ARE_ETRY3_SRAM_H	0x0C0C
#define APU_ARE_ETRY3_SRAM_L	0x080C

/* vcore offset define */
#define APUSYS_VCORE_CG_CON     0x0000
#define APUSYS_VCORE_CG_SET     0x0004
#define APUSYS_VCORE_CG_CLR     0x0008
#define APUSYS_VCORE_SW_RST     0x000C

/* rcx offset define */
#define APU_RCX_CG_CON          0x0000
#define APU_RCX_CG_SET          0x0004
#define APU_RCX_CG_CLR          0x0008
#define APU_RCX_SW_RST          0x000C

/* acx 0/1 offset define */
#define APU_ACX_CONN_CG_CON     0x3C000
#define APU_ACX_CONN_CG_CLR     0x3C008
#define APU_ACX_MVPU_CG_CON     0x2B000
#define APU_ACX_MVPU_CG_CLR     0x2B008
#define APU_ACX_MVPU_SW_RST     0x2B00C
#define APU_ACX_MVPU_RV55_CTRL0 0x2B018
#define APU_ACX_MDLA0_CG_CON    0x30000
#define APU_ACX_MDLA0_CG_CLR    0x30008
#define APU_ACX_MDLA1_CG_CON    0x34000
#define APU_ACX_MDLA1_CG_CLR    0x34008
#define APU_ACX_VP6_CG_CON      0x20100
#define APU_ACX_VP6_CG_CLR      0x20108


/* spm offset define */
#define APUSYS_BUCK_ISOLATION		(0x3EC)

/* apu_rcx_ao_ctrl  */
#define CSR_DUMMY_0_ADDR		(0x0024)

/* PCU initial data */
#define APU_PCUTOP_CTRL_SET		0x0

#define MT6359P_RG_BUCK_VMODEM_EN_ADDR	(0x1688)
#define MT6359P_RG_LDO_VSRAM_MD_EN_ADDR	(0x1f2e)

#define BUCK_VAPU_PMIC_REG_EN_ADDR	MT6359P_RG_BUCK_VMODEM_EN_ADDR
#define BUCK_VAPU_SRAM_PMIC_REG_EN_ADDR	MT6359P_RG_LDO_VSRAM_MD_EN_ADDR

#define APU_PCU_BUCK_STEP_SEL		0x0030
#define APU_PCU_BUCK_ON_DAT0_L		0x0080
#define APU_PCU_BUCK_ON_DAT0_H		0x0084
#define APU_PCU_BUCK_ON_DAT1_L		0x0088
#define APU_PCU_BUCK_ON_DAT1_H		0x008C
#define APU_PCU_BUCK_OFF_DAT0_L		0x00A0
#define APU_PCU_BUCK_OFF_DAT0_H		0x00A4
#define APU_PCU_BUCK_OFF_DAT1_L		0x00A8
#define APU_PCU_BUCK_OFF_DAT1_H		0x00AC
#define APU_PCU_BUCK_ON_SLE0		0x00C0
#define APU_PCU_BUCK_ON_SLE1		0x00C4
#define APU_PCU_BUCK_ON_SETTLE_TIME	0x12C	/* 300us */

/* xpu2apusys */
#define INFRA_FMEM_BUS_u_SI21_CTRL_0		(0x2C)
#define INFRA_FMEM_BUS_u_SI22_CTRL_0		(0x44)
#define INFRA_FMEM_BUS_u_SI11_CTRL_0		(0x48)
#define INFRA_FMEM_M6M7_BUS_u_SI24_CTRL_0	(0x1D0)

#endif /* __MT8188_APUPWR_H__ */
