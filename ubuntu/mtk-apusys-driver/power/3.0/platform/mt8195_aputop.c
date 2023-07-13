// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#include "apusys_secure.h"
#include "aputop_rpmsg.h"
#include "apu_top.h"
#include "mt8195_apupwr.h"
#include "mt8195_apupwr_prot.h"
#include "mt8195_apu_devfreq_cooling.h"

#define LOCAL_DBG	(1)

/* Below reg_name has to 1-1 mapping DTS's name */
static const char *reg_name[APUPW_MAX_REGS] = {
	"sys_spm", "apu_conn", "apu_conn1", "apu_vcore", "apu_md32_mbox",
	"apu_rpc", "apu_pcu", "apu_ao_ctl", "apu_pll", "apu_acc",
	"apu_vpu0", "apu_vpu1", "apu_mdla0", "apu_mdla1",
};

static struct apu_power apupw;
int g_pwr_log_level_MT8195 = 7;

static void aputop_dump_pwr_res(void);

/* regulator id */
static struct regulator *vapu_reg_id;
static struct regulator *vmdla_reg_id;
static struct regulator *vcore_reg_id;
static struct regulator *vsram_reg_id;

/************** IMPORTANT !! *******************
 * APU Top preclk
 * The following name of each clock struct
 * MUST mapping to clock-names @dts
 **********************************************/
/* apu clk from SOC */
static struct clk *clk_top_dsp_sel;
static struct clk *clk_top_dsp1_sel;
static struct clk *clk_top_dsp2_sel;
static struct clk *clk_top_dsp4_sel;
static struct clk *clk_top_dsp5_sel;
static struct clk *clk_top_ipu_if_sel;
/* apu clk from APUPLL */
static struct clk *clk_apupll_apupll;
static struct clk *clk_apupll_npupll;
static struct clk *clk_apupll_apupll1;
static struct clk *clk_apupll_apupll2;
static struct clk *clk_infra_ao_debugsys;

static uint32_t g_opp_cfg_acx0;

struct tiny_dvfs_opp_tbl *mt8195_opp_tbl;

static uint32_t apusys_pwr_smc_call(struct device *dev, uint32_t smc_id,
		uint32_t a2)
{
//#if APUSYS_SECURE
	/* TODO: please add back after secure ready */
	//return 0;
//#else
	return 0;
//#endif
}

static void mt8195_apu_update_vapu_opp_tbl(struct device_node *np,
						int pll_idx, int opp_idx)
{
	int volt = 0;
	uint64_t freq = 0;

	of_property_read_u64(np, "opp-hz", &freq);
	of_property_read_u32(np, "opp-microvolt", &volt);

	if (mt8195_opp_tbl->opp[opp_idx].vapu == 0) {
		mt8195_opp_tbl->opp[opp_idx].vapu = volt;
	} else if (mt8195_opp_tbl->opp[opp_idx].vapu < volt) {
		pr_info(
			"%s: vapu opp[%d] volt mismatch (%d, %d)\n",
			__func__, opp_idx,
			mt8195_opp_tbl->opp[opp_idx].vapu,
			volt);
	}

	do_div(freq, 1000); /* HZ to KHZ */
	mt8195_opp_tbl->opp[opp_idx].pll_freq[pll_idx]
		= (uint32_t)freq;
	LOG_DBG(
		"%s: vapu opp[%d] pll[%d] freq = %lld\n",
		__func__,
		opp_idx, pll_idx, freq);
}

static void mt8195_apu_update_vmdla_opp_tbl(struct device_node *np,
						int pll_idx, int opp_idx)
{
	int volt = 0;
	uint64_t freq = 0;

	of_property_read_u64(np, "opp-hz", &freq);
	of_property_read_u32(np, "opp-microvolt", &volt);

	if (mt8195_opp_tbl->opp[opp_idx].vmdla == 0) {
		mt8195_opp_tbl->opp[opp_idx].vmdla = volt;
	} else if (mt8195_opp_tbl->opp[opp_idx].vmdla < volt) {
		pr_info(
			"%s: vmdla opp[%d] volt mismatch (%d, %d)\n",
			__func__, opp_idx,
			mt8195_opp_tbl->opp[opp_idx].vapu,
			volt);
	}

	do_div(freq, 1000); /* HZ to KHZ */
	mt8195_opp_tbl->opp[opp_idx].pll_freq[pll_idx]
		= (uint32_t)freq;
	LOG_DBG(
		"%s: vmdla opp[%d] pll[%d] freq = %lld\n",
		__func__,
		opp_idx, pll_idx, freq);
}

static int init_opp_table(struct platform_device *pdev)
{
	static const char * const pll_name[] = {"PLL_CONN", "PLL_RV33",
					"PLL_VPU", "PLL_MDLA"};
	static const char * const buck_name[] = {"BUCK_VAPU", "BUCK_VMDLA",
					"BUCK_VSRAM"};
	struct tiny_dvfs_opp_entry tmp_entry = {0};
	struct of_phandle_iterator it;
	struct device_node *np, *child_np = NULL;
	struct device *dev = NULL;
	int i, j;
	int ret = 0;
	int opp_num = 0, opp_idx = 0, pll_idx = 0;
	char buf[128] = {0};

	dev = &pdev->dev;
	of_for_each_phandle(&it, ret, dev->of_node,
			    "operating-points-v2", NULL, 0) {
		np = of_node_get(it.node);
		if (!np) {
			dev_dbg(dev, "of_node_get fail\n");
			return -1;
		}

		if (mt8195_opp_tbl == NULL) {
			opp_num = of_get_child_count(np);
			mt8195_opp_tbl = kzalloc(sizeof(*mt8195_opp_tbl) +
				opp_num * sizeof(struct tiny_dvfs_opp_entry),
				GFP_KERNEL);
			if (IS_ERR_OR_NULL(mt8195_opp_tbl))
				return -1;

			mt8195_opp_tbl->tbl_size = opp_num;
		}

		opp_idx = 0;
		do {
			child_np = of_get_next_available_child(np, child_np);

			if (!child_np)
				continue;

			if (!strcmp(pll_name[pll_idx], "PLL_MDLA"))
				mt8195_apu_update_vmdla_opp_tbl(child_np,
							pll_idx, opp_idx);
			else
				mt8195_apu_update_vapu_opp_tbl(child_np,
							pll_idx, opp_idx);

			opp_idx++;
		} while (child_np);

		pll_idx++;
		of_node_put(np);
	}

	if (mt8195_opp_tbl == NULL)
		return 0;

	/* Sort opp table by vapu */
	for (i = 0; i < (mt8195_opp_tbl->tbl_size - 1); i++) {
		for (j = i + 1; j < mt8195_opp_tbl->tbl_size; j++) {
			if (mt8195_opp_tbl->opp[j].vapu > mt8195_opp_tbl->opp[i].vapu) {
				memcpy(&tmp_entry, &mt8195_opp_tbl->opp[j],
					sizeof(struct tiny_dvfs_opp_entry));
				memcpy(&mt8195_opp_tbl->opp[j], &mt8195_opp_tbl->opp[i],
					sizeof(struct tiny_dvfs_opp_entry));
				memcpy(&mt8195_opp_tbl->opp[i], &tmp_entry,
					sizeof(struct tiny_dvfs_opp_entry));
			}
		}
	}

	/* first line */
	ret = sprintf(buf, "| # | %s | %s |", buck_name[0], buck_name[1]);
	if (ret < 0)
		pr_info("%s sprintf fail\n", __func__);

	for (i = 0; i < PLL_NUM; i++) {
		ret = sprintf(buf + strlen(buf), " %s |", pll_name[i]);
		if (ret < 0)
			pr_info("%s sprintf fail\n", __func__);
	}
	pr_info("%s\n", buf);

	for (i = 0; i < mt8195_opp_tbl->tbl_size; i++) {
		buf[0] = 0;
		ret = sprintf(buf + strlen(buf), "| %d |   %d  |   %d  |",
			i,
			mt8195_opp_tbl->opp[i].vapu,
			mt8195_opp_tbl->opp[i].vmdla);
		if (ret < 0)
			pr_info("%s sprintf fail\n", __func__);

		for (j = 0; j < PLL_NUM; j++) {
			ret = sprintf(buf + strlen(buf),
				"  %07d |", mt8195_opp_tbl->opp[i].pll_freq[j]);
			if (ret < 0)
				pr_info("%s sprintf fail\n", __func__);
		}

		pr_info("%s\n", buf);
	}

	return 0;
}

static int init_plat_pwr_res(struct platform_device *pdev)
{
	int ret_clk = 0, ret = 0;

	LOG_DBG("%s %d ++\n", __func__, __LINE__);

	/* vsram */
	vsram_reg_id = regulator_get(&pdev->dev, "vsram");
	if (!vsram_reg_id)
		pr_info("regulator_get vsram_reg_id failed\n");

#if !APU_POWER_BRING_UP
	/* vcore */
	vcore_reg_id = regulator_get(&pdev->dev, "vcore");
	if (!vcore_reg_id)
		pr_info("regulator_get vcore_reg_id failed\n");
#endif

	/* vapu */
	vapu_reg_id = regulator_get(&pdev->dev, "vapu");
	if (!vapu_reg_id)
		pr_info("regulator_get vapu_reg_id failed\n");

	/* vmdla */
	vmdla_reg_id = regulator_get(&pdev->dev, "vmdla");
	if (!vmdla_reg_id)
		pr_info("regulator_get vmdla_reg_id failed\n");

	/* devm_clk_get , not real prepare_clk */
	PREPARE_CLK(clk_top_dsp_sel);
	PREPARE_CLK(clk_top_dsp1_sel);
	PREPARE_CLK(clk_top_dsp2_sel);
	PREPARE_CLK(clk_top_dsp4_sel);
	PREPARE_CLK(clk_top_dsp5_sel);
	PREPARE_CLK(clk_top_ipu_if_sel);
	PREPARE_CLK(clk_apupll_apupll);
	PREPARE_CLK(clk_apupll_npupll);
	PREPARE_CLK(clk_apupll_apupll1);
	PREPARE_CLK(clk_apupll_apupll2);
	PREPARE_CLK(clk_infra_ao_debugsys);
	if (ret_clk < 0)
		return ret_clk;

	LOG_DBG("%s %d --\n", __func__, __LINE__);
	return 0;
}

static void destroy_plat_pwr_res(void)
{
	UNPREPARE_CLK(clk_infra_ao_debugsys);
	UNPREPARE_CLK(clk_apupll_apupll2);
	UNPREPARE_CLK(clk_apupll_apupll1);
	UNPREPARE_CLK(clk_apupll_npupll);
	UNPREPARE_CLK(clk_apupll_apupll);
	UNPREPARE_CLK(clk_top_ipu_if_sel);
	UNPREPARE_CLK(clk_top_dsp5_sel);
	UNPREPARE_CLK(clk_top_dsp4_sel);
	UNPREPARE_CLK(clk_top_dsp2_sel);
	UNPREPARE_CLK(clk_top_dsp1_sel);
	UNPREPARE_CLK(clk_top_dsp_sel);

	if (vmdla_reg_id)
		regulator_put(vmdla_reg_id);
	if (vapu_reg_id)
		regulator_put(vapu_reg_id);
	if (vcore_reg_id)
		regulator_put(vcore_reg_id);
	if (vsram_reg_id)
		regulator_put(vsram_reg_id);

	vapu_reg_id  = NULL;
	vmdla_reg_id = NULL;
	vcore_reg_id = NULL;
	vsram_reg_id = NULL;
}

#if ENABLE_SW_BUCK_CTL
static void plt_pwr_res_ctl(int enable)
{
#if ENABLE_SW_BUCK_CTL
	int ret = 0;
#endif
	int ret_clk = 0;

	LOG_DBG("%s %d ++\n", __func__, __LINE__);

	if (enable) {
#if ENABLE_SW_BUCK_CTL
		if (vsram_reg_id) {
			ret = regulator_enable(vsram_reg_id);
			if (ret < 0)
				pr_info("%s fail enable vapu sram : %d\n",
					__func__, ret);
			udelay(300);
		}

		if (vapu_reg_id) {
			ret = regulator_enable(vapu_reg_id);
			if (ret < 0)
				pr_info("%s fail enable vapu : %d\n",
					__func__, ret);
			udelay(300);
		}

		if (vmdla_reg_id) {
			ret = regulator_enable(vmdla_reg_id);
			if (ret < 0)
				pr_info("%s fail enable vmdla : %d\n",
					__func__, ret);
			udelay(300);
		}

		apu_writel(
			apu_readl(apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION)
			& ~(0x00000021),
			apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION);
#else
		LOG_DBG("%s skip enable regulator since HW auto\n", __func__);
#endif

#if ENABLE_SOC_CLK_MUX
		/* enable all soc clk src of apu */
		ENABLE_CLK(clk_top_dsp_sel);
		ENABLE_CLK(clk_top_dsp1_sel);
		ENABLE_CLK(clk_top_dsp2_sel);
		ENABLE_CLK(clk_top_dsp4_sel);
		ENABLE_CLK(clk_top_dsp5_sel);
		ENABLE_CLK(clk_top_ipu_if_sel);
#endif
		/* enable all apupll */
		ENABLE_CLK(clk_apupll_apupll);
		ENABLE_CLK(clk_apupll_npupll);
		ENABLE_CLK(clk_apupll_apupll1);
		ENABLE_CLK(clk_apupll_apupll2);
		ENABLE_CLK(clk_infra_ao_debugsys);
		if (ret_clk < 0)
			pr_info("%s fail enable clk : %d\n", __func__, ret_clk);

		/* enable all acc */
		LOG_DBG("%s set all acc CGEN_APU\n", __func__);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_SET0);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_SET1);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_SET2);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_SET4);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_SET5);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_SET7);

	} else {

		/* disable all acc */
		LOG_DBG("%s clear all acc CGEN_APU\n", __func__);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_CLR7);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_CLR5);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_CLR4);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_CLR2);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_CLR1);
		apu_writel(BIT_CGEN_APU,
				apupw.regs[apu_acc] + APU_ACC_CONFG_CLR0);

#if ENABLE_SOC_CLK_MUX
		/* disable all soc clk src of apu */
		DISABLE_CLK(clk_top_ipu_if_sel);
		DISABLE_CLK(clk_top_dsp5_sel);
		DISABLE_CLK(clk_top_dsp4_sel);
		DISABLE_CLK(clk_top_dsp2_sel);
		DISABLE_CLK(clk_top_dsp1_sel);
		DISABLE_CLK(clk_top_dsp_sel);
#endif
		/* disable all apupll */
		DISABLE_CLK(clk_apupll_apupll2);
		DISABLE_CLK(clk_apupll_apupll1);
		DISABLE_CLK(clk_apupll_npupll);
		DISABLE_CLK(clk_apupll_apupll);
		DISABLE_CLK(clk_infra_ao_debugsys);

#if !APU_POWER_BRING_UP
#if ENABLE_SW_BUCK_CTL
		apu_writel(
			apu_readl(apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION)
			| 0x00000021,
			apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION);

		if (vmdla_reg_id) {
			ret = regulator_disable(vmdla_reg_id);
			if (ret < 0)
				pr_info("%s fail disable vmdla : %d\n",
					__func__, ret);
			udelay(10);
		}

		if (vapu_reg_id) {
			ret = regulator_disable(vapu_reg_id);
			if (ret < 0)
				pr_info("%s fail disable vapu : %d\n",
					__func__, ret);
			udelay(10);
		}

		if (vsram_reg_id) {
			ret = regulator_disable(vsram_reg_id);
			if (ret < 0)
				pr_info("%s fail disable vapu sram : %d\n",
					__func__, ret);
			udelay(10);
		}

#else
		LOG_DBG("%s skip disable regulator since HW auto\n", __func__);
#endif /* ENABLE_SW_BUCK_CTL */
#endif /* !APU_POWER_BRING_UP */
	}

	LOG_DBG("%s %d --\n", __func__, __LINE__);
}
#endif /*  ENABLE_SW_BUCK_CTL */

#if APU_POWER_INIT
static void __apu_pll_init(void)
{
	LOG_DBG("PLL init %s %d ++\n", __func__, __LINE__);

	/* Step2. Initial clock setting (by calling CCF driver) for all PLL */
	clk_set_rate(clk_apupll_apupll1, MNOC_DEFAULT_FREQ * MHZ_95);
	clk_set_rate(clk_apupll_apupll2, MNOC_DEFAULT_FREQ * MHZ_95);
	clk_set_rate(clk_apupll_npupll, MVPU_DEFAULT_FREQ * MHZ_95);
	clk_set_rate(clk_apupll_apupll, MDLA_DEFAULT_FREQ * MHZ_95);

	LOG_DBG("PLL init %s %d --\n", __func__, __LINE__);
}

static void __apu_acc_init(void)
{
#if LOCAL_DBG
	char buf[32];
#endif

	LOG_DBG("ACC init %s %d ++\n", __func__, __LINE__);

	/* Step4. Initial ACC setting */

	/* mnoc, uP clk setting */
	LOG_DBG("mnoc clk setting %s %d\n", __func__, __LINE__);
	apu_writel(BIT_SEL_APU, apupw.regs[apu_acc] + APU_ACC_CONFG_SET0);
	apu_writel(BIT_CGEN_SOC, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR0);

	/* iommu clk setting */
	LOG_DBG("iommu clk setting %s %d\n", __func__, __LINE__);
	apu_writel(BIT_SEL_APU, apupw.regs[apu_acc] + APU_ACC_CONFG_SET7);
	apu_writel(BIT_CGEN_SOC, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR7);

	/* vpu0/1 clk setting */
	LOG_DBG("vpu0/1 clk setting %s %d\n", __func__, __LINE__);
	apu_writel(BIT_SEL_APU, apupw.regs[apu_acc] + APU_ACC_CONFG_SET1);
	apu_writel(BIT_CGEN_SOC, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR1);

	apu_writel(BIT_SEL_APU, apupw.regs[apu_acc] + APU_ACC_CONFG_SET2);
	apu_writel(BIT_CGEN_SOC, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR2);
	apu_writel(BIT_INVEN_OUT, apupw.regs[apu_acc] + APU_ACC_CONFG_SET2);

	/* mdla0/1 clk setting */
	LOG_DBG("mdla0/1 clk setting %s %d\n", __func__, __LINE__);
	apu_writel(BIT_SEL_APU, apupw.regs[apu_acc] + APU_ACC_CONFG_SET4);
	apu_writel(BIT_CGEN_SOC, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR4);

	apu_writel(BIT_SEL_APU, apupw.regs[apu_acc] + APU_ACC_CONFG_SET5);
	apu_writel(BIT_CGEN_SOC, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR5);
	apu_writel(BIT_INVEN_OUT, apupw.regs[apu_acc] + APU_ACC_CONFG_SET5);

#if LOCAL_DBG
	memset(buf, 0, sizeof(buf));
	snprintf(buf, 32, "phys 0x%08x: ", (u32)(apupw.phy_addr[apu_acc]));
	print_hex_dump(KERN_ERR, buf, DUMP_PREFIX_OFFSET, 16, 4,
			apupw.regs[apu_acc], 0x100, true);
#endif

	LOG_DBG("ACC init %s %d --\n", __func__, __LINE__);
}

static void __apu_buck_off_cfg(void)
{
	LOG_DBG("%s %d ++\n", __func__, __LINE__);

	/*
	 * Step8. Buck els setting
	 * (Manually enable buck els enable @RPC
	 */
	apu_setl((0x1 << 3), apupw.regs[apu_rpc] + APU_RPC_TOP_CON);

	/*
	 * Manually turn off Buck (by configuring register in PMIC)
	 * (this op have move to probe last line)
	 */
	LOG_DBG("%s %d --\n", __func__, __LINE__);
}
#endif /* APU_POWER_INIT */

#if DEBUG_DUMP_REG
static void aputop_dump_all_reg(void)
{
	int idx = 0;
	char buf[32];

	for (idx = 0; idx < APUPW_MAX_REGS; idx++) {
		memset(buf, 0, sizeof(buf));
		snprintf(buf, 32, "phys 0x%08x ", (u32)(apupw.phy_addr[idx]));
		print_hex_dump(KERN_ERR, buf, DUMP_PREFIX_OFFSET, 16, 4,
				apupw.regs[idx], 0x1000, true);
	}
}
#endif /* DEBUG_DUMP_REG */

void mt8195_apu_dump_pwr_status(struct rpc_status_dump *dump)
{
	uint32_t status1 = 0x0;
	uint32_t status2 = 0x0;
	uint32_t status3 = 0x0;
	uint32_t status4 = 0x0;
	uint32_t status5 = 0x0;
	uint32_t status6 = 0x0;
	uint32_t status7 = 0x0;

	status1 = apu_readl(apupw.regs[apu_rpc]
			+ APU_RPC_INTF_PWR_RDY);
	status2 = apu_readl(apupw.regs[apu_vcore]
			+ APUSYS_VCORE_CG_CON);
	status3 = apu_readl(apupw.regs[apu_conn]
			+ APUSYS_CONN_CG_CON);
	status4 = apu_readl(apupw.regs[apu_conn1]
			+ APUSYS_CONN1_CG_CON);
	status5 = apu_readl(apupw.regs[apu_rpc]
	+ 0x004c);
	status6 = apu_readl(apupw.regs[apu_rpc]
	+ 0x0050);
	status7 = apu_readl(apupw.regs[apu_rpc]
	+ 0x0054);
	pr_info("%s APU_RPC_INTF_PWR_RDY:0x%08x APU_VCORE_CG_CON:0x%08x\n",
		__func__, status1, status2);
	pr_info("%s APU_CONN_CG_CON:0x%08x APU_CONN1_CG_CON:0x%08x\n",
		__func__, status3, status4);
	pr_info("%s status 5:0x%08x status 5:0x%08x status 7:0x%08x\n",
		__func__, status5, status6,status7);
	/*
	 * print_hex_dump(KERN_ERR, "rpc: ", DUMP_PREFIX_OFFSET,
	 *		16, 4, apupw.regs[apu_rpc], 0x100, 1);
	 */

	if (!IS_ERR_OR_NULL(dump)) {
		dump->rpc_reg_status = status1;
		dump->vcore_reg_status = status2;
		dump->conn_reg_status = status3;
		dump->conn1_reg_status = status4;
	}
}

#if APU_POWER_INIT
static void __apu_rpc_init(void)
{
	LOG_DBG("RPC init %s %d ++\n", __func__, __LINE__);

	/*
	 * Step5. RPCTop: memory types (sleep or PD type)
	 * 8195 not support md32 suspend mode (no buck off with sram retention).
	 * It is unnecessary to setup sleep type.
	 *
	 * Step6. PCUTop initial
	 * 8195 ctrl buck at kernel. It is unnecessary to init pcu if we dont
	 * need to enable buck auto ctrl feature.
	 */

	/* Step7. RPCTop initial */
#if ENABLE_SW_BUCK_CTL
	apu_setl(0x0000009E, apupw.regs[apu_rpc] + APU_RPC_TOP_SEL);
#else
	apu_setl(0x0000409E, apupw.regs[apu_rpc] + APU_RPC_TOP_SEL);
#endif

	LOG_DBG("RPC init %s %d --\n", __func__, __LINE__);
}
#endif /* APU_POWER_INIT */

#if APU_POWER_BRING_UP
static int __apu_pwr_ctl_engines(struct device *dev,
					enum t_dev_id dev_id, int pwron)
{
	int ret = 0, val = 0;
	uint32_t dev_mtcmos_ctl;
	void *dev_cg_con, *dev_cg_clr;
	uint32_t dev_mtcmos_chk;

	/* we support power only for bringup */
	switch (dev_id) {
	case VPU0:
		dev_mtcmos_ctl = (0x2 | (0x1 << 4));
		dev_mtcmos_chk = 0x4UL;
		dev_cg_con = apupw.regs[apu_vpu0] + APU_VPU_CG_CON;
		dev_cg_clr = apupw.regs[apu_vpu0] + APU_VPU_CG_CLR;
		break;
	case VPU1:
		dev_mtcmos_ctl = (0x3 | (0x1 << 4));
		dev_mtcmos_chk = 0x8UL;
		dev_cg_con = apupw.regs[apu_vpu1] + APU_VPU_CG_CON;
		dev_cg_clr = apupw.regs[apu_vpu1] + APU_VPU_CG_CLR;
		break;
	case DLA0:
		dev_mtcmos_ctl = (0x6 | (0x1 << 4));
		dev_mtcmos_chk = 0x40UL;
		dev_cg_con = apupw.regs[apu_mdla0] + APU_MDLA_CG_CON;
		dev_cg_clr = apupw.regs[apu_mdla0] + APU_MDLA_CG_CLR;
		break;
	case DLA1:
		dev_mtcmos_ctl = (0x7 | (0x1 << 4));
		dev_mtcmos_chk = 0x80UL;
		dev_cg_con = apupw.regs[apu_mdla1] + APU_MDLA_CG_CON;
		dev_cg_clr = apupw.regs[apu_mdla1] + APU_MDLA_CG_CLR;
		break;
	default:
		goto out;
	}

	dev_dbg(dev, "%s ctl p1:0x%x p2:0x%x p3:0x%x p4:0x%x\n",
		__func__, dev_mtcmos_ctl, dev_mtcmos_chk,
		dev_cg_con, dev_cg_clr);

	apu_writel(dev_mtcmos_ctl, apupw.regs[apu_rpc] + APU_RPC_SW_FIFO_WE);
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY),
			val, (val & dev_mtcmos_chk) == dev_mtcmos_chk, 50, 200000);
	if (ret) {
		pr_info("%s config rpc 0x%x fail, ret %d\n",
				__func__, dev_mtcmos_ctl, ret);
		goto out;
	}

	dev_dbg(dev, "%s APU_RPC_INTF_PWR_RDY 0x%x = 0x%x\n",
		__func__,
		(u32)(apupw.phy_addr[apu_rpc] + APU_RPC_INTF_PWR_RDY),
		readl(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY));

	apu_writel(0xFFFFFFFF, dev_cg_clr);

	dev_dbg(dev, "%s dev%d CG_CON 0x%x = 0x%x\n",
		__func__, dev_id,
		dev_cg_con, readl(dev_cg_con));
out:
	return ret;
}
#endif /* APU_POWER_BRING_UP */

static int __apu_wake_rpc_top(struct device *dev)
{
	int ret = 0, val = 0;

	/* rpc pwr from spm */
	apu_writel((1 << 0),
			apupw.regs[sys_spm] + APUSYS_SPM_CROSS_WAKE_M01_REQ);

	LOG_DBG("%s SPM: wait until PWR_ACK = 1\n", __func__);
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[sys_spm] + APUSYS_OTHER_PWR_STATUS),
			val, (val & (0x1UL << 4)), 50, 10000);
	if (ret) {
		pr_info("%s spm: wait pwr ack = 1 timeout, ret %d\n",
			__func__, ret);
		goto out;
	}

	LOG_DBG("%s RPC: wait until PWR ON complete = 1\n", __func__);
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY),
			val, (val & 0x1UL), 50, 10000);
	if (ret) {
		pr_info("%s rpc: wait pwr on complete = 1 timeout, ret %d\n",
			__func__, ret);
		goto out;
	}

	LOG_DBG("%s RCX APU_RPC_INTF_PWR_RDY 0x%x = 0x%x\n",
		__func__,
		(u32)(apupw.phy_addr[apu_rpc] + APU_RPC_INTF_PWR_RDY),
		readl(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY));


	/* bus/sleep prot CG on */
	LOG_DBG("%s bus/sleep protect CG on\n", __func__);
	apu_writel(
		apu_readl(apupw.regs[apu_ao_ctl] + CSR_DUMMY_0_ADDR) & ~(0x3),
		apupw.regs[apu_ao_ctl] + CSR_DUMMY_0_ADDR);

	/* clear vcore/conn/conn1 cgs */
	LOG_DBG("clear vcore/conn/conn1 cgs %s %d\n", __func__, __LINE__);
	apu_writel(0xFFFFFFFF, apupw.regs[apu_vcore] + APUSYS_VCORE_CG_CLR);
	apu_writel(0xFFFFFFFF, apupw.regs[apu_conn] + APUSYS_CONN_CG_CLR);
	apu_writel(0xFFFFFFFF, apupw.regs[apu_conn1] + APUSYS_CONN1_CG_CLR);

	LOG_DBG("%s APUSYS_VCORE_CG_CON 0x%x = 0x%x\n",
		__func__,
		(u32)(apupw.phy_addr[apu_vcore] + APUSYS_VCORE_CG_CON),
		readl(apupw.regs[apu_vcore] + APUSYS_VCORE_CG_CON));

	LOG_DBG("%s APUSYS_CONN_CG_CON 0x%x = 0x%x\n",
		__func__,
		(u32)(apupw.phy_addr[apu_conn] + APUSYS_CONN_CG_CON),
		readl(apupw.regs[apu_conn] + APUSYS_CONN_CG_CON));

	LOG_DBG("%s APUSYS_CONN1_CG_CON 0x%x = 0x%x\n",
		__func__,
		(u32)(apupw.phy_addr[apu_conn1] + APUSYS_CONN1_CG_CON),
		readl(apupw.regs[apu_conn1] + APUSYS_CONN1_CG_CON));

out:
	return ret;
}



#if APMCU_REQ_RPC_SLEEP
static int __apu_sleep_rpc_top(struct device *dev)
{
	/* clr vcore/conn/conn1 cgs for eng cg auto gating */
	LOG_DBG("set vcore/conn/conn1 cgs %s %d\n", __func__, __LINE__);
	apu_writel(0xFFFFFFFF, apupw.regs[apu_vcore] + APUSYS_VCORE_CG_CLR);
	apu_writel(0xFFFFFFFF, apupw.regs[apu_conn] + APUSYS_CONN_CG_CLR);
	apu_writel(0xFFFFFFFF, apupw.regs[apu_conn1] + APUSYS_CONN1_CG_CLR);


	LOG_DBG("%s SPM:  subsys power off (APU_CONN/APU_VCORE)\n", __func__);
	apu_writel(
		apu_readl(apupw.regs[sys_spm] + APUSYS_SPM_CROSS_WAKE_M01_REQ)
		& ~(0x1),
		apupw.regs[sys_spm] + APUSYS_SPM_CROSS_WAKE_M01_REQ);

	LOG_DBG("%s RPC:  sleep request enable\n", __func__);
	apu_writel((1 << 0), apupw.regs[apu_rpc] + APU_RPC_TOP_CON);

	return 0;
}
#endif

static int mt8195_apu_top_on(struct device *dev)
{
	int ret = 0;

	LOG_DBG("%s +\n", __func__);
#if ENABLE_SW_BUCK_CTL
	plt_pwr_res_ctl(1);
#endif
	ret = __apu_wake_rpc_top(dev);

	if (ret) {
		pr_info("%s fail to wakeup RPC, ret %d\n", __func__, ret);
		apupw_aee_warn("APUSYS_POWER", "APUSYS_POWER_WAKEUP_FAIL");
		return -1;
	}

	mt8195_apu_devfreq_cooling_start();

	LOG_DBG("%s -\n", __func__);
	return 0;
}

static int mt8195_apu_top_off(struct device *dev)
{
	int ret = 0, val = 0;
	int rpc_timeout_val = 500000; /* 500 ms */

	LOG_DBG("%s +\n", __func__);
	mt8195_apu_dump_pwr_status(NULL);
	/* TODO: temp mark, need to modify common part */
	mt8195_apu_devfreq_cooling_stop();
#if APMCU_REQ_RPC_SLEEP
	/* backup solution : send request for RPC sleep from APMCU */
	__apu_sleep_rpc_top(dev);
#else
	mt8195_pwr_flow_remote_sync(1); /* tell remote side I am ready to off */
#endif
	/* blocking until sleep success or timeout */
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY),
			val, (val & 0x1UL) == 0x0, 50, rpc_timeout_val);
	if (ret) {
		pr_info("%s timeout to wait RPC sleep (val:%d), ret %d\n",
				__func__, rpc_timeout_val, ret);
		apupw_aee_warn("APUSYS_POWER", "APUSYS_POWER_SLEEP_TIMEOUT");
		mt8195_apu_dump_pwr_status(NULL);
		return -1;
	}

#if ENABLE_SW_BUCK_CTL
	plt_pwr_res_ctl(0);
#endif

	return 0;
}

#if APU_POWER_INIT
static void __apu_aoc_init(void)
{
	LOG_DBG("AOC init %s %d ++\n", __func__, __LINE__);

	/* Step1. Manually disable buck els enable @SOC */
	apu_clearl(((0x1 << 5) | (0x1 << 0)),
			apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION);

	LOG_DBG("AOC init %s %d --\n", __func__, __LINE__);
}
#endif /* APU_POWER_INIT */

static int init_plat_chip_data(struct platform_device *pdev)
{
	struct plat_cfg_data plat_cfg;
	uint32_t aging_attr = 0x0;
	int i;

	memset(&plat_cfg, 0, sizeof(plat_cfg));

	of_property_read_u32(pdev->dev.of_node, "aging_load", &aging_attr);

	plat_cfg.aging_flag = (aging_attr & 0xf);
	plat_cfg.hw_id = 0x0;

	/* hint uP each apupll max freq */
	for (i = 0; i < PLL_NUM; i++)
		plat_cfg.freq[i] = mt8195_opp_tbl->opp[0].pll_freq[i] / 1000; /* MHZ */

	/* hint uP if apu volt is fixed (no DVFS) */
	for (i = 1; i < mt8195_opp_tbl->tbl_size; i++)
		if (mt8195_opp_tbl->opp[0].vapu != mt8195_opp_tbl->opp[i].vapu)
			break;

	plat_cfg.fix_volt = (i == mt8195_opp_tbl->tbl_size) ? 1 : 0;
	LOG_DBG("%s 0x%08x 0x%08x 0x%08x\n", __func__,
		plat_cfg.aging_flag,
		plat_cfg.hw_id,
		aging_attr);

	return mt8195_chip_data_remote_sync(&plat_cfg);
}
#if APU_POWER_INIT
static int init_hw_setting(struct device *dev)
{
	__apu_aoc_init();
	__apu_pll_init();
	__apu_acc_init();
	__apu_rpc_init();
	__apu_buck_off_cfg();

	return 0;
}
#endif /* APU_POWER_INIT */

static int init_reg_base(struct platform_device *pdev)
{
	struct resource *res;
	int idx = 0;

	LOG_DBG("%s %d APUPW_MAX_REGS = %d\n",
			__func__, __LINE__, APUPW_MAX_REGS);

	for (idx = 0; idx < APUPW_MAX_REGS; idx++) {

		res = platform_get_resource_byname(
				pdev, IORESOURCE_MEM, reg_name[idx]);

		if (res == NULL) {
			pr_info("%s: get resource \"%s\" fail\n",
					__func__, reg_name[idx]);
			return -ENODEV;
		}

		LOG_DBG("%s: get resource \"%s\" pass\n",
				__func__, reg_name[idx]);

		apupw.regs[idx] = ioremap(res->start,
				res->end - res->start + 1);

		if (IS_ERR_OR_NULL(apupw.regs[idx])) {
			pr_info("%s: %s remap base fail\n",
					__func__, reg_name[idx]);
			return -ENOMEM;
		}

		LOG_DBG("%s: %s remap base 0x%llx to 0x%p\n",
				__func__, reg_name[idx],
				res->start, apupw.regs[idx]);

		apupw.phy_addr[idx] = res->start;
	}

	return 0;
}


static int mt8195_apu_top_pb(struct platform_device *pdev)
{
#if APU_POWER_INIT
	int ret_clk = 0;
#endif
	int ret = 0;

	pr_info("%s fpga_type : %d\n", __func__, fpga_type);

	init_reg_base(pdev);
	init_opp_table(pdev);
	init_plat_pwr_res(pdev);

	/* enable vsram buck */
	pr_info("%s regulator_enable vsram_reg_id\n", __func__);
	if (vsram_reg_id) {
		ret = regulator_enable(vsram_reg_id);
		if (ret)
			pr_info("[%s][%d] enable vsram regulator failed\n",
				__func__, __LINE__);
		udelay(300);
	}
	pr_info("%s regulator_enable vapu_reg_id\n", __func__);
	/* enable vapu buck */
	if (vapu_reg_id) {
		ret = regulator_enable(vapu_reg_id);
		if (ret)
			pr_info("[%s][%d] enable vapu regulator failed\n",
				__func__, __LINE__);
		udelay(300);
	}

	/* enable vmdla buck */
	if (vmdla_reg_id) {
		ret = regulator_enable(vmdla_reg_id);
		if (ret)
			pr_info("[%s][%d] enable vmdla regulator failed\n",
				__func__, __LINE__);
		udelay(300);
	}

	/* set vsram to default voltage */
	if (vsram_reg_id) {
		regulator_set_voltage(vsram_reg_id, VSRAM_DEF_VOLT,
							VSRAM_DEF_VOLT);
		LOG_DBG("%s vapu sram:%d (en:%d)\n", __func__,
				regulator_get_voltage(vsram_reg_id),
				regulator_is_enabled(vsram_reg_id));
	}

	/* set vapu to default voltage */
	if (vapu_reg_id) {
		regulator_set_voltage(vapu_reg_id, VAPU_DEF_VOLT,
							VAPU_DEF_VOLT);
		LOG_DBG("%s vapu:%d (en:%d)\n", __func__,
				regulator_get_voltage(vapu_reg_id),
				regulator_is_enabled(vapu_reg_id));
	}

	/* set vmdla to default voltage */
	if (vmdla_reg_id) {
		regulator_set_voltage(vmdla_reg_id, VMDLA_DEF_VOLT,
							VMDLA_DEF_VOLT);
		LOG_DBG("%s vmdla:%d (en:%d)\n", __func__,
				regulator_get_voltage(vapu_reg_id),
				regulator_is_enabled(vapu_reg_id));
	}
#if APU_POWER_INIT
	ENABLE_CLK(clk_top_dsp_sel);
	ENABLE_CLK(clk_top_dsp1_sel);
	ENABLE_CLK(clk_top_dsp2_sel);
	ENABLE_CLK(clk_top_dsp4_sel);
	ENABLE_CLK(clk_top_dsp5_sel);
	ENABLE_CLK(clk_top_ipu_if_sel);
	ENABLE_CLK(clk_apupll_apupll);
	ENABLE_CLK(clk_apupll_npupll);
	ENABLE_CLK(clk_apupll_apupll1);
	ENABLE_CLK(clk_apupll_apupll2);
	ENABLE_CLK(clk_infra_ao_debugsys);
	/* before apu power init, need to ensure soc regulator/clk is ready */
	init_hw_setting(&pdev->dev);

#if !APU_POWER_BRING_UP
	DISABLE_CLK(clk_infra_ao_debugsys);
	DISABLE_CLK(clk_apupll_apupll2);
	DISABLE_CLK(clk_apupll_apupll1);
	DISABLE_CLK(clk_apupll_npupll);
	DISABLE_CLK(clk_apupll_apupll);
	DISABLE_CLK(clk_top_ipu_if_sel);
	DISABLE_CLK(clk_top_dsp5_sel);
	DISABLE_CLK(clk_top_dsp4_sel);
	DISABLE_CLK(clk_top_dsp2_sel);
	DISABLE_CLK(clk_top_dsp1_sel);
	DISABLE_CLK(clk_top_dsp_sel);
#endif /* !APU_POWER_BRING_UP */
#endif /* APU_POWER_INIT */

	/* set vmdla to default voltage */
	if (vmdla_reg_id)
		regulator_set_voltage(vmdla_reg_id,
				      VAPU_DEF_VOLT, VAPU_DEF_VOLT);

	/* set vapu to default voltage */
	if (vapu_reg_id)
		regulator_set_voltage(vapu_reg_id,
				      VAPU_DEF_VOLT, VAPU_DEF_VOLT);


	/* disable vmdla buck */
	if (vmdla_reg_id) {
		ret = regulator_disable(vmdla_reg_id);
		if (ret)
			return ret;
		udelay(10);
	}

	/* disable vapu buck */
	if (vapu_reg_id) {
		ret = regulator_disable(vapu_reg_id);
		if (ret)
			return ret;
		udelay(10);
	}

	/* disable vsram buck */
	if (vsram_reg_id) {
		ret = regulator_disable(vsram_reg_id);
		if (ret)
			return ret;
		udelay(10);
	}

	mt8195_init_remote_data_sync(apupw.regs[apu_md32_mbox]);
	init_plat_chip_data(pdev);

	mt8195_apu_devfreq_cooling_init(pdev, apupw.regs[apu_md32_mbox]);
#if APU_POWER_BRING_UP
	switch (fpga_type) {
	default:
	case 0: /* do not power on */
		pr_info("%s bypass pre-power-ON\n", __func__);
		break;
	case 2:
		/* use pm domain to pwr on apu top */
		pm_runtime_get_sync(&pdev->dev);

		/* pwr on apu devices */
		__apu_pwr_ctl_engines(&pdev->dev, VPU0, 1);
		__apu_pwr_ctl_engines(&pdev->dev, VPU1, 1);
		__apu_pwr_ctl_engines(&pdev->dev, DLA0, 1);
		__apu_pwr_ctl_engines(&pdev->dev, DLA1, 1);
		break;
	}
//#else
	//pm_runtime_get_sync(&pdev->dev);
#endif /* APU_POWER_BRING_UP */

	aputop_dump_pwr_res();

	platform_set_drvdata(pdev, &apupw);

	return ret;
}

static int mt8195_apu_top_rm(struct platform_device *pdev)
{
	int idx;

	LOG_DBG("%s +\n", __func__);

	destroy_plat_pwr_res();

	for (idx = 0; idx < APUPW_MAX_REGS; idx++)
		iounmap(apupw.regs[idx]);

	kfree(mt8195_opp_tbl);
	mt8195_opp_tbl = NULL;

	LOG_DBG("%s -\n", __func__);

	return 0;
}

static int mt8195_apu_top_prepare(struct device *dev)
{
	// Before entering suspend mode, we check whether the apu is still working?
	// If apu is working now, retrun EBUSY to stop suspend flow
	uint32_t status = apu_readl(apupw.regs[apu_rpc]
			+ APU_RPC_INTF_PWR_RDY);

	if (status & 1) {
		pr_info("%s apu is still working, not allow to enter suspend mode\n",
			__func__);
		return -EBUSY;
	} else {
		return 0;
	}
}

static int mt8195_apu_top_suspend(struct device *dev)
{
	g_opp_cfg_acx0 = apu_readl(
			apupw.regs[apu_md32_mbox] + ACX0_LIMIT_OPP_REG);

	LOG_DBG("%s backup data 0x%08x\n", __func__,
			g_opp_cfg_acx0);
	return 0;
}

static int mt8195_apu_top_resume(struct device *dev)
{
	LOG_DBG("%s restore data 0x%08x\n", __func__,
			g_opp_cfg_acx0);

	apu_writel(g_opp_cfg_acx0,
			apupw.regs[apu_md32_mbox] + ACX0_LIMIT_OPP_REG);

	return 0;
}
static void aputop_dump_pwr_res(void)
{
	int vapu_en = 0, vapu_mode = 0, vmdla_en = 0, vmdla_mode = 0;
	uint32_t vapu = 0;
	uint32_t vmdla = 0;
	uint32_t vcore = 0;
	uint32_t vsram = 0;

	if (vapu_reg_id) {
		vapu = regulator_get_voltage(vapu_reg_id);
		vapu_en = regulator_is_enabled(vapu_reg_id);
		vapu_mode = regulator_get_mode(vapu_reg_id);
	}

	if (vmdla_reg_id) {
		vmdla = regulator_get_voltage(vmdla_reg_id);
		vmdla_en = regulator_is_enabled(vmdla_reg_id);
		vmdla_mode = regulator_get_mode(vmdla_reg_id);
	}

	if (vcore_reg_id)
		vcore = regulator_get_voltage(vcore_reg_id);

	if (vsram_reg_id)
		vsram = regulator_get_voltage(vsram_reg_id);

	pr_info("%s vapu:%u(en:%d,mode:%d) vmdla:%u(en:%d,mode:%d) vcore:%u vsram:%u\n",
			__func__, vapu, vapu_en, vapu_mode,
			vmdla, vmdla_en, vmdla_mode,
			vcore, vsram);

	pr_info("%s d:%ld d1:%ld d2:%ld d4:%ld d5:%ld dif:%ld\n",
			__func__,
			clk_get_rate(clk_top_dsp_sel),
			clk_get_rate(clk_top_dsp1_sel),
			clk_get_rate(clk_top_dsp2_sel),
			clk_get_rate(clk_top_dsp4_sel),
			clk_get_rate(clk_top_dsp5_sel),
			clk_get_rate(clk_top_ipu_if_sel));

	pr_info("%s apupll:%ld npupll:%ld apupll1:%ld apupll2: %ld\n",
			__func__,
			clk_get_rate(clk_apupll_apupll),
			clk_get_rate(clk_apupll_npupll),
			clk_get_rate(clk_apupll_apupll1),
			clk_get_rate(clk_apupll_apupll2));

	mt8195_apu_dump_pwr_status(NULL);
}

static int mt8195_apu_top_func(struct platform_device *pdev,
		enum aputop_func_id func_id, struct aputop_func_param *aputop)
{
	char buf[32];
	int ret = 0;

	pr_info("%s func_id : %d\n", __func__, aputop->func_id);

	switch (aputop->func_id) {
	case APUTOP_FUNC_PWR_OFF:
		pm_runtime_put_sync(&pdev->dev);
		break;
	case APUTOP_FUNC_PWR_ON:
		pm_runtime_get_sync(&pdev->dev);
		break;
	case APUTOP_FUNC_OPP_LIMIT_HAL:
		mt8195_aputop_opp_limit(aputop, OPP_LIMIT_HAL);
		break;
	case APUTOP_FUNC_OPP_LIMIT_DBG:
		mt8195_aputop_opp_limit(aputop, OPP_LIMIT_DEBUG);
		break;
	case APUTOP_FUNC_DUMP_REG:
		aputop_dump_pwr_res();

		memset(buf, 0, sizeof(buf));
		ret = snprintf(buf, 32, "phys 0x%08x: ",
				(u32)(apupw.phy_addr[apu_rpc]));
		if (ret <= 0)
			pr_info("%s: snprintf fail\n", __func__);
		print_hex_dump(KERN_ERR, buf, DUMP_PREFIX_OFFSET, 16, 4,
				apupw.regs[apu_rpc], 0x300, true);

		memset(buf, 0, sizeof(buf));
		ret = snprintf(buf, 32, "phys 0x%08x: ",
				(u32)(apupw.phy_addr[apu_pcu]));
		if (ret <= 0)
			pr_info("%s: snprintf fail\n", __func__);
		print_hex_dump(KERN_ERR, buf, DUMP_PREFIX_OFFSET, 16, 4,
				apupw.regs[apu_pcu], 0x100, true);

		apusys_pwr_smc_call(&pdev->dev,
				MTK_APUSYS_KERNEL_OP_APUSYS_PWR_DUMP, 0);
#if DEBUG_DUMP_REG
		aputop_dump_all_reg();
#endif
		break;
	case APUTOP_FUNC_DRV_CFG:
		mt8195_drv_cfg_remote_sync(aputop);
		break;
	case APUTOP_FUNC_IPI_TEST:
		test_ipi_wakeup_apu();
		break;
	default:
		pr_info("%s invalid func_id : %d\n", __func__, aputop->func_id);
		return -EINVAL;
	}

	return 0;
}

const struct apupwr_plat_data mt8195_plat_data = {
	.plat_name             = "mt8195_apupwr",
	.plat_aputop_on        = mt8195_apu_top_on,
	.plat_aputop_off       = mt8195_apu_top_off,
	.plat_aputop_pb        = mt8195_apu_top_pb,
	.plat_aputop_rm        = mt8195_apu_top_rm,
	.plat_aputop_prepare   = mt8195_apu_top_prepare,
	.plat_aputop_suspend   = mt8195_apu_top_suspend,
	.plat_aputop_resume    = mt8195_apu_top_resume,
	.plat_aputop_func      = mt8195_apu_top_func,
#if IS_ENABLED(CONFIG_DEBUG_FS)
	.plat_aputop_dbg_open  = mt8195_apu_top_dbg_open,
	.plat_aputop_dbg_write = mt8195_apu_top_dbg_write,
#endif
	.plat_rpmsg_callback   = mt8195_apu_top_rpmsg_cb,
	.plat_apu_devfreq_cooling_register      = mt8195_apu_devfreq_cooling_register,
	.plat_apu_devfreq_cooling_unregister    = mt8195_apu_devfreq_cooling_unregister,
	.plat_apu_devfreq_cooling_start_monitor = mt8195_apu_devfreq_cooling_start_monitor,
	.plat_apu_devfreq_cooling_stop_monitor  = mt8195_apu_devfreq_cooling_stop_monitor,
	.bypass_pwr_on         = 0,
	.bypass_pwr_off        = 0,
};
