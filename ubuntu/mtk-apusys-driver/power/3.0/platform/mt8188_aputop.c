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
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/thermal.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include "apusys_secure.h"
#include "aputop_rpmsg.h"
#include "apu_top.h"
#include "mt8188_apupwr.h"
#include "mt8188_apupwr_prot.h"
#include "mt8188_apu_devfreq_cooling.h"

#define LOCAL_DBG	(0)

int g_pwr_log_level = 7; //APUSYS_PWR_LOG_ERR;

/* Below reg_name has to 1-1 mapping DTS's name */
static const char *reg_name[APUPW_MAX_REGS] = {
	"apu_rpc", "sys_spm", "bcrm_fmem_pdn", "apu_rcx", "apu_vcore",
	"apu_md32_mbox", "apu_pcu", "apu_ao_ctl", "apu_pll", "apu_acc",
	"apu_are0", "apu_are1", "apu_are2",
	"apu_acx0", "apu_acx0_rpc_lite",
};

static struct apu_power apupw;
static uint32_t g_opp_cfg_acx0;

static void aputop_dump_pwr_res(void);

/* regulator id */
static struct regulator *vapu_reg_id;
static struct regulator *vsram_reg_id;
static struct regulator *vcore_dvfsrc_reg_id;

/* apu thermal */
#if THERMAL_TEMP_UPDATE
static const char *tz_name;
static struct work_struct apu_thermal_work;
static struct timer_list apu_thermal_timer;
#endif

/************** IMPORTANT !! *******************
 * APU Top preclk
 * The following name of each clock struct
 * MUST mapping to clock-names @dts
 **********************************************/
static struct clk *clk_top_dsp;		/* CONN */
static struct clk *clk_top_dsp1;
static struct clk *clk_top_dsp2;
static struct clk *clk_top_dsp3;
static struct clk *clk_top_dsp4;
static struct clk *clk_top_dsp5;
static struct clk *clk_top_dsp6;
static struct clk *clk_top_dsp7;

struct tiny_dvfs_opp_tbl *opp_tbl;

static uint32_t apusys_pwr_smc_call(struct device *dev, uint32_t smc_id,
		uint32_t a2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_APUSYS_CONTROL, smc_id,
			a2, 0, 0, 0, 0, 0, &res);
	if (((int) res.a0) < 0)
		dev_info(dev, "%s: smc call %d return error(%ld)\n",
				__func__,
				smc_id, res.a0);

	return res.a0;
}

#if APU_POWER_INIT
/* WARNING: can not call this API after acc initial or may cause bus hang ! */
static void dump_rpc_lite_reg(int line)
{
	pr_info("%s ln_%d acx%d APU_RPC_TOP_SEL=0x%08x\n",
			__func__, line, 0,
			apu_readl(apupw.regs[apu_acx0_rpc_lite]
				+ APU_RPC_TOP_SEL));
}
#endif

static void apu_vote_vcore_dvfsrc(int enable)
{
	if (enable) {
		/* vote vcore to vapu-safety voltage */
		if (vcore_dvfsrc_reg_id) {
			regulator_set_voltage(vcore_dvfsrc_reg_id,
						VCORE_CON_SAFE_VOLT, INT_MAX);
			pr_info("%s vote vcore-dvfsrc to %d\n", __func__,
				VCORE_CON_SAFE_VOLT);
		}
	} else {
		/* vote vcore to default voltage */
		if (vcore_dvfsrc_reg_id) {
			regulator_set_voltage(vcore_dvfsrc_reg_id,
						VCORE_CON_DEF_VOLT, INT_MAX);
			pr_info("%s vote vcore-dvfsrc to %d\n", __func__,
				VCORE_CON_DEF_VOLT);
		}
	}
}

static int init_opp_table(struct platform_device *pdev)
{
	static const char * const pll_name[] = {"PLL_CONN", "PLL_RV33",
				   "PLL_MVPU", "PLL_MDLA"};
	static const char * const buck_name[] = {"BUCK_VAPU", "BUCK_VSRAM",
					  "BUCK_VCORE"};
	struct tiny_dvfs_opp_entry tmp_entry = {0};
	struct of_phandle_iterator it;
	struct device_node *np, *child_np = NULL;
	struct device *dev = NULL;
	int i, j;
	int ret = 0;
	int opp_num = 0, opp_idx = 0, pll_idx = 0;
	int volt = 0;
	uint64_t freq = 0;
	char buf[128] = {0};

	dev = &pdev->dev;
	of_for_each_phandle(&it, ret, dev->of_node,
			    "operating-points-v2", NULL, 0) {
		np = of_node_get(it.node);
		if (!np) {
			dev_dbg(dev, "of_node_get fail\n");
			return -1;
		}

		if (opp_tbl == NULL) {
			opp_num = of_get_child_count(np);
			opp_tbl = kzalloc(sizeof(*opp_tbl) +
				opp_num * sizeof(struct tiny_dvfs_opp_entry),
				GFP_KERNEL);
			if (IS_ERR_OR_NULL(opp_tbl))
				return -1;

			opp_tbl->tbl_size = opp_num;
		}

		opp_idx = 0;
		do {
			child_np = of_get_next_available_child(np, child_np);
			if (child_np) {
				of_property_read_u64(child_np, "opp-hz", &freq);
				of_property_read_u32(child_np,
						     "opp-microvolt", &volt);

				if (opp_tbl->opp[opp_idx].vapu == 0) {
					opp_tbl->opp[opp_idx].vapu = volt;
				} else if (opp_tbl->opp[opp_idx].vapu < volt) {
					pr_info(
						"[%s][%d] opp[%d] volt mismatch (%d, %d)\n",
						__func__, __LINE__, opp_idx,
						opp_tbl->opp[opp_idx].vapu,
						volt);
				}

				do_div(freq, 1000); /* HZ to KHZ*/
				opp_tbl->opp[opp_idx].pll_freq[pll_idx]
					= (uint32_t)freq;
				LOG_DBG(
					"[%s][%d] opp[%d] pll[%d] freq = %lld\n",
					__func__, __LINE__,
					opp_idx, pll_idx, freq);
				opp_idx++;
			}
		} while (child_np);

		pll_idx++;
		of_node_put(np);
	}

	if (opp_tbl == NULL)
		return 0;

	/* Sort opp table by vapu */
	for (i = 0; i < (opp_tbl->tbl_size - 1); i++) {
		for (j = i + 1; j < opp_tbl->tbl_size; j++) {
			if (opp_tbl->opp[j].vapu > opp_tbl->opp[i].vapu) {
				memcpy(&tmp_entry, &opp_tbl->opp[j],
					sizeof(struct tiny_dvfs_opp_entry));
				memcpy(&opp_tbl->opp[j], &opp_tbl->opp[i],
					sizeof(struct tiny_dvfs_opp_entry));
				memcpy(&opp_tbl->opp[i], &tmp_entry,
					sizeof(struct tiny_dvfs_opp_entry));
			}
		}
	}

	/* first line */
	ret = sprintf(buf, "| # | %s |", buck_name[0]);
	if (ret < 0)
		pr_info("%s sprintf fail\n", __func__);

	for (i = 0; i < PLL_NUM; i++) {
		ret = sprintf(buf + strlen(buf), " %s |", pll_name[i]);
		if (ret < 0)
			pr_info("%s sprintf fail\n", __func__);
	}
	pr_info("%s\n", buf);

	for (i = 0; i < opp_tbl->tbl_size; i++) {
		buf[0] = 0;
		ret = sprintf(buf + strlen(buf),
			"| %d |   %d  |", i, opp_tbl->opp[i].vapu);
		if (ret < 0)
			pr_info("%s sprintf fail\n", __func__);

		for (j = 0; j < PLL_NUM; j++) {
			ret = sprintf(buf + strlen(buf),
				"  %07d |", opp_tbl->opp[i].pll_freq[j]);
			if (ret < 0)
				pr_info("%s sprintf fail\n", __func__);
		}

		pr_info("%s\n", buf);
	}

	return 0;
}
#if THERMAL_TEMP_UPDATE
static void mt8188_apu_thermal_timer_func(struct timer_list *t)
{
	schedule_work(&apu_thermal_work);
	mod_timer(t, jiffies + msecs_to_jiffies(TEMP_UPDATE_TIME));
}

static void mt8188_apu_update_temperature_work(struct work_struct *work)
{
	mt8188_apu_temperature_sync(tz_name);
}

static int init_apu_thermal(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;

	INIT_WORK(&apu_thermal_work, &mt8188_apu_update_temperature_work);

	if (of_property_read_string(node, "apusys,thermal-zones", &tz_name)) {
		pr_info("%s: parse apu thermal-zones name failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void destroy_apu_thermal(void)
{
	/* make sure no work_func running after timer delete */
	cancel_work_sync(&apu_thermal_work);
}
#endif /* THERMAL_TEMP_UPDATE */

static int init_plat_pwr_res(struct platform_device *pdev)
{
	int ret_clk = 0, ret = 0;

	LOG_DBG("%s %d ++\n", __func__, __LINE__);

	/* vsram */
	vsram_reg_id = regulator_get(&pdev->dev, "vsram");
	if (!vsram_reg_id)
		pr_info("regulator_get vsram_reg_id failed\n");

#if !APU_POWER_BRING_UP
	/* dvfsrc-vcore */
	vcore_dvfsrc_reg_id = regulator_get(&pdev->dev, "dvfsrc-vcore");
	if (!vcore_dvfsrc_reg_id)
		pr_info("regulator_get vcore_dvfsrc_reg_id failed\n");
#endif

	/* vapu */
	vapu_reg_id = regulator_get(&pdev->dev, "vapu");
	if (!vapu_reg_id)
		pr_info("regulator_get vapu_reg_id failed\n");

	/* devm_clk_get , not real prepare_clk */
	PREPARE_CLK(clk_top_dsp);
	PREPARE_CLK(clk_top_dsp1);
	PREPARE_CLK(clk_top_dsp2);
	PREPARE_CLK(clk_top_dsp3);
	PREPARE_CLK(clk_top_dsp4);
	PREPARE_CLK(clk_top_dsp5);
	PREPARE_CLK(clk_top_dsp6);
	PREPARE_CLK(clk_top_dsp7);
	if (ret_clk < 0)
		return ret_clk;

	LOG_DBG("%s %d --\n", __func__, __LINE__);
	return 0;
}

static void destroy_plat_pwr_res(void)
{
	UNPREPARE_CLK(clk_top_dsp);
	UNPREPARE_CLK(clk_top_dsp1);
	UNPREPARE_CLK(clk_top_dsp2);
	UNPREPARE_CLK(clk_top_dsp3);
	UNPREPARE_CLK(clk_top_dsp4);
	UNPREPARE_CLK(clk_top_dsp5);
	UNPREPARE_CLK(clk_top_dsp6);
	UNPREPARE_CLK(clk_top_dsp7);

	if (vapu_reg_id)
		regulator_put(vapu_reg_id);
	if (vsram_reg_id)
		regulator_put(vsram_reg_id);
	if (vcore_dvfsrc_reg_id)
		regulator_put(vcore_dvfsrc_reg_id);

	vapu_reg_id  = NULL;
	vsram_reg_id = NULL;
	vcore_dvfsrc_reg_id = NULL;
}

#if (ENABLE_SOC_CLK_MUX || ENABLE_SW_BUCK_CTL)
static void plt_pwr_res_ctl(int enable)
{
#if ENABLE_SW_BUCK_CTL
	int ret = 0;
#endif
#if ENABLE_SOC_CLK_MUX
	int ret_clk = 0;
#endif
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

		apu_writel(
			apu_readl(apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION)
			& ~(0x00000021),
			apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION);
#else
		LOG_DBG("%s skip enable regulator since HW auto\n", __func__);
#endif

#if ENABLE_SOC_CLK_MUX
		/* clk_prepare_enable */
		ENABLE_CLK(clk_top_dsp);
		ENABLE_CLK(clk_top_dsp1);
		ENABLE_CLK(clk_top_dsp2);
		ENABLE_CLK(clk_top_dsp3);
		ENABLE_CLK(clk_top_dsp4);
		ENABLE_CLK(clk_top_dsp5);
		ENABLE_CLK(clk_top_dsp6);
		ENABLE_CLK(clk_top_dsp7);
		if (ret_clk < 0)
			pr_info("%s fail enable clk : %d\n", __func__, ret_clk);
#else
		LOG_DBG("%s skip enable soc PLL since HW auto\n", __func__);
#endif

	} else {

#if ENABLE_SOC_CLK_MUX
		/* clk_disable_unprepare */
		DISABLE_CLK(clk_top_dsp7);
		DISABLE_CLK(clk_top_dsp6);
		DISABLE_CLK(clk_top_dsp5);
		DISABLE_CLK(clk_top_dsp4);
		DISABLE_CLK(clk_top_dsp3);
		DISABLE_CLK(clk_top_dsp2);
		DISABLE_CLK(clk_top_dsp1);
		DISABLE_CLK(clk_top_dsp);
#else
		LOG_DBG("%s skip disable soc PLL since HW auto\n", __func__);
#endif

#if !APU_POWER_BRING_UP
#if ENABLE_SW_BUCK_CTL
		apu_writel(
			apu_readl(apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION)
			| 0x00000021,
			apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION);

		if (vapu_reg_id) {
			ret = regulator_disable(vapu_reg_id);
			if (ret < 0)
				pr_info("%s fail disable vapu sram : %d\n",
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
#endif /* ENABLE_SOC_CLK_MUX || ENABLE_SW_BUCK_CTL */

#if APU_POWER_INIT
#if !APU_PWR_SOC_PATH
static void get_pll_pcw(uint32_t clk_rate, uint32_t *r1, uint32_t *r2)
{
	unsigned int fvco = clk_rate;
	unsigned int pcw_val;
	unsigned int postdiv_val = 1;
	unsigned int postdiv_reg = 0;

	while (fvco <= 1500) {
		postdiv_val = postdiv_val << 1;
		postdiv_reg = postdiv_reg + 1;
		fvco = fvco << 1;
	}

	pcw_val = (fvco * 1 << 14) / 26;

	if (postdiv_reg == 0) { /* Fvco * 2 with post_divider = 2 */
		pcw_val = pcw_val * 2;
		postdiv_val = postdiv_val << 1;
		postdiv_reg = postdiv_reg + 1;
	} /* Post divider is 1 is not available */

	*r1 = postdiv_reg;
	*r2 = pcw_val;
}

static void __apu_pll_init(void)
{
	uint32_t pll_hfctl_cfg[] = {PLL4HPLL_FHCTL0_CFG, PLL4HPLL_FHCTL1_CFG,
				    PLL4HPLL_FHCTL2_CFG, PLL4HPLL_FHCTL3_CFG};
	uint32_t pll_con1[] = {PLL4H_PLL1_CON1, PLL4H_PLL2_CON1,
			       PLL4H_PLL3_CON1, PLL4H_PLL4_CON1};
	uint32_t pll_fhctl_dds[] = {PLL4HPLL_FHCTL0_DDS, PLL4HPLL_FHCTL1_DDS,
				    PLL4HPLL_FHCTL2_DDS, PLL4HPLL_FHCTL3_DDS};
	int32_t pll_freq_out[] = {MDLA_DEFAULT_FREQ, MVPU_DEFAULT_FREQ,
				  MNOC_DEFAULT_FREQ, MNOC_DEFAULT_FREQ}; /* MHz */
	uint32_t pcw_val, posdiv_val;
	int pll_idx;
	uint32_t tmp = 0;

	LOG_DBG("PLL init %s %d ++\n", __func__, __LINE__);

	/* Step4. Initial PLL setting */

	/* Hopping function reset release: ofs 0xE0C */
	apu_setl(0xF << 0, apupw.regs[apu_pll] + PLL4HPLL_FHCTL_RST_CON);

	/* PCW value always from hopping function: ofs 0xE00 */
	apu_setl(0xF << 0, apupw.regs[apu_pll] + PLL4HPLL_FHCTL_HP_EN);

	/* Hopping function clock enable: ofs 0xE08 */
	apu_setl(0xF << 0, apupw.regs[apu_pll] + PLL4HPLL_FHCTL_CLK_CON);


	for (pll_idx = 0 ; pll_idx < PLL_NUM ; pll_idx++) {
		/* Hopping function enable */
		apu_setl((0x1 << 0) | (0x1 << 2),
				apupw.regs[apu_pll] + pll_hfctl_cfg[pll_idx]);

		posdiv_val = 0;
		pcw_val = 0;
		get_pll_pcw(pll_freq_out[pll_idx], &posdiv_val, &pcw_val);
		LOG_DBG("[%s][%d] freq = %d, div = 0x%x, pcw = 0x%x\n",
			__func__, __LINE__,
			pll_freq_out[pll_idx], posdiv_val, pcw_val);

		/*
		 * postdiv offset (0x000C), [26:24] RG_PLL_POSDIV
		 * 3'b000: /1 , 3'b001: /2 , 3'b010: /4
		 * 3'b011: /8 , 3'b100: /16
		 */
		tmp = apu_readl(apupw.regs[apu_pll] + pll_con1[pll_idx]);
		tmp = (tmp & ~(7 << 24)) | (posdiv_val << 24);
		apu_writel(tmp, apupw.regs[apu_pll] + pll_con1[pll_idx]);

		/*
		 * PCW offset (0x011C)
		 * [31] FHCTL0_PLL_TGL_ORG
		 * [21:0] FHCTL0_PLL_ORG set to PCW value
		 */
		apu_writel(((0x1 << 31) | pcw_val),
				apupw.regs[apu_pll] + pll_fhctl_dds[pll_idx]);
	}

	LOG_DBG("PLL init %s %d --\n", __func__, __LINE__);
}

static void __apu_acc_init(void)
{
#if LOCAL_DBG
	char buf[32];
#endif

	LOG_DBG("ACC init %s %d ++\n", __func__, __LINE__);

	/* Step6. Initial ACC setting (@ACC) */

	/* mnoc, uP clk setting */
	LOG_DBG("mnoc clk setting %s %d\n", __func__, __LINE__);
	/* CGEN_SOC */
	apu_writel(0x00000004, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR0);
	/* HW_CTRL_EN */
	apu_writel(0x00008000, apupw.regs[apu_acc] + APU_ACC_CONFG_SET0);

	/* iommu clk setting */
	LOG_DBG("iommu clk setting %s %d\n", __func__, __LINE__);
	/* CGEN_SOC */
	apu_writel(0x00000004, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR1);
	/* HW_CTRL_EN */
	apu_writel(0x00008000, apupw.regs[apu_acc] + APU_ACC_CONFG_SET1);

	/* mvpu clk setting */
	LOG_DBG("mvpu clk setting %s %d\n", __func__, __LINE__);
	/* CGEN_SOC */
	apu_writel(0x00000004, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR2);
	/* HW_CTRL_EN */
	apu_writel(0x00008000, apupw.regs[apu_acc] + APU_ACC_CONFG_SET2);
	/* CLK_REQ_SW_EN */
	apu_writel(0x00000100, apupw.regs[apu_acc] + APU_ACC_AUTO_CTRL_SET2);

	/* mdla clk setting */
	LOG_DBG("mdla clk setting %s %d\n", __func__, __LINE__);
	/* CGEN_SOC */
	apu_writel(0x00000004, apupw.regs[apu_acc] + APU_ACC_CONFG_CLR3);
	/* HW_CTRL_EN */
	apu_writel(0x00008000, apupw.regs[apu_acc] + APU_ACC_CONFG_SET3);
	/* CLK_REQ_SW_EN */
	apu_writel(0x00000100, apupw.regs[apu_acc] + APU_ACC_AUTO_CTRL_SET3);

	/* clk invert setting */
	LOG_DBG("clk invert setting %s %d\n", __func__, __LINE__);
	/*
	 * MVPU1_CLK_INV_EN MVPU3_CLK_INV_EN MVPU5_CLK_INV_EN
	 * MDLA1_CLK_INV_EN MDLA3_CLK_INV_EN
	 * MDLA5_CLK_INV_EN MDLA7_CLK_INV_EN
	 */
	apu_writel(0x0000AAA8, apupw.regs[apu_acc] + APU_ACC_CLK_INV_EN_SET);

#if LOCAL_DBG
	memset(buf, 0, sizeof(buf));
	snprintf(buf, 32, "phys 0x%08x: ", (u32)(apupw.phy_addr[apu_acc]));
	print_hex_dump(KERN_ERR, buf, DUMP_PREFIX_OFFSET, 16, 4,
			apupw.regs[apu_acc], 0x100, true);
#endif

	LOG_DBG("ACC init %s %d --\n", __func__, __LINE__);
}
#endif /* !APU_PWR_SOC_PATH */

static void __apu_buck_off_cfg(void)
{
	/* Step11. Roll back to Buck off stage */

	LOG_DBG("%s %d ++\n", __func__, __LINE__);

	/*
	 * a. Setup Buck control signal
	 * The following setting need to in order,
	 * and wait 1uS before setup next control signal
	 */
	/* APU_BUCK_PROT_REQ */
	apu_writel(0x00004000, apupw.regs[apu_rpc] + APU_RPC_HW_CON);
	udelay(10);
	/* APU_BUCK_ELS_EN */
	apu_writel(0x00000400, apupw.regs[apu_rpc] + APU_RPC_HW_CON);
	udelay(10);
	/* APU_BUCK_RST_B */
	apu_writel(0x00002000, apupw.regs[apu_rpc] + APU_RPC_HW_CON);
	udelay(10);


	/* b. Manually turn off Buck (by configuring register in PMIC) */
	/* move to probe last line */
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
#endif

void mt8188_apu_dump_rpc_status(enum t_acx_id id, struct rpc_status_dump *dump)
{
	uint32_t status1 = 0x0;
	uint32_t status2 = 0x0;
	uint32_t status3 = 0x0;

	if (id == ACX0) {
		status1 = apu_readl(apupw.regs[apu_acx0_rpc_lite]
				+ APU_RPC_INTF_PWR_RDY);
		status2 = apu_readl(apupw.regs[apu_acx0]
				+ APU_ACX_CONN_CG_CON);
		pr_info(
		"%s ACX0 APU_RPC_INTF_PWR_RDY:0x%08x APU_ACX_CONN_CG_CON:0x%08x\n",
				__func__, status1, status2);
	} else {
		status1 = apu_readl(apupw.regs[apu_rpc]
				+ APU_RPC_INTF_PWR_RDY);
		status2 = apu_readl(apupw.regs[apu_vcore]
				+ APUSYS_VCORE_CG_CON);
		status3 = apu_readl(apupw.regs[apu_rcx]
				+ APU_RCX_CG_CON);
		pr_info(
		"%s RCX APU_RPC_INTF_PWR_RDY:0x%08x APU_VCORE_CG_CON:0x%08x APU_RCX_CG_CON:0x%08x\n",
				__func__, status1, status2, status3);
		/*
		 * print_hex_dump(KERN_ERR, "rpc: ", DUMP_PREFIX_OFFSET,
		 *		16, 4, apupw.regs[apu_rpc], 0x100, 1);
		 */
	}

	if (!IS_ERR_OR_NULL(dump)) {
		dump->rpc_reg_status = status1;
		dump->conn_reg_status = status2;
		if (id == RCX)
			dump->vcore_reg_status = status3;
	}
}

#if APU_POWER_INIT
/*
 * low 32-bit data for PMIC control
 *	APU_PCU_PMIC_TAR_BUF1
 *	[31:16] offset to update
 *	[15:00] data to update
 *
 * high 32-bit data for PMIC control
 *	APU_PCU_PMIC_TAR_BUF2
 *	[1:0] cmd_op, read:0x0 , write:0x4
 *	[3]: pmifid,
 *	[7:4]: slvid
 *	[8]: bytecnt
 */
static void __apu_pcu_init(void)
{
	uint32_t cmd_op_w = 0x4;
	uint32_t vapu_en_offset = BUCK_VAPU_PMIC_REG_EN_ADDR;
	uint32_t vapu_sram_en_offset = BUCK_VAPU_SRAM_PMIC_REG_EN_ADDR;

	LOG_DBG("PCU init %s %d ++\n", __func__, __LINE__);

	/* auto buck enable */
	apu_writel((0x1 << 16), apupw.regs[apu_pcu] + APU_PCUTOP_CTRL_SET);

	/* Step1. enable auto buck on/off function of command0/1 */
	apu_writel(0x33, apupw.regs[apu_pcu] + APU_PCU_BUCK_STEP_SEL);

	/* Step2. fill-in command0/1 for vapu_sram/vapu auto buck ON */
	apu_writel((vapu_sram_en_offset << 16) + 1,
			apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT0_L);
	apu_writel(cmd_op_w, apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT0_H);

	apu_writel((vapu_en_offset << 16) + 1,
			apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT1_L);
	apu_writel(cmd_op_w, apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT1_H);

	LOG_DBG("%s APU_PCU_BUCK_ON_DAT0_L=0x%08x, APU_PCU_BUCK_ON_DAT0_H=0x%08x\n",
		__func__,
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT0_L),
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT0_H));
	LOG_DBG("%s APU_PCU_BUCK_ON_DAT1_L=0x%08x, APU_PCU_BUCK_ON_DAT1_H=0x%08x\n",
		__func__,
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT1_L),
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_DAT1_H));

	/* Step3. fill-in command0/1 for vapu/vapu_sram auto buck OFF */
	apu_writel((vapu_en_offset << 16) + 0,
			apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT0_L);
	apu_writel(cmd_op_w, apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT0_H);

	apu_writel((vapu_sram_en_offset << 16) + 0,
			apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT1_L);
	apu_writel(cmd_op_w, apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT1_H);

	LOG_DBG("%s APU_PCU_BUCK_OFF_DAT0_L=0x%08x, APU_PCU_BUCK_OFF_DAT0_H=0x%08x\n",
		__func__,
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT0_L),
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT0_H));
	LOG_DBG("%s APU_PCU_BUCK_OFF_DAT1_L=0x%08x, APU_PCU_BUCK_OFF_DAT1_H=0x%08x\n",
		__func__,
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT1_L),
		apu_readl(apupw.regs[apu_pcu] + APU_PCU_BUCK_OFF_DAT1_H));

	/* Step4. update buck settle time for vapu_sram/vapu by SEL0/1 */
	apu_writel(APU_PCU_BUCK_ON_SETTLE_TIME,
			apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_SLE0);
	apu_writel(APU_PCU_BUCK_ON_SETTLE_TIME,
			apupw.regs[apu_pcu] + APU_PCU_BUCK_ON_SLE1);

	LOG_DBG("PCU init %s %d --\n", __func__, __LINE__);
}
static void __apu_rpclite_init(void)
{
	uint32_t sleep_type_offset[] = {0x0208, 0x020C, 0x0210, 0x0214,
					0x0218, 0x021C, 0x0220, 0x0224};
	enum apupw_reg rpc_lite_base[CLUSTER_NUM];
	int ofs_arr_size = sizeof(sleep_type_offset) / sizeof(uint32_t);
	int acx_idx, ofs_idx;

	LOG_DBG("RPC-Lite init %s %d ++\n", __func__, __LINE__);

	rpc_lite_base[0] = apu_acx0_rpc_lite;

	for (acx_idx = 0 ; acx_idx < CLUSTER_NUM ; acx_idx++) {
		for (ofs_idx = 0 ; ofs_idx < ofs_arr_size ; ofs_idx++) {
			/* Memory setting */
			apu_clearl((0x1 << 1),
					apupw.regs[rpc_lite_base[acx_idx]]
					+ sleep_type_offset[ofs_idx]);
		}

		/* Control setting */
		apu_setl(0x0000009E, apupw.regs[rpc_lite_base[acx_idx]]
					+ APU_RPC_TOP_SEL);
	}

	dump_rpc_lite_reg(__LINE__);

	LOG_DBG("RPC-Lite init %s %d --\n", __func__, __LINE__);
}

static void __apu_rpc_init(void)
{
	LOG_DBG("RPC init %s %d ++\n", __func__, __LINE__);

	/* Step7. RPC: memory types (sleep or PD type) */
	/* RPC: iTCM in uP need to setup to sleep type */
	apu_clearl((0x1 << 1), apupw.regs[apu_rpc] + 0x0200);

	/* Step9. RPCtop initial */
	/* RPC */
#if ENABLE_SW_BUCK_CTL
	apu_setl(0x0800101E, apupw.regs[apu_rpc] + APU_RPC_TOP_SEL);
#else
	apu_setl(0x0800501E, apupw.regs[apu_rpc] + APU_RPC_TOP_SEL);
#endif
	/* BUCK_PROT_SEL */
	apu_setl((0x1 << 20), apupw.regs[apu_rpc] + APU_RPC_TOP_SEL_1);

	LOG_DBG("RPC init %s %d --\n", __func__, __LINE__);
}

static int __apu_are_init(struct device *dev)
{
	u32 tmp = 0;
	int ret, val = 0;
	int are_id, idx = 0;
	uint32_t are_entry2_cfg_h[] = {0x00140000, 0x00140000, 0x00140000};
	uint32_t are_entry2_cfg_l[] = {0x004E0804, 0x004E0806, 0x004E0807};

	/* Step10. ARE initial */

	/* Turn on ARE: (8139/8188 Only) */
	tmp = apu_readl(apupw.regs[apu_ao_ctl] + CSR_DUMMY_0_ADDR) | (0x1 << 2);
	apu_writel(tmp, apupw.regs[apu_ao_ctl] + CSR_DUMMY_0_ADDR);

	/* TINFO="Wait for sare1 fsm to transition to IDLE" */
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_are2] + 0x48),
			val, (val & 0x1UL), 50, 10000);
	if (ret) {
		pr_info("%s timeout to wait sram1 fsm to idle, ret %d\n",
				__func__, ret);
		return -1;
	}

	LOG_DBG("ARE init %s %d ++\n", __func__, __LINE__);

	for (are_id = apu_are0, idx = 0; are_id <= apu_are2; are_id++, idx++) {

		LOG_DBG("%s are_id:%d\n", __func__, are_id);

		/* ARE entry 0 initial */
		apu_writel(0x01234567, apupw.regs[are_id]
						+ APU_ARE_ETRY0_SRAM_H);
		apu_writel(0x89ABCDEF, apupw.regs[are_id]
						+ APU_ARE_ETRY0_SRAM_L);

		/* ARE entry 1 initial */
		apu_writel(0xFEDCBA98, apupw.regs[are_id]
						+ APU_ARE_ETRY1_SRAM_H);
		apu_writel(0x76543210, apupw.regs[are_id]
						+ APU_ARE_ETRY1_SRAM_L);

		/* ARE entry 2 initial */
		apu_writel(are_entry2_cfg_h[idx], apupw.regs[are_id]
						+ APU_ARE_ETRY2_SRAM_H);
		apu_writel(are_entry2_cfg_l[idx], apupw.regs[are_id]
						+ APU_ARE_ETRY2_SRAM_L);

		/* dummy read ARE entry2 H/L sram */
		tmp = readl(apupw.regs[are_id] + APU_ARE_ETRY2_SRAM_H);
		tmp = readl(apupw.regs[are_id] + APU_ARE_ETRY2_SRAM_L);

		/* update ARE sram */
		apu_writel(0x00000004, apupw.regs[are_id] + APU_ARE_INI_CTRL);

		dev_dbg(dev, "%s ARE entry2_H phys 0x%x = 0x%x\n",
			__func__,
			(u32)(apupw.phy_addr[are_id] + APU_ARE_ETRY2_SRAM_H),
			readl(apupw.regs[are_id] + APU_ARE_ETRY2_SRAM_H));

		dev_dbg(dev, "%s ARE entry2_L phys 0x%x = 0x%x\n",
			__func__,
			(u32)(apupw.phy_addr[are_id] + APU_ARE_ETRY2_SRAM_L),
			readl(apupw.regs[are_id] + APU_ARE_ETRY2_SRAM_L));
	}

	LOG_DBG("ARE init %s %d --\n", __func__, __LINE__);

	return 0;
}
#endif /* APU_POWER_INIT */

#if APUPW_DUMP_FROM_APMCU
static void are_dump_entry(int are_hw)
{
	int are_id, are_entry_max_id;
	uint32_t reg, data;
	uint32_t target_data = 0x0;
	void *target_addr = 0x0;
	int entry, err_flag;

	if (are_hw == 0) {
		are_id = apu_are0;
		are_entry_max_id = 238;
	} else if (are_hw == 1) {
		are_id = apu_are1;
		are_entry_max_id = 210;
	} else {
		are_id = apu_are2;
		are_entry_max_id = 237;
	}

	pr_info("APU_ARE_DUMP are_hw:%d offset: 0x%03x = 0x%08x\n",
			are_hw, 0x4, readl(apupw.regs[are_id] + 0x4));
	pr_info("APU_ARE_DUMP are_hw:%d offset: 0x%03x = 0x%08x\n",
			are_hw, 0x40, readl(apupw.regs[are_id] + 0x40));
	pr_info("APU_ARE_DUMP are_hw:%d offset: 0x%03x = 0x%08x\n",
			are_hw, 0x44, readl(apupw.regs[are_id] + 0x44));
	pr_info("APU_ARE_DUMP are_hw:%d offset: 0x%03x = 0x%08x\n",
			are_hw, 0x48, readl(apupw.regs[are_id] + 0x48));
	pr_info("APU_ARE_DUMP are_hw:%d offset: 0x%03x = 0x%08x\n",
			are_hw, 0x4C, readl(apupw.regs[are_id] + 0x4C));

	for (entry = 0 ; entry <= 2 ; entry++) {
		pr_info(
		"APU_ARE_DUMP are_hw:%d cfg entry %d = H:0x%08x L:0x%08x\n",
			are_hw, entry,
			readl(apupw.regs[are_id] +
				APU_ARE_ETRY0_SRAM_H + entry * 4),
			readl(apupw.regs[are_id] +
				APU_ARE_ETRY0_SRAM_L + entry * 4));
	}

	for (entry = 3 ; entry <= are_entry_max_id ; entry++) {
		reg = readl(apupw.regs[are_id] +
				APU_ARE_ETRY0_SRAM_H + entry * 4);
		data = readl(apupw.regs[are_id] +
				APU_ARE_ETRY0_SRAM_L + entry * 4);
		err_flag = 0;
		target_addr = 0x0;
		target_data = 0x0;

		if (reg != 0x0) {
			//pr_info("%s: remapping 0x%08x\n", __func__, reg);
			target_addr = ioremap(reg, PAGE_SIZE);

			if (IS_ERR((void const *)target_addr)) {
				pr_info("%s: remap fail 0x%08x\n",
						__func__, reg);
			} else {
				target_data = readl(target_addr);
				iounmap(target_addr);
				if (target_data != data)
					err_flag = 1;

				pr_info(
					"APU_ARE_DUMP %d-%03d 0x%08x 0x%08x 0x%08lx 0x%08x %d\n",
					are_hw, entry, reg, data,
					(uintptr_t)target_addr,
					target_data, err_flag);
			}
		}

	}
}
#endif

/* backup solution : send request for RPC sleep from APMCU */
static int __apu_sleep_rpc_rcx(struct device *dev)
{
	apusys_pwr_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_PWR_RCX,
			SMC_RCX_PWR_OFF);

	dev_info(dev, "%s RCX APU_RPC_INTF_PWR_RDY 0x%x = 0x%x\n",
			__func__,
			(u32)(apupw.phy_addr[apu_rpc] + APU_RPC_INTF_PWR_RDY),
			readl(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY));

	return 0;
}

static int __apu_wake_rpc_rcx(struct device *dev)
{
	int ret = 0, val = 0;
	uint32_t cfg = 0x0;

	/* check rpc register is correct or not */
	cfg = apu_readl(apupw.regs[apu_rpc] + APU_RPC_TOP_SEL);

	LOG_DBG("%s before wakeup RCX APU_RPC_INTF_PWR_RDY 0x%x = 0x%x\n",
		__func__,
		(u32)(apupw.phy_addr[apu_rpc] + APU_RPC_INTF_PWR_RDY),
		readl(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY));

	apusys_pwr_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_PWR_RCX,
			SMC_RCX_PWR_AFC_EN);

	/* wake up RPC */
	apusys_pwr_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_PWR_RCX,
			SMC_RCX_PWR_WAKEUP_RPC);

	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY),
			val, (val & 0x1UL), 50, 10000);
	if (ret) {
		pr_info("%s polling RPC RDY timeout, ret %d\n", __func__, ret);
		goto out;
	}

	LOG_DBG("%s after wakeup RCX APU_RPC_INTF_PWR_RDY 0x%x = 0x%x\n",
		__func__,
		(u32)(apupw.phy_addr[apu_rpc] + APU_RPC_INTF_PWR_RDY),
		readl(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY));

	/* polling FSM @RPC-lite to ensure RPC is in on/off stage */
	ret |= readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_rpc] + APU_RPC_STATUS),
			val, (val & (0x1 << 29)), 50, 10000);
	if (ret) {
		pr_info("%s polling ARE FSM timeout, ret %d\n", __func__, ret);
		goto out;
	}

	/* clear vcore/rcx cgs */
	apusys_pwr_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_PWR_RCX,
			SMC_RCX_PWR_CG_EN);

out:
	return ret;
}

#if APU_POWER_BRING_UP
static int __apu_wake_rpc_acx(struct device *dev, enum t_acx_id acx_id)
{
	int ret = 0, val = 0;
	enum apupw_reg rpc_lite_base;
	enum apupw_reg acx_base;

	if (acx_id == ACX0) {
		rpc_lite_base = apu_acx0_rpc_lite;
		acx_base = apu_acx0;
	} else {
		dev_info(dev, "[%s] acx_id(acx_id) shouble be ACX0\n",
				__func__);
		goto out;
	}

	dev_info(dev, "%s ctl p1:%d p2:%d\n",
			__func__, rpc_lite_base, acx_base);

	/* TINFO="Enable AFC enable" */
	apu_setl((0x1 << 16), apupw.regs[rpc_lite_base] + APU_RPC_TOP_SEL_1);

	/* wake acx rpc lite */
	apu_writel(0x00000100, apupw.regs[rpc_lite_base] + APU_RPC_TOP_CON);
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[rpc_lite_base] + APU_RPC_INTF_PWR_RDY),
			val, (val & 0x1UL), 50, 10000);

	/* polling FSM @RPC-lite to ensure RPC is in on/off stage */
	ret |= readl_relaxed_poll_timeout_atomic(
			(apupw.regs[rpc_lite_base] + APU_RPC_STATUS),
			val, (val & (0x1 << 29)), 50, 10000);
	if (ret) {
		pr_info("%s wake up acx%d_rpc fail, ret %d\n",
				__func__, acx_id, ret);
		goto out;
	}

	dev_info(dev, "%s ACX%d APU_RPC_INTF_PWR_RDY 0x%x = 0x%x\n",
		__func__, acx_id,
		(u32)(apupw.phy_addr[rpc_lite_base] + APU_RPC_INTF_PWR_RDY),
		readl(apupw.regs[rpc_lite_base] + APU_RPC_INTF_PWR_RDY));

	/* clear acx0/1 CGs */
	apu_writel(0xFFFFFFFF, apupw.regs[acx_base] + APU_ACX_CONN_CG_CLR);

	dev_dbg(dev, "%s ACX%d APU_ACX_CONN_CG_CON 0x%x = 0x%x\n",
		__func__, acx_id,
		(u32)(apupw.phy_addr[acx_base] + APU_ACX_CONN_CG_CON),
		readl(apupw.regs[acx_base] + APU_ACX_CONN_CG_CON));
out:
	return ret;
}

static int __apu_pwr_ctl_acx_engines(struct device *dev,
		enum t_acx_id acx_id, enum t_dev_id dev_id, int pwron)
{
	int ret = 0, val = 0;
	enum apupw_reg rpc_lite_base;
	enum apupw_reg acx_base;
	uint32_t dev_mtcmos_ctl, dev_cg_con, dev_cg_clr;
	uint32_t dev_mtcmos_chk;

	/* we support power only for bringup */

	if (acx_id == ACX0) {
		rpc_lite_base = apu_acx0_rpc_lite;
		acx_base = apu_acx0;
	} else {
		dev_info(dev, "[%s] acx_id(acx_id) shouble be ACX0\n", __func__);
		goto out;
	}

	switch (dev_id) {
	case VPU0:
		dev_mtcmos_ctl = 0x00000012;
		dev_mtcmos_chk = 0x4UL;
		dev_cg_con = APU_ACX_VP6_CG_CON;
		dev_cg_clr = APU_ACX_VP6_CG_CLR;
		break;
	case DLA0:
		dev_mtcmos_ctl = 0x00000016;
		dev_mtcmos_chk = 0x40UL;
		dev_cg_con = APU_ACX_MDLA0_CG_CON;
		dev_cg_clr = APU_ACX_MDLA0_CG_CLR;
		break;
	case DLA1:
		dev_mtcmos_ctl = 0x00000017;
		dev_mtcmos_chk = 0x80UL;
		dev_cg_con = APU_ACX_MDLA1_CG_CON;
		dev_cg_clr = APU_ACX_MDLA1_CG_CLR;
		break;
	default:
		goto out;
	}

	dev_dbg(dev, "%s ctl p1:%d p2:%d p3:0x%x p4:0x%x p5:0x%x p6:0x%x\n",
		__func__, rpc_lite_base, acx_base,
		dev_mtcmos_ctl, dev_mtcmos_chk, dev_cg_con, dev_cg_clr);

	/* config acx rpc lite */
	apu_writel(dev_mtcmos_ctl,
			apupw.regs[rpc_lite_base] + APU_RPC_SW_FIFO_WE);
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[rpc_lite_base] + APU_RPC_INTF_PWR_RDY),
			val, (val & dev_mtcmos_chk) == dev_mtcmos_chk,
			50, 200000);
	if (ret) {
		pr_info("%s config acx%d_rpc 0x%x fail, ret %d\n",
				__func__, acx_id, dev_mtcmos_ctl, ret);
		goto out;
	}

	dev_dbg(dev, "%s ACX%d APU_RPC_INTF_PWR_RDY 0x%x = 0x%x\n",
		__func__, acx_id,
		(u32)(apupw.phy_addr[rpc_lite_base] + APU_RPC_INTF_PWR_RDY),
		readl(apupw.regs[rpc_lite_base] + APU_RPC_INTF_PWR_RDY));

	apu_writel(0xFFFFFFFF, apupw.regs[acx_base] + dev_cg_clr);

	dev_dbg(dev, "%s ACX%d dev%d CG_CON 0x%x = 0x%x\n",
		__func__, acx_id, dev_id,
		(u32)(apupw.phy_addr[acx_base] + dev_cg_con),
		readl(apupw.regs[acx_base] + dev_cg_con));
out:
	return ret;
}

#if !APU_PWR_SOC_PATH
static int __apu_on_mdla_mvpu_clk(void)
{
	int ret = 0;
	int val = 0;

	/* turn on mvpu root clk src */
	apu_writel(0x00000200, apupw.regs[apu_acc] + APU_ACC_AUTO_CTRL_SET2);
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_acc] + APU_ACC_AUTO_STATUS2),
			val, (val & 0x20UL) == 0x20UL, 50, 10000);
	if (ret) {
		pr_info("%s turn on mvpu root clk fail, ret %d\n",
				__func__, ret);
		goto out;
	}

	/* turn on mdla root clk src */
	apu_writel(0x00000200, apupw.regs[apu_acc] + APU_ACC_AUTO_CTRL_SET3);
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_acc] + APU_ACC_AUTO_STATUS3),
			val, (val & 0x20UL) == 0x20UL, 50, 10000);
	if (ret) {
		pr_info("%s turn on mdla root clk fail, ret %d\n",
				__func__, ret);
		goto out;
	}

out:
	return ret;
}
#endif /* !APU_PWR_SOC_PATH */
#endif /* APU_POWER_BRING_UP */

static int __apu_xpu2apusys_d4_slv_en(int en)
{
	void __iomem *addr = 0;
	uint32_t val = 0;

	switch (en) {
	case 0:
		pr_info("[%s] xpu2apusys d4 slv dis\n", __func__);

		addr = apupw.regs[bcrm_fmem_pdn] + INFRA_FMEM_BUS_u_SI21_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val | (0x1 << 12)), addr);

		addr = apupw.regs[bcrm_fmem_pdn] + INFRA_FMEM_BUS_u_SI22_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val | (0x1 << 13)), addr);

		addr = apupw.regs[bcrm_fmem_pdn] + INFRA_FMEM_BUS_u_SI11_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val | (0x1 << 11)), addr);

		addr = apupw.regs[bcrm_fmem_pdn]
				+ INFRA_FMEM_M6M7_BUS_u_SI24_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val | (0x1 << 15)), addr);

		break;
	case 1:
		pr_info("[%s] xpu2apusys d4 slv en\n", __func__);

		addr = apupw.regs[bcrm_fmem_pdn] + INFRA_FMEM_BUS_u_SI21_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val & ~(0x1 << 12)), addr);

		addr = apupw.regs[bcrm_fmem_pdn] + INFRA_FMEM_BUS_u_SI22_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val & ~(0x1 << 13)), addr);

		addr = apupw.regs[bcrm_fmem_pdn] + INFRA_FMEM_BUS_u_SI11_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val & ~(0x1 << 11)), addr);

		addr = apupw.regs[bcrm_fmem_pdn]
				+ INFRA_FMEM_M6M7_BUS_u_SI24_CTRL_0;
		val = apu_readl(addr);
		apu_writel((val & ~(0x1 << 15)), addr);

		break;
	default:
		pr_info("%s invalid op: %d\n", __func__, en);
		break;
	}

	return 0;
}

static int mt8188_apu_top_on(struct device *dev)
{
	int ret = 0;

	LOG_DBG("%s +\n", __func__);
	apu_vote_vcore_dvfsrc(1);

	/*
	 *  To check whether the APU image is successfully loaded,
	 *  here we write 1 to PWR_FLOW_SYNC_REG, and APU will reset
	 *  this register value to 0 when it power on.
	 */
	mt8188_pwr_flow_remote_sync(1);

#if (ENABLE_SOC_CLK_MUX || ENABLE_SW_BUCK_CTL)
	plt_pwr_res_ctl(1);
#endif
	ret = __apu_wake_rpc_rcx(dev);

	if (ret) {
		pr_info("%s fail to wakeup RPC, ret %d\n", __func__, ret);
		apupw_aee_warn("APUSYS_POWER", "APUSYS_POWER_WAKEUP_FAIL");
		return -1;
	}

	__apu_xpu2apusys_d4_slv_en(0);

	mt8188_apu_devfreq_cooling_start();

#if THERMAL_TEMP_UPDATE
	/* always update apu temp once aputop pwr on */
	mt8188_apu_temperature_sync(tz_name);

	/* use timer to schedule timer to get apu temperature periodically */
	if (pwr_data->plat_apu_thermal_timer_func) {
		timer_setup(&apu_thermal_timer,
				pwr_data->plat_apu_thermal_timer_func, 0);
		apu_thermal_timer.expires =
			jiffies + msecs_to_jiffies(TEMP_UPDATE_TIME);

		add_timer(&apu_thermal_timer);
	}
#endif
	pr_info("%s -\n", __func__);
	return 0;
}

static int mt8188_apu_top_off(struct device *dev)
{
	int ret = 0, val = 0;
	int rpc_timeout_val = 500000; /* 500 ms */

	LOG_DBG("%s +\n", __func__);

#if THERMAL_TEMP_UPDATE
	if (pwr_data->plat_apu_thermal_timer_func)
		del_timer_sync(&apu_thermal_timer);
#endif

	mt8188_apu_devfreq_cooling_stop();

	__apu_xpu2apusys_d4_slv_en(1);

#if APMCU_REQ_RPC_SLEEP
	/* backup solution : send request for RPC sleep from APMCU */
	__apu_sleep_rpc_rcx(dev);
#else
	if (mt8188_read_pwr_flow_sync_reg() == 0) {
		/* APU was loaded successfully, tell remote side I am ready to off */
		mt8188_pwr_flow_remote_sync(1);
	}
	else {
		/* APU was not loaded successfully, trigger power off from APMCU */
		__apu_sleep_rpc_rcx(dev);
	}
#endif

	/* blocking until sleep success or timeout */
	ret = readl_relaxed_poll_timeout_atomic(
			(apupw.regs[apu_rpc] + APU_RPC_INTF_PWR_RDY),
			val, (val & 0x1UL) == 0x0, 50, rpc_timeout_val);
	if (ret) {
		pr_info("%s timeout to wait RPC sleep (val:%d), ret %d\n",
				__func__, rpc_timeout_val, ret);
		apupw_aee_warn("APUSYS_POWER", "APUSYS_POWER_SLEEP_TIMEOUT");
		return -1;
	}

	/* mt8188_apu_dump_rpc_status(RCX, NULL); */

#if (ENABLE_SOC_CLK_MUX || ENABLE_SW_BUCK_CTL)
	plt_pwr_res_ctl(0);
#endif
	apu_vote_vcore_dvfsrc(0);
	pr_info("%s -\n", __func__);
	return 0;
}

#if APU_POWER_INIT
static void __apu_aoc_init(void)
{
	LOG_DBG("AOC init %s %d ++\n", __func__, __LINE__);

	/* Step1. Switch APU AOC ctrl signal from SW reg to HW path (RPC) */
	// apu_clearl((0x1 << 5), apupw.regs[sys_spm] + 0xf30);
	// apu_setl((0x1 << 0), apupw.regs[sys_spm] + 0x414);

	/*
	 * Step2. Manually disable Buck els enable @SOC
	 * (disable SW mode to manually control Buck on/off)
	 */
	apu_clearl(((0x1 << 5) | (0x1 << 0)),
			apupw.regs[sys_spm] + APUSYS_BUCK_ISOLATION);

	/*
	 * Step3. Roll back to APU Buck on stage
	 * The following setting need to in order and wait 1uS before setup
	 * next control signal
	 */
	/* APU_BUCK_ELS_EN */
	apu_writel(0x00000800, apupw.regs[apu_rpc] + APU_RPC_HW_CON);
	udelay(10);

	/* APU_BUCK_RST_B */
	apu_writel(0x00001000, apupw.regs[apu_rpc] + APU_RPC_HW_CON);
	udelay(10);

	/* APU_BUCK_PROT_REQ */
	apu_writel(0x00008000, apupw.regs[apu_rpc] + APU_RPC_HW_CON);
	udelay(10);

	/* SRAM_AOC_ISO */
	apu_writel(0x00000080, apupw.regs[apu_rpc] + APU_RPC_HW_CON);
	udelay(10);

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

	for (i = 0; i < PLL_NUM; i++)
		plat_cfg.freq[i] = opp_tbl->opp[0].pll_freq[i] / 1000; /* MHZ */

	for (i = 1; i < opp_tbl->tbl_size; i++)
		if (opp_tbl->opp[0].vapu != opp_tbl->opp[i].vapu)
			break;

	plat_cfg.fix_volt = (i == opp_tbl->tbl_size) ? 1 : 0;
	LOG_DBG("%s 0x%08x 0x%08x 0x%08x\n", __func__,
		plat_cfg.aging_flag,
		plat_cfg.hw_id,
		aging_attr);

	return mt8188_chip_data_remote_sync(&plat_cfg);
}

#if APU_POWER_INIT
static int init_hw_setting(struct device *dev)
{
	__apu_aoc_init();
	__apu_pcu_init();
	__apu_rpc_init();
	__apu_rpclite_init();

	__apu_are_init(dev);
#if !APU_PWR_SOC_PATH
	__apu_pll_init();
	__apu_acc_init();
#endif
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

static int mt8188_apu_top_pb(struct platform_device *pdev)
{
#if APU_POWER_INIT
	int ret_clk = 0;
#endif
	int ret = 0;

	LOG_DBG("%s fpga_type : %d\n", __func__, fpga_type);

	init_reg_base(pdev);
	init_opp_table(pdev);
#if THERMAL_TEMP_UPDATE
	init_apu_thermal(pdev);
#endif
	init_plat_pwr_res(pdev);

	/* enable vsram buck */
	if (vsram_reg_id) {
		ret = regulator_enable(vsram_reg_id);
		if (ret)
			pr_info("[%s][%d] enable vsram regulator failed\n",
				__func__, __LINE__);
		udelay(300);
	}

	/* enable vapu buck */
	if (vapu_reg_id) {
		ret = regulator_enable(vapu_reg_id);
		if (ret)
			pr_info("[%s][%d] enable vapu regulator failed\n",
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

#if APU_POWER_INIT
	ENABLE_CLK(clk_top_dsp);
	ENABLE_CLK(clk_top_dsp1);
	ENABLE_CLK(clk_top_dsp2);
	ENABLE_CLK(clk_top_dsp3);
	ENABLE_CLK(clk_top_dsp4);
	ENABLE_CLK(clk_top_dsp5);
	ENABLE_CLK(clk_top_dsp6);
	ENABLE_CLK(clk_top_dsp7);

	/* before apu power init, ensure soc regulator/clk is ready */
	init_hw_setting(&pdev->dev);

#if !APU_POWER_BRING_UP
	DISABLE_CLK(clk_top_dsp7);
	DISABLE_CLK(clk_top_dsp6);
	DISABLE_CLK(clk_top_dsp5);
	DISABLE_CLK(clk_top_dsp4);
	DISABLE_CLK(clk_top_dsp3);
	DISABLE_CLK(clk_top_dsp2);
	DISABLE_CLK(clk_top_dsp1);
	DISABLE_CLK(clk_top_dsp);
#endif /* !APU_POWER_BRING_UP */
#endif /* APU_POWER_INIT */

	/* set vapu to default voltage */
	if (vapu_reg_id)
		regulator_set_voltage(vapu_reg_id,
				      VAPU_DEF_VOLT, VAPU_DEF_VOLT);

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

	mt8188_init_remote_data_sync(apupw.regs[apu_md32_mbox]);
	init_plat_chip_data(pdev);

	mt8188_apu_devfreq_cooling_init(pdev, apupw.regs[apu_md32_mbox]);

#if APU_POWER_BRING_UP
#if !APU_PWR_SOC_PATH
	/* power bring up shall use soc PLL */
	__apu_on_mdla_mvpu_clk();
#endif
	switch (fpga_type) {
	default:
	case 0: /* do not power on */
		pr_info("%s bypass pre-power-ON\n", __func__);
		break;
	case 1:
		pm_runtime_get_sync(&pdev->dev);
		__apu_wake_rpc_acx(&pdev->dev, ACX0);
		__apu_pwr_ctl_acx_engines(&pdev->dev, ACX0, VPU0, 1);
		__apu_pwr_ctl_acx_engines(&pdev->dev, ACX0, DLA0, 1);
		break;
	case 2:
		pm_runtime_get_sync(&pdev->dev);
		__apu_wake_rpc_acx(&pdev->dev, ACX0);
		__apu_pwr_ctl_acx_engines(&pdev->dev, ACX0, VPU0, 1);
		__apu_pwr_ctl_acx_engines(&pdev->dev, ACX0, DLA0, 1);
		break;
	case 3:
		pm_runtime_get_sync(&pdev->dev);
		__apu_wake_rpc_acx(&pdev->dev, ACX0);
		__apu_pwr_ctl_acx_engines(&pdev->dev, ACX0, VPU0, 1);
		__apu_pwr_ctl_acx_engines(&pdev->dev, ACX0, DLA0, 1);
		break;
	}
#endif /* APU_POWER_BRING_UP */

	aputop_dump_pwr_res();

	platform_set_drvdata(pdev, &apupw);

	return ret;
}

static int mt8188_apu_top_rm(struct platform_device *pdev)
{
	int idx;

	LOG_DBG("%s +\n", __func__);

#if THERMAL_TEMP_UPDATE
	destroy_apu_thermal();
#endif
	destroy_plat_pwr_res();

	for (idx = 0; idx < APUPW_MAX_REGS; idx++)
		iounmap(apupw.regs[idx]);

	kfree(opp_tbl);
	opp_tbl = NULL;

	LOG_DBG("%s -\n", __func__);

	return 0;
}

static int mt8188_apu_top_prepare(struct device *dev)
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

static int mt8188_apu_top_suspend(struct device *dev)
{
	g_opp_cfg_acx0 = apu_readl(
			apupw.regs[apu_md32_mbox] + ACX0_LIMIT_OPP_REG);

	LOG_DBG("%s backup data 0x%08x\n", __func__,
			g_opp_cfg_acx0);
	return 0;
}

static int mt8188_apu_top_resume(struct device *dev)
{
	LOG_DBG("%s restore data 0x%08x\n", __func__,
			g_opp_cfg_acx0);

	apu_writel(g_opp_cfg_acx0,
			apupw.regs[apu_md32_mbox] + ACX0_LIMIT_OPP_REG);

	return 0;
}

static void aputop_dump_pwr_res(void)
{
	int vapu_en = 0, vapu_mode = 0;
	uint32_t vapu = 0;
	uint32_t vsram = 0;

	if (vapu_reg_id) {
		vapu = regulator_get_voltage(vapu_reg_id);
		vapu_en = regulator_is_enabled(vapu_reg_id);
		vapu_mode = regulator_get_mode(vapu_reg_id);
	}

	if (vsram_reg_id)
		vsram = regulator_get_voltage(vsram_reg_id);

	pr_info("%s vapu:%u(en:%d,mode:%d) vsram:%u\n",
			__func__, vapu, vapu_en, vapu_mode, vsram);

	pr_info("%s d:%ld d1:%ld d2:%ld d3:%ld d4:%ld d5:%ld d6:%ld d7:%ld\n",
			__func__,
			clk_get_rate(clk_top_dsp),
			clk_get_rate(clk_top_dsp1),
			clk_get_rate(clk_top_dsp2),
			clk_get_rate(clk_top_dsp3),
			clk_get_rate(clk_top_dsp4),
			clk_get_rate(clk_top_dsp5),
			clk_get_rate(clk_top_dsp6),
			clk_get_rate(clk_top_dsp7));

	/* mt8188_apu_dump_rpc_status(RCX, NULL); */

#if APU_POWER_BRING_UP
	mt8188_apu_dump_rpc_status(ACX0, NULL);
#endif
}

static int mt8188_apu_top_func(struct platform_device *pdev,
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
		mt8188_aputop_opp_limit(aputop, OPP_LIMIT_HAL);
		break;
	case APUTOP_FUNC_OPP_LIMIT_DBG:
		mt8188_aputop_opp_limit(aputop, OPP_LIMIT_DEBUG);
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
		mt8188_drv_cfg_remote_sync(aputop);
		break;
	case APUTOP_FUNC_IPI_TEST:
		test_ipi_wakeup_apu();
		break;
	case APUTOP_FUNC_ARE_DUMP1:
#if APUPW_DUMP_FROM_APMCU
		are_dump_entry(0);
		are_dump_entry(1);
#endif
		break;
	case APUTOP_FUNC_ARE_DUMP2:
#if APUPW_DUMP_FROM_APMCU
		are_dump_entry(2);
#endif
		break;
	default:
		pr_info("%s invalid func_id : %d\n", __func__, aputop->func_id);
		return -EINVAL;
	}

	return 0;
}

const struct apupwr_plat_data mt8188_plat_data = {
	.plat_name             = "mt8188_apupwr",
	.plat_aputop_on        = mt8188_apu_top_on,
	.plat_aputop_off       = mt8188_apu_top_off,
	.plat_aputop_pb        = mt8188_apu_top_pb,
	.plat_aputop_rm        = mt8188_apu_top_rm,
	.plat_aputop_prepare   = mt8188_apu_top_prepare,
	.plat_aputop_suspend   = mt8188_apu_top_suspend,
	.plat_aputop_resume    = mt8188_apu_top_resume,
	.plat_aputop_func      = mt8188_apu_top_func,
#if IS_ENABLED(CONFIG_DEBUG_FS)
	.plat_aputop_dbg_open  = mt8188_apu_top_dbg_open,
	.plat_aputop_dbg_write = mt8188_apu_top_dbg_write,
#endif
	.plat_rpmsg_callback   = mt8188_apu_top_rpmsg_cb,

	.plat_apu_devfreq_cooling_register      =
				mt8188_apu_devfreq_cooling_register,
	.plat_apu_devfreq_cooling_unregister    =
				mt8188_apu_devfreq_cooling_unregister,
	.plat_apu_devfreq_cooling_start_monitor =
				mt8188_apu_devfreq_cooling_start_monitor,
	.plat_apu_devfreq_cooling_stop_monitor  =
				mt8188_apu_devfreq_cooling_stop_monitor,
#if THERMAL_TEMP_UPDATE
	.plat_apu_thermal_timer_func            =
				mt8188_apu_thermal_timer_func,
#endif

	.bypass_pwr_on         = 0,
	.bypass_pwr_off        = 0,
};
