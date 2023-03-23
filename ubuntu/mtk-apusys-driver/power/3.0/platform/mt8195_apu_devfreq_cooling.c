// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/thermal.h>

#include "apu_top.h"
#include "mt8195_apupwr.h"
#include "mt8195_apupwr_prot.h"
#include "mt8195_apu_devfreq_cooling.h"

#define	KHZ				(1000)
#define	MHZ				(1000*KHZ)
#define DEVFREQ_POOLING_INTERVAL        (0) /* ms, 0 means event trigger */

struct apu_pwr_devfreq_st {
	struct devfreq *apu_devfreq;
	struct devfreq_dev_profile profile;
	enum dvfs_device_id dvfs_dev_id;
	int opp;
};

static spinlock_t lock;
static void __iomem *spare_reg_base;
static struct apu_pwr_devfreq_st apu_pwr_devfreq_arr[DVFS_DEV_MAX];
static enum apu_clksrc_id dvfs_dev_id_to_pll_idx[DVFS_DEV_MAX] = {
							PLL_VPU, PLL_DLA};

int mt8195_apu_devfreq_cooling_init(struct platform_device *pdev,
				    void __iomem *reg_base)
{
	spin_lock_init(&lock);
	spare_reg_base = reg_base;
	apu_writel(0x0, spare_reg_base + THERMAL_SYNC_REG);

	return 0;
}

static int _plat_opp_table_add(struct device *dev,
			       unsigned int dvfs_dev_id)
{
	unsigned long rate, volt;
	int i, ret = 0;
	int pll_idx = PLL_VPU;

	if (dvfs_dev_id >= DVFS_DEV_MAX)
		return -1;

	pll_idx = dvfs_dev_id_to_pll_idx[dvfs_dev_id];

	for (i = 0; i < mt8195_opp_tbl->tbl_size; i++) {
		rate = mt8195_opp_tbl->opp[i].pll_freq[pll_idx] * 1000;
		volt = mt8195_opp_tbl->opp[i].vapu;

		ret = dev_pm_opp_add(dev, rate, volt);
		pr_debug("[%s][%d] dvfs_dev_id = %d, rate = %ld, volt = %ld\n",
			__func__, __LINE__, dvfs_dev_id, rate, volt);

		if (ret)
			break;
	}

	return ret;
}

static uint8_t _plat_freq_to_opp(unsigned int dvfs_dev_id,
				 unsigned long freq)
{
	uint8_t opp = 0;
	uint32_t next_freq = 0;
	int pll_idx = PLL_VPU;

	if (dvfs_dev_id >= DVFS_DEV_MAX)
		return -1;

	pll_idx = dvfs_dev_id_to_pll_idx[dvfs_dev_id];

	for (opp = 0; opp < mt8195_opp_tbl->tbl_size - 1; opp++) {
		next_freq = mt8195_opp_tbl->opp[opp + 1].pll_freq[pll_idx] + 1;
		if (freq >= next_freq &&
			freq <= (mt8195_opp_tbl->opp[opp].pll_freq[pll_idx] + 1))
			break;
	}

	return opp;
}

static void _plat_thermal_throttle(void)
{
	uint32_t reg_data = 0x0;
	int max_opp = 0, opp = 0;
	int i = 0;

	spin_lock(&lock);

	for (i = 0; i < DVFS_DEV_MAX; i++) {
		opp = apu_pwr_devfreq_arr[i].opp;
		max_opp = (opp > max_opp) ? opp : max_opp;
	}

	reg_data = max_opp;
	pr_info("%s max_opp = 0x%08x\n", __func__, reg_data);
	apu_writel(reg_data, spare_reg_base + THERMAL_SYNC_REG);

	spin_unlock(&lock);
}

static int mt8195_apu_devfreq_set_target(struct device *dev,
					 unsigned long *freq, u32 flags)
{
	struct dev_pm_opp *opp;
	uint8_t normalize_opp = 0;
	unsigned long normalize_freq = 0;
	unsigned long ori_freq;
	unsigned int dvfs_dev_id;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)dev_get_drvdata(dev);
	dvfs_dev_id = (unsigned int)apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id >= DVFS_DEV_MAX)
		return -1;

	if (!freq) {
		pr_info("%s freq ptr is NULL\n", __func__);
		return -1;
	}

	/* The freq is an upper bound. opp should be lower */
	flags |= DEVFREQ_FLAG_LEAST_UPPER_BOUND;
	ori_freq = *freq;

	/*
	 * due to pm opp is not allow duplicated freq,
	 * so it is different with our freq table : pm_opp : 9 vs ours : 10
	 * we need to handle opp remap task as well.
	 * devfreq_recommended_opp will help to limit opp range (limit by therm)
	 */
	opp = devfreq_recommended_opp(dev, freq, flags);
	*freq = dev_pm_opp_get_freq(opp);
	normalize_freq = *freq / 1000;
	normalize_opp = _plat_freq_to_opp(dvfs_dev_id, normalize_freq);
	dev_pm_opp_put(opp);

	pr_info("[%s] dvfs_dev_id = %d, target:%lu limit:%lu (opp:%u)\n",
					__func__, dvfs_dev_id,
					ori_freq / 1000000,
					normalize_freq / 1000,
					normalize_opp);

	if (*freq == apu_pwr_devfreq_ptr->apu_devfreq->previous_freq) {
		pr_info("%s the same freq : %lu\n",
			__func__, normalize_freq / 1000);
		return 0;
	}

	apu_pwr_devfreq_ptr->opp = normalize_opp;
	_plat_thermal_throttle();

	return 0;
}

static int mt8195_apu_devfreq_get_status(struct device *dev,
			      struct devfreq_dev_status *stat)
{
	unsigned int dvfs_dev_id;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)dev_get_drvdata(dev);
	dvfs_dev_id = (unsigned int)apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id >= DVFS_DEV_MAX)
		return -1;

	if (!stat)
		return -1;

	/* total_time = busy_time could force simple_ondemand return max freq */
	stat->total_time = 100;
	stat->busy_time  = 100;
	stat->current_frequency =
		apu_pwr_devfreq_ptr->apu_devfreq->previous_freq;

	pr_info("%s stat->current_frequency:%lu\n",
				__func__, stat->current_frequency / 1000000);

	return 0;
}

static int mt8195_apu_devfreq_get_cur_freq(struct device *dev,
					   unsigned long *freq)
{
	unsigned int dvfs_dev_id;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;


	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)dev_get_drvdata(dev);
	dvfs_dev_id = (unsigned int)apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id >= DVFS_DEV_MAX)
		return -1;

	if (!freq)
		return -1;

	*freq = apu_pwr_devfreq_ptr->apu_devfreq->previous_freq;

	/* TODO: devfreq flow need update */
/*	pr_info("%s: %lu max:%lu min:%lu\n",
 *			__func__, *freq / 1000000,
 *			apu_pwr_devfreq_ptr->apu_devfreq->max_freq / 1000000,
 *			apu_pwr_devfreq_ptr->apu_devfreq->min_freq / 1000000);
 */
	return 0;
}

int mt8195_apu_devfreq_cooling_register(struct platform_device *pdev)
{
	unsigned int dvfs_dev_id = (unsigned int)DVFS_DEV_VPU;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr = NULL;
	struct device *dev = NULL;
	int ret = 0;

	if (strstr(dev_name(&pdev->dev), "vpu_devfreq")) {
		dvfs_dev_id = (unsigned int)DVFS_DEV_VPU;
	} else if (strstr(dev_name(&pdev->dev), "mdla_devfreq")) {
		dvfs_dev_id = (unsigned int)DVFS_DEV_MDLA;
	} else {
		pr_info("[%s][%d] Wrong dev = %s\n",
			__func__, __LINE__, dev_name(&pdev->dev));
		return -1;
	}

	pr_info("[%s][%d] dev_name(&pdev->dev) = %s, dvfs_dev_id = %d\n",
		__func__, __LINE__, dev_name(&pdev->dev), dvfs_dev_id);

	dev = &pdev->dev;
	apu_pwr_devfreq_ptr = &apu_pwr_devfreq_arr[dvfs_dev_id];
	platform_set_drvdata(pdev, (void *)apu_pwr_devfreq_ptr);

	ret = _plat_opp_table_add(dev, dvfs_dev_id);
	if (ret)
		goto err;

	apu_pwr_devfreq_ptr->dvfs_dev_id = dvfs_dev_id;

	if (dvfs_dev_id == DVFS_DEV_VPU)
		apu_pwr_devfreq_ptr->profile.initial_freq =
						MVPU_DEFAULT_FREQ * MHZ;
	else
		apu_pwr_devfreq_ptr->profile.initial_freq =
						MDLA_DEFAULT_FREQ * MHZ;

	apu_pwr_devfreq_ptr->profile.polling_ms = DEVFREQ_POOLING_INTERVAL;
	apu_pwr_devfreq_ptr->profile.target = mt8195_apu_devfreq_set_target;
	apu_pwr_devfreq_ptr->profile.get_dev_status =
						mt8195_apu_devfreq_get_status;
	apu_pwr_devfreq_ptr->profile.get_cur_freq =
						mt8195_apu_devfreq_get_cur_freq;
	apu_pwr_devfreq_ptr->profile.is_cooling_device = true;

	apu_pwr_devfreq_ptr->apu_devfreq = devm_devfreq_add_device(
						&pdev->dev,
						&apu_pwr_devfreq_ptr->profile,
						DEVFREQ_GOV_SIMPLE_ONDEMAND,
						NULL);

	if (IS_ERR(apu_pwr_devfreq_ptr->apu_devfreq)) {
		pr_info(
			"%s error in devm_devfreq_add_device, dvfs_dev_id = %d\n",
			__func__, dvfs_dev_id);
		return PTR_ERR(apu_pwr_devfreq_ptr->apu_devfreq);
	}

	ret = devm_devfreq_register_opp_notifier(
			dev, apu_pwr_devfreq_ptr->apu_devfreq);
	if (ret) {
		pr_info("%s failed to register OPP notifier (%d)\n",
							__func__, ret);
		goto remove_devfreq;
	}

	pr_info("[%s][%d] dvfs_dev_id = %d, register cooling dev ok!\n",
		__func__, __LINE__, dvfs_dev_id);

	return 0;

remove_devfreq:
	devm_devfreq_remove_device(dev, apu_pwr_devfreq_ptr->apu_devfreq);

err:
	pr_info("[%s][%d] dvfs_dev_id = %d, register cooling dev NG!\n",
		__func__, __LINE__, dvfs_dev_id);
	return ret;
}

void mt8195_apu_devfreq_cooling_unregister(struct platform_device *pdev)
{
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr = NULL;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)
					dev_get_drvdata(&pdev->dev);

	if (apu_pwr_devfreq_ptr->apu_devfreq) {
		devm_devfreq_unregister_opp_notifier(
			apu_pwr_devfreq_ptr->apu_devfreq->dev.parent,
			apu_pwr_devfreq_ptr->apu_devfreq);

		devm_devfreq_remove_device(
			apu_pwr_devfreq_ptr->apu_devfreq->dev.parent,
			apu_pwr_devfreq_ptr->apu_devfreq);
	}
}

void mt8195_apu_devfreq_cooling_start_monitor(struct platform_device *pdev)
{
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr = NULL;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)
					dev_get_drvdata(&pdev->dev);
	devfreq_resume_device(apu_pwr_devfreq_ptr->apu_devfreq);
}

void mt8195_apu_devfreq_cooling_stop_monitor(struct platform_device *pdev)
{
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr = NULL;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)
					dev_get_drvdata(&pdev->dev);
	devfreq_suspend_device(apu_pwr_devfreq_ptr->apu_devfreq);
}

void mt8195_apu_devfreq_cooling_start(void)
{
	int i = 0;

	LOG_DBG("[%s][%d] ++\n", __func__, __LINE__);
	for (i = 0; i < DVFS_DEV_MAX; i++) {
		if (apu_pwr_devfreq_arr[i].apu_devfreq)
			devfreq_resume_device(
				apu_pwr_devfreq_arr[i].apu_devfreq);
	}
}

void mt8195_apu_devfreq_cooling_stop(void)
{
	int i = 0;

	LOG_DBG("[%s][%d] ++\n", __func__, __LINE__);
	for (i = 0; i < DVFS_DEV_MAX; i++) {
		if (apu_pwr_devfreq_arr[i].apu_devfreq)
			devfreq_suspend_device(
				apu_pwr_devfreq_arr[i].apu_devfreq);
	}
}

