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
#include <linux/version.h>

#include "apu_log.h"
#include "apu_top.h"
#include "mt8195_apupwr.h"
#include "mt8195_apupwr_prot.h"
#include "mt8195_apu_devfreq_cooling.h"

#define DEVFREQ_POOLING_INTERVAL        (0) /* ms, 0 means event trigger */

struct apu_power_table {
	int table_size;
	int power[];
};

struct apu_pwr_devfreq_st {
	struct devfreq *apu_devfreq;
	struct devfreq_dev_profile profile;
	struct devfreq_cooling_power cooling_power_ops;
	struct thermal_cooling_device *apu_devfreq_cooling;
	enum dvfs_device_id dvfs_dev_id;
	struct apu_power_table *apu_power_table;
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

static int _plat_get_power_table(struct device *dev,
				 enum dvfs_device_id dvfs_dev_id)
{
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr = NULL;
	struct apu_power_table *apu_power_table = NULL;
	struct device_node *node = dev->of_node;
	int i, table_size = 0;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)dev_get_drvdata(dev);

	table_size = of_property_count_u32_elems(node, "power_table");
	if (table_size <= 0) {
		dev_info(dev, "[%s][%d] table_size = %d\n",
			__func__, __LINE__, table_size);
		return -1;
	}

	apu_power_table = kzalloc(sizeof(struct apu_power_table) +
					sizeof(int) * table_size, GFP_KERNEL);
	if (IS_ERR_OR_NULL(apu_power_table))
		return -1;

	apu_power_table->table_size = table_size;
	for (i = 0; i < table_size; i++) {
		of_property_read_u32_index(node, "power_table", i,
					   &apu_power_table->power[i]);
	}

	for (i = 0; i < table_size; i++) {
		pr_debug("[%s][%d] power_table[%d] = %d\n",
			__func__, __LINE__, i, apu_power_table->power[i]);
	}

	apu_pwr_devfreq_ptr->apu_power_table = apu_power_table;
	return 0;
}

static int _plat_opp_table_add(struct device *dev,
			       enum dvfs_device_id dvfs_dev_id)
{
	unsigned long rate, volt;
	int i, ret = 0;
	int pll_idx = PLL_VPU;

	if (dvfs_dev_id < DVFS_DEV_VPU || dvfs_dev_id >= DVFS_DEV_MAX)
		return -1;

	pll_idx = dvfs_dev_id_to_pll_idx[dvfs_dev_id];

	for (i = 0; i < mt8195_opp_tbl->tbl_size; i++) {
		rate = mt8195_opp_tbl->opp[i].pll_freq[pll_idx] * 1000;

		if (dvfs_dev_id == DVFS_DEV_VPU)
			volt = mt8195_opp_tbl->opp[i].vapu;
		else
			volt = mt8195_opp_tbl->opp[i].vmdla;

		ret = dev_pm_opp_add(dev, rate, volt);
		pr_debug("[%s][%d] dvfs_dev_id = %d, rate = %ld, volt = %ld\n",
			__func__, __LINE__, dvfs_dev_id, rate, volt);

		if (ret)
			break;
	}

	return ret;
}

static uint8_t _plat_freq_to_opp(enum dvfs_device_id dvfs_dev_id,
				 unsigned long freq)
{
	uint8_t opp = 0;
	uint32_t next_freq = 0;
	int pll_idx = PLL_VPU;

	if (dvfs_dev_id < DVFS_DEV_VPU || dvfs_dev_id >= DVFS_DEV_MAX)
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
	enum dvfs_device_id dvfs_dev_id;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)dev_get_drvdata(dev);
	dvfs_dev_id = apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id < DVFS_DEV_VPU || dvfs_dev_id >= DVFS_DEV_MAX)
		return -1;

	if (!freq) {
		pr_info("%s freq ptr is NULL\n", __func__);
		return -1;
	}

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
	enum dvfs_device_id dvfs_dev_id;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)dev_get_drvdata(dev);
	dvfs_dev_id = apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id < DVFS_DEV_VPU || dvfs_dev_id >= DVFS_DEV_MAX)
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
	enum dvfs_device_id dvfs_dev_id;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)dev_get_drvdata(dev);
	dvfs_dev_id = apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id < DVFS_DEV_VPU || dvfs_dev_id >= DVFS_DEV_MAX)
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)
static unsigned long mt8195_apu_static_power(struct devfreq *devfreq,
					     unsigned long voltage_mv)
{
	return 0;
}

static unsigned long mt8195_apu_dynamic_power(struct devfreq *devfreq,
					      unsigned long freqHz,
					      unsigned long voltage_mv)
{
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;
	enum dvfs_device_id dvfs_dev_id;
	unsigned long power = 0;
	unsigned long normalize_freq = freqHz / 1000;
	uint8_t opp;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)
					dev_get_drvdata(devfreq->dev.parent);
	dvfs_dev_id = apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id < DVFS_DEV_VPU || dvfs_dev_id >= DVFS_DEV_MAX) {
		pr_info("%s, ERROR! dvfs_dev_id = %d\n", __func__, dvfs_dev_id);
		return -1;
	}

	opp = _plat_freq_to_opp(dvfs_dev_id, normalize_freq);
	if (opp >= mt8195_opp_tbl->tbl_size)
		opp = mt8195_opp_tbl->tbl_size - 1;

	power = apu_pwr_devfreq_ptr->apu_power_table->power[opp];

	pr_info("%s freq=%lu, volt=%lu, power=%lu\n",
				__func__, freqHz, voltage_mv, power);

	return power;
}
#else
static int mt8195_apu_real_power(struct devfreq *devfreq, unsigned int *power,
			unsigned long freqHz, unsigned long voltage_mv)
{
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr;
	enum dvfs_device_id dvfs_dev_id;
	unsigned long normalize_freq = freqHz / 1000;
	uint8_t opp;

	if (!power) { 
		pr_info("%s, mt8195_apu_real_power power = NULL\n");
		return -1;
	}

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)
					dev_get_drvdata(devfreq->dev.parent);
	dvfs_dev_id = apu_pwr_devfreq_ptr->dvfs_dev_id;
	if (dvfs_dev_id < DVFS_DEV_VPU || dvfs_dev_id >= DVFS_DEV_MAX) {
		pr_info("%s, mt8195_apu_real_power ERROR! dvfs_dev_id = %d\n", __func__, dvfs_dev_id);
		return -1;
	}

	opp = _plat_freq_to_opp(dvfs_dev_id, normalize_freq);
	if (opp >= mt8195_opp_tbl->tbl_size)
		opp = mt8195_opp_tbl->tbl_size - 1;

	*power = apu_pwr_devfreq_ptr->apu_power_table->power[opp];

	pr_info("%s mt8195_apu_real_power freq=%lu, volt=%lu, power=%lu\n",
				__func__, freqHz, voltage_mv, power);

	return 0;
}
#endif

int mt8195_apu_devfreq_cooling_register(struct platform_device *pdev)
{
	enum dvfs_device_id dvfs_dev_id = DVFS_DEV_VPU;
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr = NULL;
	struct device_node *of_node = NULL;
	struct dev_pm_opp *opp = NULL;
	struct device *dev = NULL;
	unsigned long rate;
	int ret = 0;

	if (strstr(dev_name(&pdev->dev), "vpu_devfreq")) {
		dvfs_dev_id = DVFS_DEV_VPU;
	} else if (strstr(dev_name(&pdev->dev), "mdla_devfreq")) {
		dvfs_dev_id = DVFS_DEV_MDLA;
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

	ret = _plat_get_power_table(dev, dvfs_dev_id);
	if (ret)
		goto err;

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

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,15,0)					
	apu_pwr_devfreq_ptr->cooling_power_ops.get_static_power =
						mt8195_apu_static_power;
	apu_pwr_devfreq_ptr->cooling_power_ops.get_dynamic_power =
						mt8195_apu_dynamic_power;
#else
	apu_pwr_devfreq_ptr->cooling_power_ops.get_real_power =
						mt8195_apu_real_power;
#endif
						
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

	/* set devfreq's min freq */
	rate = 0;
	opp = dev_pm_opp_find_freq_ceil(dev, &rate);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		goto remove_devfreq;
	}
	/* TODO: devfreq flow need update */
	//apu_pwr_devfreq_ptr->apu_devfreq->user_min_freq_req = rate;
	dev_pm_opp_put(opp);

	/* set devfreq's max freq */
	rate = ULONG_MAX;
	opp = dev_pm_opp_find_freq_floor(dev, &rate);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		goto remove_devfreq;
	}
	/* TODO: devfreq flow need update */
	//apu_pwr_devfreq_ptr->apu_devfreq->user_max_freq_req = rate;
	dev_pm_opp_put(opp);

	ret = devm_devfreq_register_opp_notifier(
			dev, apu_pwr_devfreq_ptr->apu_devfreq);
	if (ret) {
		pr_info("%s failed to register OPP notifier (%d)\n",
							__func__, ret);
		goto remove_devfreq;
	}

	of_node = of_node_get(dev->of_node);
	apu_pwr_devfreq_ptr->apu_devfreq_cooling =
		of_devfreq_cooling_register_power(
				of_node,
				apu_pwr_devfreq_ptr->apu_devfreq,
				&apu_pwr_devfreq_ptr->cooling_power_ops);
	of_node_put(of_node);

	if (IS_ERR(apu_pwr_devfreq_ptr->apu_devfreq_cooling)) {
		pr_info("%s error in of_devfreq_cooling_register_power\n",
				__func__);
		ret = PTR_ERR(apu_pwr_devfreq_ptr->apu_devfreq_cooling);
		goto unregister_opp_notifier;
	}

	pr_info("[%s][%d] dvfs_dev_id = %d, register cooling dev ok!\n",
		__func__, __LINE__, dvfs_dev_id);

	return 0;

unregister_opp_notifier:
	devm_devfreq_unregister_opp_notifier(dev,
					apu_pwr_devfreq_ptr->apu_devfreq);

remove_devfreq:
	devm_devfreq_remove_device(dev, apu_pwr_devfreq_ptr->apu_devfreq);

err:
	return ret;
}

void mt8195_apu_devfreq_cooling_unregister(struct platform_device *pdev)
{
	struct apu_pwr_devfreq_st *apu_pwr_devfreq_ptr = NULL;

	apu_pwr_devfreq_ptr = (struct apu_pwr_devfreq_st *)
					dev_get_drvdata(&pdev->dev);

	if (apu_pwr_devfreq_ptr->apu_devfreq_cooling)
		devfreq_cooling_unregister(
			apu_pwr_devfreq_ptr->apu_devfreq_cooling);

	if (apu_pwr_devfreq_ptr->apu_devfreq) {
		devm_devfreq_unregister_opp_notifier(
			apu_pwr_devfreq_ptr->apu_devfreq->dev.parent,
			apu_pwr_devfreq_ptr->apu_devfreq);

		devm_devfreq_remove_device(
			apu_pwr_devfreq_ptr->apu_devfreq->dev.parent,
			apu_pwr_devfreq_ptr->apu_devfreq);
	}

	kfree(apu_pwr_devfreq_ptr->apu_power_table);
	apu_pwr_devfreq_ptr->apu_power_table = NULL;

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

