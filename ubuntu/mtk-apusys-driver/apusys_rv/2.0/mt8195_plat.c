// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/sched/clock.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>

#ifdef APU_AEE_ENABLE
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include "mt-plat/aee.h"
#endif
#endif

#include "apusys_power.h"
#include "apusys_secure.h"
#include "../apu.h"
#include "../apu_debug.h"
#include "../apu_config.h"
#include "../apu_hw.h"
#include "../apu_excep.h"

#define MT8195_APUSYS_SECURE (1)
#define MT8195_USERFW_CTXT (0x100)
#define MT8195_SECUREFW_CTXT (0x104)
#define MT8195_MD32_SECFW_DOMAIN (0x010)
#define MT8195_MD32_USERFW_DOMAIN (0x014)


/* for IPI IRQ affinity tuning*/
static struct cpumask perf_cpus, normal_cpus;

static uint32_t apusys_rv_smc_call(struct device *dev, uint32_t smc_id,
	uint32_t a2)
{
#if MT8195_APUSYS_SECURE
	struct arm_smccc_res res;

	dev_info(dev, "%s: smc call %d\n",
			__func__, smc_id);

	arm_smccc_smc(MTK_SIP_APUSYS_CONTROL, smc_id,
				a2, 0, 0, 0, 0, 0, &res);
	if (((int) res.a0) < 0)
		dev_info(dev, "%s: smc call %d return error(%ld)\n",
			__func__,
			smc_id, res.a0);

	return res.a0;
#else
	return 0;
#endif
}

static int mt8195_rproc_init(struct mtk_apu *apu)
{
	return 0;
}

static int mt8195_rproc_exit(struct mtk_apu *apu)
{
	return 0;
}

static void apu_setup_reviser(struct mtk_apu *apu, int boundary, int ns, int domain)
{
	struct device *dev = apu->dev;
	unsigned long flags;

	if (apu->platdata->flags & F_SECURE_BOOT) {
		apusys_rv_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_RV_SETUP_REVISER, 0);
	} else {
		spin_lock_irqsave(&apu->reg_lock, flags);
		/* setup boundary */
		iowrite32(boundary,
			  apu->apu_sctrl_reviser + MT8195_USERFW_CTXT);
		iowrite32(boundary,
			  apu->apu_sctrl_reviser + MT8195_SECUREFW_CTXT);
		iowrite32((ns << 4) | domain,
			  apu->apu_ao_ctl + MT8195_MD32_SECFW_DOMAIN);
		iowrite32((ns << 4) | domain,
			  apu->apu_ao_ctl + MT8195_MD32_USERFW_DOMAIN);
		spin_unlock_irqrestore(&apu->reg_lock, flags);
		pr_info("%s: USERFW_CTXT = 0x%x, MT8195_SECUREFW_CTXT = 0x%x\n",
			__func__,
			ioread32(apu->apu_sctrl_reviser + MT8195_USERFW_CTXT),
			ioread32(apu->apu_sctrl_reviser + MT8195_SECUREFW_CTXT));

		pr_info("%s: MT8195_MD32_SECFW_DOMAIN = 0x%x, MT8195_MD32_USERFW_DOMAIN = 0x%x\n",
			__func__,
			ioread32(apu->apu_ao_ctl + MT8195_MD32_SECFW_DOMAIN),
			ioread32(apu->apu_ao_ctl + MT8195_MD32_USERFW_DOMAIN));
	}
}

static void apu_reset_mp(struct mtk_apu *apu)
{
	struct device *dev = apu->dev;
	unsigned long flags;

	if (apu->platdata->flags & F_SECURE_BOOT) {
		apusys_rv_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_RV_RESET_MP, 0);
	} else {
		spin_lock_irqsave(&apu->reg_lock, flags);
		/* reset uP */
		iowrite32(0, apu->md32_sysctrl + MD32_SYS_CTRL);
		spin_unlock_irqrestore(&apu->reg_lock, flags);

		udelay(10);

		spin_lock_irqsave(&apu->reg_lock, flags);
		/* Enable IOMMU */
		iowrite32(0xEA9, apu->md32_sysctrl + MD32_SYS_CTRL);
		spin_unlock_irqrestore(&apu->reg_lock, flags);
		apu_drv_debug("%s: MD32_SYS_CTRL = 0x%x\n",
			__func__,
			ioread32(apu->md32_sysctrl + MD32_SYS_CTRL));
	}
}

static void apu_setup_boot(struct mtk_apu *apu)
{
	struct device *dev = apu->dev;
	unsigned long flags;
	int boot_from_tcm;

	if (TCM_OFFSET == 0)
		boot_from_tcm = 1;
	else
		boot_from_tcm = 0;

	if (apu->platdata->flags & F_SECURE_BOOT) {
		apusys_rv_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_RV_SETUP_BOOT, 0);
	} else {
		/* Set uP boot addr to DRAM.
		 * If boot from tcm == 1, boot addr will always map to
		 * 0x1d000000 no matter what value boot_addr is
		 */
		spin_lock_irqsave(&apu->reg_lock, flags);
		if ((apu->platdata->flags & F_BYPASS_IOMMU)
			|| (apu->platdata->flags & F_PRELOAD_FIRMWARE)) {
			iowrite32((u32)(apu->code_da << 1),	apu->apu_ao_ctl + MD32_BOOT_CTRL);
			apu_drv_debug("%s: t << 1 = 0x%x, code_da = 0x%x\n",
			__func__, apu->code_da << 1, apu->code_da);
			apu_drv_debug("%s: t MD32_BOOT_CTRL 1 = 0x%x\n",
			__func__, ioread32(apu->apu_ao_ctl + MD32_BOOT_CTRL));
		} else {
			iowrite32((u32)CODE_BUF_DA | boot_from_tcm,
				apu->apu_ao_ctl + MD32_BOOT_CTRL);
						apu_drv_debug("%s: t MD32_BOOT_CTRL 2 = 0x%x\n",
			__func__, ioread32(apu->apu_ao_ctl + MD32_BOOT_CTRL));
		}
		spin_unlock_irqrestore(&apu->reg_lock, flags);
		apu_drv_debug("%s: MD32_BOOT_CTRL = 0x%x\n",
			__func__, ioread32(apu->apu_ao_ctl + MD32_BOOT_CTRL));

		spin_lock_irqsave(&apu->reg_lock, flags);
		/* set predefined MPU region for cache access */
		iowrite32(0xAB, apu->apu_ao_ctl + MD32_PRE_DEFINE);
		spin_unlock_irqrestore(&apu->reg_lock, flags);
		apu_drv_debug("%s: MD32_PRE_DEFINE = 0x%x\n",
			__func__, ioread32(apu->apu_ao_ctl + MD32_PRE_DEFINE));
	}
}

static void apu_start_mp(struct mtk_apu *apu)
{
	struct device *dev = apu->dev;
	int i;
	unsigned long flags;

	if (apu->platdata->flags & F_SECURE_BOOT) {
		apusys_rv_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_RV_START_MP, 0);
	} else {
		spin_lock_irqsave(&apu->reg_lock, flags);
		/* Release runstall */
		iowrite32(0x0, apu->apu_ao_ctl + MD32_RUNSTALL);
		spin_unlock_irqrestore(&apu->reg_lock, flags);

		usleep_range(0, 20);
		for (i = 0; i < 20; i++) {
			dev_info(dev, "apu boot: pc=%08x, sp=%08x\n",
				ioread32(apu->md32_sysctrl + 0x838),
				ioread32(apu->md32_sysctrl + 0x840));
			usleep_range(0, 20);
		}
	}
}

static int mt8195_rproc_start(struct mtk_apu *apu)
{
	int ns = 1; /* Non Secure */
	int domain = 0;
	int boundary = (u32) upper_32_bits(apu->code_da);
    pr_info("mt8195_rproc_start + ");
	apu_setup_reviser(apu, boundary, ns, domain);
	apu_reset_mp(apu);
	apu_setup_boot(apu);
	apu_start_mp(apu);
	pr_info("mt8195_rproc_start - ");

	return 0;
}

static int mt8195_rproc_stop(struct mtk_apu *apu)
{
	struct device *dev = apu->dev;
	pr_info("%s +\n", __func__);
	/* Hold runstall */
	if (apu->platdata->flags & F_SECURE_BOOT)
		apusys_rv_smc_call(dev,
			MTK_APUSYS_KERNEL_OP_APUSYS_RV_STOP_MP, 0);
	else
		iowrite32(0x1, apu->apu_ao_ctl + MD32_RUNSTALL);
	return 0;
}

static int mt8195_rproc_resume(struct mtk_apu *apu)
{
	unsigned long flags;
	pr_info("%s +\n", __func__);

	spin_lock_irqsave(&apu->reg_lock, flags);
	/* Enable IOMMU */
	iowrite32(0xEA9, apu->md32_sysctrl);
	spin_unlock_irqrestore(&apu->reg_lock, flags);
	/* Release Runstall */
	apu_start_mp(apu);
	pr_info("%s -\n", __func__);

	return 0;
}

static int mt8195_apu_power_init(struct mtk_apu *apu)
{
	struct device *dev = apu->dev;
	struct device_node *np;
	struct platform_device *pdev;

	/* power dev */
	np = of_parse_phandle(dev->of_node, "mediatek,apusys_power", 0);
	if (!np) {
		dev_info(dev, "failed to parse apusys_power node\n");
		return -EINVAL;
	}

	if (!of_device_is_available(np)) {
		dev_info(dev, "unable to find apusys_power node\n");
		of_node_put(np);
		return -ENODEV;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_info(dev, "apusys_power is not ready yet\n");
		of_node_put(np);
		return -EPROBE_DEFER;
	}

	dev_info(dev, "%s: get power_dev, name=%s\n", __func__, pdev->name);

	apu->power_dev = &pdev->dev;
	of_node_put(np);

	/* apu iommu 0 */
	np = of_parse_phandle(dev->of_node, "apu_iommu0", 0);
	if (!np) {
		dev_info(dev, "failed to parse apu_iommu0 node\n");
		return -EINVAL;
	}
	if (!of_device_is_available(np)) {
		dev_info(dev, "unable to find apu_iommu0 node\n");
		of_node_put(np);
		return -ENODEV;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_info(dev, "apu_iommu0 is not ready yet\n");
		of_node_put(np);
		return -EPROBE_DEFER;
	}

	dev_info(dev, "%s: get apu_iommu0 device, name=%s\n", __func__, pdev->name);

	apu->apu_iommu0 = &pdev->dev;
	of_node_put(np);

	/* apu iommu 1 */
	np = of_parse_phandle(dev->of_node, "apu_iommu1", 0);
	if (!np) {
		dev_info(dev, "failed to parse apu_iommu1 node\n");
		return -EINVAL;
	}

	if (!of_device_is_available(np)) {
		dev_info(dev, "unable to find apu_iommu1 node\n");
		of_node_put(np);
		return -ENODEV;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_info(dev, "apu_iommu1 is not ready yet\n");
		of_node_put(np);
		return -EPROBE_DEFER;
	}

	dev_info(dev, "%s: get apu_iommu1 device, name=%s\n", __func__, pdev->name);

	apu->apu_iommu1 = &pdev->dev;
	of_node_put(np);

	return 0;
}

static int mt8195_apu_power_on(struct mtk_apu *apu)
{
	struct device *dev = apu->dev;
	int ret, timeout, i;

	/* to force apu top power on synchronously */
	ret = pm_runtime_get_sync(apu->power_dev);
	if (ret < 0) {
		dev_info(dev,
			 "%s: call to get_sync(power_dev) failed, ret=%d\n",
			 __func__, ret);
		/* apusys_rv_aee_warn("APUSYS_RV", "APUSYS_RV_RPM_GET_PWR_ERROR"); */
		return ret;
	}

	/* to notify IOMMU power on */
	/* workaround possible nested disable issue */
	i = 0;
	do {
		ret = pm_runtime_get_sync(apu->apu_iommu0);
		/*try atmost 7 times since disable_depth is 3-bit wide */
		if (ret == -EACCES && i <= 7) {
			pm_runtime_enable(apu->apu_iommu0);
			pm_runtime_put_sync(apu->apu_iommu0);
			i++;
			dev_info(apu->dev,
				 "%s: %s is disabled. Enable and retry(%d)\n",
				 __func__,
				 to_platform_device(apu->apu_iommu0)->name, i);
		} else if (ret < 0)
			goto iommu_get_error;

	} while (ret < 0);

	i = 0;
	/* workaround possible nested disable issue */
	do {
		ret = pm_runtime_get_sync(apu->apu_iommu1);
		/*try atmost 7 times since disable_depth is 3-bit wide */
		if (ret == -EACCES && i <= 7) {
			pm_runtime_enable(apu->apu_iommu1);
			pm_runtime_put_sync(apu->apu_iommu1);
			i++;
			dev_info(apu->dev,
				 "%s: %s is disabled. Enable and retry(%d)\n",
				 __func__,
				 to_platform_device(apu->apu_iommu1)->name, i);
			continue;
		} else if (ret < 0)
			pm_runtime_put_sync(apu->apu_iommu0);

	} while (ret < 0);

iommu_get_error:
	if (ret < 0) {
		dev_info(apu->dev,
			 "%s: call to get_sync(iommu) failed, ret=%d\n",
			 __func__, ret);
		apusys_rv_aee_warn("APUSYS_RV", "APUSYS_IOMMU_RPM_GET_ERROR");
		goto error_put_power_dev;
	}

	/* polling IOMMU rpm state till active */
	timeout = 5000;
	while ((!pm_runtime_active(apu->apu_iommu0) ||
			!pm_runtime_active(apu->apu_iommu1)) && timeout-- > 0)
		msleep(20);
	if (timeout <= 0) {
		dev_info(apu->dev, "%s: polling iommu on timeout!!\n",
			 __func__);
		WARN_ON(0);
		apusys_rv_aee_warn("APUSYS_RV", "APUSYS_RV_IOMMU_ON_TIMEOUT");
		ret = -ETIMEDOUT;
		goto error_put_iommu_dev;
	}

	ret = pm_runtime_get_sync(apu->dev);
	if (ret < 0) {
		dev_info(apu->dev,
			 "%s: call to get_sync(dev) failed, ret=%d\n",
			 __func__, ret);
		apusys_rv_aee_warn("APUSYS_RV", "APUSYS_RV_RPM_GET_ERROR");
		goto error_put_iommu_dev;
	}

	return 0;

error_put_iommu_dev:
	pm_runtime_put_sync(apu->apu_iommu1);
	pm_runtime_put_sync(apu->apu_iommu0);

error_put_power_dev:
	pm_runtime_put_sync(apu->power_dev);

	return ret;
}

static int mt8195_apu_power_off(struct mtk_apu *apu)
{
	struct device *dev = apu->dev;
	int ret, timeout, i;
	struct mtk_apu_hw_ops *hw_ops = &apu->platdata->ops;

	ret = pm_runtime_put_sync(apu->dev);
	if (ret) {
		dev_info(dev,
			 "%s: call to put_sync(dev) failed, ret=%d\n",
			 __func__, ret);
		apusys_rv_aee_warn("APUSYS_RV", "APUSYS_RV_RPM_PUT_ERROR");
		return ret;
	}

	/* to notify IOMMU power off */
	/* workaround possible nested disable issue */
	i = 0;
	do {
		ret = pm_runtime_put_sync(apu->apu_iommu1);
		/*try atmost 7 times since disable_depth is 3-bit wide */
		if (ret == -EACCES && i <= 7) {
			pm_runtime_enable(apu->apu_iommu1);
			pm_runtime_get_sync(apu->apu_iommu1);
			i++;
			dev_info(apu->dev,
				 "%s: %s is disabled. Enable and retry(%d)\n",
				 __func__,
				 to_platform_device(apu->apu_iommu1)->name, i);
		} else if (ret < 0)
			goto iommu_put_error;

	} while (ret < 0);

	i = 0;
	do {
		ret = pm_runtime_put_sync(apu->apu_iommu0);
		/*try atmost 7 times since disable_depth is 3-bit wide */
		if (ret == -EACCES && i <= 7) {
			pm_runtime_enable(apu->apu_iommu0);
			pm_runtime_get_sync(apu->apu_iommu0);
			i++;
			dev_info(apu->dev,
				 "%s: %s is disabled. Enable and retry(%d)\n",
				 __func__,
				 to_platform_device(apu->apu_iommu0)->name, i);
		} else if (ret < 0)
			pm_runtime_get_sync(apu->apu_iommu1);

	} while (ret < 0);

iommu_put_error:
	if (ret < 0) {
		dev_info(apu->dev,
			 "%s: call to put_sync(iommu) failed, ret=%d\n",
			 __func__, ret);
		apusys_rv_aee_warn("APUSYS_RV", "APUSYS_IOMMU_RPM_PUT_ERROR");
		goto error_get_rv_dev;
	}

	/* polling IOMMU rpm state till suspended */
	timeout = 5000;
	while ((!pm_runtime_suspended(apu->apu_iommu0) ||
		   !pm_runtime_suspended(apu->apu_iommu1)) && timeout-- > 0)
		msleep(20);
	if (timeout <= 0) {
		dev_info(apu->dev, "%s: polling iommu off timeout!!\n",
			 __func__);
		WARN_ON(0);
		apusys_rv_aee_warn("APUSYS_RV", "APUSYS_RV_IOMMU_OFF_TIMEOUT");
		ret = -ETIMEDOUT;
		goto error_get_iommu_dev;
	}
	dev_info(apu->dev, "%s: pm_runtime_put_sync apu->power_dev\n",
				 __func__);

	/* to force apu top power off synchronously */
	ret = pm_runtime_put_sync(apu->power_dev);
	dev_info(apu->dev, "%s: pm_runtime_put_sync apu->power_dev end\n",
				 __func__);

	if (ret) {
		dev_info(apu->dev,
			 "%s: call to put_sync(power_dev) failed, ret=%d\n",
			 __func__, ret);
		/* apusys_rv_aee_warn("APUSYS_RV", "APUSYS_RV_RPM_PUT_PWR_ERROR"); */
		goto error_get_iommu_dev;
	}

	/* polling APU TOP rpm state till suspended */
	timeout = 500;
	while (!pm_runtime_suspended(apu->power_dev) && timeout-- > 0)
		msleep(20);
	if (timeout <= 0) {
		dev_info(apu->dev, "%s: polling power off timeout!!\n",
			 __func__);
		WARN_ON(0);
		apusys_rv_aee_warn("APUSYS_RV", "APUSYS_RV_PWRDN_TIMEOUT");
		ret = -ETIMEDOUT;
		goto error_get_power_dev;
	}
	/* hold runstall after top pwr off */
	if (hw_ops->stop)
		hw_ops->stop(apu);

	return 0;

error_get_power_dev:
	pm_runtime_get_sync(apu->power_dev);
error_get_iommu_dev:
	pm_runtime_get_sync(apu->apu_iommu0);
	pm_runtime_get_sync(apu->apu_iommu1);
error_get_rv_dev:
	pm_runtime_get_sync(apu->dev);

	return ret;
}


static int mt8195_irq_affin_init(struct mtk_apu *apu)
{
	int i;

	/* init perf_cpus mask 0x80, CPU7 only */
	cpumask_clear(&perf_cpus);
	cpumask_set_cpu(7, &perf_cpus);

	/* init normal_cpus mask 0x0f, CPU0~CPU4 */
	cpumask_clear(&normal_cpus);
	for (i = 0; i < 4; i++)
		cpumask_set_cpu(i, &normal_cpus);

	irq_set_affinity_hint(apu->mbox0_irq_number, &normal_cpus);

	return 0;
}

static int mt8195_irq_affin_set(struct mtk_apu *apu)
{
	irq_set_affinity_hint(apu->mbox0_irq_number, &perf_cpus);

	return 0;
}

static int mt8195_irq_affin_unset(struct mtk_apu *apu)
{
	irq_set_affinity_hint(apu->mbox0_irq_number, &normal_cpus);

	return 0;
}

static int mt8195_apu_memmap_init(struct mtk_apu *apu)
{
	struct platform_device *pdev = apu->pdev;
	struct device *dev = apu->dev;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apu_mbox");
	if (res == NULL) {
		dev_info(dev, "%s: apu_mbox get resource fail\n", __func__);
		return -ENODEV;
	}
	apu->apu_mbox = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *)apu->apu_mbox)) {
		dev_info(dev, "%s: apu_mbox remap base fail\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(
		pdev, IORESOURCE_MEM, "md32_sysctrl");
	if (res == NULL) {
		dev_info(dev, "%s: md32_sysctrl get resource fail\n", __func__);
		return -ENODEV;
	}
	apu->md32_sysctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *)apu->md32_sysctrl)) {
		dev_info(dev, "%s: md32_sysctrl remap base fail\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(
		pdev, IORESOURCE_MEM, "md32_debug_apb");
	if (res == NULL) {
		dev_info(dev, "%s: md32_debug_apb get resource fail\n", __func__);
		return -ENODEV;
	}
	apu->md32_debug_apb = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *)apu->md32_debug_apb)) {
		dev_info(dev, "%s: md32_debug_apb remap base fail\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apu_wdt");
	if (res == NULL) {
		dev_info(dev, "%s: apu_wdt get resource fail\n", __func__);
		return -ENODEV;
	}
	apu->apu_wdt = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *)apu->apu_wdt)) {
		dev_info(dev, "%s: apu_wdt remap base fail\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(
		pdev, IORESOURCE_MEM, "apu_sctrl_reviser");
	if (res == NULL) {
		dev_info(dev, "%s: apu_sctrl_reviser get resource fail\n", __func__);
		return -ENODEV;
	}
	apu->apu_sctrl_reviser = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *)apu->apu_sctrl_reviser)) {
		dev_info(dev, "%s: apu_sctrl_reviser remap base fail\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apu_ao_ctl");
	if (res == NULL) {
		dev_info(dev, "%s: apu_ao_ctl get resource fail\n", __func__);
		return -ENODEV;
	}
	apu->apu_ao_ctl = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *)apu->apu_ao_ctl)) {
		dev_info(dev, "%s: apu_ao_ctl remap base fail\n", __func__);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "md32_tcm");
	if (res == NULL) {
		dev_info(dev, "%s: md32_tcm get resource fail\n", __func__);
		return -ENODEV;
	}
	apu->md32_tcm = devm_ioremap_wc(dev, res->start, res->end - res->start + 1);
	if (IS_ERR((void const *)apu->md32_tcm)) {
		dev_info(dev, "%s: md32_tcm remap base fail\n", __func__);
		return -ENOMEM;
	}
	return 0;
}

static void mt8195_apu_memmap_remove(struct mtk_apu *apu)
{
}

static void mt8195_rv_cg_gating(struct mtk_apu *apu)
{
}

static void mt8195_rv_cg_ungating(struct mtk_apu *apu)
{
}

static void mt8195_rv_cachedump(struct mtk_apu *apu)
{
}

const struct mtk_apu_platdata mt8195_platdata = {
	.flags		= F_PRELOAD_FIRMWARE | F_AUTO_BOOT | F_MT8195_PLAT
				| F_SECURE_BOOT |F_SECURE_COREDUMP
				| F_KERNALLOAD_IMAGE,
	.ops		= {
		.init	= mt8195_rproc_init,
		.exit	= mt8195_rproc_exit,
		.start	= mt8195_rproc_start,
		.stop	= mt8195_rproc_stop,
		.resume = mt8195_rproc_resume,
		.apu_memmap_init = mt8195_apu_memmap_init,
		.apu_memmap_remove = mt8195_apu_memmap_remove,
		.cg_gating = mt8195_rv_cg_gating,
		.cg_ungating = mt8195_rv_cg_ungating,
		.rv_cachedump = mt8195_rv_cachedump,
		.power_init = mt8195_apu_power_init,
		.power_on = mt8195_apu_power_on,
		.power_off = mt8195_apu_power_off,
		.irq_affin_init = mt8195_irq_affin_init,
		.irq_affin_set = mt8195_irq_affin_set,
		.irq_affin_unset = mt8195_irq_affin_unset,
	},
};
