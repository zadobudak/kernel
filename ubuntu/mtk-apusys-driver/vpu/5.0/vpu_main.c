// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/remoteproc.h>
#include <linux/printk.h>
#include <linux/uaccess.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#ifndef APUSYS_AIOT
#include <base.h>
#endif

/* internal headers */
#include "vpu_cmn.h"
#include "vpu_debug.h"
#include "vpu_loadimage.h"

/* Test code, should modify */
#include "apu.h"
#include "apu_config.h"


struct vpu_platform_data {
	int core_num;
};

#define VPU_NUM_MAX 3
struct vpu_probe_info probe_info[VPU_NUM_MAX];

int vpu_set_init_info(struct mtk_apu *apu)
{
	int i;
	bool ret;
	struct vpu_probe_info *p;
	struct platform_device *pdev;
	struct device *dev;
	struct vpu_init_info *info;
	struct vpu_device *vd;
	struct vpu_misc *misc;
	#if defined(APUSYS_AIOT)
	struct device_link *sup_link;
	#endif
	if (!apu || !apu->conf_buf) {
		vpu_cmd_debug("invalid argument: apu(%p) conf_buf(%p)\n",
			apu, apu->conf_buf);
		return -EINVAL;
	}

	if (apu->platdata &&  !(apu->platdata->flags & F_REQUIRE_VPU_INIT)) {
		vpu_cmd_debug("no need to prepare vpu init info\n");
		return 0;
	}

	info = (struct vpu_init_info *)
		get_apu_config_user_ptr(apu->conf_buf, eVPU_INIT_INFO);

	for (i = 0; i < vpu_drv->core_num; i++) {
		p = &probe_info[i];

		if (!p->np) {
			vpu_cmd_debug("probe_info %d invalid\n", i);
			return -EPROBE_DEFER;
		}

		if (p->bound)
			continue;

		pdev = of_find_device_by_node(p->np);

		if (!pdev) {
			vpu_cmd_debug("vpu pdev%d failed\n", i);
			return -EINVAL;
		}

		dev = &(pdev->dev);
		#if defined(APUSYS_AIOT)
		sup_link = device_link_add(apu->dev,dev,DL_FLAG_AUTOREMOVE_CONSUMER);
		if(sup_link->supplier->links.status != DL_DEV_DRIVER_BOUND) {
			pr_err(" vpu links status %d\n", sup_link->supplier->links.status);
			return -EPROBE_DEFER;
		}
		#else
		device_lock(dev);
		ret = device_is_bound(dev);
		device_unlock(dev);
		if (ret != true)
			return -EPROBE_DEFER;
		#endif
		vd = (struct vpu_device *)platform_get_drvdata(pdev);
		info->algo_info_ptr[2 * i] = vd->iova_algo_info.m.pa;
		info->algo_info_ptr[2 * i + 1] = vd->iova_preload_info.m.pa;
		info->rst_vec[i]   = vd->iova_reset.addr;
		info->dmem_addr[i] = vd->dmem.res->start;
		info->imem_addr[i] = vd->imem.res->start;
		info->iram_addr[i] = vd->iova_iram.addr;
		info->cmd_addr[i]  = vd->iova_cmd.m.pa;
		info->log_addr[i]  = vd->iova_work.m.pa;
		info->log_size[i]  = vd->iova_work.m.length;
		p->bound = 1;
	}

	info->vpu_num = i;

	if (!info->cfg_addr && info->vpu_num) {
		misc = (void *)vpu_drv->iova_cfg.m.va;
		memset(misc, 0, sizeof(*misc));
		misc->ulog_lv = 0x3;
		misc->pwr_off_delay = VPU_PWR_OFF_LATENCY;
		vpu_iova_sync_for_device(vd->dev, &vpu_drv->iova_cfg);
		info->cfg_addr = vpu_drv->iova_cfg.addr;
		info->cfg_size = vpu_drv->iova_cfg.size;
	}
	return 0;
}
EXPORT_SYMBOL(vpu_set_init_info);

int vpu_init_info_remove(struct mtk_apu *apu)
{
	int i;
	struct vpu_probe_info *p;

	for (i = 0; i < VPU_NUM_MAX; i++) {
		p = &probe_info[i];
		p->bound = 0;
	}
	return 0;
}
EXPORT_SYMBOL(vpu_init_info_remove);

static struct vpu_device *vpu_alloc(struct platform_device *pdev)
{
	struct vpu_device *vd;

	vd = kzalloc(sizeof(struct vpu_device), GFP_KERNEL);
	return vd;
}

static void vpu_free(struct platform_device *pdev)
{
	struct vpu_device *vd = platform_get_drvdata(pdev);

	kfree(vd);
	platform_set_drvdata(pdev, NULL);
}

struct vpu_driver *vpu_drv;

void vpu_drv_release(struct kref *ref)
{
	vpu_mem_exit();
	vpu_drv_debug("%s:\n", __func__);
	kfree(vpu_drv);
	vpu_drv = NULL;
}

void vpu_drv_put(void)
{
	if (!vpu_drv)
		return;

	if (vpu_drv->wq) {
		flush_workqueue(vpu_drv->wq);
		destroy_workqueue(vpu_drv->wq);
		vpu_drv->wq = NULL;
	}

	vpu_drv_debug("%s:\n", __func__);
	kref_put(&vpu_drv->ref, vpu_drv_release);
}

void vpu_drv_get(void)
{
	kref_get(&vpu_drv->ref);
}

static int vpu_init_bin(struct device_node *node)
{
	uint32_t phy_addr;
	uint32_t phy_size;
	uint32_t bin_head_ofs;
	uint32_t bin_preload_ofs;

	/* skip, if vpu firmware had ready been mapped */
	if (vpu_drv && vpu_drv->bin_va)
		return 0;

	if (of_property_read_u32(node, "bin-phy-addr", &phy_addr) ||
		of_property_read_u32(node, "bin-size", &phy_size) ||
		of_property_read_u32(node, "img-head", &bin_head_ofs) ||
		of_property_read_u32(node, "pre-bin", &bin_preload_ofs)) {
		pr_info("%s: unable to get vpu firmware.\n", __func__);
		vpu_drv->vpu_load_image = true;
		return 0;
	}

	/* map vpu firmware to kernel virtual address */
	vpu_drv->bin_va = ioremap_wc(phy_addr, phy_size);
	vpu_drv->bin_pa = phy_addr;
	vpu_drv->bin_size = phy_size;

	vpu_drv->bin_head_ofs = bin_head_ofs;
	vpu_drv->bin_preload_ofs = bin_preload_ofs;

	pr_info("%s: mapped vpu firmware: pa: 0x%lx, size: 0x%x, kva: 0x%lx, header: 0x%x, preload:0x%x\n",
		__func__, vpu_drv->bin_pa, vpu_drv->bin_size,
		(unsigned long)vpu_drv->bin_va,
		vpu_drv->bin_head_ofs, vpu_drv->bin_preload_ofs);

	return 0;
}

static void vpu_shared_release(struct kref *ref)
{
	vpu_drv_debug("%s:\n", __func__);

	if (vpu_drv->mva_algo) {
		vpu_iova_free(vpu_drv->iova_dev, &vpu_drv->iova_algo);
		vpu_drv->mva_algo = 0;
	}

	if (vpu_drv->mva_cfg) {
		vpu_iova_free(vpu_drv->iova_dev, &vpu_drv->iova_cfg);
		vpu_drv->mva_cfg = 0;
	}
}

static int vpu_shared_put(struct platform_device *pdev,
	struct vpu_device *vd)
{
	vpu_drv->iova_dev = &pdev->dev;
	kref_put(&vpu_drv->iova_ref, vpu_shared_release);
	return 0;
}

static int vpu_shared_get(struct platform_device *pdev,
	struct vpu_device *vd)
{
	dma_addr_t iova = 0;

	if (vpu_drv->mva_algo) {
		kref_get(&vpu_drv->iova_ref);
		return 0;
	}

	kref_init(&vpu_drv->iova_ref);

	if (!vpu_drv->mva_algo && vd->id == 0) {

		if (vpu_drv->vpu_load_image == false) {
			if (vpu_iova_dts(pdev, "algo", &vpu_drv->iova_algo))
				goto error;
		}

		if (vpu_drv->iova_algo.size) {
			iova = vpu_iova_alloc(pdev, &vpu_drv->iova_algo);
			if (!iova)
				goto error;
			vpu_drv->mva_algo = iova;
			vpu_drv->iova_algo.addr = iova;
		}
	}

	if (!vpu_drv->mva_cfg) {
		vpu_drv->iova_cfg.bin = VPU_MEM_ALLOC;
		vpu_drv->iova_cfg.size = __ALIGN_KERNEL(sizeof(struct vpu_misc),
			0x1000);
		iova = vpu_iova_alloc(pdev, &vpu_drv->iova_cfg);
		if (!iova)
			goto error;
		vpu_drv->mva_cfg = iova;
		vpu_drv->iova_cfg.addr = iova;
	}

	return 0;

error:
	vpu_shared_put(pdev, vd);
	return -ENOMEM;
}

static int vpu_exit_dev_mem(struct platform_device *pdev,
	struct vpu_device *vd)
{
	vpu_iova_free(&pdev->dev, &vd->iova_reset);
	vpu_iova_free(&pdev->dev, &vd->iova_main);
	vpu_iova_free(&pdev->dev, &vd->iova_kernel);
	vpu_iova_free(&pdev->dev, &vd->iova_work);
	vpu_iova_free(&pdev->dev, &vd->iova_iram);
	vpu_iova_free(&pdev->dev, &vd->iova_cmd);
	vpu_shared_put(pdev, vd);

	return 0;
}

static int vpu_iomem_dts(struct platform_device *pdev,
	const char *name, int i, struct vpu_iomem *m)
{
	if (!m)
		return 0;

	m->res = platform_get_resource(pdev, IORESOURCE_MEM, i);

	if (!m->res) {
		dev_info(&pdev->dev, "unable to get resource: %s\n", name);
		return -ENODEV;
	}

	m->m = devm_ioremap_resource(&pdev->dev, m->res);

	if (!m->m) {
		dev_info(&pdev->dev, "unable to map iomem: %s\n", name);
		return -ENODEV;
	}

	dev_info(&pdev->dev, "mapped %s: 0x%lx: 0x%lx ~ 0x%lx\n", name,
		(unsigned long)m->m,
		(unsigned long)m->res->start,
		(unsigned long)m->res->end);

	return 0;
}

static int vpu_init_dev_mem(struct platform_device *pdev,
	struct vpu_device *vd)
{
	struct resource *res;
	dma_addr_t iova = 0;
	int ret = 0;

	/* registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (vpu_iomem_dts(pdev, "reg", 0, &vd->reg) ||
		vpu_iomem_dts(pdev, "dmem", 1, &vd->dmem) ||
		vpu_iomem_dts(pdev, "imem", 2, &vd->imem) ||
		vpu_iomem_dts(pdev, "dbg", 3, &vd->dbg)) {
		goto error;
	}

	/* iova */
	if (vpu_iova_dts(pdev, "reset-vector", &vd->iova_reset) ||
		vpu_iova_dts(pdev, "main-prog", &vd->iova_main) ||
		vpu_iova_dts(pdev, "kernel-lib", &vd->iova_kernel) ||
		vpu_iova_dts(pdev, "work-buf", &vd->iova_work)) {
		goto error;
	}
	if (vpu_drv->vpu_load_image == false) {
		if (vpu_iova_dts(pdev, "iram-data", &vd->iova_iram))
			goto error;
	}

	if (vd->iova_work.size < (VPU_LOG_OFFSET))
		goto error;

	vd->wb_log_size = vd->iova_work.size;
	vd->wb_log_data = vd->wb_log_size - VPU_LOG_OFFSET;

	ret = vpu_shared_get(pdev, vd);
	if (ret)
		goto error;
	iova = vpu_iova_alloc(pdev, &vd->iova_reset);
	if (!iova)
		goto error;
	iova = vpu_iova_alloc(pdev, &vd->iova_main);
	if (!iova)
		goto error;

	iova = vpu_iova_alloc(pdev, &vd->iova_kernel);
	if (!iova)
		goto error;

	iova = vpu_iova_alloc(pdev, &vd->iova_work);
	if (!iova)
		goto error;

	iova = vpu_iova_alloc(pdev, &vd->iova_iram);
	vd->mva_iram = iova;
	vd->iova_iram.addr = iova;

	return 0;
error:
	return -ENOMEM;
}

static int vpu_init_dev_plat(struct platform_device *pdev,
	struct vpu_device *vd)
{
	int ret = 0;

	mutex_lock(&vpu_drv->lock);
	ret = vpu_mem_init();
	mutex_unlock(&vpu_drv->lock);
	return ret;
}

static int vpu_probe(struct platform_device *pdev)
{
	struct vpu_device *vd;
	int ret;

	vd = vpu_alloc(pdev);
	if (!vd)
		return -ENOMEM;

	vd->dev = &pdev->dev;
	platform_set_drvdata(pdev, vd);


	if (of_property_read_u32(pdev->dev.of_node, "id", &vd->id)) {
		dev_info(&pdev->dev, "unable to get core id from dts\n");
		ret = -ENODEV;
		goto out;
	}

	snprintf(vd->name, sizeof(vd->name), "vpu%d", vd->id);

	if (list_empty(&vpu_drv->devs) && vd->id != 0) {
		ret = -EPROBE_DEFER;
		if (ret) {
			pr_info("%s: wait for vpu_core0 %d\n", __func__, ret);
			goto out;
		}
	}

	if (vd->id >= 0 && vd->id < VPU_NUM_MAX)
		probe_info[vd->id].np = pdev->dev.of_node;

	if (vd->id == 0) {
		struct vpu_platform_data *data;

		data = (struct vpu_platform_data *)of_device_get_match_data(vd->dev);
		if (!data) {
			pr_info("%s: of_device_get_match_data fail\n", __func__);
			ret = -EINVAL;
			goto out;
		}
		vpu_drv->core_num = data->core_num;

		ret = vpu_init_bin(pdev->dev.of_node);
		if (ret) {
			pr_info("%s: init vpu_core0 %d\n", __func__, ret);
			goto out;
		}
		vpu_init_debug();
	}

	vpu_cmd_debug("check core_num %d\n", vpu_drv->core_num);

	if (vpu_drv->vpu_load_image) {
		ret = vpu_lk(vd, pdev);
		if (ret) {
			pr_info("%s: probe_error: vpu_load_image with error %d\n", __func__, ret);
			goto out;
		}
	}

	ret = vpu_init_dev_plat(pdev, vd);
	if (ret)
		goto out;

	/* put efuse judgement at beginning */
	if (vpu_is_disabled(vd)) {
		ret = -ENODEV;
		vd->state = VS_DISALBED;
		goto out;
	} else {
		vd->state = VS_DOWN;
	}

	/* allocate resources */
	ret = vpu_init_dev_mem(pdev, vd);
	if (ret)
		goto free;

	/* cmd buffer initialization */
	ret = vpu_alloc_cmd(pdev, vd);
	if (ret)
		goto free;

	/* device algo initialization */
	ret = vpu_init_dev_algo(pdev, vd);
	if (ret)
		goto free;

	/* register debugfs nodes */
	ret = vpu_init_dev_debug(pdev, vd);
	if (ret)
		goto free;

	/* add to vd list */
	mutex_lock(&vpu_drv->lock);
	vpu_drv_get();
	list_add_tail(&vd->list, &vpu_drv->devs);
	mutex_unlock(&vpu_drv->lock);

	dev_info(&pdev->dev, "%s: succeed\n", __func__);
	return 0;

	// TODO: add error handling free algo
free:
	vpu_exit_dev_mem(pdev, vd);
out:
	vpu_free(pdev);
	dev_info(&pdev->dev, "%s: failed\n", __func__);
	return ret;
}

static int vpu_remove(struct platform_device *pdev)
{
	struct vpu_device *vd = platform_get_drvdata(pdev);

	vpu_exit_dev_debug(pdev, vd);
	vpu_exit_dev_algo(pdev, vd);
	vpu_exit_dev_mem(pdev, vd);
	vpu_free(pdev);
	vpu_drv_put();

	return 0;
}

struct vpu_platform_data vpu_plat_mt8188 = {
	.core_num = 1,
};

struct vpu_platform_data vpu_plat_mt8195 = {
	.core_num = 2,
};

static const struct of_device_id vpu_of_ids[] = {
	{.compatible = "mediatek,mt8188-vpu_core0", .data = &vpu_plat_mt8188},
	{.compatible = "mediatek,mt8195-vpu_core", .data = &vpu_plat_mt8195},
	{},
};

static struct platform_driver vpu_plat_drv = {
	.probe   = vpu_probe,
	.remove  = vpu_remove,
	.driver  = {
	.name = "vpu",
	.owner = THIS_MODULE,
	.of_match_table = vpu_of_ids,
	}
};

int vpu_init(void)
{
	int ret;

	vpu_drv = kzalloc(sizeof(struct vpu_driver), GFP_KERNEL);

	if (!vpu_drv)
		return -ENOMEM;

	kref_init(&vpu_drv->ref);

	INIT_LIST_HEAD(&vpu_drv->devs);
	mutex_init(&vpu_drv->lock);

	vpu_drv->mva_algo = 0;
	vpu_drv->mva_cfg = 0;
	vpu_drv->wq = create_workqueue("vpu_wq");

	ret = platform_driver_register(&vpu_plat_drv);

	return ret;
}

void vpu_exit(void)
{
	struct vpu_device *vd;
	struct list_head *ptr, *tmp;

	/* notify all devices that we are going to be removed
	 *  wait and stop all on-going requests
	 **/
	if (vpu_drv) {
		mutex_lock(&vpu_drv->lock);
		list_for_each_safe(ptr, tmp, &vpu_drv->devs) {
			vd = list_entry(ptr, struct vpu_device, list);
			list_del(ptr);
			vd->state = VS_REMOVING;
		}
		mutex_unlock(&vpu_drv->lock);
	}

	vpu_exit_debug();

	if (vpu_drv) {
		vpu_drv_debug("%s: iounmap\n", __func__);
		if (vpu_drv->bin_va) {
			iounmap(vpu_drv->bin_va);
			vpu_drv->bin_va = NULL;
		}

		vpu_drv_put();
	}

	vpu_drv_debug("%s: platform_driver_unregister\n", __func__);
	platform_driver_unregister(&vpu_plat_drv);
}

#ifdef INIT_VPU_BY_SELF
#ifdef BUILD_MODULE
static int vpu_mod_init(void)
{
	return vpu_init(NULL);
	/*return vpu_init();*/
}
static void vpu_mod_exit(void)
{
	vpu_exit();
}
module_init(vpu_mod_init);
module_exit(vpu_mod_exit);
MODULE_DESCRIPTION("Mediatek VPU Driver");
MODULE_LICENSE("GPL");
#else
static int vpu_mod_init(void)
{
	return vpu_init(NULL);
	/*return vpu_init();*/
}
late_initcall(vpu_mod_init);
#endif
#endif

