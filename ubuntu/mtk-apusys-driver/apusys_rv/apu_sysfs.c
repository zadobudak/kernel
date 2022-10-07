// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#ifdef APU_AEE_ENABLE
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include <mt-plat/mrdump.h>
#endif
#endif

#include "apu.h"
#include "apu_excep.h"
#include "apu_regdump.h"

#define PT_MAGIC (0x58901690)

#if APU_AEE_ENABLE
static size_t coredump_len, xfile_len;
static void *coredump_base, *xfile_base;
#endif

static ssize_t rv33_coredump(struct file *filep,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t offset, size_t size)
{
	unsigned int length = 0;
	struct platform_device *pdev;
	struct device *dev;
	struct mtk_apu *apu;
	unsigned int coredump_length;

	dev = container_of(kobj, struct device, kobj);
	pdev = container_of(dev, struct platform_device, dev);
	apu = platform_get_drvdata(pdev);
	coredump_length = sizeof(struct apu_coredump);

	if (offset >= 0 && offset < coredump_length) {
		if ((offset + size) > coredump_length)
			size = coredump_length - offset;

		if (apu->platdata->flags & F_SECURE_COREDUMP)
			memcpy(buf, apu->apu_aee_coredump_mem_base +
				apu->apusys_aee_coredump_info->up_coredump_ofs + offset,
				size);
		else
			/* XXX invalid cache here? */
			memcpy(buf, apu->coredump_buf + offset, size);
		length = size;
	}

	return length;
}

static ssize_t rv33_xfile(struct file *filep,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t offset, size_t size)
{
	unsigned int length = 0;
	struct platform_device *pdev;
	struct device *dev;
	struct mtk_apu *apu;
	unsigned int xfile_length;

	dev = container_of(kobj, struct device, kobj);
	pdev = container_of(dev, struct platform_device, dev);
	apu = platform_get_drvdata(pdev);
	if (apu->platdata->flags & F_PRELOAD_FIRMWARE) {
		WARN_ON(!apu->apusys_aee_coredump_info->up_xfile_sz);
		xfile_length = apu->apusys_aee_coredump_info->up_xfile_sz;
	} else {
		xfile_length = 0;
	}

	if (offset >= 0 && offset < xfile_length) {
		if ((offset + size) > xfile_length)
			size = xfile_length - offset;

		memcpy(buf,
			apu->apu_aee_coredump_mem_base +
			apu->apusys_aee_coredump_info->up_xfile_ofs + offset,
			size);
		length = size;
	}

	return length;
}

static ssize_t rv33_regdump(struct file *filep,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t offset, size_t size)
{
	unsigned int length = 0;
	struct platform_device *pdev;
	struct device *dev;
	struct mtk_apu *apu;
	const struct apusys_regdump_info *info = NULL;
	const struct apusys_regdump_region_info *region_info = NULL;
	void *base_va = NULL;
	uint32_t region_info_num = 0;
	unsigned int region_offset = 0;
	unsigned int regdump_length;
	int i;

	dev = container_of(kobj, struct device, kobj);
	pdev = container_of(dev, struct platform_device, dev);
	apu = platform_get_drvdata(pdev);
	//configs = &apu->platdata->configs;

	//info = configs->apu_regdump.region_info;
	base_va = apu->apu_aee_coredump_mem_base +
		apu->apusys_aee_coredump_info->regdump_ofs;
	info = (struct apusys_regdump_info *) base_va;
	if (info == NULL) {
		dev_info(dev, "%s: apusys_regdump_info == NULL\n",
			__func__);
		return length;
	}

	//region_num = configs->apu_regdump.region_num;
	//base_va = apu->apu_aee_coredump_mem_base +
	//		apu->apusys_aee_coredump_info->regdump_ofs;

	region_info = info->region_info;
	region_info_num = info->region_info_num;

	for (i = 0; i < region_info_num; i++) {
		regdump_length = region_info[i].size;
		if (offset >= 0 && offset < regdump_length) {
			if ((offset + size) > regdump_length)
				size = regdump_length - offset;

			memcpy(buf, base_va + region_offset + offset, size);
			region_offset += regdump_length;
			length += size;
		}
	}

	return length;
}

struct bin_attribute bin_attr_core_dump = {
	.attr = {
		.name = "coredump.bin",
		.mode = 0444,
	},
	.size = 0,
	.read = rv33_coredump,
};

struct bin_attribute bin_attr_xfile = {
	.attr = {
		.name = "SYS_APUSYS_RV_XFILE",
		.mode = 0444,
	},
	.size = 0,
	.read = rv33_xfile,
};

struct bin_attribute bin_attr_regdump = {
	.attr = {
		.name = "apusys_regdump",
		.mode = 0444,
	},
	.size = 0,
	.read = rv33_regdump,
};

static void apu_mrdump_register(struct mtk_apu *apu)
{
#if APU_AEE_ENABLE
	struct device *dev = apu->dev;
	int ret = 0;
	const struct apusys_regdump_info *info = NULL;
	unsigned long base_va = 0;
	unsigned long base_pa = 0;
	unsigned long size = 0;

	if (apu->platdata->flags & F_SECURE_COREDUMP) {
		base_pa = apu->apusys_aee_coredump_mem_start +
			apu->apusys_aee_coredump_info->up_coredump_ofs;
		base_va = (unsigned long) apu->apu_aee_coredump_mem_base +
			apu->apusys_aee_coredump_info->up_coredump_ofs;
		size = sizeof(struct apu_coredump);
	} else {
		base_pa = __pa_nodebug(apu->coredump_buf);
		base_va = (unsigned long) apu->coredump_buf;
		size = sizeof(struct apu_coredump);
	}
	ret = mrdump_mini_add_extra_file(base_va, base_pa, size,
		"APUSYS_RV_COREDUMP");
	if (ret)
		dev_info(dev, "%s: APUSYS_RV_COREDUMP add fail(%d)\n",
			__func__, ret);
	coredump_len = (size_t) size;
	coredump_base = (void *) base_va;

	if (apu->platdata->flags & F_PRELOAD_FIRMWARE) {
		base_pa = apu->apusys_aee_coredump_mem_start +
			apu->apusys_aee_coredump_info->up_xfile_ofs;
		base_va = (unsigned long) apu->apu_aee_coredump_mem_base +
			apu->apusys_aee_coredump_info->up_xfile_ofs;
		if (ioread32((void *) base_va) != PT_MAGIC) {
			dev_info(dev, "%s: reserve memory corrupted!\n", __func__);
			size = 0;
		} else {
			size = apu->apusys_aee_coredump_info->up_xfile_sz;
			dev_info(dev, "%s: up_xfile_sz = 0x%x\n", __func__, size);
		}

		ret = mrdump_mini_add_extra_file(base_va, base_pa, size,
			"APUSYS_RV_XFILE");
		if (ret)
			dev_info(dev, "%s: APUSYS_RV_XFILE add fail(%d)\n",
				__func__, ret);
	}
	xfile_len = (size_t) size;
	xfile_base = (void *) base_va;

	base_pa = apu->apusys_aee_coredump_mem_start +
		apu->apusys_aee_coredump_info->regdump_ofs;
	base_va = (unsigned long) apu->apu_aee_coredump_mem_base +
		apu->apusys_aee_coredump_info->regdump_ofs;
	info = (struct apusys_regdump_info *) base_va;
	size = apu->apusys_aee_coredump_info->regdump_sz;

	if (info != NULL) {
		ret = mrdump_mini_add_extra_file(base_va, base_pa, size,
			"APUSYS_REGDUMP");
		if (ret)
			dev_info(dev, "%s: APUSYS_REGDUMP add fail(%d)\n",
				__func__, ret);
	}
#endif
}

int apu_sysfs_init(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct mtk_apu *apu = (struct mtk_apu *) platform_get_drvdata(pdev);

	ret = sysfs_create_bin_file(&dev->kobj, &bin_attr_core_dump);
	if (ret < 0) {
		dev_info(dev, "%s: sysfs create fail for core_dump(%d)\n",
			__func__, ret);
		goto end;
	}

	ret = sysfs_create_bin_file(&dev->kobj, &bin_attr_xfile);
	if (ret < 0) {
		dev_info(dev, "%s: sysfs create fail for xfile(%d)\n",
			__func__, ret);
		goto end;
	}

	ret = sysfs_create_bin_file(&dev->kobj, &bin_attr_regdump);
	if (ret < 0) {
		dev_info(dev, "%s: sysfs create fail for apusys_regdump(%d)\n",
			__func__, ret);
		goto end;
	}
	apu_regdump_init(pdev);

	apu_mrdump_register(apu);
end:
	return ret;
}

void apu_sysfs_remove(struct platform_device *pdev)
{
	sysfs_remove_bin_file(&pdev->dev.kobj, &bin_attr_regdump);
	sysfs_remove_bin_file(&pdev->dev.kobj, &bin_attr_xfile);
	sysfs_remove_bin_file(&pdev->dev.kobj, &bin_attr_core_dump);
}
