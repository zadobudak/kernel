// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "apusys_rv.h"

/*add in kernel*/
#include <linux/firmware.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/libfdt_env.h>
#include <linux/libfdt.h>
#include <linux/slab.h>

#define LOCAL_TRACE 0

static int apusys_rv_load_fw(struct apusys_secure_info_t *sec_info, const char *part_name,
	off_t sec_mem_size, void *sec_mem_addr_va, struct mtk_apu *apu)
{
	unsigned int apusys_pmsize, apusys_xsize;
	void *apusys_pmimg, *apusys_ximg;
	void *tmp_addr;
	void *img;
	struct ptimg_hdr_t *hdr;
	unsigned int *img_size;

	pr_info("%s: start\n", __func__);
	/* initialize apusys sec_info */
	tmp_addr = sec_info->up_code_buf_ofs + sec_info->up_code_buf_sz +
		(void *)sec_mem_addr_va + ROUNDUP(sizeof(*sec_info), APUSYS_FW_ALIGN);

	/* separate ptimg */
	apusys_pmimg = apusys_ximg = NULL;
	apusys_pmsize = apusys_xsize = 0;

	hdr = (void *)apu->fw->data + (0x200);
	pr_info("%s: hdr->magic is 0x%x\n", __func__, hdr->magic);
	img_size = (void *)apu->fw->data + (0x4);
	pr_info("%s: apu_partition_sz is 0x%x\n", __func__, *img_size);
	memcpy(tmp_addr, ((void *)apu->fw->data + (0x200)), (*img_size));

	hdr = tmp_addr;

	while (hdr->magic == PT_MAGIC) {
		img = ((void *) hdr) + hdr->hdr_size;
		pr_info("Rhdr->hdr_size= 0x%x\n", hdr->hdr_size);
		pr_info("img address is 0x%llx\n", (u64)img);

		switch (hdr->id) {
		case PT_ID_APUSYS_FW:
			pr_debug("PT_ID_APUSYS_FW\n");
			apusys_pmimg = img;
			apusys_pmsize = hdr->img_size;
			sec_info->up_fw_ofs = ((u64) img - (u64) sec_mem_addr_va);
			sec_info->up_fw_sz = apusys_pmsize;
			pr_debug("up_fw_ofs = 0x%x, up_fw_sz = 0x%x\n",
				sec_info->up_fw_ofs, sec_info->up_fw_sz);
			break;
		case PT_ID_APUSYS_XFILE:
			pr_debug("PT_ID_APUSYS_XFILE\n");
			apusys_ximg = img;
			apusys_xsize = hdr->img_size;
			sec_info->up_xfile_ofs = (u64) hdr - (u64) sec_mem_addr_va;
			sec_info->up_xfile_sz = hdr->hdr_size + apusys_xsize;
			pr_debug("up_xfile_ofs = 0x%x, up_xfile_sz = 0x%x\n",
				sec_info->up_xfile_ofs, sec_info->up_xfile_sz);
			break;
		case PT_ID_MDLA_FW_BOOT:
			pr_debug("PT_ID_MDLA_FW_BOOT\n");
			sec_info->mdla_fw_boot_ofs = (u64) img - (u64) sec_mem_addr_va;
			sec_info->mdla_fw_boot_sz = hdr->img_size;
			pr_debug("mdla_fw_boot_ofs = 0x%x, mdla_fw_boot_sz = 0x%x\n",
				sec_info->mdla_fw_boot_ofs, sec_info->mdla_fw_boot_sz);
			break;
		case PT_ID_MDLA_FW_MAIN:
			pr_debug("PT_ID_MDLA_FW_MAIN\n");
			sec_info->mdla_fw_main_ofs = (u64) img - (u64) sec_mem_addr_va;
			sec_info->mdla_fw_main_sz = hdr->img_size;
			pr_debug("mdla_fw_main_ofs = 0x%x, mdla_fw_main_sz = 0x%x\n",
				sec_info->mdla_fw_main_ofs, sec_info->mdla_fw_main_sz);
			break;
		case PT_ID_MDLA_XFILE:
			pr_debug("PT_ID_MDLA_XFILE\n");
			sec_info->mdla_xfile_ofs = (u64) hdr - (u64) sec_mem_addr_va;
			sec_info->mdla_xfile_sz = hdr->hdr_size + hdr->img_size;
			pr_debug("mdla_xfile_ofs = 0x%x, mdla_xfile_sz = 0x%x\n",
				sec_info->mdla_xfile_ofs, sec_info->mdla_xfile_sz);
			break;
		case PT_ID_MVPU_FW:
			pr_debug("PT_ID_MVPU_FW\n");
			sec_info->mvpu_fw_ofs = (u64) img - (u64) sec_mem_addr_va;
			sec_info->mvpu_fw_sz = hdr->img_size;
			pr_debug("mvpu_fw_ofs = 0x%x, mvpu_fw_sz = 0x%x\n",
				sec_info->mvpu_fw_ofs, sec_info->mvpu_fw_sz);
			break;
		case PT_ID_MVPU_XFILE:
			pr_debug("PT_ID_MVPU_XFILE\n");
			sec_info->mvpu_xfile_ofs = (u64) hdr - (u64) sec_mem_addr_va;
			sec_info->mvpu_xfile_sz = hdr->hdr_size + hdr->img_size;
			pr_debug("mvpu_xfile_ofs = 0x%x, mvpu_xfile_sz = 0x%x\n",
				sec_info->mvpu_xfile_ofs, sec_info->mvpu_xfile_sz);
			break;
		case PT_ID_MVPU_SEC_FW:
			pr_debug("PT_ID_MVPU_SEC_FW\n");
			sec_info->mvpu_sec_fw_ofs = (u64) img - (u64) sec_mem_addr_va;
			sec_info->mvpu_sec_fw_sz = hdr->img_size;
			pr_debug("mvpu_sec_fw_ofs = 0x%x, mvpu_sec_fw_sz = 0x%x\n",
				sec_info->mvpu_sec_fw_ofs, sec_info->mvpu_sec_fw_sz);
			break;
		case PT_ID_MVPU_SEC_XFILE:
			pr_debug("PT_ID_MVPU_SEC_XFILE\n");
			sec_info->mvpu_sec_xfile_ofs = (u64) hdr - (u64) sec_mem_addr_va;
			sec_info->mvpu_sec_xfile_sz = hdr->hdr_size + hdr->img_size;
			pr_debug("mvpu_sec_xfile_ofs = 0x%x, mvpu_sec_xfile_sz = 0x%x\n",
				sec_info->mvpu_sec_xfile_ofs, sec_info->mvpu_sec_xfile_sz);
			break;
		default:
			pr_info("Warning: Ignore unknown APUSYS image_%d\n", hdr->id);
			break;
		}

		pr_info("hdr->img_size = 0x%x, ROUNDUP(hdr->img_size, hdr->align) = 0x%x\n",
			hdr->img_size, ROUNDUP(hdr->img_size, hdr->align));
		img += ROUNDUP(hdr->img_size, hdr->align);
		hdr = (struct ptimg_hdr_t *) img;
	}

	if (!apusys_pmimg || !apusys_ximg) {
		pr_info("APUSYS partition missing - PM:0x%llx, XM:0x%llx (@0x%llx)\n",
			(u64)apusys_pmimg, (u64)apusys_ximg, (u64)tmp_addr);
		return -ENOENT;
	}

	pr_debug("%s APUSYS part load finished: PM:0x%llx(up_fw_ofs = 0x%x), XM:0x%llx(up_xfile_ofs = 0x%x)\n",
		__func__, (u64)apusys_pmimg, sec_info->up_fw_ofs,
		(u64)apusys_ximg, sec_info->up_xfile_ofs);

	return 0;
}

static int apusys_rv_initialize_aee_coredump_buf(
	struct apusys_aee_coredump_info_t *aee_coredump_info,
	struct apusys_secure_info_t *sec_info, void *aee_coredump_mem_addr_va,
	void *sec_mem_addr_va)
{
	aee_coredump_info = (struct apusys_aee_coredump_info_t *)aee_coredump_mem_addr_va;

	aee_coredump_info->up_coredump_ofs = sizeof(*aee_coredump_info);
	aee_coredump_info->up_coredump_sz = sec_info->up_coredump_sz;
	aee_coredump_info->mdla_coredump_ofs = aee_coredump_info->up_coredump_ofs +
		aee_coredump_info->up_coredump_sz;
	aee_coredump_info->mdla_coredump_sz = sec_info->mdla_coredump_sz;
	aee_coredump_info->mvpu_coredump_ofs = aee_coredump_info->mdla_coredump_ofs +
		aee_coredump_info->mdla_coredump_sz;
	aee_coredump_info->mvpu_coredump_sz = sec_info->mvpu_coredump_sz;
	aee_coredump_info->mvpu_sec_coredump_ofs = aee_coredump_info->mvpu_coredump_ofs +
		aee_coredump_info->mvpu_coredump_sz;
	aee_coredump_info->mvpu_sec_coredump_sz = sec_info->mvpu_sec_coredump_sz;

	aee_coredump_info->up_xfile_ofs = aee_coredump_info->mvpu_sec_coredump_ofs +
		aee_coredump_info->mvpu_sec_coredump_sz;
	aee_coredump_info->up_xfile_sz = sec_info->up_xfile_sz;
	aee_coredump_info->mdla_xfile_ofs = aee_coredump_info->up_xfile_ofs +
		aee_coredump_info->up_xfile_sz;
	aee_coredump_info->mdla_xfile_sz = sec_info->mdla_xfile_sz;
	aee_coredump_info->mvpu_xfile_ofs = aee_coredump_info->mdla_xfile_ofs +
		aee_coredump_info->mdla_xfile_sz;
	aee_coredump_info->mvpu_xfile_sz = sec_info->mvpu_xfile_sz;
	aee_coredump_info->mvpu_sec_xfile_ofs = aee_coredump_info->mvpu_xfile_ofs +
		aee_coredump_info->mvpu_xfile_sz;
	aee_coredump_info->mvpu_sec_xfile_sz = sec_info->mvpu_sec_xfile_sz;

	/* copy uP xfile to aee_coredump buffer */
	memcpy((void *) (aee_coredump_mem_addr_va + aee_coredump_info->up_xfile_ofs),
		(void *) (sec_mem_addr_va + sec_info->up_xfile_ofs), sec_info->up_xfile_sz);

	/* copy mdla xfile to aee_coredump buffer */
	memcpy((void *) (aee_coredump_mem_addr_va + aee_coredump_info->mdla_xfile_ofs),
		(void *) (sec_mem_addr_va + sec_info->mdla_xfile_ofs), sec_info->mdla_xfile_sz);

	/* copy mvpu xfile to aee_coredump buffer */
	memcpy((void *) (aee_coredump_mem_addr_va + aee_coredump_info->mvpu_xfile_ofs),
		(void *) (sec_mem_addr_va + sec_info->mvpu_xfile_ofs), sec_info->mvpu_xfile_sz);

	/* copy mvpu_sec xfile to aee_coredump buffer */
	memcpy((void *) (aee_coredump_mem_addr_va + aee_coredump_info->mvpu_sec_xfile_ofs),
		(void *) (sec_mem_addr_va + sec_info->mvpu_sec_xfile_ofs),
		sec_info->mvpu_sec_xfile_sz);

	return 0;
}

static int apusys_load_image(struct apusys_secure_info_t *sec_info, const char *part_name,
	struct mtk_apu *apu)
{
	unsigned int *img_size;
	int ret = 0;

	ret = request_firmware(&(apu->fw), "apusys.img", apu->dev);
	if (ret < 0) {
		pr_info("%s: apu_fw fail: %d\n", __func__, ret);
		return -EPROBE_DEFER;
	}
	pr_info("%s: apu_fw request_firmware %s success\n", __func__, part_name);

	img_size = (void *)apu->fw->data + (0x4);
	sec_info->total_sz = *img_size;

	return ret;
}

int platform_load_apusys_rv(struct mtk_apu *apu, struct platform_device *pdev)
{
	int ret = 0;
	u64 sec_mem_addr_pa;//aee_coredump_mem_addr_pa
	void *sec_mem_addr_va, *aee_coredump_mem_addr_va;
	static const char part_name[15] = "apusys.img";
	struct apusys_secure_info_t *sec_info = NULL;
	struct apusys_aee_coredump_info_t *aee_coredump_info = NULL;
	unsigned int coredump_buf_sz = 0, xfile_buf_sz = 0;
	off_t sec_mem_size = 0, aee_coredump_mem_size = 0;
	off_t apusys_part_size = 0;
	int out_value;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *data_np;
	int rc;
	struct resource r;

	pr_info("%s: enter\n", __func__);

	sec_info = kzalloc(sizeof(struct apusys_secure_info_t), GFP_KERNEL);

	if (!sec_info)
		return -ENOMEM;
	memset((void *) sec_info, 0x0, sizeof(struct apusys_secure_info_t));
	pr_info("%s: kzalloc sec_info success\n", __func__);

	if (1) {
		data_np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
		if (!data_np) {
			pr_info("No %s specified\n", "apu-reserve-memory");
			goto err_apusys_rv_get_buf_sz;
			}
		rc = of_address_to_resource(data_np, 0, &r);
		if (rc) {
			pr_info("No memory address assigned to the region with error %d\n", rc);
			goto err_apusys_rv_get_buf_sz;
		}
		sec_mem_addr_pa = r.start;
		sec_mem_addr_va = memremap(r.start, resource_size(&r), MEMREMAP_WB);
		sec_mem_size = resource_size(&r);
		pr_info("Allocated reserved memory, vaddr: 0x%llx, paddr: 0x%llx, size:0x%lx\n",
			(u64)sec_mem_addr_va, sec_mem_addr_pa, sec_mem_size);
	}

	memset(sec_mem_addr_va, 0, sec_mem_size);

	ret = apusys_load_image(sec_info, part_name, apu);
	if (ret != 0) {
		pr_info("%s: request_firmware fail(%d)\n", __func__, ret);
		goto err_load_partition;
	}

	//apusys_part_size = get_apusys_rv_part_size(part_name);
	apusys_part_size = sec_info->total_sz; //load from apu.image
	if (apusys_part_size < 0)
		goto err_apusys_rv_get_buf_sz;
	pr_info("%s: apusys_part_size is 0x%lx\n", __func__, apusys_part_size);

	/* fill in dts property sec_info */
	sec_info->up_code_buf_ofs = 0;
	ret = of_property_read_u32(np, "up_code_buf_sz", &out_value);
	if (ret < 0) {
		pr_info("%s: up_code_buf_sz not found\n", __func__);
		goto err_apusys_rv_get_buf_sz;
	}
	sec_info->up_code_buf_sz = out_value;
	pr_info("%s: up_code_buf_sz is 0x%x\n", __func__, out_value);

	sec_info->up_coredump_ofs = sec_info->up_code_buf_ofs + sec_info->up_code_buf_sz
		+ apusys_part_size;

	ret = of_property_read_u32(np, "up_coredump_buf_sz", &out_value);
	if (ret < 0)
		goto err_apusys_rv_get_buf_sz;
	sec_info->up_coredump_sz = out_value;
	pr_info("%s: up_coredump_ofs is 0x%x, up_coredump_sz is 0x%x\n",
		__func__, sec_info->up_coredump_ofs, out_value);

	sec_info->mdla_coredump_ofs = sec_info->up_coredump_ofs + sec_info->up_coredump_sz;
	ret = of_property_read_u32(np, "mdla_coredump_buf_sz", &out_value);
	if (ret < 0)
		goto err_apusys_rv_get_buf_sz;
	sec_info->mdla_coredump_sz = out_value;
	pr_info("%s: mdla_coredump_ofs is 0x%x, mdla_coredump_sz is 0x%x\n",
		__func__, sec_info->mdla_coredump_ofs, out_value);

	sec_info->mvpu_coredump_ofs = sec_info->mdla_coredump_ofs + sec_info->mdla_coredump_sz;
	ret = of_property_read_u32(np, "mvpu_coredump_buf_sz", &out_value);
	if (ret < 0)
		goto err_apusys_rv_get_buf_sz;
	sec_info->mvpu_coredump_sz = out_value;
	pr_info("%s: mvpu_coredump_ofs is 0x%x, mvpu_coredump_sz is 0x%x\n",
		__func__, sec_info->mvpu_coredump_ofs, out_value);

	sec_info->mvpu_sec_coredump_ofs = sec_info->mvpu_coredump_ofs + sec_info->mvpu_coredump_sz;
	ret = of_property_read_u32(np, "mvpu_sec_coredump_buf_sz", &out_value);
	if (ret < 0)
		goto err_apusys_rv_get_buf_sz;
	sec_info->mvpu_sec_coredump_sz = out_value;
	pr_info("%s: mvpu_sec_coredump_ofs is 0x%x, mvpu_sec_coredump_sz is 0x%x\n",
		__func__, sec_info->mvpu_sec_coredump_ofs, out_value);

	coredump_buf_sz = sec_info->up_coredump_sz + sec_info->mdla_coredump_sz +
		sec_info->mvpu_coredump_sz + sec_info->mvpu_sec_coredump_sz;

	sec_mem_size = ROUNDUP(apusys_part_size + sec_info->up_code_buf_sz + coredump_buf_sz,
		APUSYS_MEM_ALIGN);
	sec_info->total_sz = sec_mem_size;

	apu->apusys_sec_mem_start = (unsigned int)sec_mem_addr_pa;
	apu->apusys_sec_mem_size = (unsigned int)sec_mem_size;
	apu->apu_sec_mem_base = sec_mem_addr_va;

	pr_info("%s: apu_apusys-rv_secure: start = 0x%x, size = 0x%x\n",
		__func__, apu->apusys_sec_mem_start, apu->apusys_sec_mem_size);

	/* load apusys FWs to rv_secure mem */
	ret = apusys_rv_load_fw(sec_info, part_name, sec_mem_size, sec_mem_addr_va, apu);
	if (ret != 0) {
		pr_info("%s: apusys_rv_load_fw fail(%d)\n", __func__, ret);
		goto err_load_partition;
	}

	/* copy uP firmware to code buffer */
	memcpy((void *) (sec_mem_addr_va + sec_info->up_code_buf_ofs),
		(void *) (sec_mem_addr_va + sec_info->up_fw_ofs), sec_info->up_fw_sz);

	/* initialize apusys sec_info */
	memcpy((void *)(sec_info->up_code_buf_ofs + sec_info->up_code_buf_sz + sec_mem_addr_va),
		(void *)sec_info, sizeof(*sec_info));

	/* flush to DRAM */
	//arch_clean_invalidate_cache_range((void *)sec_mem_addr_va, sec_mem_size);

	/* TODO: add smc call for MTK_APUSYS_SMC_OP_APUSYS_RV_SETUP_SECURE_MEM */

	xfile_buf_sz = sec_info->up_xfile_sz + sec_info->mdla_xfile_sz +
		sec_info->mvpu_xfile_sz + sec_info->mvpu_sec_xfile_sz;
	pr_info("%s: xfile_buf_sz = 0x%x, sizeof(*aee_coredump_info) = 0x%x, coredump_buf_sz = 0x%x\n",
		__func__, xfile_buf_sz, (u32)sizeof(*aee_coredump_info), coredump_buf_sz);
	aee_coredump_mem_size = ROUNDUP(sizeof(*aee_coredump_info) + coredump_buf_sz + xfile_buf_sz,
		APUSYS_MEM_ALIGN);

	aee_coredump_mem_addr_va = sec_mem_addr_va + sec_mem_size;

	memset(aee_coredump_mem_addr_va, 0, aee_coredump_mem_size);

	apu->apusys_aee_coredump_mem_start = (unsigned int)sec_mem_addr_pa + sec_mem_size;
	apu->apusys_aee_coredump_mem_size = (unsigned int)aee_coredump_mem_size;
	apu->apu_aee_coredump_mem_base = aee_coredump_mem_addr_va;

	ret = apusys_rv_initialize_aee_coredump_buf(aee_coredump_info, sec_info,
		aee_coredump_mem_addr_va, sec_mem_addr_va);
	if (ret != 0) {
		pr_info("%s: apusys_rv_initialize_aee_coredump_buf fail(%d)\n", __func__, ret);
		goto err_apusys_rv_initialize_aee_coredump_buf;
	}

	/* flush to DRAM */
	//arch_clean_invalidate_cache_range((void *)aee_coredump_mem_addr_va,
	//	aee_coredump_mem_size);

	/* TODO: add smc call for MTK_APUSYS_SMC_OP_APUSYS_RV_SETUP_AEE_COREDUMP_MEM */

	kfree(sec_info);

	return ret;

err_apusys_rv_initialize_aee_coredump_buf:

err_load_partition:
	release_firmware(apu->fw);

err_apusys_rv_get_buf_sz:
	kfree(sec_info);

	return ret;
}

