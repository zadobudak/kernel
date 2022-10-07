// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/list_sort.h>
#include "vpu_hw.h"
#include "vpu_debug.h"

#define PRELOAD_IRAM 0xFFFFFFFF

static void *bin_header(int index)
{
	int i;
	uint64_t ptr = (unsigned long)vpu_drv->bin_va;
	struct vpu_image_header *header;

	ptr += vpu_drv->bin_head_ofs;
	header = (void *)ptr;

	for (i = 0; i < index; i++) {
		ptr += header->header_size;
		header = (void *)ptr;
	}

	return (void *)header;
}

static struct vpu_algo_info *bin_algo_info(struct vpu_image_header *h, int j)
{
	struct vpu_algo_info *algo_info;

	algo_info = (void *)((unsigned long)h + h->alg_info);
	return &algo_info[j];
}

static int preload_iova_check(struct vpu_iova *i)
{
	unsigned int bin_end = vpu_drv->bin_pa + vpu_drv->bin_size;

#define is_align_4k(x)   ((x & 0xFFF))
#define is_align_64k(x)  ((x & 0xFFFF))
	if (is_align_64k(i->size) && i->addr) {
		pr_info("%s: size(0x%x) not 64k aligned\n", __func__, i->size);
		return -EINVAL;
	}

	if (is_align_4k(i->addr) ||
	    is_align_4k(i->size) ||
	    is_align_4k(i->bin)) {
		pr_info("%s: addr/size not 4k aligned\n", __func__);
		return -EINVAL;
	}

	if ((i->bin + i->size) > bin_end) {
		pr_info("%s: wrong size\n", __func__);
		return -EINVAL;
	}
#undef is_align_4k
#undef is_align_64k

	return 0;
}

dma_addr_t preload_iova_alloc(struct platform_device *pdev,
	struct vpu_device *vd, struct vpu_iova *vi,
	uint32_t addr, uint32_t size, uint32_t bin)
{
	dma_addr_t mva;

	vi->addr = addr;
	vi->size = size;
	vi->bin = bin;

	if (preload_iova_check(vi))
		return 0;

	mva = vpu_iova_alloc(pdev, vi);

	if (!mva)
		pr_info("%s: vpu%d: iova allcation failed\n", __func__, vd->id);

	vpu_drv_debug(
		"%s: vpu%d: addr:0x%X, size: 0x%X, bin: 0x%X, mva: 0x%lx\n",
		__func__, vd->id, vi->addr, vi->size, vi->bin,
		(unsigned long) mva);

	return mva;
}

static uint32_t vpu_init_dev_algo_preload_entry(
	struct platform_device *pdev, struct vpu_device *vd,
	struct vpu_algo_list *al, struct vpu_pre_info *info,
	uint32_t bin)
{
	struct __vpu_algo *alg;
	struct vpu_iova dummy_iova;
	struct vpu_iova *vi;
	uint64_t mva;
	uint32_t addr;
	uint32_t size = __ALIGN_KERNEL(info->file_sz, 0x1000);

	alg = al->ops->get(al, info->name, NULL);

	/* algo is already existed in the list */
	if (alg) {
		al->ops->put(alg);
		if (info->pAddr == PRELOAD_IRAM) {
			addr = 0;  /* dynamic alloc iova */
			vi = &alg->iram;
			mva = preload_iova_alloc(pdev, vd, vi, addr,
				size, info->off);
			alg->a.iram_mva = mva;
			goto added;
		}

		/* other segments had been merged into EXE_SEG */
		size = 0;
		goto out;
	}

	/* add new algo to the list */
	addr = info->pAddr & 0xFFF00000; /* static alloc iova */
	if (info->info)
		size = info->info;  // already aligned at packing stage

	alg = vpu_alg_alloc(al);
	if (!alg)
		goto out;

	al->cnt++;
	strncpy(alg->a.name, info->name, (ALGO_NAMELEN - 1));

	if (info->flag & 0x1 /* EXE_SEG */) {
		vi = &alg->prog;
		alg->a.entry_off = info->pAddr & 0xFFFF;
	} else {
		vi = &dummy_iova;
		pr_info("%s: vpu%d: unexpected segment: flags: %x\n",
			__func__, vd->id, info->flag);
	}

	mva = preload_iova_alloc(pdev, vd, vi, addr, size, info->off);
	alg->a.mva = mva;

	if (!alg->a.mva)
		goto out;

	alg->builtin = true;
	list_add_tail(&alg->list, &al->a);

added:
	vpu_drv_debug("%s: vpu%d(%xh): %s <%s>: off: %x, mva: %llx, size: %x, addr: %x\n",
		__func__, vd->id, info->vpu_core, info->name,
		(info->pAddr == PRELOAD_IRAM) ? "IRAM" : "PROG",
		info->off, mva, size, addr);
out:
	return bin + size;
}

static void vpu_exit_dev_algo_general(struct platform_device *pdev,
	struct vpu_device *vd, struct vpu_algo_list *al)
{
	struct __vpu_algo *alg, *tmp;

	vpu_alg_debug("%s: vd: %p, vpu%d, al->a: %p\n",
		__func__, vd, vd->id, &al->a);

	list_for_each_entry_safe(alg, tmp, &al->a, list) {
		vpu_alg_debug("%s: vd: %p, vpu%d, vd->al.a: %p, alg: %p\n",
			__func__, vd, vd->id, &al->a, alg);
		al->ops->put(alg);
	}
}

#if defined(APUSYS_AIOT)
static int vpu_algo_cmp(void *priv, const struct list_head *a,
	const struct list_head *b)
#else
static int vpu_algo_cmp(void *priv, struct list_head *a,
	struct list_head *b)
#endif
{
	struct __vpu_algo *la, *lb;

	la = list_entry(a, struct __vpu_algo, list);
	lb = list_entry(b, struct __vpu_algo, list);

	return strcmp(la->a.name, lb->a.name);
}

static int vpu_init_algo_info(struct vpu_device *vd,
	struct vpu_iova *iova_algo_info, struct vpu_algo_list *al)
{
	struct platform_device *pdev
		= container_of(vd->dev, struct platform_device, dev);
	struct algo_head *head;
	struct algo_list *algo, *array;
	struct __vpu_algo *alg, *tmp;
	dma_addr_t iova = 0;
	unsigned int size = al->cnt * sizeof(struct algo_list) +
		sizeof(struct algo_head);

	if (iova_algo_info->m.va)
		return 0;

	memset(iova_algo_info, 0, sizeof(struct vpu_iova));

	iova_algo_info->size = __ALIGN_KERNEL(size, 0x1000);
	iova_algo_info->bin = 0xFFFFFFFF;

	iova = vpu_iova_alloc(pdev, iova_algo_info);

	if (!iova) {
		pr_info("%s: vpu%d: iova allcation failed\n", __func__, vd->id);
		return 0;
	}

	head = (struct algo_head *)iova_algo_info->m.va;

	memset(head, 0, size);

	head->size = sizeof(struct algo_head);
	head->id = vd->id;
	head->number = al->cnt;
	array = (void *)((unsigned long)head + sizeof(struct algo_head));
	algo = array;

	spin_lock(&al->lock);
	list_sort(NULL, &al->a, vpu_algo_cmp);
	list_for_each_entry_safe(alg, tmp, &al->a, list) {
		strncpy(algo->name, alg->a.name, ALGO_NAMELEN);
		algo->mva = alg->a.mva;
		algo->len = alg->a.len;
		algo->entry_off = alg->a.entry_off;
		algo->iram_len = alg->a.iram_len;
		algo->iram_mva = alg->a.iram_mva;

		if (algo->entry_off)
			head->preload = 1;

		vpu_drv_debug("%s: vpu%d algo(0x%lX): %s, va: 0x%08X, len: 0x%X, entry: 0x%X\n",
			__func__, vd->id,
			(unsigned long)algo,
			algo->name, algo->mva, algo->len,
			algo->entry_off);

		algo++;
	}
	spin_unlock(&al->lock);

	vpu_iova_sync_for_device(vd->dev, iova_algo_info);

	vpu_drv_debug("%s: vpu%d [%s] iova: 0x%lX, size: 0x%X, num: %d\n",
		__func__, vd->id, (head->preload) ? "preload" : "normal",
		(unsigned long)iova, size, vd->aln.cnt);
	return 0;
}

static int vpu_free_algo_info(struct vpu_device *vd)
{
	struct platform_device *pdev
		= container_of(vd->dev, struct platform_device, dev);

	if (!vd->iova_algo_info.m.va)
		return 0;

	vpu_iova_free(&pdev->dev, &vd->iova_algo_info);
	vpu_iova_free(&pdev->dev, &vd->iova_preload_info);
	vd->iova_algo_info.m.va = 0;
	vd->iova_preload_info.m.va = 0;
	return 0;
}

static int vpu_init_dev_algo_preload(struct platform_device *pdev,
	struct vpu_device *vd, struct vpu_algo_list *al)
{
	int i, j, ret = 0;
	uint32_t offset;
	struct vpu_pre_info *info = NULL;
	struct vpu_image_header *header = NULL;

	vpu_algo_list_init(vd, al, &vpu_prelaod_aops, "Preload");

	offset = vpu_drv->bin_preload_ofs;

	for (i = 0; i < VPU_NUMS_IMAGE_HEADER; i++) {
		header = bin_header(i);
		info = (void *)((unsigned long)header + header->pre_info);

		for (j = 0; j < header->pre_info_count; j++, info++) {
			if (!((info->vpu_core & 0xF) & (1 << vd->id)))
				continue;

			offset = vpu_init_dev_algo_preload_entry(
				pdev, vd, al, info, offset);
		}
	}

	vpu_init_algo_info(vd, &vd->iova_preload_info, al);

	return ret;
}

static int vpu_init_dev_algo_normal(struct platform_device *pdev,
	struct vpu_device *vd, struct vpu_algo_list *al)
{
	int i, j;
	int ret = 0;
	unsigned int mva;
	struct vpu_algo_info *algo_info;
	struct vpu_image_header *header = NULL;

	vpu_algo_list_init(vd, al, &vpu_normal_aops, "Normal");

	/* for each algo in the image header, add them to device's algo list */
	for (i = 0; i < VPU_NUMS_IMAGE_HEADER; i++) {
		header = bin_header(i);
		for (j = 0; j < header->algo_info_count; j++) {
			struct __vpu_algo *alg;

			algo_info = bin_algo_info(header, j);
			mva = algo_info->offset - vpu_drv->iova_algo.bin +
				vpu_drv->mva_algo;

			/* skips, if the core mask mismatch */
			if (!((algo_info->vpu_core & 0xF) & (1 << vd->id)))
				continue;

			vpu_drv_debug("%s: vpu%d(%xh): %s: off: %x, mva: %x, len: %x\n",
				__func__,
				vd->id,
				algo_info->vpu_core,
				algo_info->name,
				algo_info->offset,
				mva,
				algo_info->length);

			alg = vpu_alg_alloc(al);
			if (!alg) {
				ret = -ENOMEM;
				goto out;
			}

			strncpy(alg->a.name,
				algo_info->name, (ALGO_NAMELEN - 1));
			alg->a.mva = mva;
			alg->a.len = algo_info->length;
			alg->builtin = true;

			list_add_tail(&alg->list, &al->a);
			al->cnt++;
		}
	}

	vpu_init_algo_info(vd, &vd->iova_algo_info, al);
out:
	return ret;
}

/* called by vpu_probe() */
int vpu_init_dev_algo(struct platform_device *pdev, struct vpu_device *vd)
{
	int ret;

	ret = vpu_init_dev_algo_normal(pdev, vd, &vd->aln);
	if (ret)
		goto out;

	ret = vpu_init_dev_algo_preload(pdev, vd, &vd->alp);

out:
	return ret;
}

/* called by vpu_remove() */
void vpu_exit_dev_algo(struct platform_device *pdev, struct vpu_device *vd)
{
	vpu_exit_dev_algo_general(pdev, vd, &vd->aln);
	vpu_exit_dev_algo_general(pdev, vd, &vd->alp);
	vpu_free_algo_info(vd);
}

/* command buffer init */
int vpu_alloc_cmd(struct platform_device *pdev, struct vpu_device *vd)
{
	int ret = 0;
	dma_addr_t iova = 0;

	vd->iova_cmd.bin = VPU_MEM_ALLOC;
	vd->iova_cmd.size = VPU_CMD_SIZE * VPU_MAX_PRIORITY;

	iova = vpu_iova_alloc(pdev, &vd->iova_cmd);
	if (!iova)
		ret = -ENOMEM;

	return ret;
}

