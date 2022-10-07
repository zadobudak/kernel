// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>

#include <linux/iommu.h>
#include <linux/bitmap.h>
#include <linux/list_sort.h>
#include <linux/seq_file.h>
#include <linux/sched/clock.h>
#include <linux/spinlock.h>

#include "vpu_cfg.h"
#include "vpu_mem.h"
#include "vpu_debug.h"

#define is_au_align(ab, val) (!((val) & (ab->au - 1)))

static int vpu_map_kva_to_sgt(
	const char *buf, size_t len, struct sg_table *sgt);

static dma_addr_t vpu_map_sg_to_iova(
	struct platform_device *pdev, struct scatterlist *sg,
	unsigned int nents, size_t len, dma_addr_t given_iova);

/**
 * Initialize bitmap<br>
 * <b>Inputs</b><br>
 *   ab->start: start address.<br>
 *   ab->end: end address.<br>
 *   ab->au: allocation unit.<br>
 *   ab->align_mask: alignment mask in terms of au, 0 = no alignment.<br>
 * <b>Outputs</b><br>
 *   ab->size: equals to (ab->end - ab->start).<br>
 *   ab->b: allocated bitmap.<br>
 *   ab->nbits: number of bits in the bitmap(ab->b).<br>
 *   ab->lock: spin lock used to protect the bitmap.<br>
 * @param[in,out] ab The apu bitmap object
 * @param[in] name The name of this bitmap object
 * @return 0: Success.<br>
 *   EINVAL: start, end, or size is not aligned to allocation unit.
 */
int apu_bmap_init(struct apu_bmap *ab, const char *name)
{
	if (!ab || ab->start > ab->end)
		return -EINVAL;

	memset(ab->name, 0, APU_BMAP_NAME_LEN);
	strncpy(ab->name, name, APU_BMAP_NAME_LEN - 1);

	/* size must be multiple of allocation unit */
	ab->size = ab->end - ab->start;
	vpu_mem_debug("%s: %s: start: 0x%x, end: 0x%x, size: 0x%x, au: 0x%x\n",
		__func__, ab->name, ab->start, ab->end, ab->size, ab->au);
	if (!is_au_align(ab, ab->size)) {
		pr_info("%s: %s: size 0x%x is un-aligned to AU 0x%x\n",
			__func__, ab->name, ab->size, ab->au);
		return -EINVAL;
	}

	ab->nbits = ab->size / ab->au;
	ab->b = bitmap_zalloc(ab->nbits, GFP_KERNEL);
	spin_lock_init(&ab->lock);

	return 0;
}

/**
 * Release bitmap
 * @param[in] ab The apu bitmap object to be released
 */
void apu_bmap_exit(struct apu_bmap *ab)
{
	if (!ab || !ab->b)
		return;

	bitmap_free(ab->b);
	ab->b = NULL;
}

/**
 * Allocate addresses from bitmap
 * @param[in] ab The apu bitmap object to be allocated from.
 * @param[in] size Desired allocation size in bytes.
 * @param[in] given_addr Search begin from the given address,
 *            0 = Searches from ab->start
 * @return 0: Allocation failed.<br>
 *    Others: Allocated address.<br>
 * @remark: Searches free addresses from begin (ab->start).<br>
 *    You have to check the returned address for static iova mapping
 */
uint32_t apu_bmap_alloc(struct apu_bmap *ab, unsigned int size,
	uint32_t given_addr)
{
	uint32_t addr = 0;
	unsigned int nr;
	unsigned long offset;
	unsigned long start = 0;
	unsigned long flags;

	if (!ab)
		return 0;

	if (given_addr) {
		start = given_addr - ab->start;
		if (!is_au_align(ab, start)) {
			pr_info("%s: %s: size: 0x%x, given addr: 0x%x, start 0x%x is un-aligned to AU 0x%x\n",
				__func__, ab->name, size, addr, start, ab->au);
			return 0;
		}
		start = start / ab->au;
	}

	spin_lock_irqsave(&ab->lock, flags);
	nr = round_up(size, ab->au) / ab->au;

	offset = bitmap_find_next_zero_area(ab->b, ab->nbits, start,
		nr, ab->align_mask);

	if (offset >= ab->nbits) {
		pr_info("%s: %s: Out of memory: size: 0x%x, given addr: 0x%x, offset: %d, nbits: %d\n",
			__func__, ab->name, size, addr, offset, ab->nbits);
		goto out;
	}

	addr = offset * ab->au + ab->start;
	__bitmap_set(ab->b, offset, nr);

out:
	spin_unlock_irqrestore(&ab->lock, flags);

	if (addr)
		vpu_mem_debug("%s: %s: size: 0x%x, given_addr: 0x%x, allocated addr: 0x%x\n",
		__func__, ab->name, size, given_addr, addr);

	return addr;
}

/**
 * Free occupied addresses from bitmap
 * @param[in] ab The apu bitmap object.
 * @param[in] addr Allocated start address returned by apu_bmap_alloc().
 * @param[in] size Allocated size.
 */
void apu_bmap_free(struct apu_bmap *ab, uint32_t addr, unsigned int size)
{
	unsigned int nr;
	unsigned long offset;
	unsigned long flags;

	if (!ab || addr < ab->start || (addr + size) > ab->end)
		return;

	nr = round_up(size, ab->au) / ab->au;
	offset = addr - ab->start;

	vpu_mem_debug("%s: %s: addr: 0x%x, size: 0x%x, nr_bits: %d\n",
		__func__, ab->name, addr, size, nr);

	if (!is_au_align(ab, offset)) {
		pr_info("%s: %s: addr 0x%x, offset 0x%x is un-aligned to AU 0x%x\n",
			__func__, ab->name, addr, offset, ab->au);
		return;
	}

	offset = offset / ab->au;
	if (offset >= ab->nbits) {
		pr_info("%s: %s: addr 0x%x, offset-bit %d is out of limit %d\n",
			__func__, ab->name, addr, offset, ab->nbits);
		return;
	}

	spin_lock_irqsave(&ab->lock, flags);
	__bitmap_clear(ab->b, offset, nr);
	spin_unlock_irqrestore(&ab->lock, flags);
}

#if defined(APUSYS_AIOT)
static int vpu_iova_cmp(void *priv, const struct list_head *a,
	const struct list_head *b)
#else
static int vpu_iova_cmp(void *priv, struct list_head *a,
	struct list_head *b)
#endif
{
	struct vpu_iova *ia, *ib;

	ia = list_entry(a, struct vpu_iova, list);
	ib = list_entry(b, struct vpu_iova, list);

	if (ia->iova < ib->iova)
		return -1;
	if (ia->iova > ib->iova)
		return 1;

	return 0;
}

static void vpu_dump_sg(struct scatterlist *s)
{
	unsigned int i = 0;

	if (!s || !vpu_debug_on(VPU_DBG_MEM))
		return;

	while (s) {
		struct page *p = sg_page(s);
		phys_addr_t phys;

		if (!p)
			break;

		phys = page_to_phys(p);
		pr_info("%s: sg[%d]: pfn: %lx, pa: %lx, len: %lx, dma_addr: %lx\n",
			__func__, i,
			(unsigned long) page_to_pfn(p),
			(unsigned long) phys,
			(unsigned long) s->length,
			(unsigned long) s->dma_address);
		s = sg_next(s);
		i++;
	}
}

static void vpu_dump_sgt(struct sg_table *sgt)
{
	if (!sgt || !sgt->sgl)
		return;

	vpu_dump_sg(sgt->sgl);
}

static int
vpu_mem_alloc(struct platform_device *pdev,
	struct vpu_iova *i, dma_addr_t given_iova)
{
	int ret = 0;
	void *kva;
	dma_addr_t iova;

	if (!i) {
		ret = -EINVAL;
		goto out;
	}

	vpu_mem_debug("%s: size: 0x%x, given iova: 0x%llx (%s alloc)\n",
		__func__, i->size, (u64)given_iova,
		(given_iova == VPU_IOVA_END) ? "dynamic" : "static");

	kva = kvmalloc(i->size, GFP_KERNEL);

	if (!kva) {
		ret = -ENOMEM;
		goto error;
	}

	vpu_mem_debug("%s: kvmalloc: %llx\n", __func__, (uint64_t)kva);

	ret = vpu_map_kva_to_sgt(kva, i->size, &i->sgt);

	if (ret)
		goto error;

	iova = vpu_map_sg_to_iova(pdev, i->sgt.sgl, i->sgt.nents,
		i->size, given_iova);

	if (!iova)
		goto error;

	i->m.va = (uint64_t)kva;
	i->m.pa = (uint32_t)iova;
	i->m.length = i->size;

	goto out;
error:
	kvfree(kva);
out:
	return ret;
}

void vpu_mem_free(struct vpu_mem *m)
{
	kvfree((void *)m->va);
}

static int
vpu_map_kva_to_sgt(const char *buf, size_t len, struct sg_table *sgt)
{
	struct page **pages = NULL;
	unsigned int nr_pages;
	unsigned int index;
	const char *p;
	int ret;

	vpu_mem_debug("%s: buf: %p, len: %lx, sgt: %p\n",
		__func__, buf, len, sgt);

	nr_pages = DIV_ROUND_UP((unsigned long)buf + len, PAGE_SIZE)
		- ((unsigned long)buf / PAGE_SIZE);
	pages = kmalloc_array(nr_pages, sizeof(struct page *), GFP_KERNEL);

	if (!pages)
		return -ENOMEM;

	p = buf - offset_in_page(buf);

	for (index = 0; index < nr_pages; index++) {
		if (is_vmalloc_addr(p))
			pages[index] = vmalloc_to_page(p);
		else
			pages[index] = kmap_to_page((void *)p);
		if (!pages[index]) {
			pr_info("%s: map failed\n", __func__);
			ret = -EFAULT;
			goto out;
		}
		p += PAGE_SIZE;
	}

	vpu_mem_debug("%s: nr_pages: %d\n", __func__, nr_pages);

	ret = sg_alloc_table_from_pages(sgt, pages, index,
		offset_in_page(buf), len, GFP_KERNEL);

	if (ret) {
		pr_info("%s: sg_alloc_table_from_pages: %d\n",
			__func__, ret);
		goto out;
	}

	vpu_dump_sgt(sgt);
out:
	kfree(pages);
	return ret;
}

static dma_addr_t vpu_map_sg_to_iova(
	struct platform_device *pdev, struct scatterlist *sg,
	unsigned int nents, size_t len, dma_addr_t given_iova)
{
	struct iommu_domain *domain;
	dma_addr_t iova = 0;
	size_t size = 0;
	int prot = IOMMU_READ | IOMMU_WRITE;
	u64 bank = VPU_IOVA_BANK;
	u32 iova_end = VPU_IOVA_END;
	u32 iova_heap = VPU_IOVA_HEAP;
	struct vpu_device *vd = dev_get_drvdata(&pdev->dev);

	domain = iommu_get_domain_for_dev(&pdev->dev);

	vpu_mem_debug("%s: %s: len: %zx, given_iova: %llx (%s alloc)\n",
		__func__, vd->name, len, (u64)given_iova,
		(given_iova < iova_end) ? "static" : "dynamic");

	if (given_iova < iova_end) {  /* Static IOVA allocation */
		iova = apu_bmap_alloc(&vpu_drv->ab, len, given_iova);
		if (!iova)
			goto err;
		/* Static: must be allocated on the given address */
		if (iova != given_iova) {
			dev_info(&pdev->dev,
				"%s: given iova: %llx, apu_bmap_alloc returned: %llx\n",
				__func__, (u64)given_iova, (u64)iova);
			apu_bmap_free(&vpu_drv->ab, iova, len);
			goto err;
		}
	} else {  /* Dynamic IOVA allocation */
		/* Dynamic: Allocate from heap first */
		iova = apu_bmap_alloc(&vpu_drv->ab, len, iova_heap);
		if (!iova) {
			/* Dynamic: Try to allocate again from iova start */
			iova = apu_bmap_alloc(&vpu_drv->ab, len, 0);
			if (!iova)
				goto err;
		}
	}

	iova = iova | bank;
	vpu_mem_debug("%s: %s: len: %zx, iova: %llx\n",
		__func__, vd->name, len, (u64)iova);

	size = iommu_map_sg(domain, iova, sg, nents, prot);

	if (size == 0) {
		dev_info(&pdev->dev,
			"%s: iommu_map_sg: len: %zx, iova: %llx, failed\n",
			__func__, len, (u64)iova, nents);
		goto err;
	} else if (size != len) {
		dev_info(&pdev->dev,
			"%s: iommu_map_sg: len: %zx, iova: %llx, mismatch with mapped size: %zx\n",
			__func__, len, (u64)iova, size);
		goto err;
	}

	return iova;

err:
	if (iova)
		apu_bmap_free(&vpu_drv->ab, len, iova);

	return 0;
}

static dma_addr_t
vpu_map_to_iova(struct platform_device *pdev, void *addr, size_t len,
	dma_addr_t given_iova, struct sg_table *sgt)
{
	dma_addr_t iova = 0;
	int ret;

	if (!sgt)
		goto out;

	ret = vpu_map_kva_to_sgt(addr, len, sgt);

	if (ret)
		goto out;

	iova = vpu_map_sg_to_iova(pdev, sgt->sgl, sgt->nents, len, given_iova);
out:
	return iova;
}

static void vpu_unmap_iova_from_sg(struct device *dev, struct vpu_iova *i)
{
	struct iommu_domain *domain;
	struct vpu_device *vd = dev_get_drvdata(dev);
	dma_addr_t iova = i->iova;
	size_t size = i->size;
	size_t ret;

	vpu_mem_debug("%s: %s: len: %zx, iova: %llx\n",
		__func__, vd->name, size, (u64)iova);

	domain = iommu_get_domain_for_dev(dev);
	if (i->sgt.sgl) {
		ret = iommu_unmap(domain, iova, size);
		if (ret != size)
			dev_info(dev,
				"%s: iommu_unmap iova: %llx, returned: %zx, expected: %zx\n",
				__func__, (u64)iova, ret, size);

		sg_free_table(&i->sgt);
	}
	apu_bmap_free(&vpu_drv->ab, i->m.pa, i->m.length);
}

int vpu_mem_init(void)
{
	if (vpu_drv->ab.b)
		return 0;

	vpu_drv->ab.au = PAGE_SIZE;
	vpu_drv->ab.start = VPU_IOVA_START;
	vpu_drv->ab.end = VPU_IOVA_END;
	apu_bmap_init(&vpu_drv->ab, "vpu_mem");

	INIT_LIST_HEAD(&vpu_drv->vi);
	mutex_init(&vpu_drv->vi_lock);
	return 0;
}

void vpu_mem_exit(void)
{
	struct vpu_iova *i;
	struct list_head *ptr, *tmp;
	uint32_t nsec;
	uint64_t t;
	int remain = 0;

	mutex_lock(&vpu_drv->vi_lock);
	list_sort(NULL, &vpu_drv->vi, vpu_iova_cmp);
	list_for_each_safe(ptr, tmp, &vpu_drv->vi) {
		i = list_entry(ptr, struct vpu_iova, list);
		t = i->time;
		nsec = do_div(t, 1000000000);
		pr_info(
			"%s: [%lu.%06lu] iova: %llx, addr: %x, size %x, bin: %x, m.pa: %x, m.len: %x\n",
			__func__, (unsigned long)t, (unsigned long)nsec/1000,
			i->iova, i->addr, i->size, i->bin,
			i->m.pa, i->m.length);
		list_del(&i->list);
		i->time = 0;
		vpu_iova_free(vpu_drv->iova_dev, i);
		remain++;
	}
	mutex_unlock(&vpu_drv->vi_lock);

	if (remain)
		pr_info("%s: WARNING: there were %d unrelease iova.\n",
			__func__, remain);

	apu_bmap_exit(&vpu_drv->ab);
}

dma_addr_t vpu_iova_alloc(struct platform_device *pdev,
	struct vpu_iova *i)
{
	int ret = 0;
	dma_addr_t iova = 0;
	unsigned long base = (unsigned long)vpu_drv->bin_va;

	if (!pdev || !i || !i->size)
		goto out;

	iova = i->addr ? i->addr : VPU_IOVA_END;

	i->sgt.sgl = NULL;
	i->m.handle = NULL;
	i->m.va = 0;
	i->m.pa = 0;
	i->m.length = 0;

	/* allocate kvm and map */
	if (i->bin == VPU_MEM_ALLOC) {
		ret = vpu_mem_alloc(pdev, i, iova);
		iova = i->m.pa;
	/* map from vpu firmware loaded at bootloader */
	} else if (i->size) {
		iova = vpu_map_to_iova(pdev,
			(void *)(base + i->bin), i->size, iova,
			&i->sgt);
	} else {
		dev_info(&pdev->dev,
			"%s: unknown setting (%x, %x, %x)\n",
			__func__, i->addr, i->bin, i->size);
		iova = 0;
	}

out:
	return iova;
}

void vpu_iova_free(struct device *dev, struct vpu_iova *i)
{
	struct vpu_device *vd = dev_get_drvdata(dev);

	if (!i->iova || !i->size)
		return;

	/* skip, if already deleted by .exit() */
	if (i->time) {
		mutex_lock(&vpu_drv->vi_lock);
		list_del(&i->list);
		i->time = 0;
		mutex_unlock(&vpu_drv->vi_lock);
	}

	vpu_mem_debug("%s: %s: iova: 0x%llx, size: %x\n",
		__func__, vd->name, i->iova, i->size);
	vpu_mem_free(&i->m);
	vpu_unmap_iova_from_sg(dev, i);
}

void vpu_iova_sync_for_device(struct device *dev,
	struct vpu_iova *i)
{
	dma_sync_sg_for_device(dev, i->sgt.sgl, i->sgt.nents,
		DMA_TO_DEVICE);
}

void vpu_iova_sync_for_cpu(struct device *dev,
	struct vpu_iova *i)
{
	dma_sync_sg_for_cpu(dev, i->sgt.sgl, i->sgt.nents,
		DMA_FROM_DEVICE);
}

int vpu_iova_dts(struct platform_device *pdev,
	const char *name, struct vpu_iova *i)
{
	if (of_property_read_u32_array(pdev->dev.of_node,
			name, &i->addr, 3)) {
		dev_info(&pdev->dev, "%s: vpu: unable to get %s\n",
			__func__, name);
		return -ENODEV;
	}

	dev_info(&pdev->dev, "%s: %s: addr: %08xh, size: %08xh, bin: %08xh\n",
		__func__, name, i->addr, i->size, i->bin);

	return 0;
}

