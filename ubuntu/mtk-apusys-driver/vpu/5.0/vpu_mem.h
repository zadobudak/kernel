/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __VPU_MEM_H__
#define __VPU_MEM_H__

#include <linux/platform_device.h>
#include <linux/scatterlist.h>

#define VPU_MEM_ALLOC  (0xFFFFFFFF)
#define APU_BMAP_NAME_LEN 16

struct vpu_mem {
	void *handle;
	unsigned long va;
	uint32_t pa;
	uint32_t length;
};

struct vpu_iova {
	/* settings from dts */
	uint32_t addr;  /* iova */
	uint32_t size;
	uint32_t bin;   /* offset in binary */
	/* allocated memory */
	struct vpu_mem m;
	/* allocated iova */
	struct sg_table sgt;
	uint64_t time;  /* allocated time */
	uint64_t iova;  /* allocated iova */
	/* link in vpu driver */
	struct list_head list;
};

struct apu_bmap {
	/* input */
	uint32_t start;
	uint32_t end;
	uint32_t au;  // allocation unit (in bytes)
	unsigned long align_mask;
	char name[APU_BMAP_NAME_LEN];

	// output
	uint32_t size;
	unsigned long *b;     // bitmap
	unsigned long nbits;  // number of bits
	spinlock_t lock;
};

int vpu_mem_init(void);
void vpu_mem_exit(void);
dma_addr_t vpu_iova_alloc(struct platform_device *pdev,
	struct vpu_iova *i);
void vpu_iova_free(struct device *dev, struct vpu_iova *i);
void vpu_iova_sync_for_device(struct device *dev, struct vpu_iova *i);
void vpu_iova_sync_for_cpu(struct device *dev, struct vpu_iova *i);
int vpu_iova_dts(struct platform_device *pdev,
	const char *name, struct vpu_iova *i);

#endif

