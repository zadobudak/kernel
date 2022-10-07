/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __VPU_ALGO_H__
#define __VPU_ALGO_H__

#include <linux/slab.h>
#include "vpu_mem.h"

#define ALGO_NAMELEN 32

struct vpu_device;
struct vpu_algo_list;

/* vpu_algo.c */
struct __vpu_algo *vpu_alg_alloc(struct vpu_algo_list *al);
void vpu_alg_free(struct __vpu_algo *alg);

/* vpu_hw.c */
int vpu_init_dev_algo(struct platform_device *pdev, struct vpu_device *vd);
void vpu_exit_dev_algo(struct platform_device *pdev, struct vpu_device *vd);
int vpu_hw_alg_init(struct vpu_algo_list *al, struct __vpu_algo *alg);

/* vpu_algo.c
 * handleing dynamic load/unload algo
 */

struct vpu_algo {
	char name[ALGO_NAMELEN];

	uint32_t len;     /* binary length */
	uint64_t mva;     /* mapped mva address to the binary */

	/* preload algo */
	uint32_t entry_off;  /* algo entry offset */
	uint32_t iram_len;   /* iram data length */
	uint64_t iram_mva;   /* iram data iova */
};

struct __vpu_algo {
	struct vpu_algo a;
	struct vpu_iova prog;   /* preloaded and dynamic loaded algo */
	struct vpu_iova iram;   /* preloaded algo iram */
	bool builtin;           /* from vpu binary */
	struct kref ref;        /* reference count */
	struct list_head list;  /* link to device algo list */
	struct vpu_algo_list *al;
};

struct vpu_algo_ops {
	/* driver controls */
	struct __vpu_algo * (*get)(struct vpu_algo_list *al,
		const char *name, struct __vpu_algo *alg);
	void (*put)(struct __vpu_algo *alg);
	void (*release)(struct kref *ref);
};

extern struct vpu_algo_ops vpu_normal_aops;
extern struct vpu_algo_ops vpu_prelaod_aops;

struct vpu_algo_list {
	char name[ALGO_NAMELEN];
	spinlock_t lock;
	struct list_head a;
	unsigned int cnt;    /* # of algorithms */
	struct vpu_device *vd;
	struct vpu_algo_ops *ops;
};

static inline void
vpu_algo_list_init(struct vpu_device *vd, struct vpu_algo_list *al,
	struct vpu_algo_ops *ops, const char *name) {
	if (!vd || !al)
		return;
	strncpy(al->name, name, (ALGO_NAMELEN - 1));
	spin_lock_init(&al->lock);
	INIT_LIST_HEAD(&al->a);
	al->cnt = 0;
	al->vd = vd;
	al->ops = ops;
}

#endif
