// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include "vpu_cmn.h"
#include "vpu_debug.h"

static struct __vpu_algo *vpu_alg_get(struct vpu_algo_list *al,
	const char *name, struct __vpu_algo *alg)
{
	if (alg) {
		kref_get(&alg->ref);
		goto out;
	}

	if (!name)
		goto out;

	/* search from tail, so that existing algorithm can be
	 * overidden by dynamic loaded ones.
	 **/
	spin_lock(&al->lock);
	list_for_each_entry_reverse(alg, &al->a, list) {
		if (!strcmp(alg->a.name, name)) {
			/* found, reference count++ */
			kref_get(&alg->ref);
			goto unlock;
		}
	}
	alg = NULL;
unlock:
	spin_unlock(&al->lock);
out:
	if (alg)
		vpu_alg_debug("%s: vpu%d: %s: %s: ref: %d builtin: %d\n",
			__func__, al->vd->id, al->name, alg->a.name,
			kref_read(&alg->ref), alg->builtin);
	else
		vpu_alg_debug("%s: vpu%d: %s: %s was not found\n",
			__func__, al->vd->id, al->name, name);

	return alg;
}

static void vpu_alg_release(struct kref *ref)
{
	struct __vpu_algo *alg
		= container_of(ref, struct __vpu_algo, ref);
	struct vpu_algo_list *al = alg->al;

	spin_lock(&al->lock);
	list_del(&alg->list);
	al->cnt--;
	spin_unlock(&al->lock);

	vpu_alg_debug("%s: vpu%d: %s: %s, algo_cnt: %d builtin: %d\n",
		__func__, al->vd->id, al->name, alg->a.name,
		al->cnt, alg->builtin);

	/* free __vpu_algo memory */
	vpu_alg_free(container_of(ref, struct __vpu_algo, ref));
}

static void vpu_alg_put(struct __vpu_algo *alg)
{
	vpu_alg_debug("%s: vpu%d: %s: %s: ref: %d builtin: %d\n",
		__func__, alg->al->vd->id, alg->al->name, alg->a.name,
		kref_read(&alg->ref), alg->builtin);
	kref_put(&alg->ref, alg->al->ops->release);
}


struct __vpu_algo *vpu_alg_alloc(struct vpu_algo_list *al)
{
	struct __vpu_algo *algo;

	algo = kzalloc(sizeof(struct __vpu_algo), GFP_KERNEL);
	if (!algo)
		return NULL;

	algo->builtin = false;
	algo->al = al;

	INIT_LIST_HEAD(&algo->list);
	kref_init(&algo->ref);  /* init count = 1 */

	return algo;
}

void vpu_alg_free(struct __vpu_algo *alg)
{
	struct device *dev = alg->al->vd->dev;

	vpu_iova_free(dev, &alg->prog);
	vpu_iova_free(dev, &alg->iram);
	kfree(alg);
}

struct vpu_algo_ops vpu_normal_aops = {
	.get = vpu_alg_get,
	.put = vpu_alg_put,
	.release = vpu_alg_release,
};

struct vpu_algo_ops vpu_prelaod_aops = {
	.get = vpu_alg_get,
	.put = vpu_alg_put,
	.release = vpu_alg_release,
};

