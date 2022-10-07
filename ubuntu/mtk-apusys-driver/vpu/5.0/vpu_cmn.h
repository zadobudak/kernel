/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __VPU_CMN_H__
#define __VPU_CMN_H__

#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/workqueue.h>

#include "apusys_power.h"
#include "vpu_cfg.h"
#include "vpu_mem.h"
#include "vpu_algo.h"

/*
 * mutex lock order
 * 1. driver lock (vpu_drv->lock)
 *    - protects driver's data (vpu_drv)
 * 2. command lock (vd->cmd[i].lock)
 *    - protects device's command execution of priority "i"
 * 3. device lock (vd->lock)
 *    - protects device's power control, and boot sequence
 **/

enum vpu_mutex_class {
	VPU_MUTEX_DRV = 0,
	VPU_MUTEX_CMD = 1,
	VPU_MUTEX_DEV = (VPU_MAX_PRIORITY + VPU_MUTEX_CMD),
};

struct vpu_misc {
	uint32_t ulog_lv;
	uint32_t pwr_off_delay;
	uint32_t reserved[7];
} __attribute__((__packed__));

struct vpu_probe_info {
	void *np;
	int bound; // device is bound
};

// data for load image
struct prop_params {
	u64 dsp_num;
	u64 mblock;
	u64 aligned_size;
	u64 head_offset;
	u64 preload_offset;
	u64 iram_off;
	u64 alg_offset;
	u64 total_alg_sz;
};

// driver data
struct vpu_driver {
	void *bin_va;
	unsigned long bin_pa;
	unsigned int bin_size;
	unsigned int bin_head_ofs;
	unsigned int bin_preload_ofs;
	unsigned int core_num;

	/* power work queue */
	struct workqueue_struct *wq;

	/* iova settings */
	struct kref iova_ref;
	struct device *iova_dev;
	struct vpu_iova iova_algo;
	struct vpu_iova iova_cfg;

	/* shared */
	uint64_t mva_algo;
	uint64_t mva_cfg;

	/* memory */
	struct apu_bmap ab;  /* bitmap used by v2 allocator */
	struct list_head vi; /* list of all mapped vpu_iova */
	struct mutex vi_lock;

	/* list of devices */
	struct list_head devs;
	struct mutex lock;

	/* debugfs entry */
	struct dentry *droot;

	/* device references */
	struct kref ref;

	/*vpu fw image*/
	bool vpu_load_image;
	const struct firmware *fw[VPU_NUMS_IMAGE_HEADER];
	struct prop_params prop;
};

enum vpu_state {
	VS_UNKNOWN = 0,
	VS_DISALBED,   // disabled by e-fuse
	VS_DOWN,       // power down
	VS_REMOVING,
	VS_TOTAL
};

struct vpu_iomem {
	void __iomem *m;
	struct resource *res;
};

// device data
struct vpu_device {
	/* APU power management data. MUST be placed at the begin. */
	struct apu_dev_power_data pd;

	/* general */
	int id;
	char name[8];
	struct list_head list;   // link in vpu driver
	enum vpu_state state;
	//struct mutex lock;
	struct device *dev;      // platform device

	/* iomem */
	struct vpu_iomem reg;
	struct vpu_iomem dmem;
	struct vpu_iomem imem;
	struct vpu_iomem dbg;

	/* iova settings */
	struct vpu_iova iova_reset;
	struct vpu_iova iova_main;
	struct vpu_iova iova_kernel;
	struct vpu_iova iova_iram;
	struct vpu_iova iova_work;
	struct vpu_iova iova_algo_info;
	struct vpu_iova iova_preload_info;
	struct vpu_iova iova_cmd;

	/* work buffer */
	uint32_t wb_log_size;
	uint32_t wb_log_data;

	/* algorithm */
	struct vpu_algo_list aln;  /* normal */
	struct vpu_algo_list alp;  /* preload */

	/* memory */
	uint64_t mva_iram;

	/* debug */
	struct dentry *droot;  /* debugfs entry */
	bool ftrace_avail;     /* trace */
	bool jtag_enabled;     /* jtag */
	struct vpu_dmp *dmp;   /* dump */
};

extern struct vpu_driver *vpu_drv;

int vpu_init_dev_hw(struct platform_device *pdev, struct vpu_device *vd);
int vpu_init_drv_hw(void);

int vpu_exit_dev_hw(struct platform_device *pdev, struct vpu_device *vd);
int vpu_exit_drv_hw(void);

int vpu_alloc_cmd(struct platform_device *pdev, struct vpu_device *vd);

int vpu_alloc_algo(struct __vpu_algo **ralgo);
int vpu_free_algo(struct __vpu_algo *algo);
#endif

