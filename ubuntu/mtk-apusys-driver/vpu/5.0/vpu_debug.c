// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sched/clock.h>

#include "vpu_cmn.h"
#include "vpu_debug.h"
#include "vpu_hw.h"

u32 vpu_klog;

static int vpu_debug_info(struct seq_file *s);

/**
 * vpu_seq_time() - print time to seq_file
 *
 * @time: schedule clock time got from sched_clock()
 */
void vpu_seq_time(struct seq_file *s, uint64_t t)
{
	uint32_t nsec;

	if (!t) {
		seq_puts(s, "N/A");
		return;
	}

	nsec = do_div(t, 1000000000);
	seq_printf(s, "%lu.%06lu", (unsigned long)t,
		(unsigned long)nsec/1000);
}

static int vpu_debug_algo_entry(struct seq_file *s,
	struct vpu_algo_list *al, struct __vpu_algo *alg)
{
	if (!alg)
		return -ENOENT;

	if (!al || !al->ops || !al->ops->get || !al->ops->put)
		goto out;

	al->ops->get(al, NULL, alg);
	seq_printf(s, "[%s: prog: 0x%llx/0x%x, iram: 0x%llx/0x%x, entry: 0x%x, ref: %d, builtin: %d]\n",
		alg->a.name, alg->a.mva, alg->a.len,
		alg->a.iram_mva, alg->a.iram_len, alg->a.entry_off,
		kref_read(&alg->ref), alg->builtin);
	al->ops->put(alg);
out:
	return 0;
}

static int vpu_debug_algo_list(struct seq_file *s, struct vpu_algo_list *al)
{
	struct __vpu_algo *alg;
	int ret;
	struct list_head *ptr, *tmp;

	if (!al)
		return -ENOENT;

	seq_printf(s, "\n<%s Algorithms>\n", al->name);
	list_for_each_safe(ptr, tmp, &al->a) {
		alg = list_entry(ptr, struct __vpu_algo, list);
		ret = vpu_debug_algo_entry(s, al, alg);
		if (ret)
			break;
	}
	return ret;
}

static int vpu_debug_algo(struct seq_file *s)
{
	struct vpu_device *vd;

	if (!s)
		return -ENOENT;

	vd = (struct vpu_device *) s->private;
	if (!vd)
		return -ENODEV;

	vpu_debug_algo_list(s, &vd->aln);
	vpu_debug_algo_list(s, &vd->alp);

	return 0;
}

static void *vpu_mesg_pa_to_va(struct vpu_mem *work_buf, unsigned int phys_addr)
{
	unsigned long ret = 0;
	int offset = 0;

	if (!phys_addr)
		return NULL;

	offset = phys_addr - work_buf->pa;
	ret = work_buf->va + offset;

	return (void *)(ret);
}

static void vpu_mesg_clr(struct vpu_device *vd)
{
	char *data = NULL;
	struct vpu_message_ctrl *msg = vpu_mesg(vd);

	if (!msg)
		return;

	data = (char *)vpu_mesg_pa_to_va(&vd->iova_work.m, msg->data);

	msg->head = 0;
	msg->tail = 0;

	if (data)
		memset(data, 0,
		       vd->wb_log_data - sizeof(struct vpu_message_ctrl));
}

static int vpu_mesg_level_set(void *data, u64 val)
{
	struct vpu_message_ctrl *msg = NULL;
	struct vpu_device *vd = data;
	int level = (int)val;

	if (!vd)
		return -ENOENT;

	if (!level) {
		vpu_mesg_clr(vd);
		return 0;
	}

	if (level < -1 || level >= VPU_DBG_MSG_LEVEL_TOTAL) {
		pr_info("val: %d\n", level);
		return -ENOENT;
	}

	msg = vpu_mesg(vd);
	if (!msg)
		return -ENOENT;

	if (level != -1)
		msg->level_mask ^= (1 << level);
	else
		msg->level_mask = 0;
	return 0;
}

static int vpu_mesg_level_get(void *data, u64 *val)
{
	struct vpu_message_ctrl *msg = vpu_mesg(data);

	if (!msg)
		return -ENOENT;
	*val = msg->level_mask;
	return 0;
}

int vpu_mesg_seq(struct seq_file *s, struct vpu_device *vd)
{
	int i, wrap = false;
	char *data = NULL;
	struct vpu_message_ctrl *msg = vpu_mesg(vd);

	if (!s || !msg)
		return -ENOENT;

	vpu_iova_sync_for_cpu(vd->dev, &vd->iova_work);
	data = (char *)vpu_mesg_pa_to_va(&vd->iova_work.m, msg->data);
	i = msg->head;
	do {
		if (msg->head == msg->tail || i == msg->tail)
			seq_printf(s, "%s", "<empty log>\n");
		while (i != msg->tail && data) {
			if (i > msg->tail && wrap)
				break;

			seq_printf(s, "%s", data + i);
			i += strlen(data + i) + 1;

			if (i >= msg->buf_size) {
				i = 0;
				wrap = true;
			}
		}
	} while (0);

	return 0;
}

static int vpu_debug_mesg(struct seq_file *s)
{
	if (!s)
		return -ENOENT;

	return vpu_mesg_seq(s, (struct vpu_device *)s->private);
}

static int vpu_ulog_set(void *data, u64 val)
{
	struct vpu_device *vd;
	struct vpu_misc *misc = (void *)vpu_drv->iova_cfg.m.va;

	vd = list_first_entry(&vpu_drv->devs, struct vpu_device, list);

	if (!misc || !vd)
		return -ENOENT;

	misc->ulog_lv = (u32)val;
	vpu_iova_sync_for_device(vd->dev, &vpu_drv->iova_cfg);
	return 0;
}

static int vpu_ulog_get(void *data, u64 *val)
{
	struct vpu_misc *misc = (void *)vpu_drv->iova_cfg.m.va;

	if (!misc)
		return -ENOENT;

	*val = misc->ulog_lv;
	return 0;
}

static int vpu_delay_off_set(void *data, u64 val)
{
	struct vpu_device *vd;
	struct vpu_misc *misc = (void *)vpu_drv->iova_cfg.m.va;

	vd = list_first_entry(&vpu_drv->devs, struct vpu_device, list);

	if (!misc || !vd)
		return -ENOENT;

	misc->pwr_off_delay = (u32)val;
	vpu_iova_sync_for_device(vd->dev, &vpu_drv->iova_cfg);
	return 0;
}

static int vpu_delay_off_get(void *data, u64 *val)
{
	struct vpu_misc *misc = (void *)vpu_drv->iova_cfg.m.va;

	if (!misc)
		return -ENOENT;

	*val = misc->pwr_off_delay;
	return 0;
}

static int vpu_debug_reg_dev(struct seq_file *s, struct vpu_device *vd)
{
	seq_puts(s, "name\toffset\tvalue\n");

#define seq_vpu_reg(r) \
	seq_printf(s, "%s\t%4xh\t%08xh\n", #r, r, vpu_reg_read(vd, r))

	seq_vpu_reg(CG_CON);
	seq_vpu_reg(SW_RST);
	seq_vpu_reg(DONE_ST);
	seq_vpu_reg(CTRL);
	seq_vpu_reg(XTENSA_INT);
	seq_vpu_reg(CTL_XTENSA_INT);
	seq_vpu_reg(DEFAULT0);
	seq_vpu_reg(DEFAULT1);
	seq_vpu_reg(XTENSA_INFO00);
	seq_vpu_reg(XTENSA_INFO01);
	seq_vpu_reg(XTENSA_INFO02);
	seq_vpu_reg(XTENSA_INFO03);
	seq_vpu_reg(XTENSA_INFO04);
	seq_vpu_reg(XTENSA_INFO05);
	seq_vpu_reg(XTENSA_INFO06);
	seq_vpu_reg(XTENSA_INFO07);
	seq_vpu_reg(XTENSA_INFO08);
	seq_vpu_reg(XTENSA_INFO09);
	seq_vpu_reg(XTENSA_INFO10);
	seq_vpu_reg(XTENSA_INFO11);
	seq_vpu_reg(XTENSA_INFO12);
	seq_vpu_reg(XTENSA_INFO13);
	seq_vpu_reg(XTENSA_INFO14);
	seq_vpu_reg(XTENSA_INFO15);
	seq_vpu_reg(XTENSA_INFO16);
	seq_vpu_reg(XTENSA_INFO17);
	seq_vpu_reg(XTENSA_INFO18);
	seq_vpu_reg(XTENSA_INFO19);
	seq_vpu_reg(XTENSA_INFO20);
	seq_vpu_reg(XTENSA_INFO21);
	seq_vpu_reg(XTENSA_INFO22);
	seq_vpu_reg(XTENSA_INFO23);
	seq_vpu_reg(XTENSA_INFO24);
	seq_vpu_reg(XTENSA_INFO25);
	seq_vpu_reg(XTENSA_INFO26);
	seq_vpu_reg(XTENSA_INFO27);
	seq_vpu_reg(XTENSA_INFO28);
	seq_vpu_reg(XTENSA_INFO29);
	seq_vpu_reg(XTENSA_INFO30);
	seq_vpu_reg(XTENSA_INFO31);
	seq_vpu_reg(MBOX_INBOX_0);
	seq_vpu_reg(MBOX_INBOX_1);
	seq_vpu_reg(MBOX_INBOX_2);
	seq_vpu_reg(MBOX_INBOX_3);
	seq_vpu_reg(MBOX_INBOX_4);
	seq_vpu_reg(MBOX_INBOX_5);
	seq_vpu_reg(MBOX_INBOX_6);
	seq_vpu_reg(MBOX_INBOX_7);
	seq_vpu_reg(MBOX_INBOX_8);
	seq_vpu_reg(MBOX_INBOX_9);
	seq_vpu_reg(MBOX_INBOX_10);
	seq_vpu_reg(MBOX_INBOX_11);
	seq_vpu_reg(MBOX_INBOX_12);
	seq_vpu_reg(MBOX_INBOX_13);
	seq_vpu_reg(MBOX_INBOX_14);
	seq_vpu_reg(MBOX_INBOX_15);
	seq_vpu_reg(MBOX_INBOX_16);
	seq_vpu_reg(MBOX_INBOX_17);
	seq_vpu_reg(MBOX_INBOX_18);
	seq_vpu_reg(MBOX_INBOX_19);
	seq_vpu_reg(MBOX_INBOX_20);
	seq_vpu_reg(MBOX_INBOX_21);
	seq_vpu_reg(MBOX_INBOX_22);
	seq_vpu_reg(MBOX_INBOX_23);
	seq_vpu_reg(MBOX_INBOX_24);
	seq_vpu_reg(MBOX_INBOX_25);
	seq_vpu_reg(MBOX_INBOX_26);
	seq_vpu_reg(MBOX_INBOX_27);
	seq_vpu_reg(MBOX_INBOX_28);
	seq_vpu_reg(MBOX_INBOX_29);
	seq_vpu_reg(MBOX_INBOX_30);
	seq_vpu_reg(MBOX_INBOX_31);
	seq_vpu_reg(DEBUG_INFO00);
	seq_vpu_reg(DEBUG_INFO01);
	seq_vpu_reg(DEBUG_INFO02);
	seq_vpu_reg(DEBUG_INFO03);
	seq_vpu_reg(DEBUG_INFO04);
	seq_vpu_reg(DEBUG_INFO05);
	seq_vpu_reg(DEBUG_INFO06);
	seq_vpu_reg(DEBUG_INFO07);
	seq_vpu_reg(XTENSA_ALTRESETVEC);
#undef seq_vpu_reg
	return 0;
}

static int vpu_debug_reg(struct seq_file *s)
{
	if (!s)
		return -ENOENT;
	return vpu_debug_reg_dev(s, s->private);
}

/**
 * vpu_debug_jtag_set - enable/disable jtag
 *
 * @data: vpu device
 * @val: user write value
 *
 * return 0: enable/disable success
 */
static int vpu_debug_jtag_set(void *data, u64 val)
{
	struct vpu_device *vd = data;
	int ret = 0;

	if (!vd)
		return -ENOENT;

	if (val) {
		/* enable jtag */
		vd->jtag_enabled = 1;
	} else {
		/* disable jtag */
		vd->jtag_enabled = 0;
	}

	return ret;
}

/**
 * vpu_debug_jtag_get - show jtag status of vpu
 * @data: vpu device
 * @val: return value
 *
 * return 0: operation success
 */
static int vpu_debug_jtag_get(void *data, u64 *val)
{
	struct vpu_device *vd = data;

	if (!vd)
		return -ENOENT;

	*val = vd->jtag_enabled;
	return 0;
}

static int vpu_debug_iova_seq(struct seq_file *s,
	struct vpu_iova *i, const char *prefix)
{
	if (i->bin == VPU_MEM_ALLOC) {
		if (i->addr)
			seq_printf(s, "<%s> iova: 0x%x, size: 0x%x (static alloc)\n",
				prefix,	i->addr, i->size);
		else
			seq_printf(s, "<%s> iova: 0x%x, size: 0x%x (dynamic alloc)\n",
				prefix,	i->m.pa, i->size);
	} else if (i->size) {
		seq_printf(s, "<%s> iova: 0x%x, size: 0x%x, bin offset: 0x%x\n",
			prefix, i->addr, i->size, i->bin);
	}
	return 0;
}

static int vpu_debug_iomem_seq(struct seq_file *s,
	struct vpu_iomem *i, const char *prefix)
{
	struct resource *r;

	if (!i || !i->res)
		return -ENOENT;

	r = i->res;
	seq_printf(s, "<%s> start: 0x%lx, end: 0x%lx, size: 0x%lx\n",
		prefix,	(unsigned long)r->start, (unsigned long)r->end,
		(unsigned long)(r->end - r->start + 1));

	return 0;
}

static int vpu_debug_info_dev(struct seq_file *s, struct vpu_device *vd)
{
	if (!s || !vd)
		return -ENOENT;

	if (vd->state >= VS_REMOVING || vd->state <= VS_DISALBED)
		return 0;

	seq_printf(s, "\n----- %s: settings -----\n", vd->name);
	seq_printf(s, "mva_iram: 0x%llx\n", vd->mva_iram);
	seq_printf(s, "wb_log_size: 0x%x\n", vd->wb_log_size);
	seq_printf(s, "wb_log_data: 0x%x\n", vd->wb_log_data);
	seq_puts(s, "iova:\n");
	vpu_debug_iova_seq(s, &vd->iova_reset, "reset");
	vpu_debug_iova_seq(s, &vd->iova_main, "main");
	vpu_debug_iova_seq(s, &vd->iova_kernel, "kernel");
	vpu_debug_iova_seq(s, &vd->iova_iram, "iram");
	vpu_debug_iova_seq(s, &vd->iova_work, "work");
	seq_puts(s, "iomem:\n");
	vpu_debug_iomem_seq(s, &vd->reg, "reg");
	vpu_debug_iomem_seq(s, &vd->dmem, "dmem");
	vpu_debug_iomem_seq(s, &vd->imem, "imem");
	vpu_debug_iomem_seq(s, &vd->dbg, "dbg");
	seq_printf(s, "----- %s: current registers -----\n", vd->name);
	vpu_debug_reg_dev(s, vd);

	return 0;
}

static int vpu_debug_info(struct seq_file *s)
{
	struct vpu_device *vd;
	struct list_head *ptr, *tmp;

	if (!s)
		return -ENOENT;

	mutex_lock(&vpu_drv->lock);

	seq_puts(s, "======== Driver Info ========\n");
	seq_puts(s, "Queried Time: ");
	vpu_seq_time(s, sched_clock());
	seq_printf(s, "bin_pa: 0x%lx\n", vpu_drv->bin_pa);
	seq_printf(s, "bin_size: 0x%x\n", vpu_drv->bin_size);
	seq_printf(s, "bin_head_ofs: 0x%x\n", vpu_drv->bin_head_ofs);
	seq_printf(s, "bin_preload_ofs: 0x%x\n", vpu_drv->bin_preload_ofs);
	seq_printf(s, "mva_algo: 0x%llx\n", vpu_drv->mva_algo);
	vpu_debug_iova_seq(s, &vpu_drv->iova_algo, "algo");

	list_for_each_safe(ptr, tmp, &vpu_drv->devs) {
		vd = list_entry(ptr, struct vpu_device, list);
		vpu_debug_info_dev(s, vd);
	}
	mutex_unlock(&vpu_drv->lock);
	seq_puts(s, "\n");

	return 0;
}

#define VPU_DEBUGFS_FOP_DEF(name) \
static struct dentry *vpu_d##name; \
static int vpu_debug_## name ##_show(struct seq_file *s, void *unused) \
{ \
	vpu_debug_## name(s); \
	return 0; \
} \
static int vpu_debug_## name ##_open(struct inode *inode, struct file *file) \
{ \
	return single_open(file, vpu_debug_ ## name ## _show, \
		inode->i_private); \
} \

#define VPU_DEBUGFS_DEF(name) \
	VPU_DEBUGFS_FOP_DEF(name) \
static const struct file_operations vpu_debug_ ## name ## _fops = { \
	.open = vpu_debug_ ## name ## _open, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release, \
}

#define VPU_DEBUGFS_RW_DEF(name) \
	VPU_DEBUGFS_FOP_DEF(name) \
static const struct file_operations vpu_debug_ ## name ## _fops = { \
	.open = vpu_debug_ ## name ## _open, \
	.read = seq_read, \
	.write = vpu_debug_ ## name ## _write, \
	.llseek = seq_lseek, \
	.release = single_release, \
}

VPU_DEBUGFS_DEF(algo);
VPU_DEBUGFS_DEF(mesg);
VPU_DEBUGFS_DEF(reg);
VPU_DEBUGFS_DEF(info);

#undef VPU_DEBUGFS_DEF

static struct dentry *vpu_djtag;
DEFINE_SIMPLE_ATTRIBUTE(vpu_debug_jtag_fops, vpu_debug_jtag_get,
			vpu_debug_jtag_set, "%lld\n");

static struct dentry *vpu_dmesg_level;
DEFINE_SIMPLE_ATTRIBUTE(vpu_debug_mesg_level_fops, vpu_mesg_level_get,
			vpu_mesg_level_set, "%lld\n");

static struct dentry *vpu_dulog;
DEFINE_SIMPLE_ATTRIBUTE(vpu_debug_ulog_fops, vpu_ulog_get,
			vpu_ulog_set, "%lld\n");

static struct dentry *vpu_dpw_off_latency;
DEFINE_SIMPLE_ATTRIBUTE(vpu_debug_pw_off_latency_fops, vpu_delay_off_get,
			vpu_delay_off_set, "%lld\n");

#if !defined(USER_BUILD_KERNEL) && defined(CONFIG_MTK_ENG_BUILD)
#define VPU_DEBUGFS_MDOE		0644
#else
#define VPU_DEBUGFS_MDOE		0444
#endif

#define VPU_DEBUGFS_CREATE(name) \
{ \
	vpu_d##name = debugfs_create_file(#name, VPU_DEBUGFS_MDOE, \
		droot,         \
			NULL, &vpu_debug_ ## name ## _fops); \
	if (IS_ERR_OR_NULL(vpu_d##name)) { \
		ret = PTR_ERR(vpu_d##name); \
		pr_info("%s: vpu%d: " #name "): %d\n", \
			__func__, (vd) ? (vd->id) : 0, ret); \
		goto out; \
	} \
	vpu_d##name->d_inode->i_private = vd; \
}

int vpu_init_dev_debug(struct platform_device *pdev, struct vpu_device *vd)
{
	int ret = 0;
	struct dentry *droot;

	vpu_mesg_init(vd);

	if (!vpu_drv->droot)
		return -ENODEV;

	droot = debugfs_create_dir(vd->name, vpu_drv->droot);

	if (IS_ERR_OR_NULL(droot)) {
		ret = PTR_ERR(droot);
		pr_info("%s: failed to create debugfs node: vpu/%s: %d\n",
			__func__, vd->name, ret);
		goto out;
	}

	VPU_DEBUGFS_CREATE(algo);
	VPU_DEBUGFS_CREATE(mesg);
	VPU_DEBUGFS_CREATE(reg);
	VPU_DEBUGFS_CREATE(jtag);
	VPU_DEBUGFS_CREATE(mesg_level);

out:
	return ret;
}

void vpu_exit_dev_debug(struct platform_device *pdev, struct vpu_device *vd)
{
	if (!vpu_drv || !vpu_drv->droot || !vd || !vd->droot)
		return;

	debugfs_remove_recursive(vd->droot);
	vd->droot = NULL;
}

int vpu_init_debug(void)
{
	int ret = 0;
	struct dentry *droot;
	struct vpu_device *vd = NULL;

	droot = debugfs_create_dir("vpu", NULL);

	if (IS_ERR_OR_NULL(droot)) {
		ret = PTR_ERR(droot);
		pr_info("%s: failed to create debugfs node: %d\n",
			__func__, ret);
		goto out;
	}

	vpu_drv->droot = droot;
	vpu_klog = VPU_DBG_DRV;
	debugfs_create_u32("klog", 0660, droot, &vpu_klog);

	VPU_DEBUGFS_CREATE(info);
	VPU_DEBUGFS_CREATE(ulog);
	VPU_DEBUGFS_CREATE(pw_off_latency);
out:
	return ret;
}

#undef VPU_DEBUGFS_CREATE

void vpu_exit_debug(void)
{
	if (!vpu_drv || !vpu_drv->droot)
		return;

	debugfs_remove_recursive(vpu_drv->droot);
	vpu_drv->droot = NULL;
}

