/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _MT6885_VPU_REG_H_
#define _MT6885_VPU_REG_H_

#include "vpu_cfg.h"
#include "vpu_cmn.h"

static inline
unsigned long vpu_reg_base(struct vpu_device *vd)
{
	return (unsigned long)vd->reg.m;
}

static inline
uint32_t vpu_reg_read(struct vpu_device *vd, int offset)
{
	return ioread32((void *) (vpu_reg_base(vd) + offset));
}

#endif
