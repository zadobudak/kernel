/* SPDX-License-Identifier: GPL-2.0 */
/*
 *Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _VPU_LOAD_IMAGE_H_
#define _VPU_LOAD_IMAGE_H_

#include "vpu_cmn.h"

int vpu_loadimage(struct platform_device *pdev, struct vpu_device *vpu_device);
int set_property(struct vpu_device *vpu_device, struct prop_params *prop);

#endif /* #ifndef _VPU_LOAD_IMAGE_H_ */
