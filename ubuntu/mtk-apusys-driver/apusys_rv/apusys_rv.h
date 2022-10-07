/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#pragma once
#include <apu.h>

#include <linux/firmware.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/libfdt_env.h>
#include <linux/libfdt.h>

//typedef int status_t;
//typedef void *addr_t;//uintptr_t
//typedef void *vaddr_t;//uintptr_t
//typedef void *paddr_t;//uintptr_t

#define ROUNDUP(a, b) (((a) + ((b)-1)) & ~((b)-1))

/* platform dependent address */

/* #define APUSYS_IMG_SIZE 0x300000 */ /* 3M */
/* #define APUSYS_SEC_MEM_SIZE 0x600000 */ /* 6M */
#define APUSYS_COREDUMP_SHADOW_MEM_SIZE 0x200000  /* 2M */
#define APUSYS_MEM_ALIGN 0x10000  /* 64K (minimal size for EMI MPU) */
#define APUSYS_MEM_LIMIT 0x90000000  /* max address can APUSYS remap */
#define APUSYS_FW_ALIGN 16 /* for mdla dma alignment limitation */

#define APUSYS_UP_ROM_EMI_REGION 23
#define ENABLE_APUSYS_EMI_PROTECTION 1

/* image name definition */
#define IMG_NAME_APUSYS_A	"tinysys-apusys-RV33_A"

/******************************************************************************
 * 1. New entries must be appended to the end of the structure.
 * 2. Do NOT use conditional option such as #ifdef inside the structure.
 */
enum PT_ID_APUSYS {
	PT_ID_APUSYS_FW,
	PT_ID_APUSYS_XFILE,
	PT_ID_MDLA_FW_BOOT,
	PT_ID_MDLA_FW_MAIN,
	PT_ID_MDLA_XFILE,
	PT_ID_MVPU_FW,
	PT_ID_MVPU_XFILE,
	PT_ID_MVPU_SEC_FW,
	PT_ID_MVPU_SEC_XFILE
};

#define PART_HEADER_DEFAULT_ADDR	(0xFFFFFFFF)
#define LOAD_ADDR_MODE_BACKWARD		(0x00000000)
#define PART_MAGIC			0x58881688
#define EXT_MAGIC			0x58891689
#define PT_MAGIC			0x58901690

struct ptimg_hdr_t {
	u32 magic;     /* magic number*/
	u32 hdr_size;  /* header size */
	u32 img_size;  /* img size */
	u32 align;     /* alignment */
	u32 id;        /* image id */
	u32 addr;      /* memory addr */
};

int apu_load_apusys_rv(void *fdt);
/* apusys_rv loader function implemented by platform */
int platform_load_apusys_rv(struct mtk_apu *apu, struct platform_device *pdev);

