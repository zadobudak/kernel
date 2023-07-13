// SPDX-License-Identifier: GPL-2.0
/*
 *Copyright (c) 2020 MediaTek Inc.
 */

#include "vpu_loadimage.h"
#include "vpu_debug.h"
#include <linux/firmware.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/libfdt_env.h>
#include <linux/libfdt.h>

/* The buffer layout in DDR is as follows:
 * The total is 45MBs.
 * [phyaddr offset]
 *  0x00000000  +---------------------------+  0
 *              |    Core#0 reset vector    |
 *  0x00100000  +---------------------------+  DSP_RST_VEC_SZ * 1
 *              |    Core#0 main program    |
 *  0x00400000  +---------------------------+  DSP_RST_VEC_SZ * 1 + DSP_PROG_SZ * 1
 *              |    Core#1 reset vector    |
 *  0x00500000  +---------------------------+  DSP_RST_VEC_SZ * 2 + DSP_PROG_SZ * 1
 *              |    Core#1 main program    |
 *  0x00800000  +---------------------------+  DSP_RST_VEC_SZ * 2 + DSP_PROG_SZ * 2
 *              |    Core#2 reset vector    |
 *  0x00900000  +---------------------------+  DSP_RST_VEC_SZ * 3 + DSP_PROG_SZ * 2
 *              |    Core#2 main program    |
 *  0x00C00000  +---------------------------+  DSP_RST_VEC_SZ * 3 + DSP_PROG_SZ * 3
 *              |         Algo area         |
 *  0x02950000  +---------------------------+  default IRAM_OFFSET
 *              |  main program#0 IRAM data |
 *  0x02980000  +---------------------------+
 *              |  main program#1 IRAM data |
 *  0x029B0000  +---------------------------+
 *              |  main program#2 IRAM data |
 *  0x029E0000  +---------------------------+
 *              |    Merged image header    |
 *  0x02A10000  +---------------------------+
 *              |          PRELOAD          |
 *              +---------------------------+
 *
 * the layout of main program IRAM data
 *
 *              +--------------------------+
 *              |  total iram segment num  |
 *              +--------------------------+
 *              |     offset to iram#0     |
 *              +--------------------------+
 *              |      dest of iram#0      |
 *              +--------------------------+
 *              |      size of iram#0      |
 *              +--------------------------+
 *              |     offset to iram#1     |
 *              +--------------------------+
 *              |      dest of iram#1      |
 *              +--------------------------+
 *              |      size of iram#1      |
 *              +--------------------------+
 *              |     offset to iram#2     |
 *              +--------------------------+
 *              |      dest of iram#2      |
 *              +--------------------------+
 *              |      size of iram#2      |
 *              +--------------------------+
 *              |        iram#0 data       |
 *              +--------------------------+
 *              |        iram#1 data       |
 *              +--------------------------+
 *              |        iram#2 data       |
 *              +--------------------------+
 *
 * num: number of segment for IRAM program
 * off: bin data offset from start of Main Prog. IRAM
 * dst: the dst address of bin data at IRAM
 * size: size of bin data
 * seg_data: bin data
 *
 *
 * layout of Preload
 *              +-----------------------+
 *              |   preload#0 program   |
 *              +-----------------------+
 *              |  preload#0 IRAM data  |
 *              +-----------------------+
 *              |   preload#1 program   |
 *              +-----------------------+
 *              |  preload#1 IRAM data  |
 *              +-----------------------+
 *              |          ...          |
 *              +-----------------------+
 *
 *
 */

/*****************************************************************************
 *                             Constants/Macros                              *
 *****************************************************************************/
#define MODULE_NAME	"[LK_BOOT_VPU] "
#define VERSION		(0x21005000)

#define ADDR_ALIGN	(0x00010000)
#define ADDR_MAX	(0xC0000000)
#define DEF_RSRV_SIZE	(0x02A10000) // default reserved ddr size for vpu
#define PROG_OFFSET	(0x00100000) // start address of main program (core0)

#define DSP_RST_VEC_SZ	(0x00100000) // Reset vector for 1 VPU: 1MB
#define DSP_PROG_SZ	(0x00300000) // Main program for 1 VPU: 3MB
#define DEF_ALG_SZ	(0x01D50000) // default algo bank size for 3 VPUs

#define IRAM_SEG_SZ	(0x00010000) // max 64k for each iram region
#define DSP_BOOT_SZ	(DSP_RST_VEC_SZ + DSP_PROG_SZ)
#define SPARE_SZ	(0x00001000) // spare size for VPU

#define MAX_PART_SIZE	(0x00F00000)
#define ADDR_MASK	(0xFFF00000)
#define ALIGN_16	(0x00000010)
// 16 bytes alignment for algo bin and iram prog so that vpu core can use DMA to copy
#define ALIGN_32	(0x00000020)
#define ALIGN_4K	(0x00001000)
#define ALIGN_64K	(0x00010000)
#define ALIGN_2M	(0x00200000)

#define MAIN_SEG_MAX	(50)
#define ALG_NAME_SIZE	(32)
#define HDR_STR_SIZE	(32)
#define VP6_MAGIC	(0x60)
#define PROP_LEN	(16)
#define MAX_DSP_NUM	(3)
#define NODE_CONT_LEN	(3)
#define MAJOR_VER(x)	(x & 0xFF000000)

#define ALIGN_MASK(x,a)		__ALIGN_MASK(x,(a)-1)
#define __ALIGN_MASK(x,mask)	(((x)+(mask))&~(mask))

#define IRAM_MAIN_HDR_SIZE	(4 + MAIN_SEG_MAX * sizeof(struct iram_desc))
#define MAIN_IRAM_DESC_HDR_SIZE	ALIGN_MASK(IRAM_MAIN_HDR_SIZE, ALIGN_16)


#define DT_ID            "id"
#define DT_IRAM_DAT      "iram-data"
#define DT_ALGO          "algo"
#define DT_COMP          "compatible"
#define DT_BIN           "bin-phy-addr"
#define DT_BIN_SZ        "bin-size"
#define DT_HEAD          "img-head"
#define DT_PRE_BIN       "pre-bin"

#define PARTITION_HEADER_SZ	(0x200)

/*****************************************************************************
 *                             Type declarations                             *
 *****************************************************************************/
enum CORE_NUM {
	CORE0 = 0,
	CORE1,
	CORE2,
	CORE_MAX,
};

enum _VPU_CORE_ENUM_ {
	VPU_CORE_0 = (VP6_MAGIC | (1 << CORE0)), /* bitwise to represent core index */
	VPU_CORE_1 = (VP6_MAGIC | (1 << CORE1)),
	VPU_CORE_2 = (VP6_MAGIC | (1 << CORE2)),
	VPU_CORE_MAX = 0x03,
};

enum auth_cmd {
	AUTH_INIT,
	AUTH_DO,
};

enum dt_prop {
	PROP_INIT,
	PRE_BIN,
};

enum DSP_ADDR {
	RESET_VEC = 0,
	MAIN_PROG = 1,
	IRAM = 2,
};

enum pre_flag {
	EXE_SEG  = (1 << 0),
	ZERO_SEG = (1 << 1),
};

enum {
	TOTAL_DSP,
	TOTAL_ALG_SZ,
	RESERVE_VAR2,
	RESERVE_VAR3,
	RESERVE_TOTAL,
};

struct iram_desc {
	unsigned int offset;
	unsigned int addr;
	unsigned int size;
};

struct seg_info {
	u32 vpu_core; /* core index */
	u32 off;      /* offset */
	u32 pAddr;    /* destination */
	u32 mem_sz;   /* mem. size byte to occupied */
	u32 file_sz;  /* file size byte to copy */
};

struct addr_param {
	int is_preload;
	int core;
	int iram_segment;
	int iram_num;
	u64 iram_hdr;
	u64 mblock;
	u64 iram_off;
	u64 phys_addr;
	u64 mem_sz;
	u64 ihdr_size;
	u64 preload_base;
	u64 preload_sz;
};

struct alg_info {
	u32 vpu_core; /* core index */
	u32 off;      /* offset */
	u32 file_sz;  /* file size byte to copy */
	char name[ALG_NAME_SIZE];
};

struct pre_info {
	u32 vpu_core;
	u32 off;
	u32 pAddr;
	u32 mem_sz;
	u32 file_sz;
	u32 flag;
	u32 info; // already aligned (to 4k or 64k) at packing stage
	u32 reserve;
	char name[ALG_NAME_SIZE];
};

struct img_info {
	int total_num;
	int hdr_size;
	int img_max;
	int pre_cnt;
	u32 dsp_num;
	u32 total_extra_sz;
	u32 total_alg_sz;
	void *head;
	char *name;
};

struct install_params {
	int   img_max;
	u64   mblock;
	u64   alg_dst;
	u64   pre_dst;
	u64   iram_off;
	char *name;
	int  *overlap;
	struct img_header *part_hdr;
};

/*
struct prop_params {
	u32 dsp_num;
	u32 mblock;
	u32 aligned_size;
	u32 head_offset;
	u32 preload_offset;
	u32 iram_off;
	u32 alg_offset;
	u32 total_alg_sz;
};
 */

struct img_header {
	int version;
	int build_date;
	int hdr_str[HDR_STR_SIZE / 4];
	int hdr_size;
	int img_size;
	int mem_size; // total preload segments size

	int segment_num;
	int seg_info; // offset
	int seg_size;

	int pre_num;
	int pre_info; // offset
	int pre_size;

	int alg_num;
	int alg_info; // offset

	int reserved[RESERVE_TOTAL];
};

#define ENABLE_VPU_EMI_PROTECTION	(0)

/*****************************************************************************
 *                             Global variables                              *
 *****************************************************************************/
static const char *g_vpu_image_name[VPU_NUMS_IMAGE_HEADER] = {
	"cam_vpu1.img", "cam_vpu2.img", "cam_vpu3.img"};
static char *part_name[] = {"cam_vpu1", "cam_vpu2", "cam_vpu3"};
static int   part_size[] = {0x000C00000, 0x000F00000, 0x000F00000};
static u32 vpu_m4u_addr[3][3] = {
	{0x7DA00000, 0x7DB00000, 0x7FF00000},
	{0x7E300000, 0x7E400000, 0x7FF00000},
	{0x7EC00000, 0x7ED00000, 0x7FF00000}
};

/*****************************************************************************
 *                             Functions/Interfaces                          *
 *****************************************************************************/
static int is_dsp_addr(unsigned int addr, unsigned int core, unsigned int block)
{
	return (addr & ADDR_MASK) == vpu_m4u_addr[core][block];
}

static inline int ascen(const int a, const int b)
{
	return a == b ? 0 : a < b ? -1 : 1;
}

static inline int descen(const int a, const int b)
{
	return -ascen(a, b);
}

static inline void kswap(int *const a, int *const b)
{
	int tmp;

	tmp = *a;
	*a = *b;
	*b = tmp;
}

int check_overlap(int *addr, int len)
{
	int cnt = 0, i, cmp;
	int (*compare)(const int a, const int b);

	for (i = 0; i < len; ++i) {
		cmp = ascen(addr[i - 1], addr[i]);
		if (cmp == 0)
			return true;
		cnt += cmp;
	}

	compare = cnt <= 0 ? ascen : descen;

	for (i = 1; i < len; i += cmp == -1 || i == 1 ? 1 : -1) {
		cmp = compare(addr[i - 1], addr[i]);
		if (cmp == 0)
			return true;
		if (cmp == 1)
			kswap(&addr[i], &addr[i - 1]);
	}
	return false;
}

int set_property(struct vpu_device *vpu_device, struct prop_params *prop)
{
	unsigned int i = 0, dsp_num = 0;
	fdt32_t iram_prop[3][NODE_CONT_LEN] = {{0}, {0}, {0}};
	fdt32_t alg_prop[NODE_CONT_LEN] = {0};
	u32 vpu_iram_data[3][NODE_CONT_LEN] = {
		{0x00000000, IRAM_SEG_SZ, 0x02950000},
		{0x00000000, IRAM_SEG_SZ, 0x02950000 + IRAM_SEG_SZ},
		{0x00000000, IRAM_SEG_SZ, 0x02950000 + (IRAM_SEG_SZ * 2)}
	};
	u32 vpu_alg[NODE_CONT_LEN] = {
		0x00000000, DEF_ALG_SZ, (DSP_BOOT_SZ * 3)};

	dsp_num = prop->dsp_num;

	vpu_alg[1] = prop->total_alg_sz;
	vpu_alg[2] = prop->alg_offset;

	alg_prop[0] = cpu_to_fdt32(vpu_alg[0]);
	alg_prop[1] = cpu_to_fdt32(vpu_alg[1]);
	alg_prop[2] = cpu_to_fdt32(vpu_alg[2]);

	vpu_drv->iova_algo.addr = vpu_alg[0];
	vpu_drv->iova_algo.size = vpu_alg[1];
	vpu_drv->iova_algo.bin  = vpu_alg[2];
	pr_info("algo: addr: %08xh, size: %08xh, bin: %08xh\n",
		vpu_drv->iova_algo.addr, vpu_drv->iova_algo.size,
		vpu_drv->iova_algo.bin);

	i = vpu_device->id;
	vpu_iram_data[i][2] = prop->alg_offset + prop->total_alg_sz +
				(i * IRAM_SEG_SZ);
	iram_prop[i][0] = cpu_to_fdt32(vpu_iram_data[i][0]);
	iram_prop[i][1] = cpu_to_fdt32(vpu_iram_data[i][1]);
	iram_prop[i][2] = cpu_to_fdt32(vpu_iram_data[i][2]);
	vpu_device->iova_iram.addr = vpu_iram_data[i][0];
	vpu_device->iova_iram.size = vpu_iram_data[i][1];
	vpu_device->iova_iram.bin = vpu_iram_data[i][2];
	pr_info("iram: addr: %08xh, size: %08xh, bin: %08xh\n",
		vpu_device->iova_iram.addr, vpu_device->iova_iram.size,
		vpu_device->iova_iram.bin);

	vpu_drv->bin_type = VPU_IMG_PRELOAD;

	return 0;
}

static unsigned long long get_addr(struct addr_param *param)
{
	int offset = 0;
	u32 ret = true;
	u64 addr = param->phys_addr;
	u64 mblock = param->mblock;
	u32 *iram_num = NULL;
	struct iram_desc *desc = NULL;

	static u32 iram1_seg = MAIN_IRAM_DESC_HDR_SIZE;
	static u32 iram2_seg = MAIN_IRAM_DESC_HDR_SIZE;
	static u32 iram3_seg = MAIN_IRAM_DESC_HDR_SIZE;
	static u32 pre_seg = 0;

	if (is_dsp_addr(addr, CORE0, MAIN_PROG)) {
		offset = (int)(addr - vpu_m4u_addr[CORE0][MAIN_PROG]);
		mblock += DSP_RST_VEC_SZ;
	} else if (is_dsp_addr(addr, CORE1, MAIN_PROG)) {
		offset = (int)(addr - vpu_m4u_addr[CORE1][MAIN_PROG]);
		mblock += (DSP_BOOT_SZ + DSP_RST_VEC_SZ);
	} else if (is_dsp_addr(addr, CORE2, MAIN_PROG)) {
		offset = (int)(addr - vpu_m4u_addr[CORE2][MAIN_PROG]);
		mblock += ((DSP_BOOT_SZ << 1) + DSP_RST_VEC_SZ);
	} else if (is_dsp_addr(addr, CORE0, RESET_VEC)) {
		offset = (int)(addr - vpu_m4u_addr[CORE0][RESET_VEC]);
	} else if (is_dsp_addr(addr, CORE1, RESET_VEC)) {
		offset = (int)(addr - vpu_m4u_addr[CORE1][RESET_VEC]);
		mblock += DSP_BOOT_SZ;
	} else if (is_dsp_addr(addr, CORE2, RESET_VEC)) {
		offset = (int)(addr - vpu_m4u_addr[CORE2][RESET_VEC]);
		mblock += (DSP_BOOT_SZ << 1);
	} else if (is_dsp_addr(addr, CORE0, IRAM) && !param->is_preload) {
		param->iram_segment = 1;

		switch (param->core) {
		case VPU_CORE_0:
			offset = iram1_seg;
			mblock += param->iram_off;
			iram_num  = (u32 *)mblock;
			iram1_seg += ALIGN_MASK(param->mem_sz, ALIGN_16);
			break;
		case VPU_CORE_1:
			offset = iram2_seg;
			mblock += (param->iram_off + IRAM_SEG_SZ);
			iram_num  = (u32 *)mblock;
			iram2_seg += ALIGN_MASK(param->mem_sz, ALIGN_16);
			break;
		case VPU_CORE_2:
			offset = iram3_seg;
			mblock += param->iram_off + (IRAM_SEG_SZ << 1);
			iram_num  = (u32 *)mblock;
			iram3_seg += ALIGN_MASK(param->mem_sz, ALIGN_16);
			break;
		default:
			pr_err("invalid segment core 0x%x\n", param->core);
			return 0;
		}

		param->iram_num = (*iram_num);

		//update iram descriptor
		desc = (void *)(mblock + sizeof(int));
		desc[param->iram_num].offset = offset;
		desc[param->iram_num].addr = addr;
		desc[param->iram_num].size = param->mem_sz;
		*iram_num = (param->iram_num + 1);
	} else if (param->is_preload) {
		if (is_dsp_addr(addr, CORE0, IRAM)) {
			param->iram_segment = 1;
			mblock += param->iram_off;
			param->iram_off += param->mem_sz;
		} else {
			param->iram_segment = 0;

			if (addr == 0xFFFFFFFF) {
				pre_seg += param->preload_sz;
				mblock += pre_seg;
				param->iram_off = pre_seg + param->ihdr_size;
				pre_seg += param->mem_sz;
			} else {
				mblock += (pre_seg +
					  (addr - param->preload_base));
			}
		}

		if (param->iram_hdr && param->iram_segment) {
			iram_num  = (unsigned int *)param->iram_hdr;
			param->iram_num = (*iram_num);
			desc = (void *)(param->iram_hdr + sizeof(int));
		}
	} else {
		pr_err("invalid segment addr 0x%x\n", addr);
		return 0;
	}

	return (mblock + offset);
}

static int install_main(struct install_params *install)
{
	struct img_header *part_hdr = install->part_hdr;
	struct addr_param param;
	int j, ret = 0;
	int offset = 0, read_size = 0, zero = 0, info = part_hdr->seg_info;
	u64 dst = 0;
	struct seg_info *seg = (void *)((unsigned long)part_hdr + info);

	memset(&param, 0, sizeof(struct addr_param));

	for (j = 0; j < part_hdr->segment_num; j++) {
		offset = seg->off + part_hdr->hdr_size;
		read_size = seg->file_sz;

		param.core = seg->vpu_core;
		param.phys_addr = seg->pAddr;
		param.mem_sz = seg->mem_sz;
		param.mblock = install->mblock;
		param.iram_off = install->iram_off;
		param.iram_segment = 0;

		dst = get_addr(&param);
		if (!(dst)) {
			ret = -EINVAL;
			break;
		}

		if (param.iram_segment) {
			pr_info("seg[%2d] [0x%05x:0x%05x] -> [0x%x/0x%x]: iram[0x%x:%d]\n",
				j, offset, read_size, seg->pAddr, dst,
				seg->vpu_core, param.iram_num);
		} else {
			pr_info("seg[%2d] [0x%05x:0x%05x] -> [0x%x/0x%llx]\n",
				j, offset, read_size, seg->pAddr, dst);
		}

		if (seg->file_sz > 0) {
			if (offset + read_size > install->img_max) {
				pr_err("%s partition offset error, offset 0x%x, read_size 0x%x\n",
					install->name, offset, read_size);
				ret = -EINVAL;
				break;
			} else {
				memcpy((void *)dst,
					(void *)((u8 *)part_hdr + offset),
					read_size);
			}
		}

		zero = seg->mem_sz - seg->file_sz;

		if (zero) {
			dst += seg->file_sz;
			pr_info("%-7s [0x%x:0x%x]\n", "zero", dst, zero);
			memset((void *)(dst), 0, zero);
		}
		seg++;
	}

	return ret;
}

static int install_preload(struct install_params *install)
{
	struct img_header *part_hdr = install->part_hdr;
	struct addr_param param;
	int j, ret = 0, idx = install->overlap[0];
	int offset = 0, read_size = 0, zero = 0, info = part_hdr->pre_info;
	u64 dst = 0;
	struct pre_info *pre = (void *)((unsigned long)part_hdr + info);

	memset(&param, 0, sizeof(struct addr_param));

	for (j = 0; j < part_hdr->pre_num; j++) {
		offset = pre->off + part_hdr->hdr_size + part_hdr->seg_size;
		read_size = (pre->pAddr == (unsigned int)0xFFFFFFFF) ?
			     pre->mem_sz : pre->file_sz;

		param.is_preload = true;
		param.core   = pre->vpu_core;
		param.phys_addr = pre->pAddr;
		param.mem_sz = pre->file_sz;
		param.mblock = install->pre_dst;
		param.iram_segment = 0;
		param.iram_hdr = 0;
		param.ihdr_size = (pre->pAddr == (unsigned int)0xFFFFFFFF) ?
			pre->mem_sz : 0;

		if ((pre->flag & EXE_SEG) == EXE_SEG) {
			param.phys_addr = pre->pAddr & 0xFFF00000;
			param.preload_base = param.phys_addr;
			param.preload_sz = pre->info;

			install->overlap[idx] = pre->pAddr & 0xFFF00000;
			install->overlap[0] = ++idx;
		}

		dst = get_addr(&param);
		if (!(dst)) {
			ret = -EINVAL;
			break;
		}

		if (param.iram_segment) {
			pr_info("pre[%2d] [0x%05x:0x%06x] -> [0x%x/0x%llx]: iram[0x%x:%d]\n",
				 j, offset, read_size, pre->pAddr, dst,
				 pre->vpu_core, param.iram_num);
		} else {
			if (pre->pAddr == 0xFFFFFFFF) {
				param.iram_hdr = dst;
				pr_info("pre[%2d] [0x%05x:0x%06x] -> [0x%x/0x%llx]: iram[0x%x] head\n",
					 j, offset, pre->file_sz, pre->pAddr,
					 dst, pre->vpu_core);
			} else {
				param.iram_hdr = 0;
				pr_info("pre[%2d] [0x%05x:0x%06x] -> [0x%x/0x%llx]\n",
					j, offset, read_size, pre->pAddr, dst);
			}
		}

		if (pre->file_sz > 0) {
			if (offset + read_size > install->img_max) {
				pr_err("%s partition offset error, offset 0x%x, read_size 0x%x\n",
					install->name, offset, read_size);
				ret = -EINVAL;
				break;
			} else {
				memcpy((void *)dst,
					(void *)((u8 *)part_hdr + offset),
					read_size);
			}
		}

		zero = pre->mem_sz - pre->file_sz;
		pre->off = dst - install->mblock;

		if (zero && pre->pAddr != 0xFFFFFFFF) {
			dst += pre->file_sz;
			pr_info("%-7s [0x%x:0x%x]\n", "zero", dst, zero);
			memset((void *)(dst), 0, zero);
		}
		pre++;
	}

	return ret;
}

static int install_algo(struct install_params *install)
{
	struct img_header *part_hdr = install->part_hdr;
	int j, ret = 0;
	int offset = 0, read_size = 0, info = part_hdr->alg_info;
	u64 dst = install->alg_dst;
	struct alg_info *alg = (void *)((unsigned long)part_hdr + info);

	for (j = 0; j < part_hdr->alg_num; j++) {
		int new_offset = (int)(dst - install->mblock);

		offset = alg->off + part_hdr->hdr_size;
		read_size = alg->file_sz;

		pr_info("alg[%2d] [0x%06x:0x%06x] -> [0x%llx] => 0x%x\n",
			j, offset, read_size, dst, new_offset);

		if (offset + read_size > install->img_max) {
			pr_err("%s partition offset error, offset 0x%x, read_size 0x%x\n",
				install->name, offset, read_size);
			ret = -EINVAL;
			break;
		} else {
			memcpy((void *)dst,
				(void *)((u8 *)part_hdr + offset),
				read_size);
		}

		// change alg offset in partition header for driver to get real offset
		alg->off = new_offset;
		dst += ALIGN_MASK(alg->file_sz, ALIGN_16);
		alg++;
	}

	install->alg_dst = dst;
	return ret;
}

void *read_img(struct vpu_device *vpu_device)
{
	int i, len, pre_cnt = 0, alg_sum = 0, dsp_num = 0;
	int total_num = sizeof(part_name)/sizeof(*part_name);
	char *name = NULL;
	int max = 0;
	unsigned long bin_size = 0, hdr_total = 0;
	struct img_header *head = NULL;
	struct img_info *info = kzalloc(total_num * sizeof(struct img_info),
					GFP_KERNEL);

	if (!info) {
		pr_err("kzalloc fail\n");
		return NULL;
	}

	// confirm that all firmwares exist before processing
	for (i = 0; i < total_num; i++) {
		// parse vpu firmware name from 'firmware-name' in dts
		int ret = of_property_read_string_index(vpu_device->dev->of_node, "firmware-name", i, &g_vpu_image_name[i]);

		len = request_firmware(&(vpu_drv->fw[i]),
				       g_vpu_image_name[i], vpu_device->dev);
		if (len < 0) {
			pr_info("%s: vpu_fw request_firmware cam_vpu%d not exist\n",
				__func__, i + 1);
			kfree(info);
			return NULL;
		}
	}

	// process all firmwares
	for (i = 0; i < total_num; i++) {
		info[i].total_num = total_num;
		name = part_name[i];
		max = part_size[i];

		pr_info("%s: vpu_fw request_firmware cam_vpu%d success\n",
			__func__, i + 1);

		head = (void *)vpu_drv->fw[i]->data + PARTITION_HEADER_SZ;
		head->hdr_str[HDR_STR_SIZE/4 - 1] = 0;

		if (head->img_size > max) {
			pr_err("img size(0x%x) exceeds!\n", head->img_size);
			kfree(info);
			return NULL;
		}

		if (MAJOR_VER(VERSION) != MAJOR_VER(head->version)) {
			pr_err("version mismatch (%x/[%d]%x)\n",
				VERSION, i, head->version);
			kfree(info);
			return NULL;
		}

		info[i].head = head;
		info[i].name = name;
		info[i].img_max = max;

		bin_size += head->mem_size;
		pr_info("total preload mem size: 0x%x\n", head->mem_size);
		hdr_total += head->hdr_size;
		pre_cnt += head->pre_num;

		// cam_vpu1.img carries only main program & preload and does not carry normal libraries
		if (i)
			alg_sum += head->reserved[TOTAL_ALG_SZ];
		else
			dsp_num = head->reserved[TOTAL_DSP];
	}

	info[0].hdr_size = ALIGN_MASK(hdr_total, ALIGN_32);
	info[0].pre_cnt = pre_cnt;
	info[0].dsp_num = dsp_num ? dsp_num : 3;
	info[0].total_alg_sz = ALIGN_MASK(alg_sum, ALIGN_64K);

	bin_size = ALIGN_MASK(bin_size, ALIGN_4K);
	info[0].total_extra_sz = bin_size;
	return info;
}

int vpu_loadimage(struct platform_device *pdev, struct vpu_device *vpu_device)
{
	int i, ret = 0, num = 0;
	int *overlap = NULL;
	u32 *pIramNum;
	u64 mblock, head_dst, iram_off = 0, offset = 0, pre_dst = 0;
	u64 extra_size = 0, mblock_size = 0, aligned_size = 0;
	u64 alg_offset = 0;
	struct img_info *img = NULL;
	struct img_header *head = NULL;
	struct install_params params;
	struct prop_params prop;
	struct device_node *np;
	int rc;
	struct resource r;

	if (vpu_device->id != 0) {
		set_property(vpu_device, &vpu_drv->prop);
		ret = 0;
		goto exit;
	}

	img = read_img(vpu_device);
	if (!img) {
		ret = -EINVAL;
		pr_info("%s: read_img error: %d\n", __func__, ret);
		goto exit;
	}

	if (vpu_device->id == 0) {
		np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
		if (!np) {
			dev_err(vpu_device->dev, "No %s specified\n", "memory-region");
			goto exit;
		}
		rc = of_address_to_resource(np, 0, &r);
		if (rc) {
			dev_err(vpu_device->dev,
				"No memory address assigned to the region with error %d\n",
				rc);
			goto exit;
		}
		vpu_drv->bin_pa = r.start;
		vpu_drv->bin_va = memremap(r.start, resource_size(&r),
					   MEMREMAP_WB);
		//vpu_drv->bin_va = ioremap(vpu_drv->bin_pa, resource_size(&r));
		vpu_drv->bin_size = resource_size(&r);
		dev_info(vpu_device->dev,
			"Allocated reserved memory, vaddr: 0x%0llX, paddr: 0x%0llX, size:0x%x\n",
			(u64)vpu_drv->bin_va,
			vpu_drv->bin_pa,
			vpu_drv->bin_size);
	}

	//overlap = calloc((img->pre_cnt + 1), sizeof(int));
	overlap = kzalloc((img->pre_cnt + 1) * sizeof(int), GFP_KERNEL);
	extra_size = img->total_extra_sz;

	pr_info("extra_size: 0x%x\n", extra_size);

	mblock_size = (DSP_BOOT_SZ + IRAM_SEG_SZ) * img->dsp_num +
		img->total_alg_sz + img->hdr_size + extra_size + SPARE_SZ;

	aligned_size = ALIGN(mblock_size, ALIGN_64K);

	if (IS_ALIGNED(aligned_size, ALIGN_2M))
		aligned_size += ALIGN_64K;

	pr_info("aligned_size is 0x%llx.\n", aligned_size);
	mblock = (u64)vpu_drv->bin_va;

	if (!mblock || !overlap) {
		pr_err("vpu memory allocation failed.\n");
		ret = -ENOMEM;
		goto exit;
	}

	memset((void *)(mblock), 0, mblock_size);

	alg_offset = img->dsp_num * DSP_BOOT_SZ;
	iram_off = alg_offset + img->total_alg_sz;
	offset = mblock_size - img->hdr_size - extra_size;
	head_dst = mblock + offset;
	pre_dst = ALIGN(mblock + mblock_size, ALIGN_64K) - extra_size;

	pr_info("lk ver: %x\n", VERSION);
	pr_info("mblk:[0x%x:0x%x] | head:0x%x | alg: 0x%x | pre: 0x%x\n",
		mblock, mblock_size, head_dst, mblock + alg_offset,
		pre_dst);

	pIramNum  = (u32 *)(mblock + iram_off);
	*pIramNum = 0;
	pIramNum  = (u32 *)(mblock + iram_off + IRAM_SEG_SZ);
	*pIramNum = 0;
	pIramNum  = (u32 *)(mblock + iram_off + (IRAM_SEG_SZ << 1));
	*pIramNum = 0;

	num = img->total_num;

	memset(&params, 0, sizeof(struct install_params));
	memset(&prop, 0, sizeof(struct prop_params));
	overlap[0] = 1;
	params.mblock = mblock;
	params.iram_off = iram_off;
	params.pre_dst = pre_dst;
	params.alg_dst = mblock + alg_offset;
	params.overlap = overlap;

	memset(&prop, 0, sizeof(struct prop_params));

	prop.dsp_num = img->dsp_num;
	prop.iram_off = iram_off;
	prop.alg_offset = alg_offset;
	prop.total_alg_sz = img->total_alg_sz;
	prop.mblock = mblock;
	prop.aligned_size = aligned_size;
	prop.head_offset = offset;
	prop.preload_offset = pre_dst - mblock;

	vpu_drv->bin_head_ofs = offset;
	vpu_drv->bin_preload_ofs = pre_dst - mblock;
	vpu_drv->prop = prop;

	pr_info("%s: mapped vpu firmware: pa: 0x%lx, size: 0x%x, kva: 0x%lx\n",
		__func__, vpu_drv->bin_pa, vpu_drv->bin_size,
		(unsigned long)vpu_drv->bin_va);
	pr_info("%s: header: 0x%x, preload:0x%x\n", __func__,
		vpu_drv->bin_head_ofs, vpu_drv->bin_preload_ofs);

	ret = set_property(vpu_device, &prop);
	if (ret < 0)
		goto exit;


	for (i = 0 ; i < num ; i++) {
		params.img_max = img[i].img_max;
		params.name = img[i].name;
		head = params.part_hdr = img[i].head;

		pr_info("%s| v:%x str:%s sz:0x%x/0x%x s:%d a:%d v:%d\n",
			params.name, head->version, (char *)head->hdr_str,
			head->hdr_size, head->mem_size, head->segment_num,
			head->alg_num, head->pre_num);

		ret = install_main(&params);
		if (ret)
			goto exit;

		ret = install_preload(&params);
		if (ret)
			goto exit;

		ret = install_algo(&params);
		if (ret)
			goto exit;

		pr_info("copy %s header(0x%X) to 0x%x\n",
			params.name, head->hdr_size, head_dst);
		memcpy((void *)head_dst, head, head->hdr_size);
		head_dst += head->hdr_size;
	}

	if (overlap[0] - 1) {
		if (check_overlap(&overlap[1], overlap[0] - 1))
			pr_err("preload overlap\n");
	}

exit:
	kfree(img);
	kfree(overlap);
	for (i = 0 ; i < num ; i++)
		if (vpu_drv->fw[i])
			release_firmware(vpu_drv->fw[i]);

	return ret;
}

