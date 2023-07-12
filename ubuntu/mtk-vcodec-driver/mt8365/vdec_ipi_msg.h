/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _VDEC_IPI_MSG_H_
#define _VDEC_IPI_MSG_H_

#include "vdec_vcu_if.h"

/**
 * enum vdec_ipi_msgid - message id between AP and VCU
 * @AP_IPIMSG_XXX	: AP to VCU cmd message id
 * @VCU_IPIMSG_XXX_ACK	: VCU ack AP cmd message id
 */
enum vdec_ipi_msgid {
	AP_IPIMSG_DEC_INIT = 0xA000,
	AP_IPIMSG_DEC_START = 0xA001,
	AP_IPIMSG_DEC_END = 0xA002,
	AP_IPIMSG_DEC_DEINIT = 0xA003,
	AP_IPIMSG_DEC_RESET = 0xA004,
	AP_IPIMSG_DEC_SET_PARAM = 0xA005,

	VCU_IPIMSG_DEC_INIT_ACK = 0xB000,
	VCU_IPIMSG_DEC_START_ACK = 0xB001,
	VCU_IPIMSG_DEC_END_ACK = 0xB002,
	VCU_IPIMSG_DEC_DEINIT_ACK = 0xB003,
	VCU_IPIMSG_DEC_RESET_ACK = 0xB004,
	VCU_IPIMSG_DEC_SET_PARAM_ACK = 0xB005,

	VCU_IPIMSG_DEC_WAITISR = 0xC000,
	VCU_IPIMSG_DEC_GET_FRAME_BUFFER = 0xC001,
	VCU_IPIMSG_DEC_CLOCK_ON = 0xC002,
	VCU_IPIMSG_DEC_CLOCK_OFF = 0xC003
};

/**
 * struct vdec_ap_ipi_cmd - generic AP to VCU ipi command format
 * @msg_id	: vdec_ipi_msgid
 * @vcu_inst_addr	: VCU decoder instance address
 */
struct vdec_ap_ipi_cmd {
	uint32_t msg_id;
	uint32_t reserved;
	uint64_t vcu_inst_addr;
};

/**
 * struct vdec_vpu_ipi_ack - generic VPU to AP ipi command format
 * @msg_id	: vdec_ipi_msgid
 * @status	: VPU exeuction result
 * @ap_inst_addr	: AP video decoder instance address
 */
struct vdec_vcu_ipi_ack {
	uint32_t msg_id;
	int32_t status;
	uint64_t ap_inst_addr;
};

/**
 * struct vdec_ap_ipi_init - for AP_IPIMSG_DEC_INIT
 * @msg_id	: AP_IPIMSG_DEC_INIT
 * @reserved	: Reserved field
 * @ap_inst_addr	: AP video decoder instance address
 */
struct vdec_ap_ipi_init {
	uint32_t msg_id;
	uint32_t reserved;
	uint64_t ap_inst_addr;
};

/**
 * struct vdec_ap_ipi_dec_start - for AP_IPIMSG_DEC_START
 * @msg_id	: AP_IPIMSG_DEC_START
 * @vcu_inst_addr	: VCU decoder instance address
 * @data	: Header info
 * @reserved	: Reserved field
 */
struct vdec_ap_ipi_dec_start {
	uint32_t msg_id;
	uint32_t reserved0;
	uint64_t vcu_inst_addr;
	uint32_t data[3];
	uint32_t reserved;
};

/**
 * struct vdec_ap_ipi_set_param - for AP_IPIMSG_DEC_SET_PARAM
 * @msg_id        : AP_IPIMSG_DEC_SET_PARAM
 * @vcu_inst_addr : VCU decoder instance address
 * @id            : set param  type
 * @data          : param data
 */
struct vdec_ap_ipi_set_param {
	uint32_t msg_id;
	uint32_t reserved;
	uint64_t vcu_inst_addr;
	uint32_t id;
	uint32_t data[4];
};

/**
 * struct vdec_vcu_ipi_init_ack - for VCU_IPIMSG_DEC_INIT_ACK
 * @msg_id        : VCU_IPIMSG_DEC_INIT_ACK
 * @status        : VCU exeuction result
 * @ap_inst_addr	: AP vcodec_vcu_inst instance address
 * @vcu_inst_addr : VCU decoder instance address
 */
struct vdec_vcu_ipi_init_ack {
	uint32_t msg_id;
	int32_t status;
	uint64_t ap_inst_addr;
	uint64_t vcu_inst_addr;
};

#define DEC_MAX_FB_NUM				32U

/**
 * struct vdec_fb - vdec decode frame buffer information
 * @vdec_fb_va  : virtual address of struct vdec_fb
 * @y_fb_dma    : dma address of Y frame buffer
 * @c_fb_dma    : dma address of C frame buffer
 * @poc         : picture order count of frame buffer
 * @reserved    : for 8 bytes alignment
 */
struct dec_fb {
	uint64_t vdec_fb_va;
	uint64_t y_fb_dma;
	uint64_t c_fb_dma;
	int32_t poc;
	uint32_t reserved;
};

/**
 * struct ring_fb_list - ring frame buffer list
 * @fb_list   : frame buffer arrary
 * @read_idx  : read index
 * @write_idx : write index
 * @count     : buffer count in list
 */
struct ring_fb_list {
	struct dec_fb fb_list[DEC_MAX_FB_NUM];
	unsigned int read_idx;
	unsigned int write_idx;
	unsigned int count;
	unsigned int reserved;
};

/**
 * struct vdec_dec_info - decode information
 * @dpb_sz		: decoding picture buffer size
 * @vdec_changed_info  : some changed flags
 * @bs_dma		: Input bit-stream buffer dma address
 * @bs_fd               : Input bit-stream buffer dmabuf fd
 * @fb_dma		: Y frame buffer dma address
 * @fb_fd             : Y frame buffer dmabuf fd
 * @vdec_fb_va		: VDEC frame buffer struct virtual address
 * @fb_num_planes	: frame buffer plane count
 * @reserved		: reserved variable for 64bit align
 */
struct vdec_dec_info {
	uint32_t dpb_sz;
	uint32_t vdec_changed_info;
	uint64_t bs_dma;
	uint64_t bs_fd;
	uint64_t fb_dma[VIDEO_MAX_PLANES];
	uint64_t fb_fd[VIDEO_MAX_PLANES];
	uint64_t vdec_fb_va;
	uint32_t fb_num_planes;
	uint32_t index;
};

/**
 * struct vdec_vsi - shared memory for decode information exchange
 *                        between VCU and Host.
 *                        The memory is allocated by VCU and mapping to Host
 *                        in vcu_dec_init()
 * @ppl_buf_dma : HW working buffer ppl dma address
 * @mv_buf_dma  : HW working buffer mv dma address
 * @list_free   : free frame buffer ring list
 * @list_disp   : display frame buffer ring list
 * @dec		: decode information
 * @pic		: picture information
 * @crop        : crop information
 */
struct vdec_vsi {
	struct ring_fb_list list_free;
	struct ring_fb_list list_disp;
	struct vdec_dec_info dec;
	struct vdec_pic_info pic;
	struct mtk_color_desc color_desc;
	struct v4l2_rect crop;
	char crc_path[256];
	char golden_path[256];
};

/**
 * struct vdec_inst - decoder instance
 * @num_nalu : how many nalus be decoded
 * @ctx      : point to mtk_vcodec_ctx
 * @vcu      : VCU instance
 * @vsi      : VCU shared information
 */
struct vdec_inst {
	unsigned int num_nalu;
	struct mtk_vcodec_ctx *ctx;
	struct vdec_vcu_inst vcu;
	struct vdec_vsi *vsi;
};


#endif
