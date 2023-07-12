/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Jungchang Tsao <jungchang.tsao@mediatek.com>
 *	   Daniel Hsiao <daniel.hsiao@mediatek.com>
 *	   Tiffany Lin <tiffany.lin@mediatek.com>
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

#ifndef _VENC_IPI_MSG_H_
#define _VENC_IPI_MSG_H_

#include "venc_vcu_if.h"

#define AP_IPIMSG_VENC_BASE 0xC000
#define VCU_IPIMSG_VENC_BASE 0xD000

/**
 * enum venc_ipi_msg_id - message id between AP and VCU
 * (ipi stands for inter-processor interrupt)
 * @AP_IPIMSG_ENC_XXX:		AP to VCU cmd message id
 * @VCU_IPIMSG_ENC_XXX_DONE:	VCU ack AP cmd message id
 */
enum venc_ipi_msg_id {
	AP_IPIMSG_ENC_INIT = AP_IPIMSG_VENC_BASE,
	AP_IPIMSG_ENC_SET_PARAM,
	AP_IPIMSG_ENC_ENCODE,
	AP_IPIMSG_ENC_DEINIT,

	VCU_IPIMSG_ENC_INIT_DONE = VCU_IPIMSG_VENC_BASE,
	VCU_IPIMSG_ENC_SET_PARAM_DONE,
	VCU_IPIMSG_ENC_ENCODE_DONE,
	VCU_IPIMSG_ENC_DEINIT_DONE,
	VCU_IPIMSG_ENC_POWER_ON,
	VCU_IPIMSG_ENC_POWER_OFF,
	VCU_IPIMSG_ENC_WAIT_ISR
};

/**
 * struct venc_ap_ipi_msg_init - AP to VCU init cmd structure
 * @msg_id:	message id (AP_IPIMSG_XXX_ENC_INIT)
 * @reserved:	reserved for future use. vcu is running in 32bit. Without
 *		this reserved field, if kernel run in 64bit. this struct size
 *		will be different between kernel and vcu
 * @venc_inst:	AP encoder instance
 *		(struct venc_vp8_inst/venc_h264_inst *)
 */
struct venc_ap_ipi_msg_init {
	uint32_t msg_id;
	uint32_t reserved;
	uint64_t venc_inst;
};

/**
 * struct venc_ap_ipi_msg_set_param - AP to VCU set_param cmd structure
 * @msg_id:	message id (AP_IPIMSG_XXX_ENC_SET_PARAM)
 * @vcu_inst_addr:	VCU encoder instance addr
 *			(struct venc_vp8_vsi/venc_h264_vsi *)
 * @param_id:	parameter id (venc_set_param_type)
 * @data_item:	number of items in the data array
 * @data[8]:	data array to store the set parameters
 */
struct venc_ap_ipi_msg_set_param {
	uint32_t msg_id;
	uint32_t reserved;
	uint64_t vcu_inst_addr;
	uint32_t param_id;
	uint32_t data_item;
	uint32_t data[4];
};

/**
 * struct venc_ap_ipi_msg_enc - AP to VCU enc cmd structure
 * @msg_id:	message id (AP_IPIMSG_XXX_ENC_ENCODE)
 * @vcu_inst_addr:	VCU encoder instance addr
 *			(struct venc_vp8_vsi/venc_h264_vsi *)
 * @bs_mode:	bitstream mode for h264
 *		(H264_BS_MODE_SPS/H264_BS_MODE_PPS/H264_BS_MODE_FRAME)
 * @input_addr:	pointer to input image buffer plane
 * @input_size:	image buffer size
 * @bs_addr:	pointer to output bit stream buffer
 * @bs_size:	bit stream buffer size
 * @fb_num_planes:	image buffer plane number
 */
struct venc_ap_ipi_msg_enc {
	uint32_t msg_id;
	unsigned char bs_mode; /* driver bs mode */
	unsigned char  fb_num_planes; /* image buffer plane number */
	uint64_t vcu_inst_addr;
	uint32_t input_addr[3];
	uint32_t input_size[3];
	uint32_t bs_addr;
	uint32_t bs_size;
};

/**
 * struct venc_ap_ipi_msg_deinit - AP to VCU deinit cmd structure
 * @msg_id:	message id (AP_IPIMSG_XXX_ENC_DEINIT)
 * @vcu_inst_addr:	VCU encoder instance addr
 *			(struct venc_vp8_vsi/venc_h264_vsi *)
 */
struct venc_ap_ipi_msg_deinit {
	uint32_t msg_id;
	uint32_t reserved;
	uint64_t vcu_inst_addr;

};

/**
 * enum venc_ipi_msg_status - VCU ack AP cmd status
 */
enum venc_ipi_msg_status {
	VENC_IPI_MSG_STATUS_OK,
	VENC_IPI_MSG_STATUS_FAIL,
};

/**
 * struct venc_vcu_ipi_msg_common - VCU ack AP cmd common structure
 * @msg_id:	message id (VCU_IPIMSG_XXX_DONE)
 * @status:	cmd status (venc_ipi_msg_status)
 * @venc_inst:	AP encoder instance (struct venc_vp8_inst/venc_h264_inst *)
 */
struct venc_vcu_ipi_msg_common {
	uint32_t msg_id;
	uint32_t status;
	uint64_t venc_inst;
};

/**
 * struct venc_vcu_ipi_msg_init - VCU ack AP init cmd structure
 * @msg_id:	message id (VCU_IPIMSG_XXX_ENC_SET_PARAM_DONE)
 * @status:	cmd status (venc_ipi_msg_status)
 * @venc_inst:	AP encoder instance (struct venc_vp8_inst/venc_h264_inst *)
 * @vcu_inst_addr:	VCU encoder instance addr
 *			(struct venc_vp8_vsi/venc_h264_vsi *)
 * @reserved:	reserved for future use. vcu is running in 32bit. Without
 *		this reserved field, if kernel run in 64bit. this struct size
 *		will be different between kernel and vcu
 */
struct venc_vcu_ipi_msg_init {
	uint32_t msg_id;
	uint32_t status;
	uint64_t venc_inst;
	uint64_t vcu_inst_addr;
	uint32_t reserved;
};

/**
 * struct venc_vcu_ipi_msg_set_param - VCU ack AP set_param cmd structure
 * @msg_id:	message id (VCU_IPIMSG_XXX_ENC_SET_PARAM_DONE)
 * @status:	cmd status (venc_ipi_msg_status)
 * @venc_inst:	AP encoder instance (struct venc_vp8_inst/venc_h264_inst *)
 * @param_id:	parameter id (venc_set_param_type)
 * @data_item:	number of items in the data array
 * @data[6]:	data array to store the return result
 */
struct venc_vcu_ipi_msg_set_param {
	uint32_t msg_id;
	uint32_t status;
	uint64_t venc_inst;
	uint32_t param_id;
	uint32_t data_item;
	uint32_t data[6];
};

/**
 * enum venc_ipi_msg_enc_state - Type of encode state
 * VEN_IPI_MSG_ENC_STATE_FRAME:	one frame being encoded
 * VEN_IPI_MSG_ENC_STATE_PART:	bit stream buffer full
 * VEN_IPI_MSG_ENC_STATE_SKIP:	encoded skip frame
 * VEN_IPI_MSG_ENC_STATE_ERROR:	encounter error
 */
enum venc_ipi_msg_enc_state {
	VEN_IPI_MSG_ENC_STATE_FRAME,
	VEN_IPI_MSG_ENC_STATE_PART,
	VEN_IPI_MSG_ENC_STATE_SKIP,
	VEN_IPI_MSG_ENC_STATE_ERROR,
};

/**
 * struct venc_vcu_ipi_msg_enc - VCU ack AP enc cmd structure
 * @msg_id:	message id (VCU_IPIMSG_XXX_ENC_ENCODE_DONE)
 * @status:	cmd status (venc_ipi_msg_status)
 * @venc_inst:	AP encoder instance (struct venc_vp8_inst/venc_h264_inst *)
 * @state:	encode state (venc_ipi_msg_enc_state)
 * @is_key_frm:	whether the encoded frame is key frame
 * @bs_size:	encoded bitstream size
 * @reserved:	reserved for future use. vcu is running in 32bit. Without
 *		this reserved field, if kernel run in 64bit. this struct size
 *		will be different between kernel and vcu
 */
struct venc_vcu_ipi_msg_enc {
	uint32_t msg_id;
	uint32_t status;
	uint64_t venc_inst;
	uint32_t state;
	uint32_t is_key_frm;
	uint32_t bs_size;
	uint32_t reserved;
};

/**
 * struct venc_vcu_ipi_msg_deinit - VCU ack AP deinit cmd structure
 * @msg_id:   message id (VCU_IPIMSG_XXX_ENC_DEINIT_DONE)
 * @status:   cmd status (venc_ipi_msg_status)
 * @venc_inst:	AP encoder instance (struct venc_vp8_inst/venc_h264_inst *)
 */
struct venc_vcu_ipi_msg_deinit {
	uint32_t msg_id;
	uint32_t status;
	uint64_t venc_inst;
};

/**
 * struct venc_vcu_ipi_msg_waitisr - VCU ack AP wait isr cmd structure
 * @msg_id:   message id (VCU_IPIMSG_XXX_ENC_DEINIT_DONE)
 * @status:   cmd status (venc_ipi_msg_status)
 * @venc_inst:	AP encoder instance (struct venc_vp8_inst/venc_h264_inst *)
 * @irq_status: encoder irq status
 * @timeout: 1 indicate encode timeout, 0 indicate no error
 */
struct venc_vcu_ipi_msg_waitisr {
	uint32_t msg_id;
	uint32_t status;
	uint64_t venc_inst;
	uint32_t irq_status;
	uint32_t timeout;
};


/*
 * struct venc_vcu_config - Structure for encoder configuration
 *                               AP-W/R : AP is writer/reader on this item
 *                               VCU-W/R: VCU is write/reader on this item
 * @input_fourcc: input fourcc
 * @bitrate: target bitrate (in bps)
 * @pic_w: picture width. Picture size is visible stream resolution, in pixels,
 *         to be used for display purposes; must be smaller or equal to buffer
 *         size.
 * @pic_h: picture height
 * @buf_w: buffer width. Buffer size is stream resolution in pixels aligned to
 *         hardware requirements.
 * @buf_h: buffer height
 * @gop_size: group of picture size (idr frame)
 * @intra_period: intra frame period
 * @framerate: frame rate in fps
 * @profile: as specified in standard
 * @level: as specified in standard
 * @wfd: WFD mode 1:on, 0:off
 */
struct venc_vcu_config {
	u32 input_fourcc;
	u32 bitrate;
	u32 pic_w;
	u32 pic_h;
	u32 buf_w;
	u32 buf_h;
	u32 gop_size;
	u32 intra_period;
	u32 framerate;
	u32 profile;
	u32 level;
	u32 wfd;
	u32 scenario;
};

/*
 * struct venc_vsi - Structure for VCU driver control and info share
 *                        AP-W/R : AP is writer/reader on this item
 *                        VCU-W/R: VCU is write/reader on this item
 * This structure is allocated in VCU side and shared to AP side.
 * @config: h264 encoder configuration
 * @work_bufs: working buffer information in VCU side
 * The work_bufs here is for storing the 'size' info shared to AP side.
 * The similar item in struct venc_inst is for memory allocation
 * in AP side. The AP driver will copy the 'size' from here to the one in
 * struct mtk_vcodec_mem, then invoke mtk_vcodec_mem_alloc to allocate
 * the buffer. After that, bypass the 'dma_addr' to the 'iova' field here for
 * register setting in VCU side.
 */
struct venc_vsi {
	struct venc_vcu_config config;
	unsigned int sizeimage[MTK_VCODEC_MAX_PLANES];
};

/*
 * struct venc_inst - encoder AP driver instance
 * @hw_base: encoder hardware register base
 * @work_bufs: working buffer
 * @pps_buf: buffer to store the pps bitstream
 * @work_buf_allocated: working buffer allocated flag
 * @frm_cnt: encoded frame count
 * @prepend_hdr: when the v4l2 layer send VENC_SET_PARAM_PREPEND_HEADER cmd
 *  through venc_set_param interface, it will set this flag and prepend the
 *  sps/pps in venc_encode function.
 * @vcu_inst: VCU instance to exchange information between AP and VCU
 * @vsi: driver structure allocated by VCU side and shared to AP side for
 *	 control and info share
 * @ctx: context for v4l2 layer integration
 */
struct venc_inst {
	void __iomem *hw_base;
	struct mtk_vcodec_mem pps_buf;
	bool work_buf_allocated;
	unsigned int frm_cnt;
	unsigned int prepend_hdr;
	struct venc_vcu_inst vcu_inst;
	struct venc_vsi *vsi;
	struct mtk_vcodec_ctx *ctx;
};

#endif /* _VENC_IPI_MSG_H_ */
