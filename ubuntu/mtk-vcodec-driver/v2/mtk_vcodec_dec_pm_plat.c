// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Tiffany Lin <tiffany.lin@mediatek.com>
 */

#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <soc/mediatek/smi.h>
#include <linux/slab.h>
#include "mtk_vcodec_dec_pm.h"
#include "mtk_vcodec_dec_pm_plat.h"
#include "mtk_vcodec_util.h"
#include "mtk_vcu.h"

#if DEC_DVFS
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#define STD_VDEC_FREQ 218
#endif

#define VDEC_DRV_UFO_ON (1 << 0)
#define VDEC_DRV_UFO_AUO_ON (1 << 1)
#if DEC_EMI_BW
#include "mtk-interconnect.h"
#endif

void mtk_prepare_vdec_dvfs(struct mtk_vcodec_dev *dev)
{
#if DEC_DVFS
	int ret;
	struct dev_pm_opp *opp = 0;
	unsigned long freq = 0;
	int i = 0;

	ret = dev_pm_opp_of_add_table(&dev->plat_dev->dev);
	if (ret < 0) {
		mtk_v4l2_err("Failed to get opp table (%d)\n", ret);
		return;
	}

	dev->vdec_reg = devm_regulator_get(&dev->plat_dev->dev,
						"dvfsrc-vcore");
	if (IS_ERR(dev->vdec_reg)) {
		mtk_v4l2_err("%s, %d Failed to get regulator\n",
			__func__, __LINE__);
		return;
	}

	dev->vdec_freq_cnt = dev_pm_opp_get_opp_count(&dev->plat_dev->dev);
	dev->vdec_freqs = kcalloc(
		dev->vdec_freq_cnt, sizeof(unsigned long), GFP_KERNEL);
	freq = 0;
	while (!IS_ERR(opp =
		dev_pm_opp_find_freq_ceil(&dev->plat_dev->dev, &freq))) {
		dev->vdec_freqs[i] = freq;
		mtk_v4l2_debug(1, "i:%d, freq:%lu\n", i, freq);
		freq++;
		i++;
		dev_pm_opp_put(opp);
	}
#endif
}

void mtk_unprepare_vdec_dvfs(struct mtk_vcodec_dev *dev)
{
#if DEC_DVFS
	kfree(dev->vdec_freqs);
#endif
}

void mtk_prepare_vdec_emi_bw(struct mtk_vcodec_dev *dev)
{
#if DEC_EMI_BW
	int i = 0;
	struct platform_device *pdev = 0;

	pdev = dev->plat_dev;
	for (i = 0; i < MTK_VDEC_PORT_NUM; i++)
		dev->vdec_qos_req[i] = 0;

	i = 0;
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_mc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_ufo");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_pp");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_pred_rd");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_pred_wr");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_ppwrap");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_tile");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_vld");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_vld2");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_avc_mv");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l21_ufo_ext_c");//10

	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_vld");//11
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_vld2");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_avc_mc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_pred_rd");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_tile");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_wdma");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_ufo_enc");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_ufo_enc_ext_c");
	dev->vdec_qos_req[i++] = of_mtk_icc_get(&pdev->dev, "l23_lat0_mc"); //19
	for (i = 0; i < MTK_VDEC_PORT_NUM; i++) {
		mtk_v4l2_debug(3, "%s:%d, vdec_qos_req[%d]:%pK\n",
			__func__, __LINE__, i, dev->vdec_qos_req[i]);
	}
#endif
}

void mtk_unprepare_vdec_emi_bw(struct mtk_vcodec_dev *dev)
{
#if DEC_EMI_BW
#endif
}

#if DEC_DVFS
#define MTK_VDEC_DVFS_THRESHOLD      (16)
#define MTK_VDEC_DVFS_AREAS_1080P30     (1920 * 1088 * 30)
#define MTK_VDEC_DVFS_AREAS_1080P60     (1920 * 1088 * 60)
#define MTK_VDEC_DVFS_AREAS_1440P60     (2560 * 1440 * 60)
#define MTK_VDEC_DVFS_AREAS_2160P60     (3840 * 2160 * 60)
#if VDEC_POWER_CLOCK_IMPROVEMENT
bool vdec_decode_check_need_power_always_on(struct mtk_vcodec_ctx *ctx)
{
	bool check_value = FALSE;

	if (ctx->dev->dec_cnt >= MTK_VDEC_DVFS_THRESHOLD)
		check_value = TRUE;
	else
		check_value = FALSE;

	return check_value;
}
#endif
u64 mtk_vdec_dvfs_get_freq(struct mtk_vcodec_ctx *ctx)
{
	struct mtk_vcodec_ctx *ctx_interator;
	unsigned int area = 0;
	u64 freq_vdec = 218;

	mutex_lock(&ctx->dev->dec_dvfs_inst_mutex);
	list_for_each_entry(ctx_interator, &ctx->dev->ctx_list, list) {
		area += ctx_interator->q_data[MTK_Q_DATA_DST].coded_width *
			ctx_interator->q_data[MTK_Q_DATA_DST].coded_height *
			ctx_interator->frame_rate;
	}
#if VDEC_POWER_CLOCK_IMPROVEMENT
	if (vdec_decode_check_need_power_always_on(ctx))
		area = MTK_VDEC_DVFS_AREAS_2160P60;
#endif
	mutex_unlock(&ctx->dev->dec_dvfs_inst_mutex);

	if (area <= MTK_VDEC_DVFS_AREAS_1080P30)
		freq_vdec = 218;
	else if (area <= MTK_VDEC_DVFS_AREAS_1080P60)
		freq_vdec = 249;
	else if (area <= MTK_VDEC_DVFS_AREAS_1440P60)
		freq_vdec = 416;
	else
		freq_vdec = 594;

	return freq_vdec;
}

static unsigned long mtk_vdec_get_max_freq(struct mtk_vcodec_ctx *ctx)
{
	unsigned long max_freq = 0;
	int idx = 0;

	for (idx = 0; idx < MAX_VDEC_HW; idx++) {
		if (max_freq < ctx->dev->vdec_freq.freq[idx])
			max_freq = ctx->dev->vdec_freq.freq[idx];
	}

	return max_freq * ONE_MHZ;//convert to HZ
}
#endif

void mtk_vdec_dvfs_begin(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_DVFS
	struct dev_pm_opp *opp = NULL;
	struct platform_device *pdev = NULL;
	int low_volt = 0;
	int ret;

	pdev = ctx->dev->plat_dev;

	mutex_lock(&ctx->dev->dec_dvfs_mutex);
	ctx->dev->vdec_freq.freq[hw_id] = mtk_vdec_dvfs_get_freq(ctx);

	ctx->dev->vdec_freq.active_freq = mtk_vdec_get_max_freq(ctx);

	opp = dev_pm_opp_find_freq_ceil(&pdev->dev,
		&ctx->dev->vdec_freq.active_freq);
	if (!IS_ERR(opp))
		opp = dev_pm_opp_find_freq_floor(&pdev->dev,
			&ctx->dev->vdec_freq.active_freq);
	else {
		mtk_v4l2_debug(1, "%s:%d, get opp fail\n",
			__func__, __LINE__);
		mutex_unlock(&ctx->dev->dec_dvfs_mutex);
		return;
	}
	low_volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	mtk_v4l2_debug(2, "%s:%d, vdec_freq:%ld, low_volt:%d\n",
		__func__, __LINE__,
		ctx->dev->vdec_freq.active_freq,
		low_volt);
	if (ctx->dev->vdec_reg != 0) {
		ret = regulator_set_voltage(ctx->dev->vdec_reg,
			low_volt, INT_MAX);
		if (ret)
			mtk_v4l2_debug(0, "%s:%d, set voltage failed\n",
				__func__, __LINE__);
	}
	mutex_unlock(&ctx->dev->dec_dvfs_mutex);
#endif
}

void mtk_vdec_dvfs_end(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_DVFS
	struct dev_pm_opp *opp = NULL;
	struct platform_device *pdev = NULL;
	int low_volt = 0;
	int ret;

	pdev = ctx->dev->plat_dev;

	mutex_lock(&ctx->dev->dec_dvfs_mutex);
	ctx->dev->vdec_freq.freq[hw_id] = 218;

	ctx->dev->vdec_freq.active_freq = mtk_vdec_get_max_freq(ctx);

	opp = dev_pm_opp_find_freq_ceil(&pdev->dev,
		&ctx->dev->vdec_freq.active_freq);
	if (!IS_ERR(opp))
		opp = dev_pm_opp_find_freq_floor(&pdev->dev,
			&ctx->dev->vdec_freq.active_freq);
	else {
		mtk_v4l2_debug(1, "%s:%d, get opp fail\n",
			__func__, __LINE__);
		mutex_unlock(&ctx->dev->dec_dvfs_mutex);
		return;
	}
	low_volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	mtk_v4l2_debug(2, "%s:%d, hw_id %d vdec_freq:%ld, low_volt:%d\n",
		__func__, __LINE__,
		hw_id,
		ctx->dev->vdec_freq.active_freq,
		low_volt);
	if (ctx->dev->vdec_reg != 0) {
		ret = regulator_set_voltage(ctx->dev->vdec_reg,
			low_volt, INT_MAX);
		if (ret)
			mtk_v4l2_debug(0, "%s:%d, set voltage failed\n",
				__func__, __LINE__);
	}
	mutex_unlock(&ctx->dev->dec_dvfs_mutex);
#endif
}

void mtk_vdec_emi_bw_begin(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_EMI_BW
	long emi_bw = 0;
	long emi_bw_input = 0;
	long emi_bw_output = 0;
	struct mtk_vcodec_dev *pdev = 0;
	unsigned int    coded_width = 0;
	unsigned int    coded_height = 0;
	unsigned int    larb21_bw;
	unsigned int    larb23_bw;

	larb21_bw = larb23_bw = 0;
	pdev = ctx->dev;
	coded_width = ctx->q_data[MTK_Q_DATA_DST].coded_width;
	coded_height = ctx->q_data[MTK_Q_DATA_DST].coded_height;

	if (hw_id == MTK_VDEC_LAT) {
		switch (ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc) {
		case V4L2_PIX_FMT_H264:
		case V4L2_PIX_FMT_HEVC:
			emi_bw_input = 31L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			break;
		case V4L2_PIX_FMT_VP9:
		case V4L2_PIX_FMT_AV1:
			emi_bw_input = 13L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			break;
		default:
			emi_bw_input = 31L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
		}

		if (pdev->vdec_qos_req[11] != 0) {
			//lat
			larb23_bw = emi_bw_input * 5 + 10;
			mtk_icc_set_bw(pdev->vdec_qos_req[11],
					MBps_to_icc(larb23_bw), 0);
		}
	} else if (hw_id == MTK_VDEC_CORE) {
		emi_bw = 8L * 1920 * 1088 * 9 * 9 * 5 *
			pdev->vdec_freq.active_freq / 1000000 / 2 / 3;
		emi_bw_output = 1920L * 1088 * 9 * 30 * 9 * 5 *
				pdev->vdec_freq.active_freq / 1000000 /
				4 / 3 / 3 / STD_VDEC_FREQ / 1024 / 1024;

		switch (ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc) {
		case V4L2_PIX_FMT_H264:
			emi_bw_input = 62L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_HEVC:
			emi_bw_input = 62L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_VP8:
			emi_bw_input = 13L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_VP9:
			emi_bw_input = 26L * pdev->vdec_freq.active_freq /
					1000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_AV1:
			emi_bw_input = 26L * pdev->vdec_freq.active_freq /
					10000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 24 /
					(2 * STD_VDEC_FREQ);
			break;
		case V4L2_PIX_FMT_MPEG4:
		case V4L2_PIX_FMT_H263:
		case V4L2_PIX_FMT_MPEG1:
		case V4L2_PIX_FMT_MPEG2:
			emi_bw_input = 13L * pdev->vdec_freq.active_freq /
					10000000 / STD_VDEC_FREQ;
			emi_bw = emi_bw * 20 /
					(2 * STD_VDEC_FREQ);
			break;
		}
		emi_bw = emi_bw / (1024 * 1024) / 8;
		emi_bw = emi_bw - emi_bw_output - emi_bw_input;
		if (emi_bw < 0)
			emi_bw = 0;

		if (pdev->vdec_qos_req[0] != 0) {
			if (ctx->picinfo.layout_mode & VDEC_DRV_UFO_AUO_ON ||
				ctx->picinfo.layout_mode & VDEC_DRV_UFO_ON) {
				larb21_bw += emi_bw * 2;
				larb23_bw += emi_bw;
			} else {
				larb21_bw += emi_bw * 2;
			}
			larb21_bw += 1;
			if ((ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc ==
				V4L2_PIX_FMT_AV1) ||
				(ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc ==
				V4L2_PIX_FMT_VP9)) {
				larb21_bw += emi_bw;
			} else {
				larb21_bw += 1;
			}
			larb21_bw += emi_bw_input * 2;
			if (pdev->vdec_qos_req[0] != 0) {
				mtk_icc_set_bw(pdev->vdec_qos_req[0],
						MBps_to_icc(larb21_bw), 0);
				if (pdev->vdec_qos_req[11] != 0) {
					mtk_icc_set_bw(pdev->vdec_qos_req[11],
							MBps_to_icc(larb23_bw), 0);
				}
			}
		}
	} else {
		pr_debug("%s unknown hw_id %d\n", __func__, hw_id);
	}
#endif
}

void mtk_vdec_emi_bw_end(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_EMI_BW
	if (hw_id == MTK_VDEC_LAT) {
		mtk_icc_set_bw(ctx->dev->vdec_qos_req[11],
			MBps_to_icc(0), 0);
	} else if (hw_id == MTK_VDEC_CORE) {
		mtk_icc_set_bw(ctx->dev->vdec_qos_req[0],
			MBps_to_icc(0), 0);
	}
#endif
}

void mtk_vdec_pmqos_prelock(struct mtk_vcodec_ctx *ctx, int hw_id)
{
#if DEC_DVFS
	mutex_lock(&ctx->dev->dec_dvfs_mutex);
	/* add_job(&ctx->id, &vdec_jobs); */
	mutex_unlock(&ctx->dev->dec_dvfs_mutex);
#endif
}

void mtk_vdec_pmqos_begin_frame(struct mtk_vcodec_ctx *ctx, int hw_id)
{
	mtk_vdec_dvfs_begin(ctx, hw_id);
	mtk_vdec_emi_bw_begin(ctx, hw_id);
}

void mtk_vdec_pmqos_end_frame(struct mtk_vcodec_ctx *ctx, int hw_id)
{
	mtk_vdec_dvfs_end(ctx, hw_id);
	mtk_vdec_emi_bw_end(ctx, hw_id);
}

