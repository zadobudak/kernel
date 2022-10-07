// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/rpmsg.h>

#include "apu_top.h"
#include "aputop_rpmsg.h"

struct aputop_rpmsg {
	struct rpmsg_endpoint *ept;
	struct rpmsg_device *rpdev;
	struct mutex send_lock;
	struct completion comp;
	enum aputop_rpmsg_cmd curr_rpmsg_cmd;
	int initialized;
};

static struct aputop_rpmsg *top_tx_rpmsg;
static struct aputop_rpmsg *top_rx_rpmsg;

enum aputop_rpmsg_cmd get_curr_tx_rpmsg_cmd(void)
{
	if (IS_ERR_OR_NULL(top_tx_rpmsg) || !top_tx_rpmsg->initialized) {
		pr_info("%s: rpmsg not ready yet\n", __func__);
		return APUTOP_RPMSG_CMD_MAX;
	}

	return top_tx_rpmsg->curr_rpmsg_cmd;
}

int aputop_send_tx_rpmsg(struct aputop_rpmsg_data *rpmsg_data, int timeout)
{
	int ret;

	if (IS_ERR_OR_NULL(top_tx_rpmsg) || !top_tx_rpmsg->initialized) {
		pr_info("%s: failed to send tx msg to remote side\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&top_tx_rpmsg->send_lock);

	reinit_completion(&top_tx_rpmsg->comp);

	top_tx_rpmsg->curr_rpmsg_cmd = rpmsg_data->cmd;
	ret = rpmsg_send(top_tx_rpmsg->ept, (void *)rpmsg_data,
			sizeof(struct aputop_rpmsg_data));
	if (ret) {
		pr_info("%s: failed to send tx msg to remote side, ret=%d\n",
				__func__, ret);
		goto unlock;
	}

	if (timeout > 0) {
		ret = wait_for_completion_interruptible_timeout(
				&top_tx_rpmsg->comp,
				msecs_to_jiffies(timeout));

		if (ret < 0) {
			pr_info("%s waiting for ack interrupted, ret : %d\n",
					__func__, ret);
			goto unlock;
		}

		if (ret == 0) {
			pr_info("%s waiting for ack timeout\n", __func__);
			ret = -ETIMEDOUT;
			goto unlock;
		}

		ret = 0;
	}

unlock:
	mutex_unlock(&top_tx_rpmsg->send_lock);

	return ret;
}

int aputop_send_rx_rpmsg(struct aputop_rpmsg_data *rpmsg_data)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(top_rx_rpmsg) || !top_rx_rpmsg->initialized) {
		pr_info("%s: failed to send rx msg to remote side\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&top_rx_rpmsg->send_lock);

	top_rx_rpmsg->curr_rpmsg_cmd = rpmsg_data->cmd;
	ret = rpmsg_send(top_rx_rpmsg->ept, (void *)rpmsg_data,
			sizeof(struct aputop_rpmsg_data));
	if (ret)
		pr_info("%s: failed to send rx msg to remote side, ret=%d\n",
				__func__, ret);
	mutex_unlock(&top_rx_rpmsg->send_lock);

	return 0;
}

/* receive tx-reply data from remote */
static int aputop_tx_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
		int len, void *priv, u32 src)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(top_tx_rpmsg) || !top_tx_rpmsg->initialized) {
		pr_info("%s: failed to send msg to remote side\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (pwr_data != NULL &&
		pwr_data->plat_tx_rpmsg_callback != NULL) {
		ret = pwr_data->plat_tx_rpmsg_callback(
					(int)top_tx_rpmsg->curr_rpmsg_cmd,
					data, len, priv, src);
	}
out:
	complete(&top_tx_rpmsg->comp);
	return ret;
}

/* receive rx-send data from remote */
static int aputop_rx_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
		int len, void *priv, u32 src)
{
	int ret = 0;
	struct aputop_rpmsg_data *rpmsg_data = (struct aputop_rpmsg_data *)data;
	enum aputop_rpmsg_cmd recv_cmd =
			(enum aputop_rpmsg_cmd)rpmsg_data->cmd;

	if (IS_ERR_OR_NULL(top_rx_rpmsg) || !top_rx_rpmsg->initialized) {
		pr_info("%s: failed to recv msg from remote side\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (pwr_data != NULL &&
		pwr_data->plat_rx_rpmsg_callback != NULL) {
		ret = pwr_data->plat_rx_rpmsg_callback((int)recv_cmd,
				data, len, priv, src);
	}

out:
	return ret;
}

void test_ipi_wakeup_apu(void)
{
	struct aputop_rpmsg_data rpmsg_data;

	pr_info("%s ++\n", __func__);
	memset(&rpmsg_data, 0, sizeof(struct aputop_rpmsg_data));
	rpmsg_data.cmd = APUTOP_CURR_STATUS;
	rpmsg_data.data0 = 0x0; /* pseudo data */
	aputop_send_tx_rpmsg(&rpmsg_data, 100);
	pr_info("%s --\n", __func__);
}

static int aputop_tx_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_channel_info chinfo = {};
	struct rpmsg_endpoint *ept;
	struct device *dev = &rpdev->dev;

	dev_info(dev, "%s: name=%s, src=%d\n", __func__,
			rpdev->id.name, rpdev->src);

	top_tx_rpmsg = devm_kzalloc(dev, sizeof(struct aputop_rpmsg),
			GFP_KERNEL);
	if (!top_tx_rpmsg)
		return -ENOMEM;

	strscpy(chinfo.name, rpdev->id.name, RPMSG_NAME_SIZE);
	chinfo.src = rpdev->src;
	chinfo.dst = RPMSG_ADDR_ANY;
	ept = rpmsg_create_ept(rpdev, aputop_tx_rpmsg_callback, NULL, chinfo);
	if (!ept) {
		dev_info(dev, "failed to create ept\n");
		return -ENODEV;
	}

	init_completion(&top_tx_rpmsg->comp);
	mutex_init(&top_tx_rpmsg->send_lock);
	top_tx_rpmsg->ept = ept;
	top_tx_rpmsg->rpdev = rpdev;
	top_tx_rpmsg->initialized = 1;

	dev_set_drvdata(dev, top_tx_rpmsg);

	return 0;
}

static int aputop_rx_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_channel_info chinfo = {};
	struct rpmsg_endpoint *ept;
	struct device *dev = &rpdev->dev;

	dev_info(dev, "%s: name=%s, src=%d\n", __func__,
			rpdev->id.name, rpdev->src);

	top_rx_rpmsg = devm_kzalloc(dev, sizeof(struct aputop_rpmsg),
			GFP_KERNEL);
	if (!top_rx_rpmsg)
		return -ENOMEM;

	strscpy(chinfo.name, rpdev->id.name, RPMSG_NAME_SIZE);
	chinfo.src = rpdev->src;
	chinfo.dst = RPMSG_ADDR_ANY;
	ept = rpmsg_create_ept(rpdev, aputop_rx_rpmsg_callback, NULL, chinfo);
	if (!ept) {
		dev_info(dev, "failed to create ept\n");
		return -ENODEV;
	}

	init_completion(&top_rx_rpmsg->comp);
	mutex_init(&top_rx_rpmsg->send_lock);
	top_rx_rpmsg->ept = ept;
	top_rx_rpmsg->rpdev = rpdev;
	top_rx_rpmsg->initialized = 1;

	dev_set_drvdata(dev, top_rx_rpmsg);

	return 0;
}

static void aputop_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct aputop_rpmsg *top_rpmsg;

	top_rpmsg = dev_get_drvdata(&rpdev->dev);
	rpmsg_destroy_ept(top_rpmsg->ept);
}

static const struct of_device_id aputop_tx_rpmsg_of_match[] = {
	{ .compatible = "mediatek,aputop-tx-rpmsg", },
	{ },
};

static const struct of_device_id aputop_rx_rpmsg_of_match[] = {
	{ .compatible = "mediatek,aputop-rx-rpmsg", },
	{ },
};

static struct rpmsg_driver aputop_tx_rpmsg_drv = {
	.drv	= {
		.name	= "apu_top_3_tx_rpmsg",
		.owner = THIS_MODULE,
		.of_match_table = aputop_tx_rpmsg_of_match,
	},
	.probe = aputop_tx_rpmsg_probe,
	.remove = aputop_rpmsg_remove,
};

static struct rpmsg_driver aputop_rx_rpmsg_drv = {
	.drv	= {
		.name	= "apu_top_3_rx_rpmsg",
		.owner = THIS_MODULE,
		.of_match_table = aputop_rx_rpmsg_of_match,
	},
	.probe = aputop_rx_rpmsg_probe,
	.remove = aputop_rpmsg_remove,
};

int aputop_register_rpmsg(void)
{
	int ret = 0;
	int ret_all = 0;

	ret = register_rpmsg_driver(&aputop_tx_rpmsg_drv);
	if (ret)
		pr_info("%s: failed to register aputop tx rpmsg\n", __func__);
	ret_all |= ret;

	ret = register_rpmsg_driver(&aputop_rx_rpmsg_drv);
	if (ret)
		pr_info("%s: failed to register aputop rx rpmsg\n", __func__);
	ret_all |= ret;

	return ret_all;
}

void aputop_unregister_rpmsg(void)
{
	unregister_rpmsg_driver(&aputop_tx_rpmsg_drv);
	unregister_rpmsg_driver(&aputop_rx_rpmsg_drv);
}
