/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef __APUTOP_RPMSG_H__
#define __APUTOP_RPMSG_H__

enum aputop_rpmsg_cmd {
	APUTOP_DEV_CTL,
	APUTOP_DEV_SET_OPP,
	APUTOP_DUMP_OPP_TBL,
	APUTOP_CURR_STATUS,
	APUTOP_PWR_PROFILING,
	APUTOP_THERMAL_GET_TEMP,
	APUTOP_DPIDLE_SKIP,
	APUTOP_RPMSG_CMD_MAX,
};

struct aputop_rpmsg_data {
	enum aputop_rpmsg_cmd cmd;
	int data0;
	int data1;
	int data2;
	int data3;
};

/* send a tx rpmsg message to remote side and wait ack */
int aputop_send_tx_rpmsg(struct aputop_rpmsg_data *rpmsg_data, int timeout);
/* send a rx rpmsg message to remote side */
int aputop_send_rx_rpmsg(struct aputop_rpmsg_data *rpmsg_data);

void test_ipi_wakeup_apu(void);
enum aputop_rpmsg_cmd get_curr_tx_rpmsg_cmd(void);
int aputop_register_rpmsg(void);
void aputop_unregister_rpmsg(void);

#endif /* __APUTOP_RPMSG_H__ */
