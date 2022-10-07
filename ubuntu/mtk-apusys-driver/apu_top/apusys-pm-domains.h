/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __APUSYS_PM_DOMAINS_H
#define __APUSYS_PM_DOMAINS_H

#include <dt-bindings/apusys/mt8195-apusys-power.h>

struct apusys_domain_data {
	int domain_idx;
	u32 acc_inverse;
	u32 acc_set_offs;
	u32 acc_clr_offs;
	u32 acc2nd_set_offs;
	u32 acc2nd_clr_offs;
};

struct apusys_subdomain {
	int origin;
	int subdomain;
};

struct apusys_data {
	const struct apusys_domain_data *domains_data;
	int num_domains;
	const struct apusys_subdomain *subdomains;
	int num_subdomains;
};

#endif /* __SOC_MEDIATEK_MTK_PM_DOMAINS_H */
