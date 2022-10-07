/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#ifndef __APU_DEVFREQ_COOLING_H__
#define __APU_DEVFREQ_COOLING_H__

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
int apu_devfreq_init(void);
void apu_devfreq_exit(void);
#else
static inline int apu_devfreq_init(void)
{
	return 0;
}
static inline void apu_devfreq_exit(void)
{
}
#endif /* IS_ENABLED(CONFIG_DEVFREQ_THERMAL) */
#endif /* __APU_DEVFREQ_COOLING_H__ */
