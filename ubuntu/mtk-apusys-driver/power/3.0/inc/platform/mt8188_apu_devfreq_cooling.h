/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __MT8188_DEVFREQ_COOLING_H__
#define __MT8188_DEVFREQ_COOLING_H__

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
int mt8188_apu_devfreq_cooling_init(struct platform_device *pdev,
				    void __iomem *reg_base);
void mt8188_apu_devfreq_cooling_start(void);
void mt8188_apu_devfreq_cooling_stop(void);

int mt8188_apu_devfreq_cooling_register(struct platform_device *pdev);
void mt8188_apu_devfreq_cooling_unregister(struct platform_device *pdev);
void mt8188_apu_devfreq_cooling_start_monitor(struct platform_device *pdev);
void mt8188_apu_devfreq_cooling_stop_monitor(struct platform_device *pdev);
#else
static inline int mt8188_apu_devfreq_cooling_init(struct platform_device *pdev,
						  void __iomem *reg_base)
{
	return 0;
}
static inline void mt8188_apu_devfreq_cooling_start(void)
{
}
static inline void mt8188_apu_devfreq_cooling_stop(void)
{
}
static inline int mt8188_apu_devfreq_cooling_register
						(struct platform_device *pdev)
{
	return 0;
}
static inline void mt8188_apu_devfreq_cooling_unregister
						(struct platform_device *pdev)
{
}
static inline void mt8188_apu_devfreq_cooling_start_monitor
						(struct platform_device *pdev)
{
}
static inline void mt8188_apu_devfreq_cooling_stop_monitor
						(struct platform_device *pdev)
{
}
#endif /* IS_ENABLED(CONFIG_DEVFREQ_THERMAL) */
#endif /* __MT8188_DEVFREQ_COOLING_H__ */
