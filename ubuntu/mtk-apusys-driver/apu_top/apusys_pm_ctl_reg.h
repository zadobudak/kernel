/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef __APUSYS_PM_CTL_REG_H
#define __APUSYS_PM_CTL_REG_H

/**************************************************
 * APUSYS_RPC related register
 *************************************************/
#define APUSYS_RPC_TOP_CON		(0x000)
#define APUSYS_RPC_TOP_SEL		(0x004)
#define APUSYS_RPC_SW_FIFO_WE		(0x008)
#define APUSYS_RPC_INTF_PWR_RDY		(0x044)

#define APUSYS_RPC_SW_TYPE0		(0x200)
#define APUSYS_RPC_SW_TYPE1		(0x204)
#define APUSYS_RPC_SW_TYPE2		(0x208)
#define APUSYS_RPC_SW_TYPE3		(0x20C)
#define APUSYS_RPC_SW_TYPE4		(0x210)
#define APUSYS_RPC_SW_TYPE6		(0x218)
#define APUSYS_RPC_SW_TYPE7		(0x21C)

#define BIT_ABORT_DISABLE		(1)
#define BIT_ABORT_IRQ_MASK		(2)
#define BIT_WAKEUP_IRQ_MASK		(3)
#define BIT_DDR_EN_SEL0			(4)
#define BIT_DDR_EN_SEL1			(5)
#define BIT_BYASS_WFI			(7)

#define BIT_APU_BUCK_ELS_EN		(3)

/**************************************************
 * APUSYS_VCORE related register
 *************************************************/
#define APUSYS_VCORE_CG_CON		(0x000)
#define APUSYS_VCORE_CG_SET		(0x004)
#define APUSYS_VCORE_CG_CLR		(0x008)

/**************************************************
 * SPM and related register
 *************************************************/
#define APUSYS_OTHER_PWR_STATUS		(0x198)
#define APUSYS_BUCK_ISOLATION		(0x3EC)
#define APUSYS_SPM_CROSS_WAKE_M01_REQ	(0x670)

#define APMCU_WAKEUP_APU		(0x1 << 0)

/**************************************************
 * APUSYS_CONN related register
 *************************************************/
#define APUSYS_CONN_CG_CON		(0x000)
#define APUSYS_CONN_CG_CLR		(0x008)

/**************************************************
 * APU CONN1 related register
 *************************************************/
#define APUSYS_CONN1_CG_CON		(0x000)
#define APUSYS_CONN1_CG_CLR		(0x008)

/**************************************************
 * PLL and related register
 *************************************************/
#define APUSYS_PLL4H_PLL1_CON0		(0x008)
#define APUSYS_PLL4H_PLL1_CON1		(0x00C)
#define APUSYS_PLL4H_PLL1_CON3		(0x014)

#define APUSYS_PLL4H_PLL2_CON0		(0x018)
#define APUSYS_PLL4H_PLL2_CON1		(0x01C)
#define APUSYS_PLL4H_PLL2_CON3		(0x024)

#define APUSYS_PLL4H_PLL3_CON0		(0x028)
#define APUSYS_PLL4H_PLL3_CON1		(0x02C)
#define APUSYS_PLL4H_PLL3_CON3		(0x034)

#define APUSYS_PLL4H_PLL4_CON0		(0x038)
#define APUSYS_PLL4H_PLL4_CON1		(0x03C)
#define APUSYS_PLL4H_PLL4_CON3		(0x044)

#define APUSYS_PLL4H_FQMTR_CON0		(0x200)
#define APUSYS_PLL4H_FQMTR_CON1		(0x204)

#define BIT_RG_PLL_EN			(0)
#define BIT_DA_PLL_SDM_PWR_ON		(0)
#define BIT_DA_PLL_SDM_ISO_EN		(1)
#define POSDIV_SHIFT			(24)

/**************************************************
 * ACC and related register
 *************************************************/
#define APUSYS_ACC_CONFG_SET0		(0x000)
#define APUSYS_ACC_CONFG_SET1		(0x004)
#define APUSYS_ACC_CONFG_SET2		(0x008)
#define APUSYS_ACC_CONFG_SET4		(0x010)
#define APUSYS_ACC_CONFG_SET5		(0x014)
#define APUSYS_ACC_CONFG_SET7		(0x01C)

#define APUSYS_ACC_CONFG_CLR0		(0x040)
#define APUSYS_ACC_CONFG_CLR1		(0x044)
#define APUSYS_ACC_CONFG_CLR2		(0x048)
#define APUSYS_ACC_CONFG_CLR4		(0x050)
#define APUSYS_ACC_CONFG_CLR5		(0x054)
#define APUSYS_ACC_CONFG_CLR7		(0x05C)

#define APUSYS_ACC_FM_CONFG_SET		(0x0C0)
#define APUSYS_ACC_FM_CONFG_CLR		(0x0C4)
#define APUSYS_ACC_FM_SEL		(0x0C8)
#define APUSYS_ACC_FM_CNT		(0x0CC)

#define BIT_LOOP_REF			(16)
#define BIT_CLKEN			(0)
#define BIT_FUNCEN			(1)
#define BIT_FM_DONE			(4)

#define BIT_CGEN_F26M			(0)
#define BIT_CGEN_PARK			(1)
#define BIT_CGEN_SOC			(2)
#define BIT_CGEN_APU			(3)
#define BIT_CGEN_OUT			(4)
#define BIT_SEL_PARK			(8)
#define BIT_SEL_F26M			(9)
#define BIT_SEL_APU_DIV2		(10)
#define BIT_SEL_APU			(11)
#define BIT_SEL_PARK_SRC_OUT		(12)
#define BIT_INVEN_OUT			(15)

/**************************************************
 * APU_PCU related register
 *************************************************/
#define APUSYS_PCU_PMIC_STATUS		(0x180)
#define APUSYS_PCU_PMIC_CMD_GEN		(0x184)
#define APUSYS_PCU_PMIC_RDATA		(0x188)
#define	APUSYS_PCU_PMIC_TAR_BUF1	(0x190)
#define	APUSYS_PCU_PMIC_TAR_BUF2	(0x194)

/**************************************************
 * APU AO CTRL related register
 *************************************************/
#define APUSYS_CSR_DUMMY_0		(0x024)


#endif /* __APUSYS_PM_CTL_REG_H */
