#
# Copyright (C) 2019 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#


SRC := $(shell pwd)
#MTK_PLATFORM = mt8195
top := $(srctree)/$(src)

export top
#export MTK_PLATFORM

# config
CONFIG_MTK_APUSYS_VPU = y
CONFIG_MTK_APUSYS_DEBUG = n
CONFIG_MTK_APUSYS_VPU_DEBUG = n
#CONFIG_FTRACE = y
#CONFIG_DEBUG_FS = y

ifeq ($(CONFIG_DEBUG_FS),y)
CONFIG_MTK_APUSYS_DEBUG = y
CONFIG_MTK_APUSYS_VPU_DEBUG = y
endif

EXTRA_CFLAGS += -DCONFIG_MTK_APUSYS_VPU
#EXTRA_CFLAGS += -DCONFIG_FTRACE
ifeq ($(CONFIG_DEBUG_FS),y)
EXTRA_CFLAGS += -DCONFIG_MTK_APUSYS_DEBUG
EXTRA_CFLAGS += -DCONFIG_MTK_APUSYS_VPU_DEBUG
EXTRA_CFLAGS += -DCONFIG_DEBUG_FS
endif

# assign value
UTIL_DIR = util

# declare apusys version
APUSYS_VERSION_MIDWARE = 2.0
APUSYS_VERSION_SAMPLE = 2.0
APUSYS_VERSION_DEBUG = 1.0
APUSYS_VERSION_MNOC = 1.5
APUSYS_VERSION_VPU = 4.5
APUSYS_VERSION_REVISER = 2.0

# target object
obj-m += apusys.o

# common include path
EXTRA_CFLAGS  += -I$(srctree)/$(src)/include/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/misc/mediatek/geniezone/public/

# core makefile
apusys-objs += core/apusys_core.o

# middleware makefile
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/staging/android/ion
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/misc/mediatek/include/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/misc/mediatek/include/mt-plat/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/misc/mediatek/slbc/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/debug/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/ap/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/rv/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/ap/dummy/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/$(UTIL_DIR)/

apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_mem_drv.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_mem_rsc.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_drv.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_ctx.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_dev.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_ioctl.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_mem.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_mem_dma.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_mem_aram.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_mem_pool.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_cmd.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_cmn.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_hs.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_sysfs.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_dbg.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_import.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/mdw_util.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv_cmd.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv_cmd_v3.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv_dev.o

apusys-$(CONFIG_FTRACE) += $(UTIL_DIR)/apusys_trace.o
apusys-objs += $(UTIL_DIR)/apu_bmap.o

# edma driver makefile
apusys-objs += edma/edma_driver.o
ifeq ($(CONFIG_DEBUG_FS),y)
apusys-objs += edma/edma_dbgfs.o
endif
apusys-objs += edma/edma_cmd_hnd.o
apusys-objs += edma/edma_plat_device.o
apusys-objs += edma/edma_v20.o
apusys-objs += edma/edma_v30.o
apusys-objs += edma/edma_rv.o
#apusys-objs += edma/apusys_power_dummy.o


# reviser driver makefile
REVISER=$(srctree)/$(src)
REVISER_PLAT=reviser/$(APUSYS_VERSION_REVISER)/platform
REVISER_COMN=reviser/$(APUSYS_VERSION_REVISER)/common

EXTRA_CFLAGS  += -I$(REVISER)/$(REVISER_PLAT)
EXTRA_CFLAGS  += -I$(REVISER)/$(REVISER_COMN)
EXTRA_CFLAGS  += -I$(REVISER)/$(REVISER_PLAT)/v_1_0

apusys-objs += $(REVISER_COMN)/reviser_drv.o
apusys-objs += $(REVISER_COMN)/reviser_dbg.o
apusys-objs += $(REVISER_COMN)/reviser_export.o
apusys-objs += $(REVISER_COMN)/reviser_import.o
apusys-objs += $(REVISER_COMN)/reviser_hw_mgt.o
apusys-objs += $(REVISER_COMN)/reviser_mem.o
apusys-objs += $(REVISER_COMN)/reviser_power.o
apusys-objs += $(REVISER_COMN)/reviser_table_mgt.o
apusys-objs += $(REVISER_COMN)/reviser_remote.o
apusys-objs += $(REVISER_COMN)/reviser_remote_cmd.o

apusys-objs += $(REVISER_PLAT)/reviser_device.o
apusys-objs += $(REVISER_PLAT)/reviser_plat.o

apusys-objs += $(REVISER_PLAT)/v_1_0/reviser_hw_v1_0.o
apusys-objs += $(REVISER_PLAT)/v_1_0/reviser_plat_v1_0.o
apusys-objs += $(REVISER_PLAT)/v_1_0/reviser_reg_v1_0.o
apusys-objs += $(REVISER_PLAT)/v_rv/reviser_hw_vrv.o
apusys-objs += $(REVISER_PLAT)/v_rv/reviser_plat_vrv.o



# mnoc makefile
EXTRA_CFLAGS  += -I$(srctree)/$(src)/mnoc/$(APUSYS_VERSION_MNOC)/interface/
apusys-objs += mnoc/$(APUSYS_VERSION_MNOC)/common/mnoc_rv.o

# debug makefile
EXTRA_CFLAGS  += -I$(srctree)/$(src)/debug/$(APUSYS_VERSION_DEBUG)/common/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/debug/$(APUSYS_VERSION_DEBUG)/interface/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/debug/$(APUSYS_VERSION_DEBUG)/platform/

ifeq ($(CONFIG_MTK_APUSYS_DEBUG),y)
apusys-objs += debug/apu_tags.o
apusys-objs += debug/apu_tp.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv_tag.o

apusys-objs += debug/$(APUSYS_VERSION_DEBUG)/common/debug_drv.o
apusys-objs += debug/$(APUSYS_VERSION_DEBUG)/platform/debug_plat_device.o
apusys-objs += debug/$(APUSYS_VERSION_DEBUG)/platform/debug_platform.o
endif

EXTRA_CFLAGS  += -I$(srctree)/$(src)/debug/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/include/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/

apusys-objs += power/2.5/common/apu-tag.o
EXTRA_CFLAGS  += -I$(srctree)/$(src)/power/2.5/inc/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/power/3.0/inc/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/3.0/inc/platform/
obj-m += power/

# mdla
EXTRA_CFLAGS  += -I$(srctree)/$(src)/mdla/
apusys-objs += $(shell cd $(srctree)/$(src) && find mdla -name "*.c" | sed 's/\.c/\.o/g')

# vpu
VPU = vpu/$(APUSYS_VERSION_VPU)
EXTRA_CFLAGS += -I$(srctree)/$(src)/$(VPU)
ifeq ($(CONFIG_MTK_APUSYS_VPU_DEBUG),y)
apusys-objs += $(VPU)/vpu_tag.o
apusys-objs += $(VPU)/vpu_met.o
endif
#ifeq ($(CONFIG_MTK_APUSYS_VPU),y)
apusys-objs += $(VPU)/vpu_main.o
apusys-objs += $(VPU)/vpu_algo.o
apusys-objs += $(VPU)/vpu_cmd.o
apusys-objs += $(VPU)/vpu_hw.o
apusys-objs += $(VPU)/vpu_mem.o
apusys-objs += $(VPU)/vpu_power.o
apusys-objs += $(VPU)/vpu_cfg.o
apusys-objs += $(VPU)/vpu_debug.o
apusys-objs += $(VPU)/vpu_dump.o
apusys-objs += $(VPU)/vpu_loadimage.o
#endif

# devapc
apusys-objs += devapc/dapc.o
apusys-objs += devapc/dapc_cfg.o

# apusys rv
EXTRA_CFLAGS  += -I$(srctree)/$(src)/apusys_rv/
apusys-objs += apusys_rv/apu_config.o
apusys-objs += apusys_rv/apu_coredump.o
apusys-objs += apusys_rv/apu_excep.o
apusys-objs += apusys_rv/apu_ipi.o
apusys-objs += apusys_rv/apu_mbox.o
apusys-objs += apusys_rv/apu_mem.o
apusys-objs += apusys_rv/apu_rproc.o
apusys-objs += apusys_rv/apu_procfs.o
apusys-objs += apusys_rv/apu_timesync.o
apusys-objs += apusys_rv/apu_deepidle.o
apusys-objs += apusys_rv/apu_regdump.o
apusys-objs += apusys_rv/apu_debug.o
apusys-objs += apusys_rv/apu_loadimage.o
ifneq ($(wildcard $(srctree)/$(src)/apusys_rv/2.0/mt8188_plat.c),)
subdir-EXTRA_CFLAGS  += -DMT8188_APUSYS_RV_PLAT_DATA
apusys-objs += apusys_rv/2.0/mt8188_plat.o
endif
ifneq ($(wildcard $(srctree)/$(src)/apusys_rv/2.0/mt8195_plat.c),)
subdir-EXTRA_CFLAGS  += -DMT8195_APUSYS_RV_PLAT_DATA
apusys-objs += apusys_rv/2.0/mt8195_plat.o
endif
# ipi
EXTRA_CFLAGS  += -I$(srctree)/$(src)/ipi/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/rpmsg/
apusys-objs += ipi/mtk_apu_rpmsg.o

# sw logger
EXTRA_CFLAGS  += -I$(srctree)/$(src)/sw_logger/
apusys-objs += sw_logger/sw_logger.o

# aps
EXTRA_CFLAGS  += -I$(srctree)/$(src)/aps/
apusys-objs += aps/aps_driver.o
apusys-objs += aps/aps_ipi.o
apusys-objs += aps/aps_sysfs.o

# hw logger
EXTRA_CFLAGS  += -I$(srctree)/$(src)/hw_logger/
apusys-objs += hw_logger/hw_logger.o

# aov
#EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/misc/mediatek/scp/include
#apusys-objs += aov/aov_init.o
#apusys-objs += aov/aov_rpmsg.o
#apusys-objs += aov/aov_recovery.o

# sapu
#obj-$(CONFIG_MTK_APUSYS_SUPPORT) += sapu/

# hw_sema
EXTRA_CFLAGS  += -I$(srctree)/$(src)/power/3.0/inc/


all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
