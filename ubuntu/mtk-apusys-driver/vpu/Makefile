# SPDX-License-Identifier: GPL-2.0-only

PDIR = $(MTK_PLATFORM)
VDIR = 5.0

# RemoteRPC
ccflags-y += -I$(srctree)/drivers/remoteproc/

# Memory
ccflags-y += -I$(srctree)/drivers/staging/android/ion
ccflags-y += -I$(srctree)/drivers/misc/mediatek/m4u/$(PDIR)/

# MTK platform
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(PDIR)/include

# APUSYS
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/debug
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/midware/2.0
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/power/3.0/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/apusys_rv/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/ipi

# VPU driver
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/vpu/$(VDIR)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/vpu/$(PDIR)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/apusys/util

# power related
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/qos/$(PDIR)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
ccflags-y += -I$(srctree)/drivers/devfreq

# SMI debug
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/

obj-y += vpu.o

#vpu-$(CONFIG_MTK_APUSYS_VPU_DEBUG) += $(VDIR)/vpu_debug.o $(IPDIR)/vpu_dump.o $(IPDIR)/vpu_met.o
#vpu-y += $(VDIR)/vpu_main.o $(VDIR)/vpu_algo.o
#vpu-y += $(IPDIR)/vpu_hw.o $(IPDIR)/vpu_mem.o $(IPDIR)/vpu_power.o
vpu-y += $(VDIR)/vpu_debug.o
vpu-y += $(VDIR)/vpu_main.o
vpu-y += $(VDIR)/vpu_mem.o
vpu-y += $(VDIR)/vpu_cfg.o
vpu-y += $(VDIR)/vpu_algo.o
vpu-y += $(VDIR)/vpu_hw.o
vpu-y += $(VDIR)/vpu_loadimage.o
