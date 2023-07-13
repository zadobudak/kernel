CONFIG_DEVFREQ_THERMAL = y

# common part
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/devfreq/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/soc/mediatek/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/drivers/misc/mediatek/include
EXTRA_CFLAGS  += -I$(srctree)/$(src)/debug/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/../include/
EXTRA_CFLAGS  += -DCONFIG_DEVFREQ_THERMAL

# apusys power driver v2.5
EXTRA_CFLAGS  += -I$(srctree)/$(src)/2.5/inc/
SRC := 2.5/common/apu-clk.o 2.5/common/apu-common.o 2.5/common/apu-regulator.o 2.5/common/apu-rpc.o 2.5/common/apu-of.o 2.5/common/apu-plat.o  2.5/common/apu-driver.o 2.5/common/apu-dbg.o 2.5/common/apu-trace.o
SRC += 2.5/governor/gov-apusys.o 2.5/governor/gov-user.o 2.5/governor/common.o 2.5/governor/gov-constrain.o 2.5/governor/gov-apusys-pe.o
SRC += 2.5/devices/dev-freq-core.o 2.5/devices/dev-freq-conn.o 2.5/devices/dev-freq-vpu.o 2.5/devices/dev-freq-mdla.o 2.5/devices/dev-cb.o 2.5/devices/dev-freq-iommu.o

# apusys power driver v3.0
#EXTRA_CFLAGS  += -I$(srctree)/$(src)/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/3.0/inc/
EXTRA_CFLAGS  += -I$(srctree)/$(src)/3.0/inc/platform/

SRC += 3.0/aputop_drv.o
SRC += 3.0/aputop_rpmsg.o

ifneq ($(wildcard $(srctree)/$(src)/3.0/platform/mt8188_aputop.c),)
subdir-ccflags-y += -DMT8188_PLAT_DATA
SRC += 3.0/platform/mt8188_aputop.o
endif
ifneq ($(wildcard $(srctree)/$(src)/3.0/platform/mt8188_pwr_func.c),)
SRC += 3.0/platform/mt8188_pwr_func.o
endif
ifneq ($(wildcard $(srctree)/$(src)/3.0/platform/mt8195_aputop.c),)
subdir-ccflags-y += -DMT8195_PLAT_DATA
SRC += 3.0/platform/mt8195_aputop.o
endif
ifneq ($(wildcard $(srctree)/$(src)/3.0/platform/mt8195_pwr_func.c),)
SRC += 3.0/platform/mt8195_pwr_func.o
endif
SRC += apu_top_entry.o

ifeq ($(CONFIG_DEVFREQ_THERMAL),y)
SRC += 3.0/apu_devfreq_cooling.o
ifneq ($(wildcard $(srctree)/$(src)/3.0/platform/mt8188_apu_devfreq_cooling.c),)
SRC += 3.0/platform/mt8188_apu_devfreq_cooling.o
endif
ifneq ($(wildcard $(srctree)/$(src)/3.0/platform/mt8195_apu_devfreq_cooling.c),)
SRC += 3.0/platform/mt8195_apu_devfreq_cooling.o
endif
endif

obj-m += apu_top.o
apu_top-objs = $(SRC)
