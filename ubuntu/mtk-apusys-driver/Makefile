#obj-m := hello.o

SRC := $(shell pwd)
MTK_PLATFORM=mt8195

# assign value
UTIL_DIR = util

# declare apusys version
APUSYS_VERSION_MIDWARE = 2.0
APUSYS_VERSION_SAMPLE = 2.0
APUSYS_VERSION_DEBUG = 1.0
APUSYS_VERSION_MNOC = 1.5
APUSYS_VERSION_REVISER = 2.0
APUSYS_VERSION_VPU = 5.0
CONFIG_MTK_APUSYS_VPU = y
CONFIG_MTK_APUSYS_DEBUG = y
CONFIG_FTRACE = y
CONFIG_MTK_APUSYS_MDLA_SUPPORT = y

top := $(src)
export top
export MTK_PLATFORM

EXTRA_CFLAGS += -DAPUSYS_AIOT
EXTRA_CFLAGS += -DCONFIG_MTK_APUSYS_VPU
EXTRA_CFLAGS += -DCONFIG_MTK_APUSYS_DEBUG
EXTRA_CFLAGS += -DCONFIG_FTRACE
EXTRA_CFLAGS += -DCONFIG_MTK_APUSYS_MDLA_SUPPORT

# target object
obj-m := apusys.o

# common include path
EXTRA_CFLAGS += -I$(srctree)/$(src)/include/

# core makefile
apusys-objs += core/apusys_core.o

# middleware makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/slbc/
EXTRA_CFLAGS += -I$(srctree)/$(src)/debug/
EXTRA_CFLAGS += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/
EXTRA_CFLAGS += -I$(srctree)/$(src)/$(UTIL_DIR)/

EXTRA_CFLAGS += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/ap/
EXTRA_CFLAGS += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/rv/
EXTRA_CFLAGS += -I$(srctree)/$(src)/midware/$(APUSYS_VERSION_MIDWARE)/ap/dummy/
EXTRA_CFLAGS += -I$(srctree)/$(src)/$(UTIL_DIR)/

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
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_cmd.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_sched.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_dispr.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_queue.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_queue_deadline.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_queue_norm.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_rsc.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/mdw_ap_tag.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/ap/dummy/mdw_dmy.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv_cmd.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv_dev.o

ifeq ($(CONFIG_FTRACE),y)
apusys-objs += $(UTIL_DIR)/apusys_trace.o
endif


#edma makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/edma/

apusys-objs += edma/edma_rv.o

# reviser driver makefile
REVISER=$(src)/
REVISER_PLAT=reviser/$(APUSYS_VERSION_REVISER)/platform
REVISER_COMN=reviser/$(APUSYS_VERSION_REVISER)/common

EXTRA_CFLAGS += -I$(REVISER)/$(REVISER_PLAT)
EXTRA_CFLAGS += -I$(REVISER)/$(REVISER_COMN)
EXTRA_CFLAGS += -I$(REVISER)/$(REVISER_PLAT)/v_1_0
EXTRA_CFLAGS += -I$(REVISER)/$(REVISER_PLAT)/v_rv

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
EXTRA_CFLAGS += -I$(srctree)/$(src)/mnoc/$(APUSYS_VERSION_MNOC)



apusys-objs += mnoc/$(APUSYS_VERSION_MNOC)/common/mnoc_rv.o


# debug makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/debug/$(APUSYS_VERSION_DEBUG)/common/
EXTRA_CFLAGS += -I$(srctree)/$(src)/debug/$(APUSYS_VERSION_DEBUG)/interface/
EXTRA_CFLAGS += -I$(srctree)/$(src)/debug/$(APUSYS_VERSION_DEBUG)/platform/

ifeq ($(CONFIG_MTK_APUSYS_DEBUG),y)
apusys-objs += debug/apu_tp.o
apusys-objs += debug/apu_tags.o
apusys-objs += midware/$(APUSYS_VERSION_MIDWARE)/rv/mdw_rv_tag.o

apusys-objs += debug/$(APUSYS_VERSION_DEBUG)/common/debug_drv.o
apusys-objs += debug/$(APUSYS_VERSION_DEBUG)/platform/debug_plat_device.o
apusys-objs += debug/$(APUSYS_VERSION_DEBUG)/platform/debug_platform.o
endif


# power
EXTRA_CFLAGS += -I$(srctree)/$(src)/power/2.0/
EXTRA_CFLAGS += -I$(srctree)/$(src)/power/2.0/dummy/

EXTRA_CFLAGS += -I$(srctree)/$(src)/power/3.0/
EXTRA_CFLAGS += -I$(srctree)/$(src)/power/3.0/include/
EXTRA_CFLAGS += -I$(srctree)/$(src)/power/3.0/include/platform/
obj-m += apu_top/
obj-m += power/

# mdla makefile

EXTRA_CFLAGS += -I$(srctree)/$(src)/mdla/
apusys-objs += $(shell cd $(srctree)/$(src) && find mdla -name "*.c" | sed 's/\.c/\.o/g')

# vpu makefile
VPU = vpu/$(APUSYS_VERSION_VPU)
EXTRA_CFLAGS += -I$(srctree)/$(src)/$(VPU)
#EXTRA_CFLAGS += -I$(src)/drivers/base

ifeq ($(CONFIG_MTK_APUSYS_VPU),y)
apusys-objs += $(VPU)/vpu_debug.o
apusys-objs += $(VPU)/vpu_main.o
apusys-objs += $(VPU)/vpu_mem.o
apusys-objs += $(VPU)/vpu_cfg.o
apusys-objs += $(VPU)/vpu_algo.o
apusys-objs += $(VPU)/vpu_hw.o
apusys-objs += $(VPU)/vpu_loadimage.o
endif


# devapc makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/devapc/
apusys-objs += devapc/dapc.o
apusys-objs += devapc/dapc_cfg.o

# apusys_rv makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/apusys_rv
apusys-objs += apusys_rv/apu_config.o
apusys-objs += apusys_rv/apu_coredump.o
apusys-objs += apusys_rv/apu_excep.o
apusys-objs += apusys_rv/apu_ipi.o
apusys-objs += apusys_rv/apu_mbox.o
apusys-objs += apusys_rv/apu_mem.o
apusys-objs += apusys_rv/apu_rproc.o
apusys-objs += apusys_rv/apu_sysfs.o
apusys-objs += apusys_rv/apu_timesync.o
apusys-objs += apusys_rv/apusys_rv.o
apusys-objs += apusys_rv/apu_deepidle.o
apusys-objs += apusys_rv/apu_regdump.o
apusys-objs += apusys_rv/apu_debug.o
#ifneq ($(wildcard $(src)/apusys_rv/2.0/mt8188_plat.c),)
#EXTRA_CFLAGS += -DMT8188_APUSYS_RV_PLAT_DATA
#apusys-objs += apusys_rv/2.0/mt8188_plat.o
#endif
ifneq ($(wildcard $(src)/apusys_rv/2.0/mt8195_plat.c),)
EXTRA_CFLAGS += -DMT8195_APUSYS_RV_PLAT_DATA
apusys-objs += apusys_rv/2.0/mt8195_plat.o
endif

# ipi makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/power/3.0/
EXTRA_CFLAGS += -I$(srctree)/$(src)/midware/2.0/
EXTRA_CFLAGS += -I$(srctree)/$(src)/include/
EXTRA_CFLAGS += -I$(srctree)/$(src)/mdla/rv/
EXTRA_CFLAGS += -I$(srctree)/$(src)/mvpu/1.0/
EXTRA_CFLAGS += -I$(srctree)/$(src)/reviser/2.0/common
EXTRA_CFLAGS += -I$(srctree)/$(src)/2.0/platform/
EXTRA_CFLAGS += -I$(srctree)/$(src)/../../2.0/platform/
EXTRA_CFLAGS += -I$(srctree)/$(src)/../2.0/platform/
EXTRA_CFLAGS += -I$(srctree)/$(src)/../../../2.0/platform/
EXTRA_CFLAGS += -I$(srctree)/$(src)/reviser/2.0/platform
#EXTRA_CFLAGS += -I$(src)/ipi
#apusys-objs += ipi/mtk_apu_rpmsg.o

# hw_logger makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/hw_logger/
apusys-objs += hw_logger/hw_logger.o

# sw_logger makefile
EXTRA_CFLAGS += -I$(srctree)/$(src)/sw_logger/
apusys-objs += sw_logger/sw_logger.o



all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
