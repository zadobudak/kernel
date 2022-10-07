obj-m += apu_top.o


# 2.0

EXTRA_CFLAGS += -I$(top)/debug/
EXTRA_CFLAGS += -I$(top)/include/
EXTRA_CFLAGS += -I$(top)/power/2.0/
EXTRA_CFLAGS += -I$(top)/power/2.0/test/

# FPGA environment
ifeq ($(CONFIG_MTK_FPGA),y)
subdir-EXTRA_CFLAGS += -DMTK_FPGA_PORTING
else ifeq ($(CONFIG_FPGA_EARLY_PORTING),y)
subdir-EXTRA_CFLAGS += -DMTK_FPGA_PORTING
endif

apu_top-objs += 2.0/apusys_power_driver.o
apu_top-objs += 2.0/apusys_power_ctl.o
apu_top-objs += 2.0/apusys_power_debug.o
EXTRA_CFLAGS += -I$(top)/power/2.0/dummy/
APUPW_PLATFORM_DIR = dummy


apu_top-objs += 2.0/$(APUPW_PLATFORM_DIR)/hal_config_power.o
apu_top-objs += 2.0/$(APUPW_PLATFORM_DIR)/apusys_power_cust.o
apu_top-objs += 2.0/$(APUPW_PLATFORM_DIR)/apu_platform_resource.o
apu_top-objs += 2.0/$(APUPW_PLATFORM_DIR)/apu_platform_debug.o
apu_top-objs += 2.0/$(APUPW_PLATFORM_DIR)/apusys_power_rule_check.o


ifneq (,$(findstring aging,$(PROJECT_DTB_NAMES)))
subdir-EXTRA_CFLAGS += -DAGING_MARGIN
endif


# 3.0
EXTRA_CFLAGS += -I$(top)/include/
EXTRA_CFLAGS += -I$(top)/midware/2.0/
EXTRA_CFLAGS += -I$(top)/power/
EXTRA_CFLAGS += -I$(top)/power/3.0/inc/
EXTRA_CFLAGS += -I$(top)/power/3.0/inc/platform/

#src = $(srctree)/drivers/misc/mediatek/apusys/power/3.0
#apu_top-objs += $(shell cd $(top) && find . -name "*.c" | sed 's/\.c/\.o/g')
apu_top-objs += apu_top_entry.o
apu_top-objs += 3.0/aputop_drv.o
apu_top-objs += 3.0/aputop_rpmsg.o
ifneq ($(wildcard $(top)/power/3.0/platform/mt8188_aputop.c),)
apu_top-objs += 3.0/platform/mt8188_aputop.o
apu_top-objs += 3.0/platform/mt8188_pwr_func.o
endif
ifneq ($(wildcard $(top)/power/3.0/platform/mt8195_aputop.c),)
apu_top-objs += 3.0/platform/mt8195_aputop.o
apu_top-objs += 3.0/platform/mt8195_pwr_func.o
endif

ifeq ($(CONFIG_DEVFREQ_THERMAL),y)
apu_top-objs += 3.0/apu_devfreq_cooling.o
ifneq ($(wildcard $(top)/power/3.0/platform/mt8188_apu_devfreq_cooling.c),)
apu_top-objs += 3.0/platform/mt8188_apu_devfreq_cooling.o
endif
ifneq ($(wildcard $(top)/power/3.0/platform/mt8195_apu_devfreq_cooling.c),)
apu_top-objs += 3.0/platform/mt8195_apu_devfreq_cooling.o
endif
endif
