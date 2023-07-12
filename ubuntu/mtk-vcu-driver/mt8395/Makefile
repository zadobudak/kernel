SRC := $(shell pwd)

# extra options
MTK_PLATFORM = $(TARGET_PLATFORM)
KBUILD_EXTRA_SYMBOLS_PATH = $(EXTRA_SYMBOLS_PATH)

# declare kernel config
CONFIG_VIDEO_MEDIATEK_VCU_MT8395=y
CONFIG_MTK_IOMMU=y
CONFIG_ARM64=y
CONFIG_COMPAT=y

top := $(src)
export top
$(info mtk-vcu-driver/ src is $(src))
$(info mtk-vcu-driver/ MTK_PLATFORM is $(MTK_PLATFORM))
$(info mtk-vcu-driver/ current extra symbols $(KBUILD_EXTRA_SYMBOLS_PATH))
export MTK_PLATFORM

EXTRA_CFLAGS += -DCONFIG_VIDEO_MEDIATEK_VCU_MT8395
EXTRA_CFLAGS += -DCONFIG_MTK_IOMMU
EXTRA_CFLAGS += -DCONFIG_ARM64
EXTRA_CFLAGS += -DCONFIG_COMPAT

# target object
obj-m := mtk-vcu.o

# include
EXTRA_CFLAGS += -I$(src)/
# mtk_vcu_controls.h
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/uapi/

# mtk-vcu.ko
mtk-vcu-objs += mtk_vcu.o
mtk-vcu-objs += mtk_vcodec_mem.o

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS_PATH)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
