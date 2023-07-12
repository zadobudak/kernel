SRC := $(shell pwd)

# declare vcu config
MTK_PLATFORM = mt8395
CONFIG_VIDEO_MEDIATEK_VCU_MT8395=y
CONFIG_MTK_IOMMU=y
CONFIG_ARM64=y
CONFIG_COMPAT=y

top := $(src)
export top
$(info mtk-vcu-mt8395/ src is $(src))
$(info mtk-vcu-mt8395/ MTK_PLATFORM is $(MTK_PLATFORM))
$(info mtk-vcu-mt8395/ current extra symbols $(KBUILD_EXTRA_SYMBOLS))
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
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules KBUILD_EXTRA_SYMBOLS=`pwd`/mtk-vcodec-mt8395/Module.symvers

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
