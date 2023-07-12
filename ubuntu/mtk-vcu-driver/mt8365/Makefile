SRC := $(shell pwd)

# extra options
MTK_PLATFORM = $(TARGET_PLATFORM)
KBUILD_EXTRA_SYMBOLS_PATH = $(EXTRA_SYMBOLS_PATH)

# declare kernel config
CONFIG_VIDEO_MEDIATEK_VCU_MT8167=y

MTK_IOT_PLATFORM=mt8365

top := $(src)
export top
$(info mtk-vcu-driver/ src is $(src))
$(info mtk-vcu-driver/ MTK_PLATFORM is $(MTK_PLATFORM))
export MTK_PLATFORM

# target object
obj-m := mtk-vcu-$(MTK_IOT_PLATFORM).o
obj-m += mtk-vpu-$(MTK_IOT_PLATFORM).o

# include
EXTRA_CFLAGS += -I$(srctree)/$(src)/
# mtk_vcu_controls.h
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/uapi/

# SPDX-License-Identifier: GPL-2.0-only
mtk-vcu-$(MTK_IOT_PLATFORM)-objs += mtk_vcu.o
mtk-vcu-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_mem.o

mtk-vpu-$(MTK_IOT_PLATFORM)-objs += mtk_vpu.o



all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod *.mod.c
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
