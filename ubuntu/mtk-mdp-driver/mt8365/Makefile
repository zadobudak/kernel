SRC := $(shell pwd)

# extra options
MTK_PLATFORM = $(TARGET_PLATFORM)
KBUILD_EXTRA_SYMBOLS_PATH = $(EXTRA_SYMBOLS_PATH)

# declare kernel config
CONFIG_VIDEO_MEDIATEK_MDP_MT8167=y

MTK_IOT_PLATFORM=mt8365

top := $(src)
export top
$(info mtk-mdp-driver/ src is $(src))
$(info mtk-mdp-driver/ MTK_PLATFORM is $(MTK_PLATFORM))
export MTK_PLATFORM

# target object
obj-m := mtk-mdp-$(MTK_IOT_PLATFORM).o

# include
EXTRA_CFLAGS += -I$(srctree)/$(src)/
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/mtk-vcu-mt8167/

# SPDX-License-Identifier: GPL-2.0
# mtk-mdp-$(MTK_IOT_PLATFORM).ko
mtk-mdp-$(MTK_IOT_PLATFORM)-objs += mtk_mdp_core.o
mtk-mdp-$(MTK_IOT_PLATFORM)-objs += mtk_mdp_comp.o
mtk-mdp-$(MTK_IOT_PLATFORM)-objs += mtk_mdp_m2m.o
mtk-mdp-$(MTK_IOT_PLATFORM)-objs += mtk_mdp_regs.o
mtk-mdp-$(MTK_IOT_PLATFORM)-objs += mtk_mdp_vpu.o



all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS_PATH)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod *.mod.c
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
