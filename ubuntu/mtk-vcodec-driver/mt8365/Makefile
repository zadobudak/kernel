SRC := $(shell pwd)

# extra options
MTK_PLATFORM = $(TARGET_PLATFORM)
KBUILD_EXTRA_SYMBOLS_PATH = $(EXTRA_SYMBOLS_PATH)

# declare kernel config
CONFIG_VIDEO_MEDIATEK_VCODEC_MT8167=y

MTK_IOT_PLATFORM=mt8365

top := $(src)
export top
$(info mtk-vcodec-driver/ src is $(src))
$(info mtk-vcodec-driver/ MTK_PLATFORM is $(MTK_PLATFORM))
export MTK_PLATFORM

# target object
obj-m := mtk-vcodec-common-$(MTK_IOT_PLATFORM).o
obj-m += mtk-vcodec-dec-$(MTK_IOT_PLATFORM).o
obj-m += mtk-vcodec-enc-$(MTK_IOT_PLATFORM).o

# include
EXTRA_CFLAGS += -I$(srctree)/$(src)/
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/mtk-vcu-mt8167/
# mtk_vcu_controls.h
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/uapi/

# mtk-vcodec-common.ko
mtk-vcodec-common-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_intr.o
mtk-vcodec-common-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_util.o

# mtk-vcodec-dec.ko
mtk-vcodec-dec-$(MTK_IOT_PLATFORM)-objs += vdec/vdec_common_if.o
mtk-vcodec-dec-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_dec_drv.o
mtk-vcodec-dec-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_dec.o
mtk-vcodec-dec-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_dec_pm.o
mtk-vcodec-dec-$(MTK_IOT_PLATFORM)-objs += vdec_drv_if.o
mtk-vcodec-dec-$(MTK_IOT_PLATFORM)-objs += vdec_vcu_if.o

# mtk-vcodec-enc.ko
mtk-vcodec-enc-$(MTK_IOT_PLATFORM)-objs += venc/venc_common_if.o
mtk-vcodec-enc-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_enc_drv.o
mtk-vcodec-enc-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_enc.o
mtk-vcodec-enc-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_enc_pm.o
mtk-vcodec-enc-$(MTK_IOT_PLATFORM)-objs += venc_drv_if.o
mtk-vcodec-enc-$(MTK_IOT_PLATFORM)-objs += venc_vcu_if.o



all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS_PATH)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c *.mod
	rm -rf vdec/*.o vdec/.*.cmd
	rm -rf venc/*.o venc/.*.cmd
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
