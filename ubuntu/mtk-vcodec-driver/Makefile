SRC := $(shell pwd)

# extra options
MTK_PLATFORM = $(TARGET_PLATFORM)
KBUILD_EXTRA_SYMBOLS_PATH = $(EXTRA_SYMBOLS_PATH)

# declare kernel config
CONFIG_VIDEO_MEDIATEK_VCODEC_MT8395=y
CONFIG_VIDEO_MEDIATEK_VCU_MT8395=y
CONFIG_VIDEO_MEDIATEK_VCODEC_V1=y
CONFIG_MTK_IOMMU=y
CONFIG_64BIT=y
CONFIG_ARM64=y
CONFIG_COMPAT=y

top := $(src)
export top
$(info mtk-vcodec-driver/ src is $(src))
$(info mtk-vcodec-driver/ MTK_PLATFORM is $(MTK_PLATFORM))
export MTK_PLATFORM

EXTRA_CFLAGS += -DCONFIG_VIDEO_MEDIATEK_VCODEC_MT8395
EXTRA_CFLAGS += -DCONFIG_VIDEO_MEDIATEK_VCU_MT8395
EXTRA_CFLAGS += -DCONFIG_VIDEO_MEDIATEK_VCODEC_V1
EXTRA_CFLAGS += -DCONFIG_MTK_IOMMU
EXTRA_CFLAGS += -DCONFIG_64BIT
EXTRA_CFLAGS += -DCONFIG_ARM64
EXTRA_CFLAGS += -DCONFIG_COMPAT


# target object
obj-m := mtk-vcodec-common.o
obj-m += mtk-vcodec-dec-v1.o
obj-m += mtk-vcodec-enc-v1.o

# include
EXTRA_CFLAGS += -I$(srctree)/$(src)/
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/mtk-vcu-mt8395/
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/iommu/
# mtk_vcu_controls.h
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/uapi/

# mtk-vcodec-common.ko
mtk-vcodec-common-objs += mtk_vcodec_intr.o
mtk-vcodec-common-objs += mtk_vcodec_util.o

# mtk-vcodec-dec-v1.ko
mtk-vcodec-dec-v1-objs += v1/mtk_vcodec_dec_pm_plat.o
mtk-vcodec-dec-v1-objs += v1/vcodec_dvfs.o
mtk-vcodec-dec-v1-objs += mtk_vcodec_dec_drv.o
mtk-vcodec-dec-v1-objs += vdec_drv_if.o
mtk-vcodec-dec-v1-objs += mtk_vcodec_dec.o
mtk-vcodec-dec-v1-objs += mtk_vcodec_dec_pm.o

ifeq ($(CONFIG_VIDEO_MEDIATEK_VCU_MT8395),y)
mtk-vcodec-dec-v1-objs += vdec/vdec_common_if.o
mtk-vcodec-dec-v1-objs += vdec/vdec_vcu_if.o
endif

# mtk-vcodec-enc-v1.ko
mtk-vcodec-enc-v1-objs += v1/mtk_vcodec_enc_pm_plat.o
mtk-vcodec-enc-v1-objs += v1/vcodec_dvfs.o
mtk-vcodec-enc-v1-objs += mtk_vcodec_enc_drv.o
mtk-vcodec-enc-v1-objs += venc_drv_if.o
mtk-vcodec-enc-v1-objs += mtk_vcodec_enc.o
mtk-vcodec-enc-v1-objs += mtk_vcodec_enc_pm.o

ifeq ($(CONFIG_VIDEO_MEDIATEK_VCU_MT8395),y)
mtk-vcodec-enc-v1-objs += venc/venc_common_if.o
mtk-vcodec-enc-v1-objs += venc/venc_vcu_if.o
endif



all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
