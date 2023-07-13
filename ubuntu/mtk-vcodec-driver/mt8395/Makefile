SRC := $(shell pwd)

# extra options
MTK_PLATFORM = $(TARGET_PLATFORM)
KBUILD_EXTRA_SYMBOLS_PATH = $(EXTRA_SYMBOLS_PATH)

# declare kernel config
CONFIG_VIDEO_MEDIATEK_VCODEC_MT8395=y
CONFIG_VIDEO_MEDIATEK_VCU_MT8395=y
CONFIG_MTK_IOMMU=y
CONFIG_64BIT=y
CONFIG_ARM64=y
CONFIG_COMPAT=y


# The driver support multiple platform with v1 architecture while DVFS & EMI_BW are disabled.
# *NEED-TO-CHECK* if trying to enable DVFS & EMI_BW features (architecture dependent)
MTK_PLATFORM_VER=v1
MTK_IOT_PLATFORM=mt8395

top := $(src)
export top
$(info mtk-vcodec-driver/ src is $(src))
$(info mtk-vcodec-driver/ MTK_PLATFORM is $(MTK_PLATFORM))
export MTK_PLATFORM

EXTRA_CFLAGS += -DCONFIG_VIDEO_MEDIATEK_VCODEC_MT8395
EXTRA_CFLAGS += -DCONFIG_VIDEO_MEDIATEK_VCU_MT8395
EXTRA_CFLAGS += -DCONFIG_VIDEO_MEDIATEK_VCODEC_$(MTK_PLATFORM_VER)
EXTRA_CFLAGS += -DCONFIG_MTK_IOMMU
EXTRA_CFLAGS += -DCONFIG_64BIT
EXTRA_CFLAGS += -DCONFIG_ARM64
EXTRA_CFLAGS += -DCONFIG_COMPAT


# target object
obj-m := mtk-vcodec-common-$(MTK_IOT_PLATFORM).o
obj-m += mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM).o
obj-m += mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM).o

# include
EXTRA_CFLAGS += -I$(srctree)/$(src)/
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/mtk-vcu-mt8395/
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/iommu/
# mtk_vcu_controls.h
EXTRA_CFLAGS += -I$(srctree)/$(src)/external_include/uapi/

# mtk-vcodec-common.ko
mtk-vcodec-common-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_intr.o
mtk-vcodec-common-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_util.o

# mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_PLATFORM).ko
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += $(MTK_PLATFORM_VER)/mtk_vcodec_dec_pm_plat.o
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += $(MTK_PLATFORM_VER)/vcodec_dvfs.o
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_dec_drv.o
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += vdec_drv_if.o
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_dec.o
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_dec_pm.o

ifeq ($(CONFIG_VIDEO_MEDIATEK_VCU_MT8395),y)
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += vdec/vdec_common_if.o
mtk-vcodec-dec-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += vdec/vdec_vcu_if.o
endif

# mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_PLATFORM).ko
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += $(MTK_PLATFORM_VER)/mtk_vcodec_enc_pm_plat.o
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += $(MTK_PLATFORM_VER)/vcodec_dvfs.o
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_enc_drv.o
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += venc_drv_if.o
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_enc.o
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += mtk_vcodec_enc_pm.o

ifeq ($(CONFIG_VIDEO_MEDIATEK_VCU_MT8395),y)
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += venc/venc_common_if.o
mtk-vcodec-enc-$(MTK_PLATFORM_VER)-$(MTK_IOT_PLATFORM)-objs += venc/venc_vcu_if.o
endif



all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c *.mod
	rm -rf vdec/*.o vdec/*.cmd vdec/.*.cmd
	rm -rf venc/*.o venc/*.cmd venc/.*.cmd
	rm -rf v1/*.o v1/*.cmd v1/.*.cmd
	rm -rf v2/*.o v2/*.cmd v2/.*.cmd
	rm -rf Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
