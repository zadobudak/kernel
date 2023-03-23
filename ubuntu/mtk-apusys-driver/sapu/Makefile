ifneq ($(filter y m,$(CONFIG_MTK_GZ_TZ_SYSTEM)),)

# common part
ccflags-y += -I$(srctree)/drivers/dma-buf/heaps
ccflags-y += -I$(srctree)/drivers/misc/mediatek/geniezone/public

# sapu
SRC := sapu_init.o
SRC += sapu_driver.o

obj-$(CONFIG_MTK_APUSYS_SUPPORT) += sapu.o
sapu-objs = $(SRC)

endif
