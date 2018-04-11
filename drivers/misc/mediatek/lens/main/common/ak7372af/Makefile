#
# Makefile for misc devices that really don't fit anywhere else.
#
ifeq ($(CONFIG_ARCH_MTK_PROJECT), "titan6757_c10_n")
ccflags-y += -DAF_C10
$(info cam_cal: AK7372_C10 macro AF_C10 defined)
endif
subdir-ccflags-$(CONFIG_CAMERA_HW_BIG_DATA) += -I$(srctree)/drivers/misc/mediatek/include/hwparam
obj-y += AK7372AF.o


