#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

ifeq ($(CONFIG_MTK_TEE_GP_SUPPORT),y)
  ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/include/trustzone/utils/tlutils
  ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/trustzone/utils/tlutils
  ccflags-y += -I$(src)/$(MTK_PLATFORM)
  ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
    $(warning "##### Including Trustonic GP Headers #####")
    include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
  else ifeq ($(CONFIG_MICROTRUST_TEE_SUPPORT),y)
    $(warning "##### Including Microtrust GP Headers #####")
    include $(srctree)/drivers/misc/mediatek/teei/Makefile.include
  else ifeq ($(CONFIG_BLOWFISH_TEE_SUPPORT),y)
    $(warning "##### Including Blowfish CSMC Headers #####")
    ccflags-y += -I$(srctree)/drivers/misc/tzdev
  else
    $(warning "##### Including GP OTHER TEE Headers #####")
  endif
else # Legacy
  ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
    $(warning "##### Including Legacy GUD Headers #####")
    ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/include/trustzone/utils/tlutils
    ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/trustzone/utils/tlutils
    ccflags-y += -I$(src)/$(MTK_PLATFORM)
    include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
  endif
endif # !CONFIG_MTK_TEE_GP_SUPPORT

ifeq ($(CONFIG_MTK_TEE_GP_SUPPORT),y)
  ifeq ($(CONFIG_BLOWFISH_TEE_SUPPORT),y)
    $(warning "##### Using CSMC Secure Memory Driver #####")
    obj-y += secmem_csmc.o
  else
    $(warning "##### Using GP Secure Memory Driver #####")
    obj-y += secmem_gp.o
  endif
else
  $(warning "##### Using Legacy Secure Memory Driver #####")
  obj-y += secmem.o
endif
