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


ifneq ($(MTK_ROOT_BUILD),)
    include $(MTK_ROOT_BUILD)/Makefile
else
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
    ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/utils/tlutils
    include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
    obj-y += secwidevinemdw.o
else
    ifeq ($(CONFIG_MICROTRUST_TEE_SUPPORT),y)
        include $(srctree)/drivers/misc/mediatek/teei/Makefile.include
        obj-y += secwidevinemdw_mt_teei.o
    else
        ifeq ($(CONFIG_BLOWFISH_TEE_SUPPORT),y)
            $(info "Build blowfish widevine modular drm driver")
            ccflags-y += -I$(srctree)/drivers/misc/tzdev/
            obj-y += secwidevinemdw_mt_blowfish.o
        endif
    endif
endif
