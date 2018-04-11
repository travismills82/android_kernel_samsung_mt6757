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

LINUXINCLUDE += -include $(srctree)/kernel/sched/sched.h

#
# RQ stats for TLP estimation
#
# For user space operation
obj-$(CONFIG_MTK_SCHED_RQAVG_US) += rq_stats.o
# For kernel space operation
obj-$(CONFIG_MTK_SCHED_RQAVG_KS) += sched_avg.o

# For Enhance CPU affinity in hotplug
obj-$(CONFIG_MT_CPU_AFFINITY) += mtk_cpu_affinity/
obj-$(CONFIG_MT_RT_THROTTLE_MON) += rt_monitor.o

# For CPU topology to user space
obj-y += cputopo.o

ifeq ($(CONFIG_SCHED_HMP_BOOST),y)
ccflags-y += \
	-I$(srctree)/drivers/misc/mediatek/base/power/ppm_v2 \
	-I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(CONFIG_MTK_PLATFORM)/include \
	-I$(srctree)/drivers/misc/mediatek/base/power/ppm_v2/src/mach/$(CONFIG_MTK_PLATFORM)

obj-y += hmp_boost.o
endif
