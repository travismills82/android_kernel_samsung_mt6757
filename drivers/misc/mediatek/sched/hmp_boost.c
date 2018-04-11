/*
 * Copyright (C) 2017 Samsung, Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/mutex.h>
#include <linux/atomic.h>

#include <mach/mtk_ppm_api.h>
#include <inc/mtk_ppm_internal.h>

#include "hmp_boost.h"
#define	HMP_BOOST_CLUSTER	0

static DEFINE_MUTEX(boost_hmp_mutex);
static int bkup_fixed_root_cluster = -1;

atomic_t boost_refcount[MAX_NUM_HMP_BOOST_STATE] =  {
		[HMP_BOOST_FULL_ENABLE] = ATOMIC_INIT(0),
		[HMP_BOOST_SEMI_ENABLE] = ATOMIC_INIT(0), };

static void _sched_hmp_boost(int type)
{
	int full_on, semi_on;
	
	full_on = atomic_read(&boost_refcount[HMP_BOOST_FULL_ENABLE]);
	semi_on = atomic_read(&boost_refcount[HMP_BOOST_SEMI_ENABLE]);
		
	switch (type) {
	case HMP_BOOST_NONE:
		atomic_set(&boost_refcount[HMP_BOOST_FULL_ENABLE], 0);
		atomic_set(&boost_refcount[HMP_BOOST_SEMI_ENABLE], 0);
		
		if (full_on || semi_on)
			mt_ppm_sysboost_set_core_limit(BOOST_BY_HMP_BOOST,
				PPM_CLUSTER_L, -1, -1);
		break;
	case HMP_BOOST_FULL_ENABLE:
		atomic_inc(&boost_refcount[HMP_BOOST_FULL_ENABLE]);
		
		if (!full_on) {
			/* at least 4L core online */
			mt_ppm_sysboost_set_core_limit(
						BOOST_BY_HMP_BOOST, PPM_CLUSTER_L, 4, -1);
		}
		break;
	case HMP_BOOST_SEMI_ENABLE:
		atomic_inc(&boost_refcount[HMP_BOOST_SEMI_ENABLE]);

		if (!full_on && !semi_on) {
			/* Fix root cluster as zero */
			bkup_fixed_root_cluster = ppm_main_info.fixed_root_cluster;
			if(bkup_fixed_root_cluster == -1) {
				if (ppm_hica_root_cluster_changed(HMP_BOOST_CLUSTER) == -1)
					ppm_err("[HMP_BOOST] changing root cluster to LITTLE fail!!!");
			}
			/* at least 1L core online */
			mt_ppm_sysboost_set_core_limit(
						BOOST_BY_HMP_BOOST, PPM_CLUSTER_L, 1, -1);
		}
		break;
	case HMP_BOOST_FULL_DISABLE:
		if (full_on) {
			atomic_dec(&boost_refcount[HMP_BOOST_FULL_ENABLE]);
			
			if (full_on == 1) {
				if (semi_on) {
					/* at least 1L core online */
					mt_ppm_sysboost_set_core_limit(
						BOOST_BY_HMP_BOOST, PPM_CLUSTER_L, 1, -1);					
				}
				else {
					/* release core lock */					
					mt_ppm_sysboost_set_core_limit(
						BOOST_BY_HMP_BOOST, PPM_CLUSTER_L, -1, -1);
				}
			}
		}
		break;
	case HMP_BOOST_SEMI_DISABLE:
		if (semi_on) {
			atomic_dec(&boost_refcount[HMP_BOOST_SEMI_ENABLE]);
			
			if (!full_on && (semi_on == 1)) {
				/* release core lock */
				mt_ppm_sysboost_set_core_limit(
					BOOST_BY_HMP_BOOST, PPM_CLUSTER_L, -1, -1);
				/* revert the root cluster Fix to zero */				
				if(bkup_fixed_root_cluster == -1 && ppm_main_info.fixed_root_cluster == HMP_BOOST_CLUSTER) {
					if (ppm_hica_root_cluster_changed(bkup_fixed_root_cluster) == -1)
						ppm_err("[HMP_BOOST]restoring to previous root cluster=%d fail!!!", bkup_fixed_root_cluster);
				}
			}
		}
		break;
	default:
		WARN_ON(1);
		return;
	}
}

int sched_hmp_boost_on(int type)
{
	return atomic_read(&boost_refcount[type]);
}

int sched_hmp_boost(int type)
{
	if ((type < MIN_HMP_BOOST_STATE) || (type > MAX_HMP_BOOST_STATE))
		return -EINVAL;

	mutex_lock(&boost_hmp_mutex);

	_sched_hmp_boost(type);

	mutex_unlock(&boost_hmp_mutex);

	return 0;
}
