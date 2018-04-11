/*
 * Copyright (C) 2016 MediaTek Inc.
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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>

#include "mtk_ppm_internal.h"

#if 0
/* L+ mode utilization during app launch has not given 
 * consistence improvement so we need to revisit the 
 * implementation and results before enabling the below code
 */
#ifdef PPM_TURBO_CORE_SUPPORT

#define MAX_CORE_CAPACITY  (get_cluster_max_cpu_core(PPM_CLUSTER_L) + \
			get_cluster_max_cpu_core(PPM_CLUSTER_LL))

static int lplus_enabled = 0;
#endif
#endif //Fix me

#ifdef CONFIG_SCHED_HMP_BOOST
static int hmp_enabled = 0;

extern int sched_hmp_boost(int);
#endif

static enum ppm_power_state ppm_sysboost_get_power_state_cb(enum ppm_power_state cur_state);
static void ppm_sysboost_update_limit_cb(enum ppm_power_state new_state);
static void ppm_sysboost_status_change_cb(bool enable);
static void ppm_sysboost_mode_change_cb(enum ppm_mode mode);

/* other members will init by ppm_main */
static struct ppm_policy_data sysboost_policy = {
	.name			= __stringify(PPM_POLICY_SYS_BOOST),
	.lock			= __MUTEX_INITIALIZER(sysboost_policy.lock),
	.policy			= PPM_POLICY_SYS_BOOST,
	.priority		= PPM_POLICY_PRIO_PERFORMANCE_BASE,
	.get_power_state_cb	= ppm_sysboost_get_power_state_cb,
	.update_limit_cb	= ppm_sysboost_update_limit_cb,
	.status_change_cb	= ppm_sysboost_status_change_cb,
	.mode_change_cb		= ppm_sysboost_mode_change_cb,
};

struct ppm_sysboost_data {
	enum ppm_sysboost_user user;
	char *user_name;
	/* record user input for old interface */
	unsigned int min_freq;
	unsigned int max_freq;
	unsigned int min_core_num;
	unsigned int max_core_num;

	struct ppm_user_limit limit[NR_PPM_CLUSTERS];
	struct list_head link;
};

static LIST_HEAD(sysboost_user_list);
/* to save each user's request */
static struct ppm_sysboost_data sysboost_data[NR_PPM_SYSBOOST_USER];
/* final limit */
static struct ppm_userlimit_data sysboost_final_limit = {
	.is_freq_limited_by_user = false,
	.is_core_limited_by_user = false,
};

static void ppm_sysboost_dump_final_limit(void *data)
{
	int i;

	if (!data) {
		ppm_dbg(SYS_BOOST, "is_core_limited = %d, is_freq_limited = %d\n",
			sysboost_final_limit.is_core_limited_by_user,
			sysboost_final_limit.is_freq_limited_by_user);
		for_each_ppm_clusters(i) {
			ppm_dbg(SYS_BOOST, "cluster %d = (%d)(%d)(%d)(%d)\n", i,
				sysboost_final_limit.limit[i].min_freq_idx,
				sysboost_final_limit.limit[i].max_freq_idx,
				sysboost_final_limit.limit[i].min_core_num,
				sysboost_final_limit.limit[i].max_core_num);
		}
	} else {
		struct seq_file *m = (struct seq_file *)data;

		seq_printf(m, "is_core_limited = %d, is_freq_limited = %d\n",
			sysboost_final_limit.is_core_limited_by_user,
			sysboost_final_limit.is_freq_limited_by_user);
		for_each_ppm_clusters(i) {
			seq_printf(m, "cluster %d = (%d)(%d)(%d)(%d)\n", i,
				sysboost_final_limit.limit[i].min_freq_idx,
				sysboost_final_limit.limit[i].max_freq_idx,
				sysboost_final_limit.limit[i].min_core_num,
				sysboost_final_limit.limit[i].max_core_num);
		}
	}
}

/* must acquire sysboost_policy lock before calling this functions! */
static void ppm_sysboost_update_final_limit(void)
{
	struct ppm_sysboost_data *data;
	int min_core, max_core, min_freq_idx, max_freq_idx;
	int i;

	/* update user core setting */
	for_each_ppm_clusters(i) {
		min_core = max_core = min_freq_idx = max_freq_idx = -1;

		list_for_each_entry_reverse(data, &sysboost_user_list, link) {
			min_core = MAX(min_core, data->limit[i].min_core_num);
			max_core = (data->limit[i].max_core_num == -1) ? max_core
				: (max_core == -1) ? data->limit[i].max_core_num
				: MIN(max_core, data->limit[i].max_core_num);
			min_freq_idx = (data->limit[i].min_freq_idx == -1) ? min_freq_idx
				: (min_freq_idx == -1) ? data->limit[i].min_freq_idx
				: MIN(min_freq_idx, data->limit[i].min_freq_idx);
			max_freq_idx = MAX(max_freq_idx, data->limit[i].max_freq_idx);
		}

		/* error check */
		if (min_freq_idx != -1 && min_freq_idx < max_freq_idx)
			min_freq_idx = max_freq_idx;
		if (max_core != -1 && min_core > max_core)
			min_core = max_core;

		sysboost_final_limit.limit[i].min_freq_idx = min_freq_idx;
		sysboost_final_limit.limit[i].max_freq_idx = max_freq_idx;
		sysboost_final_limit.limit[i].min_core_num = min_core;
		sysboost_final_limit.limit[i].max_core_num = max_core;
	}

	sysboost_final_limit.is_freq_limited_by_user = false;
	sysboost_final_limit.is_core_limited_by_user = false;
	for_each_ppm_clusters(i) {
		if (!sysboost_final_limit.is_freq_limited_by_user
			&& (sysboost_final_limit.limit[i].min_freq_idx != -1
			|| sysboost_final_limit.limit[i].max_freq_idx != -1)) {
			sysboost_final_limit.is_freq_limited_by_user = true;
		}
		if (!sysboost_final_limit.is_core_limited_by_user
			&& (sysboost_final_limit.limit[i].min_core_num != -1
			|| sysboost_final_limit.limit[i].max_core_num != -1)) {
			sysboost_final_limit.is_core_limited_by_user = true;
		}
		if (sysboost_final_limit.is_freq_limited_by_user
			&& sysboost_final_limit.is_core_limited_by_user)
			break;
	}

	if (!sysboost_final_limit.is_freq_limited_by_user
			&& !sysboost_final_limit.is_core_limited_by_user)
		sysboost_policy.is_activated = false;
	else
		sysboost_policy.is_activated = true;

	ppm_sysboost_dump_final_limit(NULL);
}


void mt_ppm_sysboost_core(enum ppm_sysboost_user user, unsigned int core_num)
{
	struct ppm_sysboost_data *data;
	unsigned int i, tmp_core, target_core = core_num;

	if (core_num > num_possible_cpus() || user >= NR_PPM_SYSBOOST_USER) {
		ppm_err("@%s: Invalid input: user = %d, core_num = %d\n",
			__func__, user, core_num);
		return;
	}

	ppm_lock(&sysboost_policy.lock);

	if (!sysboost_policy.is_enabled) {
		ppm_err("@%s: sysboost policy is not enabled!\n", __func__);
		ppm_unlock(&sysboost_policy.lock);
		return;
	}

	ppm_info("sys boost by %s: req_core = %d\n", sysboost_data[user].user_name, core_num);

	/* update user core setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		if (data->user == user) {
			data->min_core_num = core_num;

			/* clear previous setting */
			if (core_num == 0) {
				for_each_ppm_clusters(i)
					data->limit[i].min_core_num = -1;
				break;
			}

			/* dispatch boost core to each cluster */
			for_each_ppm_clusters(i) {
				if (target_core <= 0)
					data->limit[i].min_core_num = -1;
				else {
					tmp_core = (sysboost_final_limit.limit[i].max_core_num == -1)
						? MIN(target_core, get_cluster_max_cpu_core(i))
						: MIN(target_core, sysboost_final_limit.limit[i].max_core_num);
					data->limit[i].min_core_num = tmp_core;
					target_core -= tmp_core;
				}
			}
		}
	}

	ppm_sysboost_update_final_limit();

	ppm_unlock(&sysboost_policy.lock);

	mt_ppm_main();
}

static void mt_ppm_sysboost_max_core(enum ppm_sysboost_user user, unsigned int core_num)
{
	struct ppm_sysboost_data *data;
	unsigned int i;

	if (core_num > num_possible_cpus() || user >= NR_PPM_SYSBOOST_USER) {
		ppm_err("@%s: Invalid input: user = %d, core_num = %d\n",
			__func__, user, core_num);
		return;
	}

	ppm_lock(&sysboost_policy.lock);

	if (!sysboost_policy.is_enabled) {
		ppm_err("@%s: sysboost policy is not enabled!\n", __func__);
		ppm_unlock(&sysboost_policy.lock);
		return;
	}

	ppm_info("sys boost by %s: req_core = %d\n", sysboost_data[user].user_name, core_num);

	/* update user core setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		if (data->user == user) {
			data->max_core_num = core_num;

			/* clear previous setting */
			if (core_num == 0) {
				for_each_ppm_clusters(i)
					data->limit[i].max_core_num = -1;
				break;
			}

			/* dispatch boost core to each cluster */
			if (ppm_main_info.cur_power_state == PPM_POWER_STATE_L_ONLY
					|| ppm_main_info.cur_power_state == PPM_POWER_STATE_4L_LL) {
				/* set to L first */
				if (core_num > get_cluster_max_cpu_core(PPM_CLUSTER_L)) {
					data->limit[PPM_CLUSTER_L].max_core_num = get_cluster_max_cpu_core(PPM_CLUSTER_L);
					data->limit[PPM_CLUSTER_LL].max_core_num = core_num - get_cluster_max_cpu_core(PPM_CLUSTER_L);
				} else {
					data->limit[PPM_CLUSTER_L].max_core_num = core_num;
					data->limit[PPM_CLUSTER_LL].max_core_num = 0;
				}
			} else {
				if (core_num > get_cluster_max_cpu_core(PPM_CLUSTER_LL)) {
					data->limit[PPM_CLUSTER_LL].max_core_num = get_cluster_max_cpu_core(PPM_CLUSTER_LL);
					data->limit[PPM_CLUSTER_L].max_core_num = core_num - get_cluster_max_cpu_core(PPM_CLUSTER_LL);
				} else {
					data->limit[PPM_CLUSTER_LL].max_core_num = core_num;
					data->limit[PPM_CLUSTER_L].max_core_num = 0;
				}
			}
		}
	}

	ppm_sysboost_update_final_limit();

	ppm_unlock(&sysboost_policy.lock);

	mt_ppm_main();
}

void mt_ppm_sysboost_freq(enum ppm_sysboost_user user, unsigned int freq)
{
	struct ppm_sysboost_data *data;
	int i, freq_idx;

	if (user >= NR_PPM_SYSBOOST_USER) {
		ppm_err("@%s: Invalid input: user = %d, freq = %d\n",
			__func__, user, freq);
		return;
	}

	ppm_lock(&sysboost_policy.lock);

	if (!sysboost_policy.is_enabled) {
		ppm_err("@%s: sysboost policy is not enabled!\n", __func__);
		ppm_unlock(&sysboost_policy.lock);
		return;
	}

	ppm_info("sys boost by %s: req_freq = %d\n", sysboost_data[user].user_name, freq);

	/* update user freq setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		if (data->user == user) {
			data->min_freq = freq;

			/* clear previous setting */
			if (freq == 0) {
				for_each_ppm_clusters(i)
					data->limit[i].min_freq_idx = -1;
				break;
			}

			for_each_ppm_clusters(i) {
				freq_idx = (freq == -1) ? -1
					: ppm_main_freq_to_idx(i, freq, CPUFREQ_RELATION_L);

				/* error check */
				if (data->limit[i].max_freq_idx != -1 && freq_idx != -1
					&& freq_idx < data->limit[i].max_freq_idx)
					freq_idx = data->limit[i].max_freq_idx;

				data->limit[i].min_freq_idx = freq_idx;
			}
		}
	}

	ppm_sysboost_update_final_limit();

	ppm_unlock(&sysboost_policy.lock);

	mt_ppm_main();
}

void mt_ppm_sysboost_max_freq(enum ppm_sysboost_user user, unsigned int freq)
{
	struct ppm_sysboost_data *data;
	int i, freq_idx;

	if (user >= NR_PPM_SYSBOOST_USER) {
		ppm_err("@%s: Invalid input: user = %d, freq = %d\n",
			__func__, user, freq);
		return;
	}

	if (user != BOOST_BY_BOOT_OPT && user != BOOST_BY_UT && user != BOOST_BY_SS)
		return;

	ppm_lock(&sysboost_policy.lock);

	if (!sysboost_policy.is_enabled) {
		ppm_err("@%s: sysboost policy is not enabled!\n", __func__);
		ppm_unlock(&sysboost_policy.lock);
		return;
	}

	ppm_info("sys boost by %s: req_max_freq = %d\n", sysboost_data[user].user_name, freq);

	/* update user freq setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		if (data->user == user) {
			data->max_freq = freq;

			/* clear previous setting */
			if (freq == 0) {
				for_each_ppm_clusters(i) {
					data->limit[i].max_freq_idx = -1;

					/* update min_freq if max_freq becomes higher */
					if (data->min_freq > 0)
						data->limit[i].min_freq_idx =
							ppm_main_freq_to_idx(i, data->min_freq, CPUFREQ_RELATION_L);
				}
				break;
			}

			for_each_ppm_clusters(i) {
				freq_idx = (freq == -1) ? -1
					: ppm_main_freq_to_idx(i, freq, CPUFREQ_RELATION_H);
				
				/* update min_freq if max_freq becomes higher */
				if (data->min_freq > 0)
					data->limit[i].min_freq_idx =
						ppm_main_freq_to_idx(i, data->min_freq, CPUFREQ_RELATION_L);

				data->limit[i].max_freq_idx = freq_idx;
				
				/* error check */
				if (data->limit[i].min_freq_idx != -1 && freq_idx != -1
					&& freq_idx > data->limit[i].min_freq_idx)
					data->limit[i].min_freq_idx = freq_idx;
			}
		}
	}

	ppm_sysboost_update_final_limit();

	ppm_unlock(&sysboost_policy.lock);

	mt_ppm_main();
}

void mt_ppm_sysboost_set_core_limit(enum ppm_sysboost_user user, unsigned int cluster,
					int min_core, int max_core)
{
	struct ppm_sysboost_data *data;

	if (cluster >= NR_PPM_CLUSTERS) {
		ppm_err("@%s: Invalid input: cluster = %d\n", __func__, cluster);
		return;
	}

	if ((max_core != -1 && max_core > get_cluster_max_cpu_core(cluster))
		|| (min_core != -1 && min_core < get_cluster_min_cpu_core(cluster))
		|| user >= NR_PPM_SYSBOOST_USER) {
		ppm_err("@%s: Invalid input: user = %d, cluster = %d, max_core = %d, min_core = %d\n",
			__func__, user, cluster, max_core, min_core);
		return;
	}

	ppm_info("sys boost by %s: cluster %d min/max core = %d/%d\n",
		sysboost_data[user].user_name, cluster, min_core, max_core);

	if (min_core > max_core && max_core != -1)
		min_core = max_core;

	ppm_lock(&sysboost_policy.lock);

	if (!sysboost_policy.is_enabled) {
		ppm_err("@%s: sysboost policy is not enabled!\n", __func__);
		ppm_unlock(&sysboost_policy.lock);
		return;
	}

	/* update user core setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		if (data->user == user) {
			data->limit[cluster].min_core_num = min_core;
			data->limit[cluster].max_core_num = max_core;
		}
	}

	ppm_sysboost_update_final_limit();

	ppm_unlock(&sysboost_policy.lock);

	mt_ppm_main();
}

void mt_ppm_sysboost_set_freq_limit(enum ppm_sysboost_user user, unsigned int cluster,
					int min_freq, int max_freq)
{
	struct ppm_sysboost_data *data;

	if (cluster >= NR_PPM_CLUSTERS) {
		ppm_err("@%s: Invalid input: cluster = %d\n", __func__, cluster);
		return;
	}

	if ((max_freq != -1 && max_freq > get_cluster_max_cpufreq(cluster))
		|| (min_freq != -1 && min_freq < get_cluster_min_cpufreq(cluster))
		|| user >= NR_PPM_SYSBOOST_USER) {
		ppm_err("@%s: Invalid input: user = %d, cluster = %d, max_freq = %d, min_freq = %d\n",
			__func__, user, cluster, max_freq, min_freq);
		return;
	}

	ppm_info("sys boost by %s: cluster %d min/max freq = %d/%d\n",
		sysboost_data[user].user_name, cluster, min_freq, max_freq);

	if (min_freq > max_freq && max_freq != -1)
		min_freq = max_freq;

	ppm_lock(&sysboost_policy.lock);

	if (!sysboost_policy.is_enabled) {
		ppm_err("@%s: sysboost policy is not enabled!\n", __func__);
		ppm_unlock(&sysboost_policy.lock);
		return;
	}

	/* update user freq setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		if (data->user == user) {
			data->limit[cluster].min_freq_idx = (min_freq == -1)
				? -1 : ppm_main_freq_to_idx(cluster, min_freq, CPUFREQ_RELATION_L);
			data->limit[cluster].max_freq_idx = (max_freq == -1)
				? -1 : ppm_main_freq_to_idx(cluster, max_freq, CPUFREQ_RELATION_H);

			/* error check */
			if (data->limit[cluster].min_freq_idx != -1
				&& data->limit[cluster].min_freq_idx < data->limit[cluster].max_freq_idx)
				data->limit[cluster].min_freq_idx = data->limit[cluster].max_freq_idx;
		}
	}

	ppm_sysboost_update_final_limit();

	ppm_unlock(&sysboost_policy.lock);

	mt_ppm_main();
}

static enum ppm_power_state ppm_sysboost_get_power_state_cb(enum ppm_power_state cur_state)
{
	if (sysboost_final_limit.is_core_limited_by_user)
		return ppm_judge_state_by_user_limit(cur_state, sysboost_final_limit);
	else
		return cur_state;
}

static void ppm_sysboost_update_limit_cb(enum ppm_power_state new_state)
{
	unsigned int i;
	struct ppm_policy_req *req = &sysboost_policy.req;

	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: sysboost policy update limit for new state = %s\n",
		__func__, ppm_get_power_state_name(new_state));

	if (sysboost_final_limit.is_freq_limited_by_user || sysboost_final_limit.is_core_limited_by_user) {
		ppm_hica_set_default_limit_by_state(new_state, &sysboost_policy);

		for (i = 0; i < req->cluster_num; i++) {
			req->limit[i].min_cpu_core = (sysboost_final_limit.limit[i].min_core_num == -1)
				? req->limit[i].min_cpu_core
				: sysboost_final_limit.limit[i].min_core_num;
			req->limit[i].max_cpu_core = (sysboost_final_limit.limit[i].max_core_num == -1)
				? req->limit[i].max_cpu_core
				: sysboost_final_limit.limit[i].max_core_num;
			req->limit[i].min_cpufreq_idx = (sysboost_final_limit.limit[i].min_freq_idx == -1)
				? req->limit[i].min_cpufreq_idx
				: sysboost_final_limit.limit[i].min_freq_idx;
			req->limit[i].max_cpufreq_idx = (sysboost_final_limit.limit[i].max_freq_idx == -1)
				? req->limit[i].max_cpufreq_idx
				: sysboost_final_limit.limit[i].max_freq_idx;
		}

		ppm_limit_check_for_user_limit(new_state, req, sysboost_final_limit);

		/* error check */
		for (i = 0; i < req->cluster_num; i++) {
			if (req->limit[i].max_cpu_core < req->limit[i].min_cpu_core)
				req->limit[i].min_cpu_core = req->limit[i].max_cpu_core;
			if (req->limit[i].max_cpufreq_idx > req->limit[i].min_cpufreq_idx)
				req->limit[i].min_cpufreq_idx = req->limit[i].max_cpufreq_idx;
		}
	}

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_sysboost_status_change_cb(bool enable)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: sysboost policy status changed to %d\n", __func__, enable);

	FUNC_EXIT(FUNC_LV_POLICY);
}

static void ppm_sysboost_mode_change_cb(enum ppm_mode mode)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	ppm_ver("@%s: ppm mode changed to %d\n", __func__, mode);

	FUNC_EXIT(FUNC_LV_POLICY);
}

static int ppm_sysboost_core_proc_show(struct seq_file *m, void *v)
{
	struct ppm_sysboost_data *data;

	/* update user core setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link)
		seq_printf(m, "[%d] %s: %d\n", data->user, data->user_name, data->min_core_num);

	ppm_sysboost_dump_final_limit(m);

	return 0;
}

static ssize_t ppm_sysboost_core_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	int user, core;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &user, &core) == 2)
		mt_ppm_sysboost_core((enum ppm_sysboost_user)user, core);
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_sysboost_core_ut_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysboost_data[BOOST_BY_UT].min_core_num);

	return 0;
}

static ssize_t ppm_sysboost_core_ut_proc_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *pos)
{
	int min_core;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	
	if (!kstrtoint(buf, 10, &min_core)) {
#if 0
	/* L+ mode utilization during app launch has not given 
	 * consistence improvement so we need to revisit the 
	 * implementation and results before enabling the below code
	 */
#ifdef PPM_TURBO_CORE_SUPPORT
		if (!lplus_enabled && min_core == MAX_CORE_CAPACITY) {
			ppm_set_smart_detect_boost(1);
			lplus_enabled = 1;
        	}
		else if (lplus_enabled && min_core <= 0) {
			ppm_set_smart_detect_boost(0);
			lplus_enabled = 0;
        	}
#endif
#endif //Fix me
		mt_ppm_sysboost_core(BOOST_BY_UT, min_core);
	}
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_sysboost_max_core_ut_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sysboost_data[BOOST_BY_UT].max_core_num);

	return 0;
}

static ssize_t ppm_sysboost_max_core_ut_proc_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *pos)
{
	int max_core;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;
	
	if (!kstrtoint(buf, 10, &max_core))
		mt_ppm_sysboost_max_core(BOOST_BY_UT, max_core);
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_sysboost_freq_proc_show(struct seq_file *m, void *v)
{
	struct ppm_sysboost_data *data;

	/* update user freq setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link)
		seq_printf(m, "[%d] %s min/max: %d/%d\n",
				data->user, data->user_name, data->min_freq, data->max_freq);

	ppm_sysboost_dump_final_limit(m);

	return 0;
}

static ssize_t ppm_sysboost_freq_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	int user, freq;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &user, &freq) == 2)
		mt_ppm_sysboost_freq((enum ppm_sysboost_user)user, freq);
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_sysboost_max_freq_proc_show(struct seq_file *m, void *v)
{
	struct ppm_sysboost_data *data;

	/* update user freq setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link)
		seq_printf(m, "[%d] %s min/max: %d/%d\n",
				data->user, data->user_name, data->min_freq, data->max_freq);

	ppm_sysboost_dump_final_limit(m);

	return 0;
}

static ssize_t ppm_sysboost_max_freq_proc_write(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	int user, freq;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d", &user, &freq) == 2)
		mt_ppm_sysboost_max_freq((enum ppm_sysboost_user)user, freq);
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_sysboost_cluster_core_limit_proc_show(struct seq_file *m, void *v)
{
	struct ppm_sysboost_data *data;
	int i;

	/* update user core setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		seq_printf(m, "[%d] %s: %d\t", data->user, data->user_name, data->min_core_num);
		for_each_ppm_clusters(i)
			seq_printf(m, "(%d)(%d)", data->limit[i].min_core_num, data->limit[i].max_core_num);
		seq_puts(m, "\n");
	}

	ppm_sysboost_dump_final_limit(m);

	return 0;
}

static ssize_t ppm_sysboost_cluster_core_limit_proc_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *pos)
{
	int user, min_core, max_core, cluster;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d %d %d", &user, &cluster, &min_core, &max_core) == 4)
		mt_ppm_sysboost_set_core_limit((enum ppm_sysboost_user)user, cluster, min_core, max_core);
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int ppm_sysboost_cluster_freq_limit_proc_show(struct seq_file *m, void *v)
{
	struct ppm_sysboost_data *data;
	int i;

	/* update user freq setting */
	list_for_each_entry_reverse(data, &sysboost_user_list, link) {
		seq_printf(m, "[%d] %s: %d\t", data->user, data->user_name, data->min_freq);
		for_each_ppm_clusters(i)
			seq_printf(m, "(%d)(%d)", data->limit[i].min_freq_idx, data->limit[i].max_freq_idx);
		seq_puts(m, "\n");
	}

	ppm_sysboost_dump_final_limit(m);

	return 0;
}

static ssize_t ppm_sysboost_cluster_freq_limit_proc_write(struct file *file,
				const char __user *buffer, size_t count, loff_t *pos)
{
	int user, min_freq, max_freq, cluster;

	char *buf = ppm_copy_from_user_for_proc(buffer, count);

	if (!buf)
		return -EINVAL;

	if (sscanf(buf, "%d %d %d %d", &user, &cluster, &min_freq, &max_freq) == 4)
		mt_ppm_sysboost_set_freq_limit((enum ppm_sysboost_user)user, cluster, min_freq, max_freq);
	else
		ppm_err("@%s: Invalid input!\n", __func__);

	free_page((unsigned long)buf);
	return count;
}

static int cpufreq_max_limit_val = -1;
static int cpufreq_min_limit_val = -1;
static ssize_t cpufreq_max_limit_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", cpufreq_max_limit_val);
}

static ssize_t cpufreq_max_limit_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int val;
	ssize_t ret = -EINVAL;

	if (sscanf(buf, "%d", &val) != 1) {
		pr_err("%s: Invalid cpufreq format\n", __func__);
		goto out;
	}

	mt_ppm_sysboost_max_freq(BOOST_BY_SS, (val == -1)? 0 : val);

	cpufreq_max_limit_val = val;

	ret = count;
out:
	return ret;
}

static ssize_t cpufreq_min_limit_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", cpufreq_min_limit_val);
}

static ssize_t cpufreq_min_limit_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int val;
	ssize_t ret = -EINVAL;

	if (sscanf(buf, "%d", &val) != 1) {
		pr_err("%s: Invalid cpufreq format\n", __func__);
		goto out;
	}

#ifdef CONFIG_SCHED_HMP_BOOST
	if(val >= (signed int)get_cluster_hmp_target_cpufreq(PPM_CLUSTER_LL)) {
		if (!hmp_enabled) {
			sched_hmp_boost(2); /* SEMI booster ON */
			hmp_enabled = 1;
		}
	}
	else if(hmp_enabled) {
		sched_hmp_boost(-2); /* SEMI booster OFF */
		hmp_enabled = 0;
	}
#endif

	mt_ppm_sysboost_freq(BOOST_BY_SS, (val == -1)? 0 : val);

	cpufreq_min_limit_val = val;

	ret = count;
out:
	return ret;
}

PROC_FOPS_RW(sysboost_core);
PROC_FOPS_RW(sysboost_core_ut);
PROC_FOPS_RW(sysboost_max_core_ut);
PROC_FOPS_RW(sysboost_freq);
PROC_FOPS_RW(sysboost_max_freq);
PROC_FOPS_RW(sysboost_cluster_core_limit);
PROC_FOPS_RW(sysboost_cluster_freq_limit);

static DEVICE_ATTR(cpufreq_min_limit, 0660, cpufreq_min_limit_show, cpufreq_min_limit_store);
static DEVICE_ATTR(cpufreq_max_limit, 0660, cpufreq_max_limit_show, cpufreq_max_limit_store);

static struct attribute *g[] = {
	&dev_attr_cpufreq_min_limit.attr,
	&dev_attr_cpufreq_max_limit.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int __init ppm_sysboost_policy_init(void)
{
	int i, j, ret = 0;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(sysboost_core),
		PROC_ENTRY(sysboost_core_ut),
		PROC_ENTRY(sysboost_max_core_ut),
		PROC_ENTRY(sysboost_freq),
		PROC_ENTRY(sysboost_max_freq),
		PROC_ENTRY(sysboost_cluster_core_limit),
		PROC_ENTRY(sysboost_cluster_freq_limit),
	};

	FUNC_ENTER(FUNC_LV_POLICY);

	/* create procfs */
	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, S_IRUGO | S_IWUSR | S_IWGRP, policy_dir, entries[i].fops)) {
			ppm_err("%s(), create /proc/ppm/policy/%s failed\n", __func__, entries[i].name);
			ret = -EINVAL;
			goto out;
		}
	}

	/* allocate mem and init policy limit_data */
	sysboost_final_limit.limit =
		kcalloc(ppm_main_info.cluster_num, sizeof(*sysboost_final_limit.limit), GFP_KERNEL);
	if (!sysboost_final_limit.limit) {
		ret = -ENOMEM;
		goto out;
	}

	for_each_ppm_clusters(i) {
		sysboost_final_limit.limit[i].min_freq_idx = -1;
		sysboost_final_limit.limit[i].max_freq_idx = -1;
		sysboost_final_limit.limit[i].min_core_num = -1;
		sysboost_final_limit.limit[i].max_core_num = -1;
	}

	/* list init */
	for (i = 0; i < NR_PPM_SYSBOOST_USER; i++) {
		sysboost_data[i].user = (enum ppm_sysboost_user)i;

		switch (sysboost_data[i].user) {
		case BOOST_BY_WIFI:
			sysboost_data[i].user_name = "WIFI";
			break;
		case BOOST_BY_PERFSERV:
			sysboost_data[i].user_name = "PERFSERV";
			break;
		case BOOST_BY_USB:
			sysboost_data[i].user_name = "USB";
			break;
		case BOOST_BY_USB_PD:
			sysboost_data[i].user_name = "USB_PD";
			break;
		case BOOST_BY_DEBUGD:
			sysboost_data[i].user_name = "DEBUGD";
			break;
		case BOOST_BY_DEBUGD_64:
			sysboost_data[i].user_name = "DEBUGD_64";
			break;
		case BOOST_BY_BOOT_OPT:
			sysboost_data[i].user_name = "BOOT_OPT";
			break;
		case BOOST_BY_SS:
			sysboost_data[i].user_name = "SS";
			break;
		case BOOST_BY_FPSENSOR:
			sysboost_data[i].user_name = "FPSENSOR";
			break;
		case BOOST_BY_HMP_BOOST:
			sysboost_data[i].user_name = "HMP_BOOST";
			break;

		case BOOST_BY_UT:
		default:
			sysboost_data[i].user_name = "UT";
			break;
		}

		for_each_ppm_clusters(j) {
			sysboost_data[i].limit[j].min_freq_idx = -1;
			sysboost_data[i].limit[j].max_freq_idx = -1;
			sysboost_data[i].limit[j].min_core_num = -1;
			sysboost_data[i].limit[j].max_core_num = -1;
		}

		list_add(&sysboost_data[i].link, &sysboost_user_list);
	}

	if (ppm_main_register_policy(&sysboost_policy)) {
		ppm_err("@%s: sysboost policy register failed\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	/* default set to max cpu freq/core for boot time optimization */
	for_each_ppm_clusters(i) {
		sysboost_data[BOOST_BY_BOOT_OPT].limit[i].min_freq_idx = get_cluster_max_cpufreq_idx(i);
		sysboost_data[BOOST_BY_BOOT_OPT].limit[i].min_core_num = get_cluster_max_cpu_core(i);
	}
	ppm_sysboost_update_final_limit();

	ret = sysfs_create_group(power_kobj, &attr_group);

	ppm_info("@%s: register %s done!\n", __func__, sysboost_policy.name);

out:
	FUNC_EXIT(FUNC_LV_POLICY);

	return ret;
}

static void __exit ppm_sysboost_policy_exit(void)
{
	FUNC_ENTER(FUNC_LV_POLICY);

	kfree(sysboost_final_limit.limit);

	ppm_main_unregister_policy(&sysboost_policy);

	FUNC_EXIT(FUNC_LV_POLICY);
}

late_initcall(ppm_sysboost_policy_init);
module_exit(ppm_sysboost_policy_exit);

