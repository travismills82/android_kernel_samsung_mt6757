/*
 * Copyright (C) 2017 MediaTek Inc.
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

/**
 * @file   secwidevinemdw_mt_blowfish.c
 * @brief  Open widevine modular drm secure driver and receive command from secure driver
 * @Author Bo Ye
 *
 **/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/switch.h>
#include <linux/completion.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>

#include "secwidevinemdw.h"
#include "tz_kernel_api.h"
#include "tz_iwnotify.h"
#include "tzdev.h"

#define secwidevinemdw_NAME     "secwidevinemdw"
#define DEFAULT_HANDLES_NUM (64)
#define MAX_OPEN_SESSIONS   10
#define BLOWFISH_DRV_OK  0

/* Debug message event */
#define DBG_EVT_NONE        (0)       /* No event */
#define DBG_EVT_CMD         (1 << 0)  /* SEC CMD related event */
#define DBG_EVT_FUNC        (1 << 1)  /* SEC function event */
#define DBG_EVT_INFO        (1 << 2)  /* SEC information event */
#define DBG_EVT_WRN         (1 << 30) /* Warning event */
#define DBG_EVT_ERR         (1 << 31) /* Error event */
#define DBG_EVT_ALL         (0xffffffff)

#define DBG_EVT_MASK        (DBG_EVT_ALL)

#define MSG(evt, fmt, args...) \
do {    \
	if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
		pr_debug("[secwidevinemdw_mt_blowfish_normalwd][%s] "fmt, secwidevinemdw_NAME, ##args); \
	}   \
} while (0)

#define MSG_FUNC() MSG(FUNC, "%s\n", __func__)

#define HDCP_VERSION_ANY 0
#define HDCP_VERSION_1_0 100
#define HDCP_VERSION_2_0 200
#define HDCP_VERSION_2_1 210
#define HDCP_VERSION_2_2 220

struct task_struct *secwidevinemdwDci_th;

#define CMD_SEC_WIDEVINE_NOTIFY_HWC 1
#define TEE_SUCCESS 0

#define IWNOTIFY_FLAG_WIDEVINE_KCMD TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_11
#define IWNOTIFY_FLAG_WIDEVINE_HDCP TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_12

struct widevinemdw_req_t {
	unsigned int requiredhdcpversion;
	unsigned int currenthdcpversion;
};

/*
 * DCI message data.
 */
struct dciMessage_t {
	uint32_t cmd;
	struct widevinemdw_req_t request;
	uint32_t iwshmem_id;
	uint32_t result;
};

static struct tz_uuid uuid = {
		.time_low = 0x00000000,
		.time_mid = 0x4D54,
		.time_hi_and_version = 0x4B5F,
		.clock_seq_and_node = {0x42, 0x46, 0x57, 0x56, 0x4E, 0x54, 0x41, 0x00}
};

static DEFINE_MUTEX(secwidevinemdw_lock);

static DECLARE_COMPLETION(kcmd_event);
static int __kcmd_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
	MSG(INFO, "Complete kcmd_event\n");
	complete(&kcmd_event);
	return 0;
}

static struct notifier_block kcmd_notifier = {
	.notifier_call = __kcmd_notifier,
};

static DECLARE_COMPLETION(hdcp_event);
static int __hdcp_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
	MSG(INFO, "Complete hdcp_event\n");
	complete(&hdcp_event);
	return 0;
}

static struct notifier_block hdcp_notifier = {
	.notifier_call = __hdcp_notifier,
};

static unsigned int secwidevinemdw_session_ref;
static struct dciMessage_t msg;
static struct switch_dev secwidevinemdw_switch_data;

static int secwidevinemdw_execute(unsigned int cmd)
{

	mutex_lock(&secwidevinemdw_lock);
	switch (cmd) {
	case CMD_SEC_WIDEVINE_NOTIFY_HWC:
		MSG(INFO, "%s: switch_set_state:%u\n",
			__func__, msg.request.requiredhdcpversion);
		switch_set_state(&secwidevinemdw_switch_data,
			msg.request.requiredhdcpversion);
		/*Send uevent to notify hwc*/
		break;
	default:
		MSG(ERR, "Unkonw cmd\n");
		break;
	}

	mutex_unlock(&secwidevinemdw_lock);

	return 0;
}

static int secwidevinemdw_listenDci(void *data)
{
	int sRet = TEE_SUCCESS;
	unsigned int cmdId;
	unsigned int clientId = 0;
	int ret = 0;
	struct page *shmem_page;
	int shmem_id = 0;
	char *shmem;
	u32 *received_data;

	while (!tzdev_is_up()) {
		MSG(INFO, "%s: blowfish is not ready. to sleep\n", __func__);
		msleep(1000);
	}
	MSG(INFO, "%s: DCI listener.\n", __func__);
	init_completion(&kcmd_event);
	init_completion(&hdcp_event);

	MSG(INFO, "%s: tz_iwnotify_chain_register()\n", __func__);
	sRet = tz_iwnotify_chain_register(IWNOTIFY_FLAG_WIDEVINE_KCMD, &kcmd_notifier);
	if (sRet < 0) {
		MSG(ERR, "%s: tz_iwnotify_chain_register failed. ret = %d\n", __func__, sRet);
		return sRet;
	}
	sRet = tz_iwnotify_chain_register(IWNOTIFY_FLAG_WIDEVINE_HDCP, &hdcp_notifier);
	if (sRet < 0) {
		MSG(ERR, "%s: tz_iwnotify_chain_register hdcp notifier failed. ret = %d\n", __func__, sRet);
		goto out_iwnotify_send;
	}

	MSG(INFO, "%s: tz_iwnotify_chain_register() SUCCESS\n", __func__);
	clientId = tzdev_kapi_open(&uuid);
	if (clientId < 0) {
		MSG(ERR, "%s: tzdev_kapi_open failed - %d\n", __func__, clientId);
		goto out_iwnotify_hdcp;
	}

	MSG(INFO, "%s: clientId = %d\n", __func__, clientId);
	shmem_page = alloc_page(GFP_KERNEL);
	if (!shmem_page) {
		MSG(ERR, "%s alloc page failed", __func__);
		goto out_close;
	}
	shmem = page_address(shmem_page);
	memset(shmem, 0, PAGE_SIZE);
	sRet = tzdev_mem_register(shmem, PAGE_SIZE, 1);
	if (sRet < 0) {
		MSG(ERR, "%s register memory failed", __func__);
		goto out_free;
	}

	shmem_id = sRet;
	MSG(INFO, "%s: memory grant with shmem_id:%d\n", __func__, shmem_id);
	sRet = tzdev_kapi_mem_grant(clientId, shmem_id);
	if (sRet < 0) {
		MSG(ERR, "%s grant permission to memory failed", __func__);
		goto out_unregister;
	}

	MSG(INFO, "%s: kernel api send message to ta\n", __func__);
	msg.cmd = 0;
	msg.request.requiredhdcpversion = 0;
	msg.iwshmem_id = shmem_id;

	sRet = tzdev_kapi_send(clientId, &msg, sizeof(msg));
	if (sRet < 0) {
		ret = sRet;
		MSG(ERR, "%s  tzdev_kapi_send failed - %d\n", __func__, ret);
		goto out_ungrant;
	}

	MSG(INFO, "%s: wait_for_completion(secure ta)_interruptible_timeout()\n", __func__);
	sRet = wait_for_completion_interruptible_timeout(&kcmd_event, 5 * HZ);
	/*if wait intteruptible with timeout, return > 0)*/
	/*if wait intteruptible without timeout, return >= 0)*/
	if (sRet <= 0) {
		MSG(ERR, "%s: wait_for_completion(secure ta)_interruptile_timeout failed, sRet=%d\n", __func__, sRet);
		ret = sRet;
		goto out_ungrant;
	}
	sRet = tzdev_kapi_recv(clientId, &msg, sizeof(msg));
	if (sRet < 0) {
		ret = sRet;
		MSG(ERR, "%s: tzdev_kapi_recv failed - %d\n", __func__, ret);
		goto out_ungrant;
	}

	if (msg.result) {
		MSG(ERR,  "%s: Bad msg.result from TA - %u\n", __func__, msg.result);
		ret = -EPERM;
		goto out_ungrant;
	}

	for (;;) {
		MSG(INFO, "%s: Waiting for notification\n", __func__);
		sRet = wait_for_completion_interruptible(&hdcp_event);
		/*if wait intteruptible with timeout, return > 0)*/
		/*if wait intteruptible without timeout, return >= 0)*/
		if (sRet < 0) {
			MSG(ERR, "%s: wait_for_completion(hdcp)_interruptile failed, sRet=%d\n", __func__, sRet);
			ret = sRet;
			break;
		}

		received_data = (u32 *)shmem;
		cmdId = received_data[0];
		msg.request.requiredhdcpversion = received_data[1];
		msg.request.currenthdcpversion = received_data[2];
		MSG(INFO, "%s: wait notification done!! cmdId = 0x%x, current = 0x%x, required = 0x%x\n",
			__func__, cmdId, received_data[2], received_data[1]);
		sRet = secwidevinemdw_execute(cmdId);
	}

out_ungrant:
	MSG(INFO, "%s: unregister memory", __func__);
	tzdev_kapi_mem_revoke(clientId, shmem_id);
	shmem_id = 0;

out_unregister:
	MSG(INFO, "%s: unregister memory", __func__);
	tzdev_mem_release(shmem_id);
	shmem_id = 0;

out_free:
	MSG(INFO, "%s: free kernel memory", __func__);
	__free_page(shmem_page);

out_close:
	MSG(INFO, "%s: tzdev_kapi_close()\n", __func__);
	sRet = tzdev_kapi_close(clientId);
	if (sRet < 0) {
		ret = sRet;
		MSG(ERR, "%s: tzdev_kapi_close failed - %d\n", __func__, sRet);
	}

out_iwnotify_hdcp:
	MSG(INFO, "%s: tz_iwnotify(hdcp)_chain_unregister()\n", __func__);
	sRet = tz_iwnotify_chain_unregister(IWNOTIFY_FLAG_WIDEVINE_HDCP, &hdcp_notifier);
	if (sRet < 0) {
		ret = sRet;
		MSG(ERR, "%s: tz_iwnotify(hdcp)_chain_unregister failed - %d\n", __func__, ret);
	}

out_iwnotify_send:
	MSG(INFO, "%s: tz_iwnotify(kcmd)_chain_unregister()\n", __func__);
	sRet = tz_iwnotify_chain_unregister(IWNOTIFY_FLAG_WIDEVINE_KCMD, &kcmd_notifier);
	if (sRet < 0) {
		ret = sRet;
		MSG(ERR, "%s: tz_iwnotify(kcmd)_chain_unregister send iwnotify failed - %d\n", __func__, ret);
	}
	return ret;
}

/*Open driver in open*/
static int secwidevinemdw_session_open(void)
{
	int sRet = TEE_SUCCESS;

	mutex_lock(&secwidevinemdw_lock);

	do {
		/* sessions reach max numbers ? */
		if (secwidevinemdw_session_ref > MAX_OPEN_SESSIONS) {
			MSG(WRN, "secwidevinemdw_session > 0x%x\n", MAX_OPEN_SESSIONS);
			break;
		}

		if (secwidevinemdw_session_ref > 0) {
			MSG(WRN, "secwidevinemdw_session already open");
			secwidevinemdw_session_ref++;
			break;
		}

		/* create a thread for listening DCI signals */
		secwidevinemdwDci_th = kthread_run(secwidevinemdw_listenDci, NULL, "secwidevinemdw_Dci");
		if (IS_ERR(secwidevinemdwDci_th)) {
			MSG(ERR, "%s, init kthread_run failed!\n", __func__);
			break;
		}
		secwidevinemdw_session_ref = 1;

	} while (0);

	MSG(INFO, "secwidevinemdw_session_open: ret=%d, ref=%d\n", sRet, secwidevinemdw_session_ref);

	mutex_unlock(&secwidevinemdw_lock);

	if (sRet != TEE_SUCCESS) {
		MSG(ERR, "secwidevinemdw_session_open fail");
		return -ENXIO;
	}

	return 0;
}

/*Close trustlet and driver*/
static int secwidevinemdw_session_close(void)
{
	int sRet = TEE_SUCCESS;

	mutex_lock(&secwidevinemdw_lock);

	do {
		/* session is already closed ? */
		if (secwidevinemdw_session_ref == 0) {
			MSG(WRN, "secwidevinemdw_session already closed\n");
			break;
		}

		if (secwidevinemdw_session_ref > 1) {
			secwidevinemdw_session_ref--;
			break;
		}
	} while (0);

	MSG(INFO, "secwidevinemdw_session_close: ret=%d, ref=%d\n", sRet, secwidevinemdw_session_ref);

	mutex_unlock(&secwidevinemdw_lock);

	return 0;
}

static int secwidevinemdw_open(struct inode *inode, struct file *file)
{
	/* open session */
	if (secwidevinemdw_session_open() < 0) {
		MSG(ERR, "secwidevinemdw_open fail - secwidevinemdw_session_open fail");
		return -ENXIO;
	}
	return 0;
}

static int secwidevinemdw_release(struct inode *inode, struct file *file)
{
	int ret = 0;

	ret = secwidevinemdw_session_close();
	return ret;
}

static ssize_t secwidevinemdw_read(struct file *file, char *buf, size_t size,
		loff_t *offset)
{
	/*ignore offsset*/
	unsigned int hdcpversion = msg.request.currenthdcpversion;
	int ret = 0;

	if (size < sizeof(hdcpversion))
		return -1;
	memset(buf, 0, sizeof(hdcpversion));
	ret = copy_to_user(buf, &hdcpversion, sizeof(hdcpversion));
	/*MSG(INFO, "secwidevinemdw_read: hdcpversion = %d, copy result: %d\n", hdcpversion, ret);*/
	return sizeof(hdcpversion);
}

static const struct file_operations secwidevinemdw_fops = {
		.owner = THIS_MODULE,
		.open = secwidevinemdw_open,
		.release = secwidevinemdw_release,
		.unlocked_ioctl = NULL,
		.write = NULL,
		.read = secwidevinemdw_read,
};

static int __init secwidevinemdw_init(void)
{
	int ret = 0;

	proc_create("secwidevinemdw1", (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH),
			NULL, &secwidevinemdw_fops);

	/*To support modular drm v9, inform HWC the event*/
	secwidevinemdw_switch_data.name  = "widevine";
	secwidevinemdw_switch_data.index = 0;
	secwidevinemdw_switch_data.state = HDCP_VERSION_ANY;
	MSG(INFO, "secwidevinemdw_session_open: switch_dev_register");
	ret = switch_dev_register(&secwidevinemdw_switch_data);

	if (ret)
		MSG(INFO, "[secwidevinemdw]switch_dev_register failed, returned:%d!\n", ret);

#if 0 /*Test purpose without really opened from native layer*/
	secwidevinemdw_session_open();
#endif

	return 0;
}

late_initcall(secwidevinemdw_init);
