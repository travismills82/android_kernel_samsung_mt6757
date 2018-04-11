/*
 * Copyright (C) 2015 MediaTek Inc.
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
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "flashlight.h"
#include "flashlight-dt.h"


/* device tree should be defined in flashlight-dt.h */
#ifndef SM5705_DTNAME
#define SM5705_DTNAME "mediatek,flashlights_sm5705"
#endif
#define SM5705_NAME "flashlights-sm5705"

#define CONFIG_LEDS_SM5705_FLASH 1
#ifndef CONFIG_LEDS_SM5705_FLASH
static int g_bLtVersion;
#endif


/* define channel, level */
#define SM5705_CHANNEL_NUM 2
#define SM5705_CHANNEL_CH1 0
#define SM5705_CHANNEL_CH2 1
/* define mutex and work queue */
static DEFINE_MUTEX(sm5705_mutex);
static struct work_struct sm5705_work_ch1;
static struct work_struct sm5705_work_ch2;
/* define usage count */
static int use_count;
static int sm5705_level_ch1 = -1;
static int sm5705_level_ch2 = -1;
static int gDuty = 0;

extern int sm5705_fled_flash_on(unsigned char index);
extern int sm5705_fled_led_off(unsigned char index);
extern int sm5705_fled_prepare_flash(unsigned char index);
extern int sm5705_fled_torch_on(unsigned char index);
extern int sm5705_fled_close_flash(unsigned char index);





static void __exit sm5705_flash_exit(void);
static int __init sm5705_flash_init(void);


/* flashlight enable function */
static int sm5705_enable_ch1(void)
{
	if(gDuty==3){//Capture
		printk("[DHL]Main Flash_ch1\n");
		sm5705_fled_flash_on(0);	
	}
	else{
		sm5705_fled_torch_on(0);
	}
	return 0;
}

static int sm5705_enable_ch2(void)
{ 
	if(gDuty==3){//Capture
		printk("[DHL]Main Flash_ch2\n");
		sm5705_fled_flash_on(1);	
	}
	else{
		sm5705_fled_torch_on(1);
	}
	return 0;
}

static int sm5705_enable(int channel)
{
	fl_err("sm5705_enable = %d\n",channel);

	if (channel == SM5705_CHANNEL_CH1)
		sm5705_enable_ch1();
	else if (channel == SM5705_CHANNEL_CH2)
		sm5705_enable_ch2();
	else {
		fl_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* flashlight disable function */
static int sm5705_disable_ch1(void)
{
	fl_err("sm5705_disable_ch1\n");
	sm5705_fled_close_flash(0);
	return 1;
}

static int sm5705_disable_ch2(void)
{
	fl_err("sm5705_disable_ch2\n");
	sm5705_fled_close_flash(1);
	return 1;
}

static int sm5705_disable(int channel)
{
	if (channel == SM5705_CHANNEL_CH1)
		sm5705_disable_ch1();
	else if (channel == SM5705_CHANNEL_CH2)
		sm5705_disable_ch2();
	else {
		fl_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* set flashlight level */
static int sm5705_set_level_ch1(int level)
{
	int ret = 0;

	fl_err("sm5705_set_level_ch1. %d\n", level);
	sm5705_fled_prepare_flash(0);
	sm5705_level_ch1 = level;
	return ret;
}

int sm5705_set_level_ch2(int level)
{
	int ret = 0;
	fl_err("sm5705_set_level_ch2. %d\n", level);
	sm5705_fled_prepare_flash(1);
	sm5705_level_ch2 = level;	
	return ret;
}

static int sm5705_set_level(int channel, int level)
{
	if (channel == SM5705_CHANNEL_CH1)
		sm5705_set_level_ch1(level);
	else if (channel == SM5705_CHANNEL_CH2)
		sm5705_set_level_ch2(level);
	else {
		fl_err("Error channel\n");
		return -1;
	}
	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer sm5705_timer_ch1;
static struct hrtimer sm5705_timer_ch2;
static unsigned int sm5705_timeout_ms[SM5705_CHANNEL_NUM];


#if 1
static void sm5705_work_disable_ch1(struct work_struct *data)
{
	fl_dbg("ht work queue callback\n");
}
static void sm5705_work_disable_ch2(struct work_struct *data)
{
	fl_dbg("lt work queue callback\n");
}
#endif
#if 1
static enum hrtimer_restart sm5705_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&sm5705_work_ch1);
	return HRTIMER_NORESTART;
}
static enum hrtimer_restart sm5705_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&sm5705_work_ch2);
	return HRTIMER_NORESTART;
}
#endif

int sm5705_timer_start(int channel, ktime_t ktime)
{
	if (channel == SM5705_CHANNEL_CH1)
		hrtimer_start(&sm5705_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == SM5705_CHANNEL_CH2)
		hrtimer_start(&sm5705_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		fl_err("Error channel\n");
		return -1;
	}
	return 0;
}

int sm5705_timer_cancel(int channel)
{
	if (channel == SM5705_CHANNEL_CH1)
		hrtimer_cancel(&sm5705_timer_ch1);
	else if (channel == SM5705_CHANNEL_CH2)
		hrtimer_cancel(&sm5705_timer_ch2);
	else {
		fl_err("Error channel\n");
		return -1;
	}
	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int sm5705_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;
	/* verify channel */
	if (channel < 0 || channel >= SM5705_CHANNEL_NUM) {
		fl_err("Failed with error channel\n");
		return -EINVAL;
	}

	fl_info("sm5705 flash command and arg(%d): (%d, %d)\n",
			channel, _IOC_NR(cmd), (int)fl_arg->arg);
	
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		fl_dbg("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		sm5705_timeout_ms[channel] = fl_arg->arg;
		break;
	case FLASH_IOC_SET_DUTY:
		fl_dbg("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		
		gDuty = (int)fl_arg->arg;
		sm5705_set_level(channel, fl_arg->arg);
		break;
	case FLASH_IOC_SET_ONOFF:
		fl_dbg("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (sm5705_timeout_ms[channel]) {
				ktime = ktime_set(sm5705_timeout_ms[channel] / 1000,
						(sm5705_timeout_ms[channel] % 1000) * 1000000);
				sm5705_timer_start(channel, ktime);
			}
			sm5705_enable(channel);
		} else {
			sm5705_disable(channel);
			sm5705_timer_cancel(channel);
		}
		break;
	default:
		fl_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}
	return 0;
}

static int sm5705_open(void *pArg)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int sm5705_release(void *pArg)
{
	/* uninit chip and clear usage count */
	//mutex_lock(&sm5705_mutex);
	//sm5705_flash_exit();
	//mutex_unlock(&sm5705_mutex);
	fl_dbg("Release: %d\n", use_count);
	return 0;
}

static int sm5705_set_driver(void)
{
	/* init chip and set usage count */
	//mutex_lock(&sm5705_mutex);
	//sm5705_flash_init();
	//mutex_unlock(&sm5705_mutex);
	fl_dbg("Set driver: %d\n", use_count);
	return 0;
}

static ssize_t sm5705_strobe_store(struct flashlight_arg arg)
{
	sm5705_set_driver();
	sm5705_set_level(arg.ct, arg.level);
	sm5705_enable(arg.ct);
	msleep(arg.dur);
	sm5705_disable(arg.ct);
	sm5705_release(NULL);
	return 0;
}

static struct flashlight_operations sm5705_ops = {
	sm5705_open,
	sm5705_release,
	sm5705_ioctl,
	sm5705_strobe_store,
	sm5705_set_driver
};




/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
	static int sm5705_flash_probe(struct platform_device *pdev)
	{
		int err;
		
		fl_dbg("Probe start.\n");

		/* init work queue for MTK FLASH ctrl*/ 
		INIT_WORK(&sm5705_work_ch1, sm5705_work_disable_ch1);
		INIT_WORK(&sm5705_work_ch2, sm5705_work_disable_ch2);
		
		/* init timer */
		hrtimer_init(&sm5705_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		sm5705_timer_ch1.function = sm5705_timer_func_ch1;
		hrtimer_init(&sm5705_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		sm5705_timer_ch2.function = sm5705_timer_func_ch2;
		sm5705_timeout_ms[SM5705_CHANNEL_CH1] = 100;
		sm5705_timeout_ms[SM5705_CHANNEL_CH2] = 100;
		
		/* register flashlight operations for MTK FLASH ctrl*/
		if (flashlight_dev_register(SM5705_NAME, &sm5705_ops)) {
				fl_err("Failed to register flashlight device.\n");
				err = -EFAULT;
				/* goto err_free; */
			}
			
			/* clear usage count */
		use_count = 0;
		fl_dbg("Probe done.\n");
		
		return 0;

	}
	
	static int sm5705_flash_remove(struct platform_device *pdev)
	{
		fl_dbg("Remove start.\n");
		/* flush work queue for MTK Flash ctrl*/
		flush_work(&sm5705_work_ch1);
		flush_work(&sm5705_work_ch2);
		/* unregister flashlight operations for MTK Flash ctrl*/
		flashlight_dev_unregister(SM5705_NAME);		
	
		fl_dbg("Remove done.\n");

		return 0;

	}
	

#ifdef CONFIG_OF
	static struct of_device_id sm5705_flash_match_table[] = {
		{ .compatible = SM5705_DTNAME,},
		{},
	};
MODULE_DEVICE_TABLE(of, sm5705_flash_match_table);
#else
#define sm5705_fled_match_table NULL
#endif

	static struct platform_driver sm5705_flash_driver = {
		.driver 	= {
			.name	= SM5705_NAME,
			.owner	= THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table	= sm5705_flash_match_table,
#endif /* CONFIG_OF */			
		},
		.probe		= sm5705_flash_probe,
		.remove 	= sm5705_flash_remove,
	};
	static int __init sm5705_flash_init(void)
	{
		int ret;
		
		printk("Init start.\n");
		ret = platform_driver_register(&sm5705_flash_driver);
		printk("Init doing =%d\n",ret);
		if (ret) {
			fl_err("Failed to register platform driver\n");
			return ret;
		}
		printk("Init done.\n");
		return 0;		
		
	}
	module_init(sm5705_flash_init);
	
	static void __exit sm5705_flash_exit(void)
	{
		platform_driver_unregister(&sm5705_flash_driver);
	}
	module_exit(sm5705_flash_exit);
	
	MODULE_DESCRIPTION("SM5705 FLASH-LED driver");
	MODULE_ALIAS("platform:sm5705-flashLED");
	MODULE_LICENSE("GPL");

