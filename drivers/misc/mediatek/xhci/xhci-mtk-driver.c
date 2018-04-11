/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/switch.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/atomic.h>

#include "xhci-mtk-driver.h"
#ifdef CONFIG_PROJECT_PHY
#include <mtk-phy-asic.h>
#endif
#if CONFIG_MTK_GAUGE_VERSION == 30
#include <mt-plat/mtk_battery.h>
#else
#include <mt-plat/battery_meter.h>
#endif


#include <mt-plat/charger_class.h>
#ifdef CONFIG_USB_XHCI_MTK_SUSPEND_SUPPORT
#include <mtk_sleep.h>
#endif

#define RET_SUCCESS 0
#define RET_FAIL 1

#ifdef CONFIG_TCPC_CLASS
#include "tcpm.h"
#include <linux/workqueue.h>
#include <linux/mutex.h>
static struct notifier_block otg_nb;
static bool usbc_otg_attached;
static struct tcpc_device *otg_tcpc_dev;
static struct workqueue_struct *otg_tcpc_power_workq;
static struct workqueue_struct *otg_tcpc_workq;
static struct work_struct tcpc_otg_power_work;
static struct work_struct tcpc_otg_work;
static bool usbc_otg_power_enable;
static bool usbc_otg_enable;
static struct mutex tcpc_otg_lock;
static struct mutex tcpc_otg_pwr_lock;
static bool tcpc_boost_on;
#endif /* CONFIG_TCPC_CLASS */

#if CONFIG_MTK_GAUGE_VERSION == 30
static struct charger_device *primary_charger;
#endif

static struct wake_lock mtk_xhci_wakelock;

enum dualrole_state {
	DUALROLE_DEVICE,
	DUALROLE_HOST,
};

static enum dualrole_state mtk_dualrole_stat = DUALROLE_DEVICE;
static struct switch_dev mtk_otg_state;

u32 xhci_debug_level = K_ALET | K_CRIT | K_ERR | K_WARNIN;

module_param(xhci_debug_level, int, 0644);

#if defined(CONFIG_MUIC_S2MU005) || defined(CONFIG_MUIC_UNIVERSAL) || defined(CONFIG_DUMMY_MUIC)
/* FIXME: workaround for BQ25890 removal */
signed int battery_meter_get_charger_voltage(void)
{
	return 5;
}

int set_chr_enable_otg(unsigned int enable)
{
	return 0;
}


int set_chr_boost_current_limit(unsigned int current_limit)
{
	return 0;
}
#endif

bool mtk_is_charger_4_vol(void)
{
#if defined(CONFIG_USBIF_COMPLIANCE) || defined(CONFIG_POWER_EXT)
	return false;
#else
	int vol = battery_meter_get_charger_voltage();

	mtk_xhci_mtk_printk(K_DEBUG, "voltage(%d)\n", vol);

	return (vol > 4000) ? true : false;
#endif
}

static void mtk_enable_otg_mode(void)
{
#if CONFIG_MTK_GAUGE_VERSION == 30
	charger_dev_enable_otg(primary_charger, true);
	charger_dev_set_boost_current_limit(primary_charger, 1500000);
	charger_dev_kick_wdt(primary_charger);
	enable_boost_polling(true);
#else
	set_chr_enable_otg(0x1);
	set_chr_boost_current_limit(1500);
#endif
}

static void mtk_disable_otg_mode(void)
{
#if CONFIG_MTK_GAUGE_VERSION == 30
	charger_dev_enable_otg(primary_charger, false);
	enable_boost_polling(false);
#else
	set_chr_enable_otg(0x0);
#endif
}

static int mtk_xhci_hcd_init(void)
{
	int retval;

	retval = xhci_mtk_register_plat();
	if (retval < 0) {
		pr_err("Problem registering platform driver.\n");
		return retval;
	}

	return 0;
}

static void mtk_xhci_wakelock_init(void)
{
	wake_lock_init(&mtk_xhci_wakelock, WAKE_LOCK_SUSPEND, "xhci.wakelock");
#ifdef CONFIG_USB_C_SWITCH
#ifndef CONFIG_TCPC_CLASS
	typec_host_driver.priv_data = NULL;
	register_typec_switch_callback(&typec_host_driver);
#endif /* if not CONFIG_TCPC_CLASS */
#endif
}

void mtk_xhci_wakelock_lock(void)
{
	if (!wake_lock_active(&mtk_xhci_wakelock))
		wake_lock(&mtk_xhci_wakelock);
	mtk_xhci_mtk_printk(K_DEBUG, "xhci_wakelock_lock done\n");
}

void mtk_xhci_wakelock_unlock(void)
{
	if (wake_lock_active(&mtk_xhci_wakelock))
		wake_unlock(&mtk_xhci_wakelock);
	mtk_xhci_mtk_printk(K_DEBUG, "xhci_wakelock_unlock done\n");
}

static void mtk_xhci_hcd_cleanup(void)
{
	xhci_mtk_unregister_plat();
}
#ifdef CONFIG_USB_NOTIFY_LAYER
void usb_host_phy_tune(void) {

	/* setting for USB host */
	U3PhyWriteField32((phys_addr_t) (uintptr_t) U3D_USBPHYACR1, RG_USB20_VRT_VREF_SEL_OFST,
			  RG_USB20_VRT_VREF_SEL, 1);
	U3PhyWriteField32((phys_addr_t) (uintptr_t) U3D_USBPHYACR1, RG_USB20_TERM_VREF_SEL_OFST,
			  RG_USB20_TERM_VREF_SEL, 1);
	U3PhyWriteField32((phys_addr_t) (uintptr_t) U3D_USBPHYACR6, RG_USB20_DISCTH_OFST,
			  RG_USB20_DISCTH, 0xE);

	printk("usb: %s: (%d,%d,%d,%d)\n", __func__, U3PhyReadField32((phys_addr_t) (uintptr_t)U3D_USBPHYACR1, RG_USB20_VRT_VREF_SEL_OFST, RG_USB20_VRT_VREF_SEL),
		U3PhyReadField32((phys_addr_t) (uintptr_t)U3D_USBPHYACR1, RG_USB20_TERM_VREF_SEL_OFST, RG_USB20_TERM_VREF_SEL),
		U3PhyReadField32((phys_addr_t) (uintptr_t)U3D_USBPHYACR6, RG_USB20_SQTH_OFST, RG_USB20_SQTH),
		U3PhyReadField32((phys_addr_t) (uintptr_t)U3D_USBPHYACR6, RG_USB20_DISCTH_OFST, RG_USB20_DISCTH));

}
#endif
int mtk_xhci_driver_load(bool vbus_on)
{
	int ret = 0;

	/* recover clock/power setting and deassert reset bit of mac */
#ifdef CONFIG_PROJECT_PHY
	usb_phy_recover(0);
#ifdef CONFIG_USB_NOTIFY_LAYER
	usb_host_phy_tune();
#endif
	usb20_pll_settings(true, true);
#endif
	ret = mtk_xhci_hcd_init();
	if (ret) {
		ret = -ENXIO;
		goto _err;
	}
#ifdef CONFIG_USB_XHCI_MTK_SUSPEND_SUPPORT
	slp_set_infra_on(true);
#else
	mtk_xhci_wakelock_lock();
#endif
#ifndef CONFIG_USBIF_COMPLIANCE
	switch_set_state(&mtk_otg_state, 1);
#endif
	mtk_dualrole_stat = DUALROLE_HOST;

	if (vbus_on)
		mtk_enable_otg_mode();

	return 0;

_err:
#ifdef CONFIG_PROJECT_PHY
	usb_phy_savecurrent(1);
#endif
	mtk_dualrole_stat = DUALROLE_DEVICE;

	return ret;
}

void mtk_xhci_disable_vbus(void)
{
	mtk_disable_otg_mode();
}


void mtk_xhci_driver_unload(bool vbus_off)
{
	mtk_xhci_hcd_cleanup();
	if (vbus_off)
		mtk_disable_otg_mode();

	/* close clock/power setting and assert reset bit of mac */
#ifdef CONFIG_PROJECT_PHY
	usb20_pll_settings(true, false);
	usb_phy_savecurrent(1);
#endif

#ifndef CONFIG_USBIF_COMPLIANCE
	switch_set_state(&mtk_otg_state, 0);
#endif
#ifdef CONFIG_USB_XHCI_MTK_SUSPEND_SUPPORT
	slp_set_infra_on(false);
#endif
	mtk_xhci_wakelock_unlock();
	mtk_dualrole_stat = DUALROLE_DEVICE;
}

void mtk_xhci_switch_init(void)
{
	mtk_otg_state.name = "otg_state";
	mtk_otg_state.index = 0;
	mtk_otg_state.state = 0;

#ifndef CONFIG_USBIF_COMPLIANCE
	if (switch_dev_register(&mtk_otg_state))
		mtk_xhci_mtk_printk(K_DEBUG, "switch_dev_register fail\n");
	else
		mtk_xhci_mtk_printk(K_DEBUG, "switch_dev register success\n");
#endif
}

bool mtk_is_host_mode(void)
{
#ifdef CONFIG_TCPC_CLASS
	return tcpc_boost_on;
#else
	return (mtk_dualrole_stat == DUALROLE_HOST) ? true : false;
#endif /* CONFIG_TCPC_CLASS */
}

#ifdef CONFIG_TCPC_CLASS
int tcpc_otg_enable(void)
{
	int ret = 0;

	if (!usbc_otg_attached) {
		/* mtk_idpin_cur_stat = IDPIN_IN_HOST; */
		ret = mtk_xhci_driver_load(false);
		if (!ret) {
			mtk_xhci_wakelock_lock();
			switch_set_state(&mtk_otg_state, 1);
		}
		usbc_otg_attached = true;
	}
	return ret;
}

int tcpc_otg_disable(void)
{
	if (usbc_otg_attached) {
		/* USB PLL Force settings */
		usb20_pll_settings(true, false);
		mtk_xhci_driver_unload(false);
		switch_set_state(&mtk_otg_state, 0);
		mtk_xhci_wakelock_unlock();
		/* mtk_idpin_cur_stat = IDPIN_OUT; */
		usbc_otg_attached = false;
	}
	return 0;
}

static void tcpc_otg_work_call(struct work_struct *work)
{
	bool enable;

	mutex_lock(&tcpc_otg_lock);
	enable = usbc_otg_enable;
	mutex_unlock(&tcpc_otg_lock);

	if (enable)
		tcpc_otg_enable();
	else
		tcpc_otg_disable();
}

static void tcpc_otg_power_work_call(struct work_struct *work)
{
	mutex_lock(&tcpc_otg_pwr_lock);
	if (usbc_otg_power_enable) {
		if (!tcpc_boost_on) {
			mtk_enable_otg_mode();
			tcpc_boost_on = true;
		}
	} else {
		if (tcpc_boost_on) {
			mtk_disable_otg_mode();
			tcpc_boost_on = false;
		}
	}
	mutex_unlock(&tcpc_otg_pwr_lock);
}
#endif /* CONFIG_TCPC_CLASS */

#ifdef CONFIG_TCPC_CLASS
static int otg_tcp_notifier_call(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct tcp_notify *noti = data;

	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("%s source vbus = %dmv\n",
				__func__, noti->vbus_state.mv);
		mutex_lock(&tcpc_otg_pwr_lock);
		usbc_otg_power_enable = (noti->vbus_state.mv) ? true : false;
		mutex_unlock(&tcpc_otg_pwr_lock);
		queue_work(otg_tcpc_power_workq, &tcpc_otg_power_work);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			pr_info("%s OTG Plug in\n", __func__);
			mutex_lock(&tcpc_otg_lock);
			usbc_otg_enable = true;
			mutex_unlock(&tcpc_otg_lock);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SRC &&
				noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_info("%s OTG Plug out\n", __func__);
			mutex_lock(&tcpc_otg_lock);
			usbc_otg_enable = false;
			mutex_unlock(&tcpc_otg_lock);
		}
		queue_work(otg_tcpc_workq, &tcpc_otg_work);
		break;
	}
	return NOTIFY_OK;
}
#endif /* CONFIG_TCPC_CLASS */


#if CONFIG_MTK_GAUGE_VERSION == 30

struct usbotg_boost_manager {
	struct platform_device *pdev;
	struct fgtimer otg_kthread_fgtimer;
	struct workqueue_struct *otg_boost_workq;
	struct work_struct kick_work;
	struct charger_device *primary_charger;
	unsigned int polling_interval;
	bool polling_enabled;
};
static struct usbotg_boost_manager *g_info;
static struct charger_device *primary_charger;


void enable_boost_polling(bool poll_en)
{
	if (g_info) {
		if (poll_en) {
			fgtimer_start(&g_info->otg_kthread_fgtimer, g_info->polling_interval);
			g_info->polling_enabled = true;
		} else {
			g_info->polling_enabled = false;
			fgtimer_stop(&g_info->otg_kthread_fgtimer);
		}
	}
}

static void usbotg_boost_kick_work(struct work_struct *work)
{

	struct usbotg_boost_manager *usb_boost_manager =
		container_of(work, struct usbotg_boost_manager, kick_work);

	mtk_xhci_mtk_printk(K_ALET, "usbotg_boost_kick_work\n");

	charger_dev_kick_wdt(usb_boost_manager->primary_charger);

	if (usb_boost_manager->polling_enabled == true)
		fgtimer_start(&usb_boost_manager->otg_kthread_fgtimer, usb_boost_manager->polling_interval);
}

static int usbotg_fgtimer_func(struct fgtimer *data)
{
	struct usbotg_boost_manager *usb_boost_manager =
		container_of(data, struct usbotg_boost_manager, otg_kthread_fgtimer);

	queue_work(usb_boost_manager->otg_boost_workq, &usb_boost_manager->kick_work);
	return 0;
}

static int usbotg_boost_manager_probe(struct platform_device *pdev)
{
	struct usbotg_boost_manager *info = NULL;

	info = devm_kzalloc(&pdev->dev, sizeof(struct usbotg_boost_manager), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);
	info->pdev = pdev;
	info->primary_charger = get_charger_by_name("primary_chg");
	if (!info->primary_charger) {
		pr_err("%s: get primary charger device failed\n", __func__);
		return -ENODEV;
	}

	fgtimer_init(&info->otg_kthread_fgtimer, &info->pdev->dev, "otg_boost");
	info->otg_kthread_fgtimer.callback = usbotg_fgtimer_func;
	info->polling_interval = 30;
	info->otg_boost_workq = create_singlethread_workqueue("otg_boost_workq");
	INIT_WORK(&info->kick_work, usbotg_boost_kick_work);
	g_info = info;
	return 0;
}

static int usbotg_boost_manager_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id usbotg_boost_manager_of_match[] = {
	{.compatible = "mediatek,usb_boost_manager"},
	{},
};
MODULE_DEVICE_TABLE(of, usbotg_boost_manager_of_match);
static struct platform_driver boost_manager_driver = {
	.remove = usbotg_boost_manager_remove,
	.probe = usbotg_boost_manager_probe,
	.driver = {
		   .name = "usb_boost_manager",
		   .of_match_table = usbotg_boost_manager_of_match,
		   },
};
#endif



#ifdef CONFIG_USBIF_COMPLIANCE
static int __init xhci_hcd_init(void)
{
#ifdef CONFIG_TCPC_CLASS
	int ret;
#endif /* CONFIG_TCPC_CLASS */

	mtk_xhci_wakelock_init();
	mtk_xhci_switch_init();
#if CONFIG_MTK_GAUGE_VERSION == 30
	primary_charger = get_charger_by_name("primary_chg");
	if (!primary_charger) {
		pr_err("%s: get primary charger device failed\n", __func__);
		return -ENODEV;
	}
	platform_driver_register(&boost_manager_driver);
#endif

#ifdef CONFIG_TCPC_CLASS
	mutex_init(&tcpc_otg_lock);
	mutex_init(&tcpc_otg_pwr_lock);
	otg_tcpc_workq = create_singlethread_workqueue("tcpc_otg_workq");
	otg_tcpc_power_workq = create_singlethread_workqueue("tcpc_otg_power_workq");
	INIT_WORK(&tcpc_otg_power_work, tcpc_otg_power_work_call);
	INIT_WORK(&tcpc_otg_work, tcpc_otg_work_call);
	otg_tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!otg_tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	otg_nb.notifier_call = otg_tcp_notifier_call;
	ret = register_tcp_dev_notifier(otg_tcpc_dev, &otg_nb);
	if (ret < 0) {
		pr_err("%s register tcpc notifer fail\n", __func__);
		return -EINVAL;
	}
#endif /* CONFIG_TCPC_CLASS */

	return 0;
}
late_initcall(xhci_hcd_init);

static void __exit xhci_hcd_cleanup(void)
{
}
module_exit(xhci_hcd_cleanup);
#else
static int __init xhci_hcd_init(void)
{
#ifdef CONFIG_TCPC_CLASS
	int ret;
#endif /* CONFIG_TCPC_CLASS */
	mtk_xhci_wakelock_init();
	mtk_xhci_switch_init();

#if CONFIG_MTK_GAUGE_VERSION == 30
	primary_charger = get_charger_by_name("primary_chg");
	if (!primary_charger) {
		pr_err("%s: get primary charger device failed\n", __func__);
		return -ENODEV;
	}
	platform_driver_register(&boost_manager_driver);
#endif

#ifdef CONFIG_TCPC_CLASS
	mutex_init(&tcpc_otg_lock);
	mutex_init(&tcpc_otg_pwr_lock);
	otg_tcpc_workq = create_singlethread_workqueue("tcpc_otg_workq");
	otg_tcpc_power_workq = create_singlethread_workqueue("tcpc_otg_power_workq");
	INIT_WORK(&tcpc_otg_power_work, tcpc_otg_power_work_call);
	INIT_WORK(&tcpc_otg_work, tcpc_otg_work_call);
	otg_tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!otg_tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	otg_nb.notifier_call = otg_tcp_notifier_call;
	ret = register_tcp_dev_notifier(otg_tcpc_dev, &otg_nb);
	if (ret < 0) {
		pr_err("%s register tcpc notifer fail\n", __func__);
		return -EINVAL;
	}
#endif /* CONFIG_TCPC_CLASS */
	return 0;
}

late_initcall(xhci_hcd_init);

static void __exit xhci_hcd_cleanup(void)
{
}

module_exit(xhci_hcd_cleanup);
#endif

