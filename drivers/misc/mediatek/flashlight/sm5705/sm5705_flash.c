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

#include "../flashlight.h"
#include "../flashlight-dt.h"
#include <linux/leds-sm5705.h>
#include <linux/mfd/sm5705/sm5705.h>
#include <linux/muic/muic_afc.h>

#define sm5705_charger_oper

#ifdef sm5705_charger_oper
#include <charger/sm5705_charger_oper.h>
#endif
#ifndef sm5705_charger_oper
#define IGNORE_AFC_STATE
#endif


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
#define FLED_PINCTRL_STATE_DEFAULT "fled_default"
#define FLED_PINCTRL_STATE_SLEEP "fled_sleep"
	
	enum {
		SM5705_FLED_OFF_MODE					= 0x0,
		SM5705_FLED_ON_MOVIE_MODE				= 0x1,
		SM5705_FLED_ON_FLASH_MODE				= 0x2,
		SM5705_FLED_ON_EXTERNAL_CONTROL_MODE	= 0x3,
	};
	
	enum {
		FLED_GPIO_ISP = 0,
		FLED_GPIO_OS = 1,
	};
	
	struct sm5705_fled_info {
		struct device *dev;
		struct i2c_client *i2c;
	
		struct sm5705_fled_platform_data *pdata;
		struct device *rear_fled_dev;
		struct mutex led_lock;
	};
	
	static struct sm5705_fled_info *g_sm5705_fled;
	static bool fimc_is_activated = 0;
	static bool assistive_light = false;
	//static int fled_gpio_config = FLED_GPIO_ISP;
#if defined(CONFIG_MUIC_UNIVERSAL_SM5705_AFC)	
	static bool muic_flash_on_status = false;
#endif	
	//struct class *camera_class; /*sys/class/camera*/
	extern int sm5705_call_fg_device_id(void);
	
	void sm5705_fled_lock(struct sm5705_fled_info *fled_info)
	{
		mutex_lock(&fled_info->led_lock);
	}
	
	void sm5705_fled_unlock(struct sm5705_fled_info *fled_info)
	{
		mutex_unlock(&fled_info->led_lock);
	}
	
	static inline int __get_revision_number(void)
	{
		return 0;//sm5705_call_fg_device_id();
	}
	
	/**
	 * SM5705 Flash-LEDs device register control functions
	 */
	static int sm5705_FLEDx_mode_enable(struct sm5705_fled_info *sm5705_fled,
		int index, unsigned char FLEDxEN)
	{
		int ret;
	
		ret = sm5705_update_reg(sm5705_fled->i2c,
			SM5705_REG_FLED1CNTL1 + (index * 4),
			(FLEDxEN & 0x3), 0x3);
		if (IS_ERR_VALUE(ret)) {
			dev_err(sm5705_fled->dev, "%s: fail to update REG:FLED%dEN (value=%d)\n",
				__func__, index, FLEDxEN);
			return ret;
		}
	
		dev_info(sm5705_fled->dev, "%s: FLED[%d] set mode = %d\n",
			__func__, index, FLEDxEN);
	
		return 0;
	}
	
	
	static inline unsigned char _calc_flash_current_offset_to_mA(
		unsigned short current_mA)
	{
		return current_mA < 700 ?
			(((current_mA - 300) / 25) & 0x1F) :((((current_mA - 700) / 50) + 0xF) & 0x1F);
	}
	
	static int sm5705_FLEDx_set_flash_current(struct sm5705_fled_info *sm5705_fled,
		int index, unsigned short current_mA)
	{
		int ret;
		unsigned char reg_val;
	
		reg_val = _calc_flash_current_offset_to_mA(current_mA);
		ret = sm5705_write_reg(sm5705_fled->i2c, SM5705_REG_FLED1CNTL3 + (index * 4),
			reg_val);
		if (IS_ERR_VALUE(ret)) {
			dev_err(sm5705_fled->dev, "%s: fail to write REG:FLED%dCNTL3 (value=%d)\n",
				__func__, index, reg_val);
			return ret;
		}
	
		return 0;
	}
	
	static inline unsigned char _calc_torch_current_offset_to_mA(
			unsigned short current_mA)
	{
		if (current_mA > 320)
			current_mA = 320;
	
		return (((current_mA - 10) / 10) & 0x1F);
	}
	
	static inline unsigned short _calc_torch_current_mA_to_offset(unsigned char offset)
	{
		return (((offset & 0x1F) + 1) * 10);
	}
	
	static int sm5705_FLEDx_set_torch_current(
		struct sm5705_fled_info *sm5705_fled, int index, unsigned short current_mA)
	{
		int ret;
		unsigned char reg_val;
		struct device *dev = sm5705_fled->dev;


		dev_info(dev, "SM5705_FLEDx_set_torch_current[IDX = %d][current_mA = %d]\n",index,current_mA);

	
		reg_val = _calc_torch_current_offset_to_mA(current_mA);
		ret = sm5705_write_reg(sm5705_fled->i2c, SM5705_REG_FLED1CNTL4 + (index * 4),
			reg_val);
		if (IS_ERR_VALUE(ret)) {
			dev_err(sm5705_fled->dev, "%s: fail to write REG:FLED%dCNTL4 (value=%d)\n",
				__func__, index, reg_val);
			return ret;
		}
	
		return 0;
	}
	
#if defined(CONFIG_MUIC_UNIVERSAL_SM5705_AFC)
	/**
	 * SM5705 Flash-LED to MUIC interface functions
	 */
	/*#define IGNORE_AFC_STATE*/
	static inline bool sm5705_fled_check_valid_vbus_from_MUIC(void)
	{
		if (muic_check_afc_state(1) == 1) {
			return true; /* Can use FLED */
		}
		return false; /* Can NOT use FLED*/
	}
	
	static inline int sm5705_fled_muic_flash_work_on(
		struct sm5705_fled_info *sm5705_fled)
	{
		/* MUIC 9V -> 5V function */
		muic_check_afc_state(1);
		/*muic_dpreset_afc();*/
		return 0;
	}
	
	static inline int sm5705_fled_muic_flash_work_off(
		struct sm5705_fled_info *sm5705_fled)
	{
		/* MUIC 5V -> 9V function */
		muic_check_afc_state(0);
		/*muic_restart_afc();*/
	
		return 0;
	}
	
	static int sm5705_fled_muic_flash_on_prepare(void)
	{
		int ret = 0;
	
		if (muic_torch_prepare(1) == 1) {
			muic_flash_on_status = true;
			ret = 0;
		} else {
			pr_err("%s: fail to prepare for AFC V_drop\n", __func__);
			ret = -1;
		}
	
		return ret;
	}
	
	static void sm5705_fled_muic_flash_off_prepare(void)
	{
		if (muic_flash_on_status == true) {
			muic_torch_prepare(0);
			muic_flash_on_status = false;
		}
	}
#endif
	/**
	 * SM5705 Flash-LED operation control functions
	 */
	static int sm5705_fled_initialize(struct sm5705_fled_info *sm5705_fled)
	{
		struct device *dev = sm5705_fled->dev;
		struct sm5705_fled_platform_data *pdata = sm5705_fled->pdata;
		int i, ret;
	
		for (i=0; i < SM5705_FLED_MAX; ++i) {
			if (pdata->led[i].used_gpio) {
				ret = gpio_request(pdata->led[i].flash_en_pin, "sm5705_fled");
				if (IS_ERR_VALUE(ret)) {
					dev_err(dev, "%s: fail to request flash gpio pin = %d (ret=%d)\n",
						__func__, pdata->led[i].flash_en_pin, ret);
					return ret;
				}
				printk("sm5705_fled_initialize %d,FLASH GPIO = %d\n",i,pdata->led[i].flash_en_pin);				
				gpio_direction_output(pdata->led[i].flash_en_pin, 0);
	
				ret = gpio_request(pdata->led[i].torch_en_pin, "sm5705_fled");
				if (IS_ERR_VALUE(ret)) {
					dev_err(dev, "%s: fail to request torch gpio pin = %d (ret=%d)\n",
						__func__, pdata->led[i].torch_en_pin, ret);
					return ret;
				}
					printk("sm5705_fled_initialize %d,TORCH GPIO = %d\n",i,pdata->led[i].torch_en_pin);				
				gpio_direction_output(pdata->led[i].torch_en_pin, 0);
	
				dev_info(dev, "SM5705 FLED[%d] used External GPIO control Mode \
					(Flash pin=%d, Torch pin=%d)\n",
					i, pdata->led[i].flash_en_pin, pdata->led[i].torch_en_pin);
			} else {
				dev_info(dev, "SM5705 FLED[%d] used I2C control Mode\n", i);
			}
			ret = sm5705_FLEDx_mode_enable(sm5705_fled, i, SM5705_FLED_OFF_MODE);
			if (IS_ERR_VALUE(ret)) {
				dev_err(dev, "%s: fail to set FLED[%d] external control mode\n", __func__, i);
				return ret;
			}
		}
	
		return 0;
	}
	
	static void sm5705_fled_deinitialize(struct sm5705_fled_info *sm5705_fled)
	{
		struct device *dev = sm5705_fled->dev;
		struct sm5705_fled_platform_data *pdata = sm5705_fled->pdata;
		int i;
	
		for (i=0; i < SM5705_FLED_MAX; ++i) {
			if (pdata->led[i].used_gpio) {
				gpio_free(pdata->led[i].flash_en_pin);
				gpio_free(pdata->led[i].torch_en_pin);
			}
			sm5705_FLEDx_mode_enable(sm5705_fled, i, SM5705_FLED_OFF_MODE);
		}
	
		dev_info(dev, "%s: FLEDs de-initialize done.\n", __func__);
	}
	
	static inline int _fled_turn_on_torch(struct sm5705_fled_info *sm5705_fled, int index)
	{
		struct sm5705_fled_platform_data *pdata = sm5705_fled->pdata;
		struct device *dev = sm5705_fled->dev;
		int ret;

		dev_info(dev, "_fled_turn_on_torch[IDX = %d]\n",index);

	
		if (pdata->led[index].used_gpio) {
			ret = sm5705_FLEDx_mode_enable(sm5705_fled, index,
				SM5705_FLED_ON_EXTERNAL_CONTROL_MODE);
			if (IS_ERR_VALUE(ret)) {
				dev_err(dev, "%s: fail to set FLED[%d] External control mode\n",
					__func__, index);
				return ret;
			}
			gpio_set_value(pdata->led[index].flash_en_pin, 0);
			gpio_set_value(pdata->led[index].torch_en_pin, 1);
		} else {
			ret = sm5705_FLEDx_mode_enable(sm5705_fled, index,
				SM5705_FLED_ON_MOVIE_MODE);
			if (IS_ERR_VALUE(ret)) {
				dev_err(dev, "%s: fail to set FLED[%d] Movie mode\n", __func__, index);
				return ret;
			}
		}
#ifdef sm5705_charger_oper
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_TORCH, 1);
#endif
		dev_info(dev, "%s: FLED[%d] Torch turn-on done.\n", __func__, index);
	
		return 0;
	}
	
	static int sm5705_fled_turn_on_torch(struct sm5705_fled_info *sm5705_fled,
		int index, unsigned short current_mA)
	{
		struct device *dev = sm5705_fled->dev;
		int ret;
#if defined(CONFIG_MUIC_UNIVERSAL_SM5705_AFC)
		ret = sm5705_fled_muic_flash_on_prepare();
	
		if (ret < 0) {
			dev_err(dev, "%s: fail to prepare for AFC V_drop\n", __func__);
		}
#endif
		ret = sm5705_FLEDx_set_torch_current(sm5705_fled, index, current_mA);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] torch current (current_mA=%d)\n",
				__func__, index, current_mA);
			return ret;
		}
	
		_fled_turn_on_torch(sm5705_fled, index);
	
		return 0;
	}
	
	
	static int sm5705_fled_turn_off(struct sm5705_fled_info *sm5705_fled, int index)
	{
		struct device *dev = sm5705_fled->dev;
		struct sm5705_fled_platform_data *pdata = sm5705_fled->pdata;
		int ret;
	
		if (pdata->led[index].used_gpio) {
			gpio_set_value(pdata->led[index].flash_en_pin, 0);
			gpio_set_value(pdata->led[index].torch_en_pin, 0);
		}
	
		ret = sm5705_FLEDx_mode_enable(sm5705_fled, index, SM5705_FLED_OFF_MODE);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] OFF mode\n", __func__, index);
			return ret;
		}
	
		ret = sm5705_FLEDx_set_flash_current(sm5705_fled, index, 0);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] flash current\n", __func__, index);
			return ret;
		}
	
		ret = sm5705_FLEDx_set_torch_current(sm5705_fled, index, 0);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to set FLED[%d] torch current\n", __func__, index);
			return ret;
		}
		
#ifdef sm5705_charger_oper	
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_FLASH, 0);
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_TORCH, 0);
#endif

#if defined(CONFIG_MUIC_UNIVERSAL_SM5705_AFC)
		sm5705_fled_muic_flash_off_prepare();
#endif
		dev_info(dev, "%s: FLED[%d] turn-off done.\n", __func__, index);
	
		return 0;
	}
	
	/**
	 *	For Export Flash control functions (external GPIO control)
	 */
	int sm5705_fled_prepare_flash(unsigned char index)
	{
		//struct pinctrl *pinctrl;
	
		if (fimc_is_activated == 1) {
			/* skip to overlapping function calls */
			return 0;
		}
	
		if (assistive_light == true) {
			pr_info("%s : assistive_light is enabled \n", __func__);
			return 0;
		}
	
		if (g_sm5705_fled == NULL) {
			pr_err("sm5705-fled: %s: invalid g_sm5705_fled, maybe not registed fled \
				device driver\n", __func__);
			return -ENXIO;
		}
	
		dev_info(g_sm5705_fled->dev, "%s: check - SM5705(rev.%d)\n",
				__func__, __get_revision_number());
	
		if (g_sm5705_fled->pdata->led[index].used_gpio == 0) {
			pr_err("sm5705-fled: %s: can't used external GPIO control, check device tree\n",
				__func__);
			return -ENOENT;
		}

#if 0
		if (__get_revision_number() < 3) {
			pinctrl = devm_pinctrl_get_select(g_sm5705_fled->dev->parent, FLED_PINCTRL_STATE_DEFAULT);
			if (IS_ERR(pinctrl))
				pr_err("%s: flash %s pins are not configured\n", __func__, FLED_PINCTRL_STATE_DEFAULT);
		}
#endif	
#if defined(CONFIG_MUIC_UNIVERSAL_SM5705_AFC)
#ifdef IGNORE_AFC_STATE
		sm5705_fled_muic_flash_work_on(g_sm5705_fled);
#else
		if (sm5705_charger_oper_get_current_op_mode() == SM5705_CHARGER_OP_MODE_CHG_ON) {
			/* W/A : for protect form VBUS drop */
			sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_FLASH, 1);
			if(!sm5705_fled_check_valid_vbus_from_MUIC()) {
				pr_err("%s: Can't used FLED, because of failed AFC V_drop\n", __func__);
				sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_FLASH, 0);
				return -1;
			}
			sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_FLASH, 0);
		} else {
			if(!sm5705_fled_check_valid_vbus_from_MUIC()) {
				pr_err("%s: Can't used FLED, because of failed AFC V_drop\n", __func__);
				return -1;
			}
		}	
#endif
#endif
	
		sm5705_FLEDx_set_torch_current(g_sm5705_fled, index,
			g_sm5705_fled->pdata->led[index].torch_current_mA);
		sm5705_FLEDx_set_flash_current(g_sm5705_fled, index,
			g_sm5705_fled->pdata->led[index].flash_current_mA);
	
		sm5705_FLEDx_mode_enable(g_sm5705_fled, index,
			SM5705_FLED_ON_EXTERNAL_CONTROL_MODE);
	
		fimc_is_activated = 1;
	
		return 0;
	}
	EXPORT_SYMBOL(sm5705_fled_prepare_flash);
	
	int sm5705_fled_torch_on(unsigned char index)
	{
		if (assistive_light == true) {
			pr_info("%s : assistive_light is enabled \n", __func__);
			return 0;
		}
	
		dev_info(g_sm5705_fled->dev, "%s: Torch - ON : E\n", __func__);
		dev_info(g_sm5705_fled->dev, "sm5705_fled_torch_on[IDX = %d]\n",index);
	
		//if (fimc_is_activated != 1) {
#if defined(CONFIG_MUIC_UNIVERSAL_SM5705_AFC)
#ifdef IGNORE_AFC_STATE
			sm5705_fled_muic_flash_work_on(g_sm5705_fled);
#else
			if(!sm5705_fled_check_valid_vbus_from_MUIC()) {
				pr_err("%s: Can't used FLED, because of failed AFC V_drop\n", __func__);
				return -1;
			}
#endif
#endif
			sm5705_FLEDx_set_torch_current(g_sm5705_fled, index,
				g_sm5705_fled->pdata->led[index].torch_current_mA);
			sm5705_FLEDx_set_flash_current(g_sm5705_fled, index,
				g_sm5705_fled->pdata->led[index].flash_current_mA);
	
			sm5705_FLEDx_mode_enable(g_sm5705_fled, index, SM5705_FLED_ON_EXTERNAL_CONTROL_MODE);
			fimc_is_activated = 1;
		//}
		sm5705_fled_lock(g_sm5705_fled);
#ifdef sm5705_charger_oper
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_FLASH, 0);
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_TORCH, 1);
#endif
		dev_info(g_sm5705_fled->dev, "flash_en_pin = %d\n",g_sm5705_fled->pdata->led[index].flash_en_pin);
		dev_info(g_sm5705_fled->dev, "torch_en_pin = %d\n",g_sm5705_fled->pdata->led[index].torch_en_pin);


		if (__get_revision_number() < 3) {
			gpio_set_value(g_sm5705_fled->pdata->led[index].flash_en_pin, 0);
			gpio_set_value(g_sm5705_fled->pdata->led[index].torch_en_pin, 1);
		}
		sm5705_fled_unlock(g_sm5705_fled);
		dev_info(g_sm5705_fled->dev, "%s: Torch - ON : X\n", __func__);
		return 0;
	}
	EXPORT_SYMBOL(sm5705_fled_torch_on);
	
	int sm5705_fled_flash_on(unsigned char index)
	{
		if (assistive_light == true) {
			pr_info("%s : assistive_light is enabled \n", __func__);
			return 0;
		}
	
		dev_info(g_sm5705_fled->dev, "%s: Flash - ON : E\n", __func__);
	
		sm5705_fled_lock(g_sm5705_fled);
		dev_info(g_sm5705_fled->dev, "%s: Flash - 1 : E\n", __func__);		
#ifdef sm5705_charger_oper		
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_TORCH, 0);
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_FLASH, 1);
#endif	
		if (__get_revision_number() < 3) {
		dev_info(g_sm5705_fled->dev, "%s: Flash - 2 : E\n", __func__);			
			gpio_set_value(g_sm5705_fled->pdata->led[index].torch_en_pin, 0);
			gpio_set_value(g_sm5705_fled->pdata->led[index].flash_en_pin, 1);
			dev_info(g_sm5705_fled->dev, "%s: Flash - 3 : E\n", __func__);
		}
			dev_info(g_sm5705_fled->dev, "%s: Flash - 4 : E\n", __func__);
		sm5705_fled_unlock(g_sm5705_fled); 
		dev_info(g_sm5705_fled->dev, "%s: Flash - ON : X\n", __func__);
		return 0;
	}
	EXPORT_SYMBOL(sm5705_fled_flash_on);
	
	int sm5705_fled_led_off(unsigned char index)
	{
		if (assistive_light == true) {
			pr_info("%s : assistive_light is enabled \n", __func__);
			return 0;
		}
	
		dev_info(g_sm5705_fled->dev, "%s: LED - OFF : E\n", __func__);
		sm5705_fled_lock(g_sm5705_fled);
		if (__get_revision_number() < 3) {
			gpio_set_value(g_sm5705_fled->pdata->led[index].flash_en_pin, 0);
			gpio_set_value(g_sm5705_fled->pdata->led[index].torch_en_pin, 0);
		}
#ifdef sm5705_charger_oper
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_TORCH, 0);
		sm5705_charger_oper_push_event(SM5705_CHARGER_OP_EVENT_FLASH, 0);
#endif	
		sm5705_fled_unlock(g_sm5705_fled);
		dev_info(g_sm5705_fled->dev, "%s: LED - OFF : X\n", __func__);
		return 0;
	}
	EXPORT_SYMBOL(sm5705_fled_led_off);
	
	int sm5705_fled_close_flash(unsigned char index)
	{
		if (fimc_is_activated == 0) {
			/* skip to overlapping function calls */
			return 0;
		}
	
		if (assistive_light == true) {
			pr_info("%s : assistive_light is enabled \n", __func__);
			return 0;
		}
	
		dev_info(g_sm5705_fled->dev, "%s: Close Process\n", __func__);
	
		if (g_sm5705_fled == NULL) {
			pr_err("sm5705-fled: %s: invalid g_sm5705_fled, maybe not registed fled \
				device driver\n", __func__);
			return -ENXIO;
		}
	
		sm5705_fled_led_off(index);
		sm5705_FLEDx_mode_enable(g_sm5705_fled, index, SM5705_FLED_OFF_MODE);
	
#if defined(CONFIG_MUIC_UNIVERSAL_SM5705_AFC)
		sm5705_fled_muic_flash_work_off(g_sm5705_fled);
#endif

#ifndef titan6757_c10_n
		fimc_is_activated = 0;
#else
		if(index) // C10 have Main1 & Main2 flash
			fimc_is_activated = 0;			
#endif

	
		return 0;
	}
	EXPORT_SYMBOL(sm5705_fled_close_flash);
	
	/**
	 * For Camera-class Rear Flash device file support functions
	 */
#ifdef titan6757_c10_n
#define REAR_FLASH_INDEX	SM5705_FLED_0
#define REAR_FLASH2_INDEX	SM5705_FLED_1
#else
#define REAR_FLASH_INDEX	SM5705_FLED_0
#define FRONT_FLASH_INDEX	SM5705_FLED_1
#endif

#ifdef titan6757_c10_n
ssize_t rear_flash_store(struct device *dev, 	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sm5705_fled_info *sm5705_fled = g_sm5705_fled;
	int ret = 0;
	int value_u32 = 0;

	/* for Dual Flash Leds in rear */
	int index = SM5705_FLED_MAX;// set defualt value for prevent
	if(strcmp(attr->attr.name,"rear_flash") == 0){
		pr_err("flash index is 0 \n");
		index = SM5705_FLED_0;
	}
	else if(strcmp(attr->attr.name,"rear_flash_2") == 0){
		pr_err("flash index is 1 \n");
		index = SM5705_FLED_1;
	}
	else{
		pr_err("flash index is not match \n");
	}

	if ((buf == NULL) || kstrtouint(buf, 10, &value_u32) || (sm5705_fled == NULL)) {
		dev_err(dev, "%s: front_flash_store can't process %d\n", __func__, value_u32);
		return -1;
	}

	dev_info(dev, "%s: value=%d\n", __func__, value_u32);

	switch (value_u32) {
	case 0:
		ret = sm5705_fled_turn_off(sm5705_fled, index);
		assistive_light = false;
		break;
		
	case 1:
		/* Turn on Torch */
		ret = sm5705_fled_turn_on_torch(sm5705_fled, index, 60);
		assistive_light = true;
		fimc_is_activated = 0;
		break;
		
	case 100:
		ret = sm5705_fled_turn_on_torch(sm5705_fled, index, 60);
		break;
		
	default:
        if (value_u32 > 1000 && value_u32 < (1000 + 32)) {
			if (1 == index)
				value_u32 -= 1;
			ret = sm5705_fled_turn_on_torch(sm5705_fled, index, _calc_torch_current_mA_to_offset(value_u32 - 1000));
		} else {
			dev_err(dev, "%s: can't process, invalid value=%d\n", __func__, value_u32);
			ret = -EINVAL;
		}
		break;
	}
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "%s: fail to rear flash file operation:store (value=%d, ret=%d)\n",
			__func__, value_u32, ret);
	}

	return count;
}
#else
	ssize_t rear_flash_store(struct device *dev, 	struct device_attribute *attr, const char *buf, size_t count)
	{
		struct sm5705_fled_info *sm5705_fled = g_sm5705_fled;
		int ret, value_u32;
	
		if ((buf == NULL) || kstrtouint(buf, 10, &value_u32) || (sm5705_fled == NULL)) {
			dev_err(dev, "%s: front_flash_store can't process %d\n", __func__, value_u32);
			return -1;
		}
	
		dev_info(dev, "%s: value=%d\n", __func__, value_u32);
	
		switch (value_u32) {
		case 0:
			ret = sm5705_fled_turn_off(sm5705_fled, REAR_FLASH_INDEX);
			assistive_light = false;
			break;
			
		case 1:
			/* Turn on Torch */
			ret = sm5705_fled_turn_on_torch(sm5705_fled, REAR_FLASH_INDEX, 60);
			assistive_light = true;
			fimc_is_activated = 0;
			break;
			
		case 100:
			ret = sm5705_fled_turn_on_torch(sm5705_fled, REAR_FLASH_INDEX, 320);
			break;
			
		default:
			if (value_u32 > 1000 && value_u32 < (1000 + 32)) {
				/* Turn on Torch : 20mA ~ 320mA */
				ret = sm5705_fled_turn_on_torch(sm5705_fled, REAR_FLASH_INDEX,
					_calc_torch_current_mA_to_offset(value_u32 - 1000));
			} else {
				dev_err(dev, "%s: can't process, invalid value=%d\n", __func__, value_u32);
				ret = -EINVAL;
			}
			break;
		}
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to rear flash file operation:store (value=%d, ret=%d)\n",
				__func__, value_u32, ret);
		}
	
		return count;
	}
#endif	
	ssize_t rear_flash_show(struct device *dev, struct device_attribute *attr, char *buf)
	{
		unsigned char offset = _calc_torch_current_offset_to_mA(320);
	
		dev_info(dev, "%s: SM5705 Movie mode max current = 320mA(offset:%d)\n",
			__func__, offset);
	
		return sprintf(buf, "%d\n", offset);
	}

#ifndef titan6757_c10_n
	ssize_t front_flash_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
	{
		struct sm5705_fled_info *sm5705_fled = g_sm5705_fled;
		int ret, value_u32;
	
		if ((buf == NULL) || kstrtouint(buf, 10, &value_u32) || (sm5705_fled == NULL)) {
			dev_err(dev, "%s: front_flash_store can't process %d\n", __func__, value_u32);
			return -1;
		}
	
		dev_info(dev, "%s: value=%d\n", __func__, value_u32);
	
		switch (value_u32) {
		case 0:

			ret = sm5705_fled_turn_off(sm5705_fled, FRONT_FLASH_INDEX);
			assistive_light = false;
			break;
			
		case 1:
			/* Turn on Torch */
			ret = sm5705_fled_turn_on_torch(sm5705_fled, FRONT_FLASH_INDEX, 60);
			assistive_light = true;
			fimc_is_activated = 0;
			break;
			
		case 100:
			ret = sm5705_fled_turn_on_torch(sm5705_fled, FRONT_FLASH_INDEX, 320);
			break;
			
		default:
			if (value_u32 > 1000 && value_u32 < (1000 + 32)) {
				/* Turn on Torch : 20mA ~ 320mA */
				ret = sm5705_fled_turn_on_torch(sm5705_fled, FRONT_FLASH_INDEX,
					_calc_torch_current_mA_to_offset(value_u32 - 1000));
			} else {
				dev_err(dev, "%s: can't process, invalid value=%d\n", __func__, value_u32);
				ret = -EINVAL;
			}
			break;
		}
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to rear flash file operation:store (value=%d, ret=%d)\n",
				__func__, value_u32, ret);
		}
	
		return count;
	}
	
	ssize_t front_flash_show(struct device *dev, struct device_attribute *attr, char *buf)
	{
		unsigned char offset = _calc_torch_current_offset_to_mA(320);
	
		dev_info(dev, "%s: SM5705 Movie mode max current = 320mA(offset:%d)\n",
			__func__, offset);
	
		return sprintf(buf, "%d\n", offset);
	}
#endif
	/**
	 * SM5705 Flash-LED device driver management functions
	 */
	
#ifdef CONFIG_OF
	static int sm5705_fled_parse_dt(struct device *dev,
		struct sm5705_fled_platform_data *pdata)
	{
		struct device_node *nproot = dev->parent->of_node;
		struct device_node *np, *c_np;
		unsigned int temp;
		int index;
		int ret = 0;
	
		np = of_find_node_by_name(nproot, "flash");
		if (unlikely(np == NULL)) {
			dev_err(dev, "%s: fail to find flash node\n", __func__);
			return ret;
		}
	
		for_each_child_of_node(np, c_np) {
			ret = of_property_read_u32(c_np, "id", &temp);
			if (ret) {
				dev_err(dev, "%s: fail to get a id\n", __func__);
				return ret;
			}
			index = temp;
	
			ret = of_property_read_u32(c_np, "flash-mode-current-mA", &temp);
			if (ret) {
				dev_err(dev, "%s: fail to get dt:flash-mode-current-mA\n", __func__);
				return ret;
			}
			pdata->led[index].flash_current_mA = temp;
	
			ret = of_property_read_u32(c_np, "torch-mode-current-mA", &temp);
			if (ret) {
				dev_err(dev, "%s: fail to get dt:torch-mode-current-mA\n", __func__);
				return ret;
			}
			pdata->led[index].torch_current_mA = temp;
	
			ret = of_property_read_u32(c_np, "used-gpio-control", &temp);
			if (ret) {
				dev_err(dev, "%s: fail to get dt:used-gpio-control\n", __func__);
				return ret;
			}
			pdata->led[index].used_gpio = (bool)(temp & 0x1);
	
			if (pdata->led[index].used_gpio) {
				ret = of_get_named_gpio(c_np, "flash-en-gpio", 0);
				if (ret < 0) {
					dev_err(dev, "%s: fail to get dt:flash-en-gpio (ret=%d)\n", __func__, ret);
					return ret;
				}
				pdata->led[index].flash_en_pin = ret;
	
				ret = of_get_named_gpio(c_np, "torch-en-gpio", 0);
				if (ret < 0) {
					dev_err(dev, "%s: fail to get dt:torch-en-gpio (ret=%d)\n", __func__, ret);
					return ret;
				}
				pdata->led[index].torch_en_pin = ret;
			}
		}
	
		return 0;
	}
#endif
	
	static inline struct sm5705_fled_platform_data *_get_sm5705_fled_platform_data(
		struct device *dev, struct sm5705_dev *sm5705)
	{
		struct sm5705_fled_platform_data *pdata;
		int i = 0;
		int ret = 0;
	
#ifdef CONFIG_OF
		pdata = devm_kzalloc(dev, sizeof(struct sm5705_fled_platform_data), GFP_KERNEL);
		if (unlikely(!pdata)) {
			dev_err(dev, "%s: fail to allocate memory for sm5705_fled_platform_data\n",
				__func__);
			goto out_p;
		}
	
		ret = sm5705_fled_parse_dt(dev, pdata);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to parse dt for sm5705 flash-led (ret=%d)\n", __func__, ret);
			goto out_kfree_p;
		}
#else
		pdata = sm5705->pdata->fled_platform_data;
		if (unlikely(!pdata)) {
			dev_err(dev, "%s: fail to get sm5705_fled_platform_data\n", __func__);
			goto out_p;
		}
#endif
	
		dev_info(dev, "sm5705 flash-LED device platform data info, \n");
		for (i=0; i < SM5705_FLED_MAX; ++i) {
			dev_info(dev, "[FLED-%d] Flash: %dmA, Torch: %dmA, used_gpio=%d, \
				GPIO_PIN(%d, %d)\n", i, pdata->led[i].flash_current_mA,
				pdata->led[i].torch_current_mA, pdata->led[i].used_gpio,
				pdata->led[i].flash_en_pin, pdata->led[i].torch_en_pin);
		}
	
		return pdata;
	
	out_kfree_p:
		devm_kfree(dev, pdata);
	out_p:
		return NULL;
	}
	
	static int sm5705_fled_probe(struct platform_device *pdev)
	{
		struct sm5705_dev *sm5705 = dev_get_drvdata(pdev->dev.parent);
		struct sm5705_fled_info *sm5705_fled;
		struct sm5705_fled_platform_data *sm5705_fled_pdata;
		struct device *dev = &pdev->dev;
		int i = 0;
		int ret = 0;

			printk("probe start  1.\n");

			dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver start..\n",
			__get_revision_number());

#if 0	
		if (IS_ERR_OR_NULL(camera_class)) {
			dev_err(dev, "%s: can't find camera_class sysfs object, didn't used rear_flash attribute\n",
				__func__);
			return -ENOENT;
		}
				printk("probe start 2.\n");
			dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver start1..\n",
			__get_revision_number());
#endif			
	
		sm5705_fled = devm_kzalloc(dev, sizeof(struct sm5705_fled_info), GFP_KERNEL);
		if (unlikely(!sm5705_fled)) {
			dev_err(dev, "%s: fail to allocate memory for sm5705_fled_info\n", __func__);
			return -ENOMEM;
		}
				printk("probe start 3.\n");
		dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver Probing..\n",
			__get_revision_number());
	
		sm5705_fled_pdata = _get_sm5705_fled_platform_data(dev, sm5705);
		if (unlikely(!sm5705_fled_pdata)) {
			dev_info(dev, "%s: fail to get platform data\n", __func__);
			goto fled_platfrom_data_err;
		}
			printk("probe start 4.\n");	
		sm5705_fled->dev = dev;
		sm5705_fled->i2c = sm5705->i2c;
		sm5705_fled->pdata = sm5705_fled_pdata;
		platform_set_drvdata(pdev, sm5705_fled);
		g_sm5705_fled = sm5705_fled;
			printk("probe start 5.\n");	
		ret = sm5705_fled_initialize(sm5705_fled);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s: fail to initialize SM5705 Flash-LED[%d] (ret=%d)\n", __func__, i, ret);
			goto fled_init_err;
		}
				printk("probe start 6.\n");

		mutex_init(&sm5705_fled->led_lock);

#if 0	
		/* create camera_class rear_flash device */
		sm5705_fled->rear_fled_dev = device_create(camera_class, NULL, 3, NULL, "flash");
		if (IS_ERR(sm5705_fled->rear_fled_dev)) {
			dev_err(dev, "%s fail to create device for rear_flash\n", __func__);
			goto fled_deinit_err;
		}
		sm5705_fled->rear_fled_dev->parent = dev;

				printk("probe start 7.\n");
		ret = device_create_file(sm5705_fled->rear_fled_dev, &dev_attr_rear_flash);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s fail to create device file for rear_flash\n", __func__);
			goto fled_rear_device_err;
		}
				printk("probe start 8.\n");
		ret = device_create_file(sm5705_fled->rear_fled_dev, &dev_attr_rear_torch_flash);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "%s fail to create device file for rear_torch_flash\n", __func__);
			goto fled_rear_device_err;
		}
#endif		
				printk("probe start 9.\n");
			dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver Probing 4..\n",
			__get_revision_number());	
		sm5705_fled_pdata->fled_pinctrl = devm_pinctrl_get(dev->parent);
		if (IS_ERR_OR_NULL(sm5705_fled_pdata->fled_pinctrl)) {
			pr_err("%s:%d Getting pinctrl handle failed\n",
					__func__, __LINE__);
			goto fled_rear_device_err;
		}
			dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver Probing 5..\n",
			__get_revision_number());
			
				printk("probe start 10.\n");

		#if 0		
		sm5705_fled_pdata->gpio_state_active
			= pinctrl_lookup_state(sm5705_fled_pdata->fled_pinctrl, FLED_PINCTRL_STATE_DEFAULT);
		if (IS_ERR_OR_NULL(sm5705_fled_pdata->gpio_state_active)) {
			pr_err("%s:%d Failed to get the active state pinctrl handle\n",
					__func__, __LINE__);
			goto fled_rear_device_err;
		}
				printk("probe start11.\n");
		dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver Probing 6..\n",
			__get_revision_number());	
		sm5705_fled_pdata->gpio_state_suspend
			= pinctrl_lookup_state(sm5705_fled_pdata->fled_pinctrl, FLED_PINCTRL_STATE_SLEEP);
		if (IS_ERR_OR_NULL(sm5705_fled_pdata->gpio_state_suspend)) {
			pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
					__func__, __LINE__);
			goto fled_rear_device_err;
		}

	
			dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver Probing 7..\n",
			__get_revision_number());	
				printk("probe start12.\n");
		ret = pinctrl_select_state(sm5705_fled_pdata->fled_pinctrl, sm5705_fled_pdata->gpio_state_suspend);
		if (ret) {
			pr_err("%s:%d cannot set pin to suspend state", __func__, __LINE__);
		} else 
		{
			fled_gpio_config = FLED_GPIO_ISP;
		}

#endif		
			dev_info(dev, "SM5705(rev.%d) Flash-LED devic driver Probing 8..\n",
			__get_revision_number());	
	
		dev_info(dev, "%s: Probe done.\n", __func__);
	
		return 0;
	
	fled_rear_device_err:
	//	device_destroy(camera_class, sm5705_fled->rear_fled_dev->devt);

	//fled_deinit_err:
		//sm5705_fled_deinitialize(sm5705_fled);
	
	fled_init_err:
		platform_set_drvdata(pdev, NULL);
#ifdef CONFIG_OF
		devm_kfree(dev, sm5705_fled_pdata);
#endif
	
	fled_platfrom_data_err:
		devm_kfree(dev, sm5705_fled);
	
		return ret;
	}
	
	static int sm5705_fled_remove(struct platform_device *pdev)
	{
		struct sm5705_fled_info *sm5705_fled = platform_get_drvdata(pdev);
		struct device *dev = &pdev->dev;
		int i;
	
		//device_remove_file(sm5705_fled->rear_fled_dev, &dev_attr_rear_flash);
		//device_remove_file(sm5705_fled->rear_fled_dev, &dev_attr_front_flash);
	
		//device_destroy(camera_class, sm5705_fled->rear_fled_dev->devt);
	
		for (i = 0; i != SM5705_FLED_MAX; ++i) {
			sm5705_fled_turn_off(sm5705_fled, i);
		}
		sm5705_fled_deinitialize(sm5705_fled);	
	
		platform_set_drvdata(pdev, NULL);
		mutex_destroy(&sm5705_fled->led_lock);
#ifdef CONFIG_OF
		devm_kfree(dev, sm5705_fled->pdata);
#endif
		devm_kfree(dev, sm5705_fled);
	
		return 0;
	}
	
	static void sm5705_fled_shutdown(struct device *dev)
	{
		struct sm5705_fled_info *sm5705_fled = dev_get_drvdata(dev);
		int i;
	
		for (i=0; i < SM5705_FLED_MAX; ++i) {
			sm5705_fled_turn_off(sm5705_fled, i);
		}
	}
#if 0
#ifdef CONFIG_OF
		static struct of_device_id sm5705_fled_match_table[] = {
			{ .compatible = SM5705_DTNAME,},
			{},
		};
MODULE_DEVICE_TABLE(of, sm5705_fled_match_table);
#else
#define sm5705_fled_match_table NULL
#endif
#endif
		static struct platform_driver sm5705_fled_driver = {
			.driver 	= {
				.name	= "sm5705-fled",
				.owner	= THIS_MODULE,
				.shutdown = sm5705_fled_shutdown,
			},
			.probe		= sm5705_fled_probe,
			.remove 	= sm5705_fled_remove,
		};
		static int __init sm5705_fled_init(void)
		{
			int ret;
			
			printk("Init start.\n");
			ret = platform_driver_register(&sm5705_fled_driver);
			printk("Init doing =%d\n",ret);
			if (ret) {
				fl_err("Failed to register platform driver\n");
				return ret;
			}
			printk("Init done.\n");
			return 0;		
			
		}
		module_init(sm5705_fled_init);
		
		static void __exit sm5705_fled_exit(void)
		{
			platform_driver_unregister(&sm5705_fled_driver);
		}
		module_exit(sm5705_fled_exit);
		
		MODULE_DESCRIPTION("SM5705 FLASH-LED driver");
		MODULE_ALIAS("platform:sm5705-flashLED");
		MODULE_LICENSE("GPL");

