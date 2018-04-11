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

#include "disp_dts_gpio.h"
#include "disp_helper.h"
#include <linux/kernel.h> /* printk */

static struct pinctrl *this_pctrl; /* static pinctrl instance */

/* DTS state mapping name */
static const char *this_state_name[DTS_GPIO_STATE_MAX] = {
	"lcd_vdd_1p8_out0",
	"lcd_vdd_1p8_out1",
	"lcm_rst_out0_gpio",
	"lcm_rst_out1_gpio",
	"lcd_vsp_5p5_en0",
	"lcd_vsp_5p5_en1",
	"lcd_vsn_5p5_en0",
	"lcd_vsn_5p5_en1",
	"lcd_bl_en0",
	"lcd_bl_en1",
	"tsp_rst_en0",
	"tsp_rst_en1",
	"disp_dsi_te"
};

/* pinctrl implementation */
static long _set_state(const char *name)
{
	long ret = 0;
	struct pinctrl_state *pState = 0;

	if (!this_pctrl) {
		pr_err("this pctrl is null\n");
		return -1;
	}

	pState = pinctrl_lookup_state(this_pctrl, name);
	if (IS_ERR(pState)) {
		pr_err("lookup state '%s' failed\n", name);
		ret = PTR_ERR(pState);
		goto exit;
	}

	/* select state! */
	pinctrl_select_state(this_pctrl, pState);

exit:
	return ret; /* Good! */
}

long disp_dts_gpio_init(struct platform_device *pdev)
{
	long ret = 0;
	struct pinctrl *pctrl;

	/* retrieve */
	pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pctrl)) {
		dev_err(&pdev->dev, "Cannot find disp pinctrl!");
		ret = PTR_ERR(pctrl);
		goto exit;
	}

	this_pctrl = pctrl;

exit:
	return ret;
}

long disp_dts_gpio_select_state(enum DTS_GPIO_STATE s)
{
	if (!((unsigned int)(s) < (unsigned int)(DTS_GPIO_STATE_MAX))) {
		pr_err("GPIO STATE is invalid,state=%d\n", (unsigned int)s);
		return -1;
	}
	return _set_state(this_state_name[s]);
}
