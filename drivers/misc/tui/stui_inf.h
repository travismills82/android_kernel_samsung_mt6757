/* tui/stui_inf.h
 *
 * Samsung TUI HW Handler driver.
 *
 * Copyright (c) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#ifndef __STUI_INF_H_
#define __STUI_INF_H_

#define STUI_MODE_OFF			0x00
#define STUI_MODE_ALL			0x07
#define STUI_MODE_TUI_SESSION		0x01
#define STUI_MODE_DISPLAY_SEC		0x02
#define STUI_MODE_TOUCH_SEC		0x04

int  stui_inc_blank_ref(void);
int  stui_dec_blank_ref(void);
int  stui_get_blank_ref(void);
void stui_set_blank_ref(int ref);

int  stui_get_mode(void);
void stui_set_mode(int mode);

int  stui_set_mask(int mask);
int  stui_clear_mask(int mask);
#ifdef CONFIG_SOC_EXYNOS5433
void stui_set_tsp_irq(int irq_num);
#endif
void set_touch_data_synaptic(void *dev_data);
void set_touch_data_melfas(void *dev_data);

#endif /* __STUI_INF_H_ */
