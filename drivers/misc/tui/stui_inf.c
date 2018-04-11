/* tui/stui_inf.c
 *
 * Samsung TUI HW Handler driver.
 *
 * Copyright (c) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "stui_inf.h"

#include <linux/spinlock.h>

static int tui_mode = STUI_MODE_OFF;
static int tui_blank_cnt;
static DEFINE_SPINLOCK(tui_lock);

int stui_inc_blank_ref(void)
{
	unsigned long fls;
	int ret_cnt;

	spin_lock_irqsave(&tui_lock, fls);
	ret_cnt = ++tui_blank_cnt;
	spin_unlock_irqrestore(&tui_lock, fls);
	return ret_cnt;
}
EXPORT_SYMBOL(stui_inc_blank_ref);

int stui_dec_blank_ref(void)
{
	unsigned long fls;
	int ret_cnt;

	spin_lock_irqsave(&tui_lock, fls);
	ret_cnt = --tui_blank_cnt;
	spin_unlock_irqrestore(&tui_lock, fls);
	return ret_cnt;
}
EXPORT_SYMBOL(stui_dec_blank_ref);

int stui_get_blank_ref(void)
{
	unsigned long fls;
	int ret_cnt;

	spin_lock_irqsave(&tui_lock, fls);
	ret_cnt = tui_blank_cnt;
	spin_unlock_irqrestore(&tui_lock, fls);
	return ret_cnt;
}
EXPORT_SYMBOL(stui_get_blank_ref);

void stui_set_blank_ref(int cnt)
{
	unsigned long fls;

	spin_lock_irqsave(&tui_lock, fls);
	tui_blank_cnt = cnt;
	spin_unlock_irqrestore(&tui_lock, fls);
}
EXPORT_SYMBOL(stui_set_blank_ref);

int stui_get_mode(void)
{
	unsigned long fls;
	int ret_mode;

	spin_lock_irqsave(&tui_lock, fls);
	ret_mode = tui_mode;
	spin_unlock_irqrestore(&tui_lock, fls);
	return ret_mode;
}
EXPORT_SYMBOL(stui_get_mode);

void stui_set_mode(int mode)
{
	unsigned long fls;

	spin_lock_irqsave(&tui_lock, fls);
	tui_mode = mode;
	spin_unlock_irqrestore(&tui_lock, fls);
}
EXPORT_SYMBOL(stui_set_mode);

int stui_set_mask(int mask)
{
	unsigned long fls;
	int ret_mode;

	spin_lock_irqsave(&tui_lock, fls);
	ret_mode = (tui_mode |= mask);
	spin_unlock_irqrestore(&tui_lock, fls);
	return ret_mode;
}
EXPORT_SYMBOL(stui_set_mask);

int stui_clear_mask(int mask)
{
	unsigned long fls;
	int ret_mode;

	spin_lock_irqsave(&tui_lock, fls);
	ret_mode = (tui_mode &= ~mask);
	spin_unlock_irqrestore(&tui_lock, fls);
	return ret_mode;
}
EXPORT_SYMBOL(stui_clear_mask);
#ifdef CONFIG_SOC_EXYNOS5433
extern int stui_tsp_irq;
void stui_set_tsp_irq(int irq_num)
{
	stui_tsp_irq = irq_num;
	printk(KERN_DEBUG "[STUI] %s called![%d]\n", __func__, irq_num);
}
EXPORT_SYMBOL(stui_set_tsp_irq);
#endif
