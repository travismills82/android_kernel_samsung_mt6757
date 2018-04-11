/* tui/stui_core.h
 *
 * Samsung TUI HW Handler driver.
 *
 * Copyright (c) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __STUI_CORE_H_
#define __STUI_CORE_H_

#include <linux/fs.h>
#include <linux/types.h>
#include <linux/wakelock.h>

#define STUI_ALIGN_1MB_SZ   0x100000  /* 1MB */
#define STUI_ALIGN_16MB_SZ  0x1000000 /* 16MB */
#define STUI_ALIGN_32MB_SZ  0x2000000 /* 32MB */

#define STUI_ALIGN_UP(size, block) ((((size) + (block) - 1) / (block)) * (block))

/* TODO:
 * Set correct framebuffer size according to the device specifications
 */
#define STUI_FRAME_BUF_SIZE (1920*1080*4) /* Frame Buffer */
#define STUI_WORK_BUF_SIZE  (1920*1080*8) /* Work Buffer for TUI lib */
#define STUI_DISP_BUF_SIZE  0x100000      /* Internal Buffer for Display Driver TA */
#define STUI_BUFFER_NUM     3

#define STUI_DEV_NAME "tuihw"

extern struct wake_lock tui_wakelock;

long stui_process_cmd(struct file *f, unsigned int cmd, unsigned long arg);

struct stui_buf_info {
	unsigned long pa[STUI_BUFFER_NUM];
	size_t size[STUI_BUFFER_NUM];
};

#endif /* __STUI_CORE_H_ */
