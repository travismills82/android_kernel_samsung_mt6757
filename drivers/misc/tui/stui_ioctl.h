/* tui/stui_ioctl.h
 *
 * Samsung TUI HW Handler driver.
 *
 * Copyright (c) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __STUI_IOCTL_H_
#define __STUI_IOCTL_H_

/* Commands and Structures for TUI */
#define STUI_HW_IOCTL_START_TUI		0x10
#define STUI_HW_IOCTL_FINISH_TUI	0x11
#ifdef SAMSUNG_TUI_TEST
#define STUI_HW_IOCTL_GET_FD_TUI	0x12
#endif

struct tui_hw_buffer {
	uint64_t fb_physical;
	uint64_t fb_size;
	uint64_t wb_physical;
	uint64_t wb_size;
	uint64_t disp_physical;
	uint64_t disp_size;
} __packed;

#define STUI_RET_OK			0x00030000
#define STUI_RET_ERR_INTERNAL_ERROR	0x00030003

#endif /* __STUI_IOCTL_H_ */
