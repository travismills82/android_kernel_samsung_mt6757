/* tui/stui_hal_display.c
 *
 * TUI HW Handler driver. Display functions.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/fb.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#include <linux/stui_inf.h>

#include "stui_core.h"
#include "stui_hal.h"
#include "ion_drv.h"
#include "tui_platform.h"

static struct ion_client *client;
static struct ion_handle *handle;
static struct dma_buf *dbuf;
static int buffer_allocated;

/* TODO:
 * Set ion driver parameters according to the device
 */
#define ION_DEV		g_ion_device
#define ION_ID_MASK	(1 << 4)		/* Use contiguous memory*/
#define ION_FLAGS	(1 << (32 - 11))	/* Use memory for video*/

static struct fb_info *get_fb_info_for_tui(struct device *fb_dev);

/* Framebuffer device driver identification
 * RETURN: 0 - Wrong device driver
 *         1 - Suitable device driver
 */
static int _is_dev_ok(struct device *fb_dev, const void *p)
{
	struct fb_info *fb_info;

	fb_info = get_fb_info_for_tui(fb_dev);
	if (!fb_info)
		return 0;

	/* TODO:
	 * Place your custom identification here
	 */

	return 1;
}

/* Find suitable framebuffer device driver */
static struct device *get_fb_dev_for_tui(void)
{
	struct device *fb_dev;

	/* get the first framebuffer device */
	fb_dev = class_find_device(fb_class, NULL, NULL, _is_dev_ok);
	if (!fb_dev)
		pr_err("[STUI] class_find_device failed\n");

	return fb_dev;
}

/* Get framebuffer's internal data */
static struct fb_info *get_fb_info_for_tui(struct device *fb_dev)
{
	struct fb_info *fb_item;

	if (!fb_dev || !fb_dev->p) {
		pr_err("[STUI] framebuffer device has no private data\n");
		return NULL;
	}
	fb_item = (struct fb_info *)dev_get_drvdata(fb_dev);
	if (!fb_item)
		pr_err("[STUI] dev_get_drvdata failed\n");

	return fb_item;
}

/* Get display controller internal structure from the framebuffer info */
static void *get_disp_dev_for_tui(void)
{
	struct device *fb_dev;
	struct fb_info *fb_info;
	void *disp_dev_data = (void *)0xDEADBEAF; /* TODO: NULL;*/

	fb_dev = get_fb_dev_for_tui();
	if (!fb_dev)
		return NULL;

	fb_info = get_fb_info_for_tui(fb_dev);
	if (!fb_info)
		return NULL;

	/* TODO:
	 * Place your custom way to get internal data here
	disp_dev_data = fb_info->internal_data;
	 */

	if (!disp_dev_data)
		pr_err("[STUI] disp_dev_data pointer is NULL\n");

	return disp_dev_data;
}

/* Prepare / restore display controller's registers */
static int disp_tui_prepare(void *disp_dev_data, bool tui_en)
{
	int ret = 0;
	
	pr_debug("[STUI] disp_tui_protection(%d) - start\n", (int)tui_en);
	if (tui_en) {
		/* TODO:
		 * Prepare display constroller's SFR registers to start TUI here

#ifdef CONFIG_PM_RUNTIME
		pm_runtime_get_sync(disp_dev_data->dev);
#endif
		 */
#ifdef TUI_ENABLE_DISPLAY
		ret = display_enter_tui();
#endif

	} else {
#ifdef TUI_ENABLE_DISPLAY
		ret = display_exit_tui();
#endif

		/* TODO:
		 * Restore display constroller's SFR registers here

#ifdef CONFIG_PM_RUNTIME
		pm_runtime_put_sync(disp_dev_data->dev);
#endif
		 */

	}
	msleep(100);
	return ret;
}


static int fb_protection_for_tui(void)
{
	void *disp_data = NULL;

	pr_debug("[STUI] fb_protection_for_tui() - start\n");
	disp_data = get_disp_dev_for_tui();
	if (!disp_data)
		return -1;

	return disp_tui_prepare(disp_data, true);
}

static int fb_unprotection_for_tui(void)
{
	void *disp_data = NULL;

	pr_debug("[STUI] fb_unprotection_for_tui() - start\n");
	disp_data = get_disp_dev_for_tui();
	if (!disp_data)
		return -1;

	return disp_tui_prepare(disp_data, false);
}

static void __maybe_unused stui_free_video_space_to_ion(void)
{
	dma_buf_put(dbuf);
	ion_free(client, handle);
	ion_client_destroy(client);
}

static unsigned long __maybe_unused stui_alloc_video_from_ion(void)
{
	unsigned long phys_addr = 0;
	unsigned long offset = 0;
	unsigned long size;
	unsigned long frame_buf_sz;
	unsigned long work_buf_sz;
	unsigned long disp_buf_sz;

	frame_buf_sz = STUI_ALIGN_UP(STUI_FRAME_BUF_SIZE, STUI_ALIGN_1MB_SZ);
	work_buf_sz = STUI_ALIGN_UP(STUI_WORK_BUF_SIZE, STUI_ALIGN_1MB_SZ);
	disp_buf_sz = STUI_ALIGN_UP(STUI_DISP_BUF_SIZE, STUI_ALIGN_1MB_SZ);
	size = frame_buf_sz + work_buf_sz + disp_buf_sz;

	client = ion_client_create(ION_DEV, "STUI module");
	if (IS_ERR_OR_NULL(client)) {
		pr_err("[STUI] ion_client_create() - failed: %ld\n", PTR_ERR(client));
		return 0;
	}

	handle = ion_alloc(client, size, 0, ION_ID_MASK,
			ION_FLAGS);

	if (IS_ERR_OR_NULL(handle)) {
		pr_err("[STUI] ion_alloc() - failed: %ld\n", PTR_ERR(handle));
		goto clean_client;
	}

	dbuf = ion_share_dma_buf(client, handle);
	if (IS_ERR_OR_NULL(dbuf)) {
		pr_err("[STUI] ion_share_dma_buf() - failed: %ld\n", PTR_ERR(dbuf));
		goto clean_alloc;
	}

	ion_phys(client, handle, (unsigned long *)&phys_addr, &dbuf->size);
	if (!phys_addr)
		goto clean_share_dma;

	/* TUI frame buffer must be aligned 1M
	 * TODO:
	 * Set different alingment, if required
	 */
	if (phys_addr % STUI_ALIGN_1MB_SZ)
		offset = STUI_ALIGN_1MB_SZ - (phys_addr % STUI_ALIGN_1MB_SZ);

	phys_addr = phys_addr + offset;
	size -= offset;
	pr_debug("[STUI] phys_addr : %lx\n", phys_addr);

	/* frame buffer pa */
	g_stui_buf_info.pa[0] = phys_addr;
	g_stui_buf_info.size[0] = frame_buf_sz;

	/* working buffer pa */
	g_stui_buf_info.pa[1] =  g_stui_buf_info.pa[0] + frame_buf_sz;
	g_stui_buf_info.size[1] = work_buf_sz;

	/* dispaly internal buffer pa */
	g_stui_buf_info.pa[2] =  g_stui_buf_info.pa[1] + work_buf_sz;
	g_stui_buf_info.size[2] = disp_buf_sz;
	return phys_addr;

clean_share_dma:
	dma_buf_put(dbuf);

clean_alloc:
	ion_free(client, handle);

clean_client:
	ion_client_destroy(client);

	return phys_addr;
}

static void stui_free_video_space_to_ssvp(void)
{
	if (buffer_allocated) {
		tui_region_online();
		buffer_allocated = 0;
	}
}

static unsigned long stui_alloc_video_from_ssvp(void)
{
	int ret = 0;
	phys_addr_t pa;
	phys_addr_t pa_next;
	unsigned long size;
	unsigned long frame_buf_sz;
	unsigned long work_buf_sz;
	unsigned long disp_buf_sz;

	frame_buf_sz = STUI_ALIGN_UP(STUI_FRAME_BUF_SIZE, STUI_ALIGN_1MB_SZ);
	work_buf_sz = STUI_ALIGN_UP(STUI_WORK_BUF_SIZE, STUI_ALIGN_1MB_SZ);
	disp_buf_sz = STUI_ALIGN_UP(STUI_DISP_BUF_SIZE, STUI_ALIGN_1MB_SZ);
	size = frame_buf_sz + work_buf_sz + disp_buf_sz;

	pr_debug("%s(%d): Requested size=0x%lx\n", __func__, __LINE__, size);

	ret = tui_region_offline(&pa, &size);
	if (ret) {
		pr_err("%s(%d): tui_region_offline failed!\n",
				__func__, __LINE__);
		return 0;
	}
	buffer_allocated = 1;
	pr_debug("alloc (p=%p s=0x%lx)\n", (void *)pa, size);

	/* frame buffer pa */
	g_stui_buf_info.pa[0] = (unsigned long)pa;
	g_stui_buf_info.size[0] = frame_buf_sz;

	/* working buffer pa */
	pa_next = g_stui_buf_info.pa[0] + frame_buf_sz;
	g_stui_buf_info.pa[1] =  (unsigned long)pa_next;
	g_stui_buf_info.size[1] = work_buf_sz;

	/* dispaly internal buffer pa */
	pa_next = g_stui_buf_info.pa[1] + work_buf_sz;
	g_stui_buf_info.pa[2] =  (unsigned long)pa_next;
	g_stui_buf_info.size[2] = disp_buf_sz;

	pr_debug("alloc buf (0x%lx,%zx),(0x%lx,%zx),(0x%lx,%zx)\n",
			g_stui_buf_info.pa[0], g_stui_buf_info.size[0],
			g_stui_buf_info.pa[1], g_stui_buf_info.size[1],
			g_stui_buf_info.pa[2], g_stui_buf_info.size[2]);

	return pa;
}

void stui_free_video_space(void)
{
#ifdef TUI_ENABLE_MEMORY_SSVP
	return stui_free_video_space_to_ssvp();
#else
	return stui_free_video_space_to_ion();
#endif

}

unsigned long stui_alloc_video_space(void)
{
#ifdef TUI_ENABLE_MEMORY_SSVP
	return stui_alloc_video_from_ssvp();
#else
	return stui_alloc_video_from_ion();
#endif
}

int stui_prepare_tui(void)
{
	pr_debug("[STUI] stui_prepare_tui() - start\n");
	return fb_protection_for_tui();
}

void stui_finish_tui(void)
{
	pr_debug("[STUI] stui_finish_tui() - start\n");
	fb_unprotection_for_tui();
	stui_free_video_space();
}
