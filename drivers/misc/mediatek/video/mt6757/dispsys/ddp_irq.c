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


#define LOG_TAG "IRQ"

#include "ddp_log.h"
#include "ddp_debug.h"

#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/timer.h>

/* #include <mach/mt_irq.h> */
#include "ddp_reg.h"
#include "ddp_irq.h"
#include "ddp_aal.h"
#include "ddp_gamma.h"
#include "ddp_drv.h"
#include "disp_helper.h"
#include "ddp_dsi.h"
#include "disp_drv_log.h"

/* IRQ log print kthread */
static struct task_struct *disp_irq_log_task;
static wait_queue_head_t disp_irq_log_wq;
static int disp_irq_log_module;

static int irq_init;

static unsigned int cnt_rdma_underflow[2];
static unsigned int cnt_rdma_abnormal[2];
static unsigned int cnt_ovl_underflow[OVL_NUM];
static unsigned int cnt_wdma_underflow[2];

unsigned long long rdma_start_time[2] = { 0 };
unsigned long long rdma_end_time[2] = { 0 };

#define DISP_MAX_IRQ_CALLBACK   10

static DDP_IRQ_CALLBACK irq_module_callback_table[DISP_MODULE_NUM][DISP_MAX_IRQ_CALLBACK];
static DDP_IRQ_CALLBACK irq_callback_table[DISP_MAX_IRQ_CALLBACK];

atomic_t ESDCheck_byCPU = ATOMIC_INIT(0);

/* dsi read by cpu should keep esd_check_bycmdq = 0.  */
/* dsi read by cmdq should keep esd_check_bycmdq = 1. */
static atomic_t esd_check_bycmdq = ATOMIC_INIT(1);

void disp_irq_esd_cust_bycmdq(int enable)
{
	atomic_set(&esd_check_bycmdq, enable);
}

int disp_irq_esd_cust_get(void)
{
	return atomic_read(&esd_check_bycmdq);
}

int disp_register_irq_callback(DDP_IRQ_CALLBACK cb)
{
	int i = 0;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_callback_table[i] == cb)
			break;
	}
	if (i < DISP_MAX_IRQ_CALLBACK)
		return 0;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_callback_table[i] == NULL)
			break;
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DDPERR("not enough irq callback entries for module\n");
		return -1;
	}
	DDPMSG("register callback on %d\n", i);
	irq_callback_table[i] = cb;
	return 0;
}

int disp_unregister_irq_callback(DDP_IRQ_CALLBACK cb)
{
	int i;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_callback_table[i] == cb) {
			irq_callback_table[i] = NULL;
			break;
		}
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DDPERR("Try to unregister callback function %p which was not registered\n", cb);
		return -1;
	}
	return 0;
}

int disp_register_module_irq_callback(enum DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb)
{
	int i;

	if (module >= DISP_MODULE_NUM) {
		DDPERR("Register IRQ with invalid module ID. module=%d\n", module);
		return -1;
	}
	if (cb == NULL) {
		DDPERR("Register IRQ with invalid cb.\n");
		return -1;
	}
	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_module_callback_table[module][i] == cb)
			break;
	}
	if (i < DISP_MAX_IRQ_CALLBACK)
		return 0;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_module_callback_table[module][i] == NULL)
			break;
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DDPERR("No enough callback entries for module %d.\n", module);
		return -1;
	}
	irq_module_callback_table[module][i] = cb;
	return 0;
}

int disp_unregister_module_irq_callback(enum DISP_MODULE_ENUM module, DDP_IRQ_CALLBACK cb)
{
	int i;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {
		if (irq_module_callback_table[module][i] == cb) {
			irq_module_callback_table[module][i] = NULL;
			break;
		}
	}
	if (i == DISP_MAX_IRQ_CALLBACK) {
		DDPERR
		    ("Try to unregister callback function with was not registered. module=%d cb=%p\n",
		     module, cb);
		return -1;
	}
	return 0;
}

void disp_invoke_irq_callbacks(enum DISP_MODULE_ENUM module, unsigned int param)
{
	int i;

	for (i = 0; i < DISP_MAX_IRQ_CALLBACK; i++) {

		if (irq_callback_table[i]) {
			/* DDPERR("Invoke callback function. module=%d param=0x%X\n", module, param); */
			irq_callback_table[i](module, param);
		}

		if (irq_module_callback_table[module][i]) {
			/* DDPERR("Invoke module callback function. module=%d param=0x%X\n", module, param); */
			irq_module_callback_table[module][i] (module, param);
		}
	}
}

static enum DISP_MODULE_ENUM disp_irq_module(unsigned int irq)
{
	enum DISP_REG_ENUM reg_module;

	for (reg_module = 0; reg_module < DISP_REG_NUM; reg_module++) {
		if (irq == dispsys_irq[reg_module])
			return ddp_get_reg_module(reg_module);
	}
	DDPERR("cannot find module for irq %d\n", irq);
	ASSERT(0);
	return DISP_MODULE_UNKNOWN;
}

static void disp_dump_rdma_underflow_regs(void)
{
	/* OVL0 */
	pr_err("OVL0[20][2C][38][3C]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_ROI_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_SRC_CON)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L0_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L0_OFFSET));
	pr_err("OVL0[58][5C][78][7C]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L1_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L1_OFFSET)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L2_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L2_OFFSET));
	pr_err("OVL0[98][9C][C0][D0]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L3_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_L3_OFFSET)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA0_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA0_FIFO_CTRL));
	pr_err("OVL0[E0][F0][100][110]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA1_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA1_FIFO_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA2_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA2_FIFO_CTRL));
	pr_err("OVL0[120][130][1E0][1E4]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA3_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA3_FIFO_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA0_MEM_GMC_S2)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + DISP_REG_OVL_RDMA1_MEM_GMC_S2));
	pr_err("OVL0[1E8][1EC][1F8][1FC]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1E8)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1EC)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1F8)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1FC));
	pr_err("OVL0[1E8][1EC][1F8][1FC]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1E8)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1EC)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1F8)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x1FC));
	pr_err("OVL0[20C][210][214][218]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x20C)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x210)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x214)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x218));
	pr_err("OVL0[21C]: %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0 + 0x21C));

	/* OVL1 */
	pr_err("OVL1[20][2C][38][218]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_ROI_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_SRC_CON)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_L0_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_L0_OFFSET));
	pr_err("OVL1[58][5C][C0][D0]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_L1_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_L1_OFFSET)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_RDMA0_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_RDMA0_FIFO_CTRL));
	pr_err("OVL1[E0][F0][1E0][1E4]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_RDMA1_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_RDMA1_FIFO_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_RDMA0_MEM_GMC_S2)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + DISP_REG_OVL_RDMA1_MEM_GMC_S2));
	pr_err("OVL1[1F8][1FC][20C][210]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + 0x1F8)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + 0x1FC)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + 0x20C)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + 0x210));
	pr_err("OVL1[214]: %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1 + 0x214));
	/* OVL0_2L */
	pr_err("OVL0_2L[20][2C][38][218]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_ROI_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_SRC_CON)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_L0_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_L0_OFFSET));
	pr_err("OVL0_2L[58][5C][C0][D0]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_L1_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_L1_OFFSET)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_RDMA0_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_RDMA0_FIFO_CTRL));
	pr_err("OVL0_2L[E0][F0][1E0][1E4]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_RDMA1_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_RDMA1_FIFO_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_RDMA0_MEM_GMC_S2)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + DISP_REG_OVL_RDMA1_MEM_GMC_S2));
	pr_err("OVL0_2L[1F8][1FC][20C][210]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + 0x1F8)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + 0x1FC)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + 0x20C)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + 0x210));
	pr_err("OVL0_2L[214]: %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL0_2L + 0x214));
	/* OVL1_2L */
	pr_err("OVL1_2L[20][2C][38][218]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_ROI_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_SRC_CON)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_L0_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_L0_OFFSET));
	pr_err("OVL1_2L[58][5C][C0][D0]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_L1_SRC_SIZE)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_L1_OFFSET)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_RDMA0_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_RDMA0_FIFO_CTRL));
	pr_err("OVL1_2L[E0][F0][1E0][1E4]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_RDMA1_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_RDMA1_FIFO_CTRL)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_RDMA0_MEM_GMC_S2)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + DISP_REG_OVL_RDMA1_MEM_GMC_S2));
	pr_err("OVL1_2L[1F8][1FC][20C][210]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + 0x1F8)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + 0x1FC)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + 0x20C)
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + 0x210));
	pr_err("OVL1_2L[214]: %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_DISP_OVL1_2L + 0x214));

	/* SMI */
	pr_err("SMI[100][104][114][220]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x100)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x104)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x114)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x220));
	pr_err("SMI[228][22C][308][30C]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x228)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x22C)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x308)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x30C));
	pr_err("SMI[310][314][318][31C]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x310)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x314)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x318)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x31C));
	pr_err("SMI[320][324]: %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x320)
		, DISP_REG_GET(DDP_REG_BASE_SMI_COMMON + 0x324));

	/* LARB0 */
	pr_err("LARB0[100][104][108][200]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB0 + 0x100)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB0 + 0x104)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB0 + 0x108)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB0 + 0x200));
	pr_err("LARB0[204][208]: %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB0 + 0x204)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB0 + 0x208));

	/* LARB4 */
	pr_err("LARB4[100][104][108][11C]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x100)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x104)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x108)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x11C));
	pr_err("LARB4[200][204][208][21C]: %08X %08X %08X %08X\n"
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x200)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x204)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x208)
		, DISP_REG_GET(DDP_REG_BASE_SMI_LARB4 + 0x21C));

	if (EMI_BASE_ADDR != NULL) {
		/* CEN_EMI */
		pr_err("EMI[60][78][100][108]: %08X %08X %08X %08X\n"
			, readl(EMI_BASE_ADDR + 0x60)
			, readl(EMI_BASE_ADDR + 0x78)
			, readl(EMI_BASE_ADDR + 0x100)
			, readl(EMI_BASE_ADDR + 0x108));
		pr_err("EMI[110][118][120][128]: %08X %08X %08X %08X\n"
			, readl(EMI_BASE_ADDR + 0x110)
			, readl(EMI_BASE_ADDR + 0x118)
			, readl(EMI_BASE_ADDR + 0x120)
			, readl(EMI_BASE_ADDR + 0x128));

		pr_err("EMI[130][138]: %08X %08X\n"
			, readl(EMI_BASE_ADDR + 0x130)
			, readl(EMI_BASE_ADDR + 0x138));

		/* EMI CHN0 */
		pr_err("CHN0[18][15C][164][1B0]: %08X %08X %08X %08X\n"
			, readl(CHN0_EMI_BASE_ADDR + 0x18)
			, readl(CHN0_EMI_BASE_ADDR + 0x15C)
			, readl(CHN0_EMI_BASE_ADDR + 0x164)
			, readl(CHN0_EMI_BASE_ADDR + 0x1B0));
		pr_err("CHN0[1B4][1B8]: %08X %08X\n"
			, readl(CHN0_EMI_BASE_ADDR + 0x1B4)
			, readl(CHN0_EMI_BASE_ADDR + 0x1B8));
	}
}

/* TODO:  move each irq to module driver */
unsigned int rdma_start_irq_cnt[2] = { 0, 0 };
unsigned int rdma_done_irq_cnt[2] = { 0, 0 };
unsigned int rdma_underflow_irq_cnt[2] = { 0, 0 };
unsigned int rdma_targetline_irq_cnt[2] = { 0, 0 };
unsigned int rdma_underflow_flag;
extern void dump_disp_info_log(void);
extern void dump_vcorefs_log(void);

irqreturn_t disp_irq_handler(int irq, void *dev_id)
{
	enum DISP_MODULE_ENUM module = DISP_MODULE_UNKNOWN;
	unsigned int reg_val = 0;
	unsigned int index = 0;
	unsigned int mutexID = 0;
	unsigned int reg_temp_val = 0;
	static unsigned int prn_count;

	DDPIRQ("disp_irq_handler, irq=%d, module=%s\n",
	       irq, ddp_get_module_name(disp_irq_module(irq)));

	if (irq == dispsys_irq[DISP_REG_DSI0] || irq == dispsys_irq[DISP_REG_DSI1]) {
		index = (irq == dispsys_irq[DISP_REG_DSI0]) ? 0 : 1;
		module = (irq == dispsys_irq[DISP_REG_DSI0]) ? DISP_MODULE_DSI0 : DISP_MODULE_DSI1;
		reg_val = (DISP_REG_GET(dsi_reg_va[index] + 0xC) & 0xff);
		reg_temp_val = reg_val;
		/* rd_rdy don't clear and wait for ESD & Read LCM will clear the bit. */
		if (disp_irq_esd_cust_get() == 1)
			reg_temp_val = reg_val & 0xfffe;
		DISP_CPU_REG_SET(dsi_reg_va[index] + 0xC, ~reg_temp_val);
				/* check dsi underrun */
		if (disp_helper_get_option(DISP_OPT_RDMA_UNDERFLOW_AEE) & 0x2) {
			if (DISP_REG_GET(dsi_reg_va[index] + 0x4) & 0x2) {
				DDPAEE("DSI%d underrun!\n", index);
				disp_helper_set_option(DISP_OPT_RDMA_UNDERFLOW_AEE,
					disp_helper_get_option(DISP_OPT_RDMA_UNDERFLOW_AEE) & (~2));
			}
		}
		MMProfileLogEx(ddp_mmp_get_events()->DSI_IRQ[index], MMProfileFlagPulse,
			reg_val, DISP_REG_GET(dsi_reg_va[index] + 0x4));
		DDPIRQ("DSI irq=%d, regval=0x%x\n", irq, reg_val);
	} else if (irq == dispsys_irq[DISP_REG_OVL0] ||
		   irq == dispsys_irq[DISP_REG_OVL1] ||
		   irq == dispsys_irq[DISP_REG_OVL0_2L] || irq == dispsys_irq[DISP_REG_OVL1_2L]) {

		module = disp_irq_module(irq);
		index = ovl_to_index(module);
		reg_val = DISP_REG_GET(DISP_REG_OVL_INTSTA + ovl_base_addr(module));
		if (reg_val & (1 << 0))
			DDPIRQ("IRQ: %s reg commit!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 1))
			DDPIRQ("IRQ: %s frame done!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 2))
			DDPERR("IRQ: %s frame underflow! cnt=%d\n", ddp_get_module_name(module),
			       cnt_ovl_underflow[index]++);

		if (reg_val & (1 << 3))
			DDPIRQ("IRQ: %s sw reset done\n", ddp_get_module_name(module));

		if (reg_val & (1 << 4))
			DDPERR("IRQ: %s hw reset done\n", ddp_get_module_name(module));

		if (reg_val & (1 << 5))
			DDPERR("IRQ: %s-L0 not complete until EOF!\n",
			       ddp_get_module_name(module));

		if (reg_val & (1 << 6))
			DDPERR("IRQ: %s-L1 not complete until EOF!\n",
			       ddp_get_module_name(module));

		if (reg_val & (1 << 7))
			DDPERR("IRQ: %s-L2 not complete until EOF!\n",
			       ddp_get_module_name(module));

		if (reg_val & (1 << 8))
			DDPERR("IRQ: %s-L3 not complete until EOF!\n",
			       ddp_get_module_name(module));
#if 0
		/* we don't care ovl underflow, it's not error */
		if (reg_val & (1 << 9))
			DDPERR("IRQ: %s-L0 fifo underflow!\n", ddp_get_module_name(module));


		if (reg_val & (1 << 10))
			DDPERR("IRQ: %s-L1 fifo underflow!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 11))
			DDPERR("IRQ: %s-L2 fifo underflow!\n", ddp_get_module_name(module));

		if (reg_val & (1 << 12))
			DDPERR("IRQ: %s-L3 fifo underflow!\n", ddp_get_module_name(module));
#endif
		if (reg_val & (1 << 13))
			DDPERR("IRQ: %s abnormal SOF!\n", ddp_get_module_name(module));

		DISP_CPU_REG_SET(DISP_REG_OVL_INTSTA + ovl_base_addr(module), ~reg_val);
		MMProfileLogEx(ddp_mmp_get_events()->OVL_IRQ[index], MMProfileFlagPulse, reg_val,
			       0);
		if (reg_val & 0x1e0)
			MMProfileLogEx(ddp_mmp_get_events()->ddp_abnormal_irq, MMProfileFlagPulse,
				       (index << 16) | reg_val, module);

	} else if (irq == dispsys_irq[DISP_REG_WDMA0] || irq == dispsys_irq[DISP_REG_WDMA1]) {
		index = (irq == dispsys_irq[DISP_REG_WDMA0]) ? 0 : 1;
		module =
		    (irq == dispsys_irq[DISP_REG_WDMA0]) ? DISP_MODULE_WDMA0 : DISP_MODULE_WDMA1;
		reg_val = DISP_REG_GET(DISP_REG_WDMA_INTSTA + index * DISP_WDMA_INDEX_OFFSET);
		if (reg_val & (1 << 0))
			DDPIRQ("IRQ: WDMA%d frame done!\n", index);

		if (reg_val & (1 << 1)) {
			DDPERR("IRQ: WDMA%d underrun! cnt=%d\n", index,
			       cnt_wdma_underflow[index]++);
			disp_irq_log_module |= 1 << module;
		}
		/* clear intr */
		DISP_CPU_REG_SET(DISP_REG_WDMA_INTSTA + index * DISP_WDMA_INDEX_OFFSET, ~reg_val);
		MMProfileLogEx(ddp_mmp_get_events()->WDMA_IRQ[index], MMProfileFlagPulse, reg_val,
			       DISP_REG_GET(DISP_REG_WDMA_CLIP_SIZE));
		if (reg_val & 0x2)
			MMProfileLogEx(ddp_mmp_get_events()->ddp_abnormal_irq, MMProfileFlagPulse,
				       (cnt_wdma_underflow[index] << 24) | (index << 16) | reg_val,
				       module);

	} else if (irq == dispsys_irq[DISP_REG_RDMA0] || irq == dispsys_irq[DISP_REG_RDMA1]) {
		if (dispsys_irq[DISP_REG_RDMA0] == irq) {
			index = 0;
			module = DISP_MODULE_RDMA0;
		} else if (dispsys_irq[DISP_REG_RDMA1] == irq) {
			index = 1;
			module = DISP_MODULE_RDMA1;
		}

		reg_val = DISP_REG_GET(DISP_REG_RDMA_INT_STATUS + index * DISP_RDMA_INDEX_OFFSET);
		if (reg_val & (1 << 0))
			DDPIRQ("IRQ: RDMA%d reg update done!\n", index);

		if (reg_val & (1 << 2)) {
			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index], MMProfileFlagEnd,
				       reg_val, DISP_REG_GET(dsi_reg_va[0] + 0x4));
			rdma_end_time[index] = sched_clock();
			DDPIRQ("IRQ: RDMA%d frame done!\n", index);
			rdma_done_irq_cnt[index]++;
		}
		if (reg_val & (1 << 1)) {
			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index],
				       MMProfileFlagStart, reg_val, DISP_REG_GET(dsi_reg_va[0] + 0x4));
			rdma_start_time[index] = sched_clock();
			DDPIRQ("IRQ: RDMA%d frame start!\n", index);
			rdma_start_irq_cnt[index]++;
		}
		if (reg_val & (1 << 3)) {
			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index], MMProfileFlagPulse,
				       reg_val, DISP_REG_GET(dsi_reg_va[0] + 0x4));

			DDPERR("IRQ: RDMA%d abnormal! cnt=%d\n", index, cnt_rdma_abnormal[index]++);
			disp_irq_log_module |= 1 << module;

		}
		if (reg_val & (1 << 4)) {

			MMProfileLogEx(ddp_mmp_get_events()->SCREEN_UPDATE[index], MMProfileFlagPulse,
				       reg_val, 1);

			DDPMSG("rdma%d, pix(%d,%d,%d,%d)\n",
			       index,
			       DISP_REG_GET(DISP_REG_RDMA_IN_P_CNT +
					    DISP_RDMA_INDEX_OFFSET * index),
			       DISP_REG_GET(DISP_REG_RDMA_IN_LINE_CNT +
					    DISP_RDMA_INDEX_OFFSET * index),
			       DISP_REG_GET(DISP_REG_RDMA_OUT_P_CNT +
					    DISP_RDMA_INDEX_OFFSET * index),
			       DISP_REG_GET(DISP_REG_RDMA_OUT_LINE_CNT +
					    DISP_RDMA_INDEX_OFFSET * index));
			DDPERR("IRQ: RDMA%d underflow! cnt=%d\n", index,
			       cnt_rdma_underflow[index]++);
			dump_vcorefs_log();
			if (disp_helper_get_option(DISP_OPT_RDMA_UNDERFLOW_AEE))
				DDPAEE("RDMA%d underflow!cnt=%d\n", index, cnt_rdma_underflow[index]++);
			disp_irq_log_module |= 1 << module;
			rdma_underflow_irq_cnt[index]++;
			
			if (!(prn_count % 1000))
				disp_dump_rdma_underflow_regs();
			prn_count++;

			if (rdma_underflow_irq_cnt[index])
				rdma_underflow_flag = TRUE;

		}
		if (reg_val & (1 << 5)) {
			DDPIRQ("IRQ: RDMA%d target line!\n", index);
			rdma_targetline_irq_cnt[index]++;
		}
		/* clear intr */
		DISP_CPU_REG_SET(DISP_REG_RDMA_INT_STATUS + index * DISP_RDMA_INDEX_OFFSET, ~reg_val);
		MMProfileLogEx(ddp_mmp_get_events()->RDMA_IRQ[index], MMProfileFlagPulse, reg_val, 0);
		if (reg_val & 0x18)
			MMProfileLogEx(ddp_mmp_get_events()->ddp_abnormal_irq, MMProfileFlagPulse,
				       (rdma_underflow_irq_cnt[index] << 24) | (index << 16) | reg_val, module);

	} else if (irq == dispsys_irq[DISP_REG_COLOR0]) {
		DDPERR("color irq happens!! %d\n", irq);
	} else if (irq == dispsys_irq[DISP_REG_MUTEX]) {
		/* mutex0: perimary disp */
		/* mutex1: sub disp */
		/* mutex2: aal */
		module = DISP_MODULE_MUTEX;
		reg_val = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA) & DISP_MUTEX_INT_MSK;
		for (mutexID = 0; mutexID < 5; mutexID++) {
			if (reg_val & (0x1 << mutexID)) {
				DDPIRQ("IRQ: mutex%d sof!\n", mutexID);
				MMProfileLogEx(ddp_mmp_get_events()->MUTEX_IRQ[mutexID],
					       MMProfileFlagPulse, reg_val, 0);
			}
			if (reg_val & (0x1 << (mutexID + DISP_MUTEX_TOTAL))) {
				DDPIRQ("IRQ: mutex%d eof!\n", mutexID);
				MMProfileLogEx(ddp_mmp_get_events()->MUTEX_IRQ[mutexID],
					       MMProfileFlagPulse, reg_val, 1);
			}
		}
		DISP_CPU_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, ~reg_val);
	} else if (irq == dispsys_irq[DISP_REG_AAL0]) {
		module = DISP_MODULE_AAL0;
		reg_val = DISP_REG_GET(DISP_AAL_INTSTA);
		disp_aal_on_end_of_frame();
	} else if (irq == dispsys_irq[DISP_REG_CCORR0]) {
		module = DISP_MODULE_CCORR0;
		reg_val = DISP_REG_GET(DISP_REG_CCORR_INTSTA);
		disp_ccorr_on_end_of_frame();
	} else if (irq == dispsys_irq[DISP_REG_CONFIG]) {	/* MMSYS error intr */
		reg_val = DISP_REG_GET(DISP_REG_CONFIG_MMSYS_INTSTA) & 0x7;
		if (reg_val & (1 << 0))
			DDPERR("MMSYS to MFG APB TX Error, MMSYS clock off but MFG clock on!\n");

		if (reg_val & (1 << 1))
			DDPERR("MMSYS to MJC APB TX Error, MMSYS clock off but MJC clock on!\n");

		if (reg_val & (1 << 2))
			DDPERR("PWM APB TX Error!\n");

		DISP_CPU_REG_SET(DISP_REG_CONFIG_MMSYS_INTSTA, ~reg_val);
	} else if (irq == dispsys_irq[DISP_REG_DPI0]) {
		module = DISP_MODULE_DPI;
		reg_val = DISP_REG_GET(DISP_REG_DPI_INSTA) & 0x7;
		DISP_CPU_REG_SET(DISP_REG_DPI_INSTA, 0);
	} else {
		module = DISP_MODULE_UNKNOWN;
		reg_val = 0;
		DDPERR("invalid irq=%d\n ", irq);
	}

	disp_invoke_irq_callbacks(module, reg_val);
	if (disp_irq_log_module != 0)
		wake_up_interruptible(&disp_irq_log_wq);

	MMProfileLogEx(ddp_mmp_get_events()->DDP_IRQ, MMProfileFlagEnd, irq, reg_val);
	return IRQ_HANDLED;
}


static int disp_irq_log_kthread_func(void *data)
{
	unsigned int i = 0;

	while (1) {
		wait_event_interruptible(disp_irq_log_wq, disp_irq_log_module);
		DDPMSG("disp_irq_log_kthread_func dump intr register: disp_irq_log_module=%d\n",
		       disp_irq_log_module);
		for (i = 0; i < DISP_MODULE_NUM; i++) {
			if ((disp_irq_log_module & (1 << i)) != 0)
				ddp_dump_reg(i);

		}
		disp_irq_log_module = 0;
	}
	return 0;
}

void disp_register_dev_irq(unsigned int irq_num, char *device_name)
{
	if (request_irq(irq_num, (irq_handler_t) disp_irq_handler,
			IRQF_TRIGGER_LOW, device_name, NULL))
		DDPERR("ddp register irq %u failed on device %s\n", irq_num, device_name);

}

int disp_init_irq(void)
{
	if (irq_init)
		return 0;

	irq_init = 1;
	DDPMSG("disp_init_irq\n");

	/* create irq log thread */
	init_waitqueue_head(&disp_irq_log_wq);
	disp_irq_log_task = kthread_create(disp_irq_log_kthread_func, NULL, "ddp_irq_log_kthread");
	if (IS_ERR(disp_irq_log_task))
		DDPERR(" can not create disp_irq_log_task kthread\n");

	/* wake_up_process(disp_irq_log_task); */
	return 0;
}
