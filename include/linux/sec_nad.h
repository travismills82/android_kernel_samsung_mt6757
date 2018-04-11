/* sec_nad.h
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef SEC_NAD_H
#define SEC_NAD_H

#if defined(CONFIG_SEC_FACTORY)
#define NAD_PARAM_NAME "/dev/block/mmcblk0p8"	// bota2 : /dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/BOTA2
#define NAD_DRAM_OFFSET 7340032

#define NAD_PARAM_READ  0
#define NAD_PARAM_WRITE 1
#define NAD_PARAM_EMPTY 2

#define NAD_DRAM_TEST_NONE  0
#define NAD_DRAM_TEST_PASS  1
#define NAD_DRAM_TEST_FAIL  2

#define NAD_BUFF_SIZE       10 
#define NAD_CMD_LIST        3

struct nad_env {
    char nad_acat[10];
    unsigned int nad_acat_loop_count;
    unsigned int nad_acat_exec_count;
	unsigned int nad_dram_test_check;

	char nad_dram_test_need[4];
	unsigned int nad_dram_test_result;
	unsigned int nad_dram_fail_data;
	volatile unsigned long nad_dram_fail_address;
};

struct sec_nad_param {
	struct work_struct sec_nad_work;
	struct delayed_work sec_nad_delay_work;
	unsigned long offset;
	int state;
	int retry_cnt;
	int curr_cnt;
};

static struct sec_nad_param sec_nad_param_data;
static struct nad_env sec_nad_env;
extern unsigned int lpcharge;
static int erase_pass;
static DEFINE_MUTEX(sec_nad_param_lock);

#endif
#endif