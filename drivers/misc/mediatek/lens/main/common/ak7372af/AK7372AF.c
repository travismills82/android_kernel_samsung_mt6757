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

/*
 * AK7372AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"
#ifdef CONFIG_CAMERA_HW_BIG_DATA
#include "hwparam.h"
#endif

#define AF_DRVNAME "AK7372AF_DRV"

#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;


static int s4AF_ReadReg(u8 a_uAddr, u16 *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff;
	char puSendCmd[1];

	puSendCmd[0] = a_uAddr;

	//g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	if (strcmp(CONFIG_ARCH_MTK_PROJECT, "titan6757_jade_n") == 0)
	{
		g_pstAF_I2Cclient->addr = 0x18;//AF_I2C_SLAVE_ADDR;
	}
	else
	{
		g_pstAF_I2Cclient->addr = 0x1e;//AF_I2C_SLAVE_ADDR;
	}
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 1);

	if (i4RetValue < 0) {
		LOG_INF("I2C read - send failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, &pBuff, 1);

	if (i4RetValue < 0) {
		LOG_INF("I2C read - recv failed!!\n");
		return -1;
	}
	*a_pu2Result = pBuff;

	return 0;
}

#if 0
static int s4AF_ReadReg(u8 a_uAddr, u16 *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff = 0;
	char puSendCmd[1];
	struct i2c_msg msg[2];

	puSendCmd[0] = a_uAddr;
	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	msg[0].addr = g_pstAF_I2Cclient->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf =  puSendCmd;
	msg[1].addr = g_pstAF_I2Cclient->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &pBuff;
	i4RetValue = i2c_transfer(g_pstAF_I2Cclient->adapter, msg, 2);
	if (i4RetValue != 2) {
	LOG_INF("Read failed.\n");
	return -1;
	}
	*a_pu2Result = pBuff;
	return 0;
}
#endif

static int s4AF_WriteReg(u16 a_u2Addr, u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[2] = { (char)a_u2Addr, (char)a_u2Data };

	//g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;
	if (strcmp(CONFIG_ARCH_MTK_PROJECT, "titan6757_jade_n") == 0)
	{
		g_pstAF_I2Cclient->addr = 0x18;//AF_I2C_SLAVE_ADDR;
	}
	else
	{
		g_pstAF_I2Cclient->addr = 0x1e;//AF_I2C_SLAVE_ADDR;
	}

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C write failed!!\n");
		return -1;
	}

	return 0;
}

static inline int getAFInfo(__user stAF_MotorInfo * pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int setVCMPos(unsigned long a_u4Position)
{
	int i4RetValue = 0;

	i4RetValue = s4AF_WriteReg(0x0, (u16) ((a_u4Position >> 2) & 0xff));

	if (i4RetValue < 0)
		return -1;

	i4RetValue = s4AF_WriteReg(0x1, (u16) ((g_u4TargetPosition & 0x3) << 6));
	return i4RetValue;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
#ifdef CONFIG_CAMERA_HW_BIG_DATA
	struct cam_hw_param *hw_param = NULL;
#endif

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos, InitPosM, InitPosL, data;

#ifdef CONFIG_CAMERA_HW_BIG_DATA
		if (s4AF_ReadReg(0x02, &data)  < 0) {
			is_sec_get_rear_hw_param(&hw_param);

			if (hw_param)
				hw_param->i2c_af_err_cnt++;				
		}
#else
		s4AF_ReadReg(0x02, &data);
#endif
		LOG_INF("Addr : 0x02 , Data : 0x%x\n", data);

		/* 00:active mode        10:Standby mode    x1:Sleep mode */
		s4AF_WriteReg(0x02, 0x00);	/* from Standby mode to Active mode */
		msleep(20);
		s4AF_ReadReg(0x0, &InitPosM);
		ret = s4AF_ReadReg(0x1, &InitPosL);
		InitPos = ((InitPosM & 0xFF) << 2) + ((InitPosL >> 6) & 0x3);
		s4AF_ReadReg(0x02, &data);
		LOG_INF("After setted, addr:0x02, data: 0x%x\n", data);

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	LOG_INF("move [curr] %d [target] %d\n", (u16)g_u4CurrPosition, (u16)g_u4TargetPosition);
	/* s4AF_WriteReg(0x02, 0x00); */

	if (setVCMPos(g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
	}

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long AK7372AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int AK7372AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		s4AF_WriteReg(0x02, 0x20);
		msleep(20);
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int AK7372AF_PowerDown(void)
{
	LOG_INF("+\n");
	if (*g_pAF_Opened == 0) {
		unsigned short data = 0;
		int cnt = 0;

		while (1) {
			data = 0;

			s4AF_WriteReg(0x02, 0x20);

			s4AF_ReadReg(0x02, &data);

			LOG_INF("Addr : 0x02 , Data : %x\n", data);

			if (data == 0x20 || cnt == 3)
				break;

			cnt++;
		}
	}
	LOG_INF("-\n");

	return 0;
}

void AK7372AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
