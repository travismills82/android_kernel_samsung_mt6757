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
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   CAT24s128c.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of EEPROM driver
 *
 *
 * Author:
 * -------
 *   Dream Yeh (MTK08783)
 *
 *============================================================================
 */
#ifndef __CAT24S128C_H
#define __CAT24S128C_H
#include <linux/i2c.h>


/*extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);*/
extern int iBurstReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
/*extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);*/
extern int iReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId);

unsigned int cat24s128c_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size);


#endif /* __EEPROM_H */

