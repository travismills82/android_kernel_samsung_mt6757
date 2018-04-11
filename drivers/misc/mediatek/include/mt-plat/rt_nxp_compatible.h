/* drivers/misc/mediatek/include/mt-plat/rt_nxp_compatible.h
 * Header of Mediatek to compatible rt and nxp smart amp
 *
 * Copyright (C) 2017 MediaTek Inc.
 * Shane Chien <Shane.Chien@mediatek.com>
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

#ifndef MISC_MEDIATEK_RT_NXP_COMPATIBLE_H
#define MISC_MEDIATEK_RT_NXP_COMPATIBLE_H

extern int NXPExtSpk_set_i2c_client(struct i2c_client *);
extern int NXPExtSpk_i2c_client_registered(void);

#endif /*MISC_MEDIATEK_RT_NXP_COMPATIBLE_H*/

