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

#ifndef _SM5705_FLAHS_H
#define _SM5705_FLAHS_H

int sm5705_fled_flash_on(unsigned char index);
int sm5705_fled_led_off(unsigned char index);
int sm5705_fled_prepare_flash(unsigned char index);
int sm5705_fled_torch_on(unsigned char index);
int sm5705_fled_flash_on(unsigned char index);
int sm5705_fled_led_off(unsigned char index);
int sm5705_fled_close_flash(unsigned char index);
ssize_t rear_flash_store(struct device *dev, 	struct device_attribute *attr, const char *buf, size_t count);
ssize_t rear_flash_show(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t front_flash_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
ssize_t front_flash_show(struct device *dev, struct device_attribute *attr, char *buf);
#endif /* _SM5705_FLAHS_H */

