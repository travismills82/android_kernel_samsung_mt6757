/*
 * Copyright (C) 2015 Samsung Electronics. All rights reserved.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/ioctl.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include "ese_p3.h"

#ifdef CONFIG_MACH_MT6757
#include "../spi/mediatek/mt6757/mtk_spi.h"
#include "../spi/mediatek/mt6757/mtk_spi_hal.h"
#endif

#include <linux/pm_runtime.h>
#include <linux/spi/spidev.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

/* Undef if want to keep eSE Power LDO ALWAYS ON */
#define FEATURE_ESE_POWER_ON_OFF

#define SPI_DEFAULT_SPEED 6500000L

/* size of maximum read/write buffer supported by driver */
#define MAX_BUFFER_SIZE   259U

/* Different driver debug lever */
enum P3_DEBUG_LEVEL {
	P3_DEBUG_OFF,
	P3_FULL_DEBUG
};

/* Variable to store current debug level request by ioctl */
static unsigned char debug_level = P3_FULL_DEBUG;

#define P3_DBG_MSG(msg...) do { \
		switch (debug_level) { \
		case P3_DEBUG_OFF: \
			break; \
		case P3_FULL_DEBUG: \
			pr_info("[ESE-P3] :  " msg); \
			break; \
		default: \
			pr_err("[ESE-P3] : debug level %d", debug_level);\
			break; \
		}; \
	} while (0);

#define P3_ERR_MSG(msg...) pr_err("[ESE-P3] : " msg);
#define P3_INFO_MSG(msg...) pr_info("[ESE-P3] : " msg);

static DEFINE_MUTEX(device_list_lock);

#ifdef CONFIG_ESE_SECURE
#define TO_STR2(s) #s
#define TO_STR(s) TO_STR2(s)
/* SPI_CLK(s) produces "spi3-sclk" */
#define SPI_CLK(x) "spi" TO_STR(CONFIG_ESE_SECURE_SPI_PORT) "-" #x "clk"
#endif
enum VDD_CONTROL_TYPE {
	VDD_GPIO,
	VDD_REGULATOR,
};

enum ese_power_state {
	ESE_POWER_OFF = 0,
	ESE_POWER_ON,
};

/* Device specific macro and structure */
struct p3_data {
	wait_queue_head_t read_wq; /* wait queue for read interrupt */
	struct mutex buffer_mutex; /* buffer mutex */
	struct spi_device *spi;  /* spi device structure */
	struct miscdevice p3_device; /* char device as misc driver */

	unsigned int users;

	bool device_opened;
#ifdef FEATURE_ESE_WAKELOCK
	struct wake_lock ese_lock;
#endif
	unsigned long speed;
	int vdd_gpio;
	enum VDD_CONTROL_TYPE vdd_type;
	const char *vdd_1p8;
#ifndef CONFIG_ESE_SECURE
	struct pinctrl *p;
	struct pinctrl_state *spi1_miso_set_cfg;
	struct pinctrl_state *spi1_miso_clr_cfg;
	struct pinctrl_state *spi1_cs_set_cfg;
	struct pinctrl_state *spi1_cs_clr_cfg;
	struct pinctrl_state *spi1_mosi_set_cfg;
	struct pinctrl_state *spi1_mosi_clr_cfg;
	struct pinctrl_state *spi1_clk_set_cfg;
	struct pinctrl_state *spi1_clk_clr_cfg;
#ifdef CONFIG_MACH_MT6757
    struct mt_chip_conf spi_conf;
#endif
#endif
};

static int p3_regulator_onoff(struct p3_data *data, int onoff)
{
	int rc = 0;
	struct regulator *regulator_vdd_1p8;

	if (data->vdd_type == VDD_GPIO) {

		if (gpio_is_valid(data->vdd_gpio)) { 
			gpio_direction_output(data->vdd_gpio, onoff);
			P3_DBG_MSG("%s - vdd %d\n", __func__, onoff);
		}


	} else {

		if (!data->vdd_1p8) {
			pr_err("%s No vdd LDO name!\n", __func__);
			return -ENODEV;
		}

		regulator_vdd_1p8 = regulator_get(NULL, data->vdd_1p8);
		pr_err("%s %s\n", __func__, data->vdd_1p8);

		if (IS_ERR(regulator_vdd_1p8) || regulator_vdd_1p8 == NULL) {
			P3_ERR_MSG("%s - vdd_1p8 regulator_get fail\n", __func__);
			return -ENODEV;
		}

		P3_DBG_MSG("%s - onoff = %d\n", __func__, onoff);
		if (onoff == 1) {
			rc = regulator_enable(regulator_vdd_1p8);
			if (rc) {
				P3_ERR_MSG("%s - enable vdd_1p8 failed, rc=%d\n",
						__func__, rc);
				goto done;
			}
			msleep(20);
		} else {
			rc = regulator_disable(regulator_vdd_1p8);
			if (rc) {
				P3_ERR_MSG("%s - disable vdd_1p8 failed, rc=%d\n",
						__func__, rc);
				goto done;
			}
		}
	/*data->regulator_is_enable = (u8)onoff;*/

done:
	regulator_put(regulator_vdd_1p8);
	}

	return rc;
}

#ifndef CONFIG_ESE_SECURE
static int p3_xfer(struct p3_data *p3_device, struct p3_ioctl_transfer *tr)
{
	int status = 0;
	struct spi_message m;
	struct spi_transfer t;
	unsigned char tx_buffer[MAX_BUFFER_SIZE] = {0x0, };
	unsigned char rx_buffer[MAX_BUFFER_SIZE] = {0x0, };

	P3_DBG_MSG("%s\n", __func__);

	if (p3_device == NULL || tr == NULL)
		return -EFAULT;

	if (tr->len > DEFAULT_BUFFER_SIZE || !tr->len)
		return -EMSGSIZE;

	if (tr->tx_buffer != NULL) {
		if (copy_from_user(tx_buffer,
				tr->tx_buffer, tr->len) != 0)
			return -EFAULT;
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));

	t.tx_buf = tx_buffer;
	t.rx_buf = rx_buffer;
	t.len = tr->len;

	spi_message_add_tail(&t, &m);

	status = spi_sync(p3_device->spi, &m);
	if (status == 0) {
		if (tr->rx_buffer != NULL) {
			unsigned int missing = 0;

			missing = (unsigned int)copy_to_user(tr->rx_buffer,
					       rx_buffer, tr->len);

			if (missing != 0)
				tr->len = tr->len - missing;
		}
	}
	pr_debug("%s p3_xfer,length=%d\n", __func__, tr->len);
	return status;

} /* vfsspi_xfer */

static int p3_rw_spi_message(struct p3_data *p3_device,
				 unsigned long arg)
{
	struct p3_ioctl_transfer   *dup = NULL;
	int err = 0;

	dup = kmalloc(sizeof(struct p3_ioctl_transfer), GFP_KERNEL);
	if (dup == NULL)
		return -ENOMEM;

	if (copy_from_user(dup, (void *)arg,
			   sizeof(struct p3_ioctl_transfer)) != 0) {
		kfree(dup);
		return -EFAULT;
	} else {
		err = p3_xfer(p3_device, dup);
		if (err != 0) {
			kfree(dup);
			P3_ERR_MSG("%s xfer failed!\n", __func__);
			return err;
		}
	}

	/*P3_ERR_MSG("%s len:%u\n", __func__, dup->len);*/
	if (copy_to_user((void *)arg, dup,
			 sizeof(struct p3_ioctl_transfer)) != 0)
		return -EFAULT;
	kfree(dup);
	return 0;
}

static void ese_spi_pin_control(struct p3_data *p3_device, int pin_set)
{
	int ret = 0;

	switch (pin_set) {
		case ESE_POWER_ON:
			if (!IS_ERR(p3_device->spi1_miso_set_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_miso_set_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_miso_set_cfg\n",	__func__);
			}
			if (!IS_ERR(p3_device->spi1_cs_set_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_cs_set_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_cs_set_cfg\n", __func__);
			}
			if (!IS_ERR(p3_device->spi1_mosi_set_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_mosi_set_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_mosi_set_cfg\n", __func__);
			}
			if (!IS_ERR(p3_device->spi1_clk_set_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_clk_set_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_clk_set_cfg\n", __func__);
			}
			break;
		case ESE_POWER_OFF:
			if (!IS_ERR(p3_device->spi1_miso_clr_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_miso_clr_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_miso_clr_cfg\n",	__func__);
			}
			if (!IS_ERR(p3_device->spi1_cs_clr_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_cs_clr_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_cs_clr_cfg\n", __func__);
			}
			if (!IS_ERR(p3_device->spi1_mosi_clr_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_mosi_clr_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_mosi_clr_cfg\n",	__func__);
			}
			if (!IS_ERR(p3_device->spi1_clk_clr_cfg)) {
				ret = pinctrl_select_state(p3_device->p,
						p3_device->spi1_clk_clr_cfg);
				if (ret)
					P3_ERR_MSG("%s: can't set spi0_clk_clr_cfg %d\n", __func__, ret);
			}
			break;
	}
	P3_DBG_MSG("%s -end %d\n", __func__, pin_set);
}

#ifdef CONFIG_COMPAT
static int p3_rw_spi_message_32(struct p3_data *p3_device, unsigned long arg)
{
	struct p3_ioctl_transfer dup;
	struct spip3_ioc_transfer_32 p3transfr_32;
	int err = 0;

	if (__copy_from_user(&p3transfr_32, (void __user *)arg,
			sizeof(struct spip3_ioc_transfer_32))) {
			P3_ERR_MSG("%s, failed to copy from user\n", __func__);
			return -EFAULT;
	}

	dup.tx_buffer = (unsigned char *)(unsigned long)(p3transfr_32.tx_buffer);
	dup.rx_buffer = (unsigned char *)(unsigned long)(p3transfr_32.rx_buffer);
	dup.len = p3transfr_32.len;

	err = p3_xfer(p3_device, &dup);
	if (err != 0) {
		P3_ERR_MSG("%s xfer failed!\n", __func__);
		return err;
	}
	P3_DBG_MSG("%s len:%u\n", __func__, dup.len);

	return 0;
}
#endif
#endif

static int spip3_open(struct inode *inode, struct file *filp)
{
	struct p3_data *p3_dev = container_of(filp->private_data,
			struct p3_data, p3_device);
	int ret = 0;

	/* for defence MULTI-OPEN */
	if (p3_dev->device_opened) {
		P3_ERR_MSG("%s - ALREADY opened!\n", __func__);
		return -EBUSY;
	}

	mutex_lock(&device_list_lock);
	p3_dev->device_opened = true;
	P3_INFO_MSG("open\n");

#ifdef FEATURE_ESE_WAKELOCK
	wake_lock(&p3_dev->ese_lock);
#endif

#ifdef CONFIG_ESE_SECURE
	p3_dev->spi->max_speed_hz = p3_dev->speed *2;
	mt_spi_enable_master_clk(p3_dev->spi);
#else
	ese_spi_pin_control(p3_dev, ESE_POWER_ON);
#endif

#ifdef FEATURE_ESE_POWER_ON_OFF
	ret = p3_regulator_onoff(p3_dev, ESE_POWER_ON);
	if (ret < 0)
		P3_ERR_MSG(" test: failed to turn on LDO()\n");
	usleep_range(2000, 2500);
#endif

	filp->private_data = p3_dev;

	p3_dev->users++;
	mutex_unlock(&device_list_lock);

	return 0;
}

static int spip3_release(struct inode *inode, struct file *filp)
{
	struct p3_data
	*p3_dev = container_of(filp->private_data,
			struct p3_data, p3_device);

	int ret = 0;

	mutex_lock(&device_list_lock);
	p3_dev = filp->private_data;

	if (!p3_dev->device_opened) {
		P3_ERR_MSG("%s - was NOT opened....\n", __func__);
		return 0;
	}

#ifdef FEATURE_ESE_WAKELOCK
	if (wake_lock_active(&p3_dev->ese_lock))
		wake_unlock(&p3_dev->ese_lock);
#endif

	filp->private_data = p3_dev;

	p3_dev->users--;
	if (!p3_dev->users) {
		p3_dev->device_opened = false;

#ifdef FEATURE_ESE_POWER_ON_OFF
		ret = p3_regulator_onoff(p3_dev, ESE_POWER_OFF);
		if (ret < 0)
			P3_ERR_MSG(" test: failed to turn off LDO()\n");

#endif
#ifdef CONFIG_ESE_SECURE
		mt_spi_disable_master_clk(p3_dev->spi);
#else
		ese_spi_pin_control(p3_dev, ESE_POWER_OFF);
#endif
	}
	mutex_unlock(&device_list_lock);

	P3_DBG_MSG("%s, users:%d, Major Minor No:%d %d\n", __func__,
			p3_dev->users, imajor(inode), iminor(inode));
	return 0;
}

static long spip3_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	struct p3_data *data = NULL;

	if (_IOC_TYPE(cmd) != P3_MAGIC) {
		P3_ERR_MSG("%s invalid magic. cmd=0x%X Received=0x%X \
		 Expected=0x%X\n", __func__, cmd, _IOC_TYPE(cmd), P3_MAGIC);
		return -ENOTTY;
	}

	data = filp->private_data;

	mutex_lock(&data->buffer_mutex);
	switch (cmd) {
	case P3_SET_DBG:
		debug_level = (unsigned char)arg;
		P3_DBG_MSG(KERN_INFO"[NXP-P3] -  Debug level %d", debug_level);
		break;
	case P3_ENABLE_SPI_CLK:
		P3_DBG_MSG("%s P3_ENABLE_SPI_CLK\n", __func__);
#ifdef CONFIG_ESE_SECURE
		data->spi->max_speed_hz = data->speed *2;
		mt_spi_enable_master_clk(data->spi);
#endif
		break;
	case P3_DISABLE_SPI_CLK:
		P3_DBG_MSG("%s P3_DISABLE_SPI_CLK\n", __func__);
#ifdef CONFIG_ESE_SECURE
		mt_spi_disable_master_clk(data->spi);
#endif
		break;
#ifndef CONFIG_ESE_SECURE
	case P3_RW_SPI_DATA:
		ret = p3_rw_spi_message(data, arg);
		if (ret < 0)
			P3_ERR_MSG("%s P3_RW_SPI_DATA failed [%d].\n",
					__func__, ret);
		break;
#ifdef CONFIG_COMPAT
	case P3_RW_SPI_DATA_32:
		ret = p3_rw_spi_message_32(data, arg);
		if (ret < 0)
			P3_ERR_MSG("%s P3_RW_SPI_DATA_32 failed [%d].\n",
					__func__, ret);
		break;
#endif
#endif

	case P3_SET_PWR:
	case P3_SET_POLL:
	case P3_SET_SPI_CLK:
	case P3_ENABLE_SPI_CS:
	case P3_DISABLE_SPI_CS:
	case P3_ENABLE_CLK_CS:
	case P3_DISABLE_CLK_CS:
	case P3_SWING_CS:
		P3_ERR_MSG("%s deprecated IOCTL:0x%X\n", __func__, cmd);
		break;

	default:
		P3_DBG_MSG("%s no matching ioctl! 0x%X\n", __func__, cmd);
		ret = -EINVAL;
	}
	mutex_unlock(&data->buffer_mutex);

	return ret;
}

#ifndef CONFIG_ESE_SECURE
static ssize_t spip3_write(struct file *filp, const char *buf, size_t count,
		loff_t *offset)
{
	int ret = -1;
	struct p3_data *p3_dev;
	unsigned char tx_buffer[MAX_BUFFER_SIZE] = {0x0, };

	P3_INFO_MSG("spip3_write -Enter count %zu\n", count);

	p3_dev = filp->private_data;

	mutex_lock(&p3_dev->buffer_mutex);
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(&tx_buffer[0], &buf[0], count)) {
		P3_ERR_MSG("%s : failed to copy from user space\n", __func__);
		mutex_unlock(&p3_dev->buffer_mutex);
		return -EFAULT;
	}

	/* Write data */
	ret = spi_write(p3_dev->spi, &tx_buffer[0], count);
	if (ret < 0)
		ret = -EIO;
	else
		ret = count;

	mutex_unlock(&p3_dev->buffer_mutex);
	P3_DBG_MSG(KERN_ALERT "spip3_write ret %d- Exit\n", ret);

	return ret;
}

static ssize_t spip3_read(struct file *filp, char *buf, size_t count,
		loff_t *offset)
{
	int ret = -EIO;
	struct p3_data *p3_dev = filp->private_data;
	unsigned char rx_buffer[MAX_BUFFER_SIZE] = {0x0, };
	unsigned char tx_buffer[MAX_BUFFER_SIZE] = {0x0, };
	struct spi_message m;
	struct spi_transfer xfer = {
		.tx_buf = tx_buffer,
		.rx_buf = rx_buffer,
		.len = count,
	};
	P3_INFO_MSG("spip3_read count %zu - Enter\n", count);

	mutex_lock(&p3_dev->buffer_mutex);

	spi_message_init(&m);
	spi_message_add_tail(&xfer, &m);
	ret = spi_sync(p3_dev->spi, &m);
	if (ret < 0) {
		P3_ERR_MSG("spi_read failed\n");
		ret = -EIO;
		goto fail;
	}

	if (copy_to_user(buf, &rx_buffer[0], count)) {
		P3_ERR_MSG("%s : failed to copy to user space\n", __func__);
		ret = -EFAULT;
		goto fail;
	}
	ret = count;
	P3_DBG_MSG("%s ret %d Exit\n", __func__, ret);

	mutex_unlock(&p3_dev->buffer_mutex);

	return ret;

fail:
	P3_ERR_MSG("Error %s ret %d Exit\n", __func__, ret);
	mutex_unlock(&p3_dev->buffer_mutex);
	return ret;
}
#endif
/* possible fops on the p3 device */
static const struct file_operations spip3_dev_fops = {
	.owner = THIS_MODULE,
#ifndef CONFIG_ESE_SECURE
	.read = spip3_read,
	.write = spip3_write,
#endif
	.open = spip3_open,
	.release = spip3_release,
	.unlocked_ioctl = spip3_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = spip3_ioctl,
#endif
};

static int p3_parse_dt(struct device *dev, struct p3_data *data)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	if(of_get_property(np, "ese_p3,gpio_control",NULL)) {
		P3_INFO_MSG("ese_p3 gpio_control vdd-gpio\n");
		data->vdd_type = VDD_GPIO;
		data->vdd_gpio = of_get_named_gpio(np, "ese_p3,pvdd-gpio", 0);
		if (gpio_is_valid(data->vdd_gpio)) { 
			ret = gpio_request(data->vdd_gpio, "vdd-gpio");
			if (ret) {
				P3_ERR_MSG("failed to get gpio ese vdd-gpio\n");
				gpio_free(data->vdd_gpio);
			}
		}
	} else {
		data->vdd_type = VDD_REGULATOR;
		data->vdd_gpio = of_get_named_gpio(np, "ese_p3,pvdd-gpio", 0);

		if (of_property_read_string(np, "p3-vdd-1p8",
					&data->vdd_1p8) < 0) {
			pr_err("%s - getting vdd_1p8 error\n", __func__);
			data->vdd_1p8 = NULL;
		}
		else {

			pr_info("%s success vdd:%s\n", __func__, data->vdd_1p8);
		}
	}

#ifndef CONFIG_ESE_SECURE
	data->p = devm_pinctrl_get(dev);
	if (IS_ERR(data->p)) {
		ret = -EINVAL;
		pr_err("%s: failed pinctrl_get\n", __func__);
		goto dt_exit;
	}

	data->spi1_miso_set_cfg =
		pinctrl_lookup_state(data->p, "spi1_miso_set_cfg");
	if (IS_ERR(data->spi1_miso_set_cfg)) {
		pr_err("%s : could not get pins spi1_miso_set_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_miso_set_cfg));
		goto fail_pinctrl_get;
	}
	data->spi1_miso_clr_cfg =
		pinctrl_lookup_state(data->p, "spi1_miso_clr_cfg");
	if (IS_ERR(data->spi1_miso_clr_cfg)) {
		pr_err("%s : could not get pins spi1_miso_clr_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_miso_clr_cfg));
		goto fail_pinctrl_get;
	}
	data->spi1_cs_set_cfg =
		pinctrl_lookup_state(data->p, "spi1_cs_set_cfg");
	if (IS_ERR(data->spi1_cs_set_cfg)) {
		pr_err("%s : could not get pins spi1_cs_set_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_cs_set_cfg));
		goto fail_pinctrl_get;
	}
	data->spi1_cs_clr_cfg =
		pinctrl_lookup_state(data->p, "spi1_cs_clr_cfg");
	if (IS_ERR(data->spi1_cs_clr_cfg)) {
		pr_err("%s : could not get pins spi1_cs_clr_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_cs_clr_cfg));
		goto fail_pinctrl_get;
	}
	data->spi1_mosi_set_cfg =
		pinctrl_lookup_state(data->p, "spi1_mosi_set_cfg");
	if (IS_ERR(data->spi1_mosi_set_cfg)) {
		pr_err("%s : could not get pins spi1_mosi_set_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_mosi_set_cfg));
		goto fail_pinctrl_get;
	}
	data->spi1_mosi_clr_cfg =
		pinctrl_lookup_state(data->p, "spi1_mosi_clr_cfg");
	if (IS_ERR(data->spi1_mosi_clr_cfg)) {
		pr_err("%s : could not get pins spi1_mosi_clr_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_mosi_clr_cfg));
		goto fail_pinctrl_get;
	}
	data->spi1_clk_set_cfg =
		pinctrl_lookup_state(data->p, "spi1_clk_set_cfg");
	if (IS_ERR(data->spi1_clk_set_cfg)) {
		pr_err("%s : could not get pins spi1_clk_set_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_clk_set_cfg));
		goto fail_pinctrl_get;
	}
	data->spi1_clk_clr_cfg =
		pinctrl_lookup_state(data->p, "spi1_clk_clr_cfg");
	if (IS_ERR(data->spi1_clk_clr_cfg)) {
		pr_err("%s : could not get pins spi1_clk_clr_cfg (%li)\n",
			__func__, PTR_ERR(data->spi1_clk_clr_cfg));
		goto fail_pinctrl_get;
	}
	pr_info("%s is successful\n", __func__);

	return ret;
fail_pinctrl_get:
	pinctrl_put(data->p);
dt_exit:
	pr_err("%s is failed\n", __func__);
#endif
	return ret;
}

static int spip3_probe(struct spi_device *spi)
{
	int ret = -1;
	struct p3_data *data = NULL;

	P3_INFO_MSG("%s chip select : %d , bus number = %d\n",
		__func__, spi->chip_select, spi->master->bus_num);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		P3_ERR_MSG("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	ret = p3_parse_dt(&spi->dev, data);
	if (ret) {
		P3_ERR_MSG("%s - Failed to parse DT\n", __func__);
		goto p3_parse_dt_failed;
	}

	ret = p3_regulator_onoff(data, ESE_POWER_ON);
	if (ret) {
		P3_ERR_MSG("%s - Failed to enable regulator\n", __func__);
		goto p3_parse_dt_failed;
	}

#ifndef CONFIG_ESE_SECURE
	ese_spi_pin_control(data, ESE_POWER_ON);
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = SPI_DEFAULT_SPEED;

#ifdef CONFIG_MACH_MT6757
	data->spi_conf.setuptime = 50;
	data->spi_conf.holdtime = 3; 
	data->spi_conf.high_time = 10;
	data->spi_conf.low_time = 10;
	data->spi_conf.cs_idletime = 2; 
	data->spi_conf.ulthgh_thrsh = 0; 
	data->spi_conf.cpol = 0; /* SPI_0 MODE */
	data->spi_conf.cpha = 0; /* SPI_0 MODE */
	data->spi_conf.com_mod = DMA_TRANSFER; /* DMA MODE */

	data->spi_conf.rx_mlsb = 1; 
	data->spi_conf.tx_mlsb = 1; 

	data->spi_conf.tx_endian = 0; 
	data->spi_conf.rx_endian = 0; 

	data->spi_conf.pause = 1; 
	data->spi_conf.finish_intr = 1; 
	data->spi_conf.deassert = 0; 
	data->spi_conf.ulthigh = 0; 
	data->spi_conf.tckdly = 0; 

	spi->controller_data = (void *)&data->spi_conf;
#endif

	ret = spi_setup(spi);
	if (ret < 0) {
		P3_ERR_MSG("failed to do spi_setup()\n");
		goto p3_parse_dt_failed;
	}
#endif
	data->speed = SPI_DEFAULT_SPEED;
	data->spi = spi;
	data->p3_device.minor = MISC_DYNAMIC_MINOR;
	data->p3_device.name = "p3";
	data->p3_device.fops = &spip3_dev_fops;
	data->p3_device.parent = &spi->dev;

	dev_set_drvdata(&spi->dev, data);

	/* init mutex and queues */
	init_waitqueue_head(&data->read_wq);
	mutex_init(&data->buffer_mutex);
#ifdef FEATURE_ESE_WAKELOCK
	wake_lock_init(&data->ese_lock,
		WAKE_LOCK_SUSPEND, "ese_wake_lock");
#endif

	data->device_opened = false;

	ret = misc_register(&data->p3_device);
	if (ret < 0) {
		P3_ERR_MSG("misc_register failed! %d\n", ret);
		goto err_misc_regi;
	}

#ifndef CONFIG_ESE_SECURE
	ese_spi_pin_control(data, ESE_POWER_OFF);
#endif
#ifdef FEATURE_ESE_POWER_ON_OFF
	ret = p3_regulator_onoff(data, ESE_POWER_OFF);
	if (ret < 0)
	{
		P3_ERR_MSG("%s failed to turn off LDO. [%d]\n",
				__func__, ret);
		goto err_ldo_off;
	}
#endif

	P3_INFO_MSG("%s finished...\n", __func__);
	return ret;

#ifdef FEATURE_ESE_POWER_ON_OFF
	err_ldo_off:
	misc_deregister(&data->p3_device);
#endif
	err_misc_regi:
#ifdef FEATURE_ESE_WAKELOCK
	wake_lock_destroy(&data->ese_lock);
#endif
	mutex_destroy(&data->buffer_mutex);
	p3_parse_dt_failed:
	kfree(data);
	err_exit:
	P3_DBG_MSG("ERROR: Exit : %s ret %d\n", __func__, ret);
	return ret;
}

static int spip3_remove(struct spi_device *spi)
{
	struct p3_data *p3_dev = dev_get_drvdata(&spi->dev);

	P3_DBG_MSG("Entry : %s\n", __func__);
	if (p3_dev == NULL) {
		P3_ERR_MSG("%s p3_dev is null!\n", __func__);
		return 0;
	}

#ifdef FEATURE_ESE_WAKELOCK
	wake_lock_destroy(&p3_dev->ese_lock);
#endif
	mutex_destroy(&p3_dev->buffer_mutex);
	misc_deregister(&p3_dev->p3_device);

	kfree(p3_dev);
	P3_DBG_MSG("Exit : %s\n", __func__);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id p3_match_table[] = {
	{ .compatible = "ese_p3",},
	{},
};
#else
#define ese_match_table NULL
#endif

static struct spi_driver spip3_driver = {
	.driver = {
		.name = "p3",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = p3_match_table,
#endif
	},
	.probe =  spip3_probe,
	.remove = spip3_remove,
};

static int __init spip3_dev_init(void)
{
	debug_level = P3_FULL_DEBUG;

	P3_INFO_MSG("Entry : %s\n", __func__);
#if (!defined(CONFIG_ESE_FACTORY_ONLY) || defined(CONFIG_SEC_FACTORY))
	return spi_register_driver(&spip3_driver);
#else
	return -1;
#endif
}

static void __exit spip3_dev_exit(void)
{
	P3_INFO_MSG("Entry : %s\n", __func__);
	spi_unregister_driver(&spip3_driver);
}

late_initcall(spip3_dev_init);
module_exit(spip3_dev_exit);

MODULE_AUTHOR("Sec");
MODULE_DESCRIPTION("ese SPI driver");
MODULE_LICENSE("GPL");

/** @} */
