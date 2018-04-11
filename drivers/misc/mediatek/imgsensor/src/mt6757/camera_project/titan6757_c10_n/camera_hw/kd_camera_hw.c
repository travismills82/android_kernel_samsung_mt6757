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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"
/******************************************************************************
 * Debug configuration
 ******************************************************************************/
#define PFX "[kd_camera_hw]"

/* #define DEBUG_CAMERA_HW_K */
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, arg...)			pr_debug(PFX fmt, ##arg)
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...)		pr_debug(PFX fmt, ##arg)
#else
#define PK_DBG(fmt, arg...)
#define PK_ERR(fmt, arg...)			pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...)		pr_debug(PFX fmt, ##arg)
#endif


#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4



u32 pinSetIdx;		/* default main sensor */
u32 pinSet[3][8] = {
	/* for main sensor */
	{CAMERA_CMRST_PIN,
		CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		GPIO_OUT_ONE,		/* ON state */
		GPIO_OUT_ZERO,		/* OFF state */
		CAMERA_CMPDN_PIN,
		CAMERA_CMPDN_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},
	/* for sub sensor */
	{CAMERA_CMRST1_PIN,
		CAMERA_CMRST1_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
		CAMERA_CMPDN1_PIN,
		CAMERA_CMPDN1_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},
	/* for Main2 sensor */
	{CAMERA_CMRST2_PIN,
		CAMERA_CMRST2_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
		CAMERA_CMPDN2_PIN,
		CAMERA_CMPDN2_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},
};

#ifndef CONFIG_MTK_LEGACY
#define CUST_AVDD		(AVDD - AVDD)
#define CUST_DVDD		(DVDD - AVDD)
#define CUST_DOVDD		(DOVDD - AVDD)
#define CUST_AFVDD		(AFVDD - AVDD)
#define CUST_SUB_AVDD		(SUB_AVDD - AVDD)
#define CUST_SUB_DVDD		(SUB_DVDD - AVDD)
#define CUST_SUB_DOVDD		(SUB_DOVDD - AVDD)
#define CUST_MAIN2_AVDD		(MAIN2_AVDD - AVDD)
#define CUST_MAIN2_DVDD		(MAIN2_DVDD - AVDD)
#define CUST_MAIN2_DOVDD	(MAIN2_DOVDD - AVDD)

#endif

PowerCust PowerCustList = {
	{
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for AVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for DVDD; */
		{GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_High},		/* for DOVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for AFVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for SUB_AVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for SUB_DVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for SUB_DOVDD; */
		{GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_High},		/* for MAIN2_AVDD; */
		{GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_High},		/* for MAIN2_DVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for MAIN2_DOVDD; */
		/* {GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_Low}, */
	}
};

PowerUp PowerOnList = {
	{
#if defined(S5K3P8SN_MIPI_RAW)
		{SENSOR_DRVNAME_S5K3P8SN_MIPI_RAW,
			{
				{RST, Vol_Low, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1050, 0},
				{DOVDD, Vol_1800, 0},
				{AFVDD, Vol_2800, 5},				
				{SensorMCLK, Vol_High, 0},
				{RST, Vol_High, 1}
			},
		},
#endif
#if defined(S5K2L2_MIPI_RAW)
		{SENSOR_DRVNAME_S5K2L2_MIPI_RAW,
			{
				{RST, Vol_Low, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 3},
				{SensorMCLK, Vol_High, 0},
				{RST, Vol_High, 5},
			},
		},
#endif
#if defined(S5K4H9_MIPI_MONO)
		{SENSOR_DRVNAME_S5K4H9_MIPI_MONO,
			{
				{RST, Vol_Low, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 3},
				{SensorMCLK, Vol_High, 0},
				{RST, Vol_High, 5}
			},
		},
#endif
		/* add new sensor before this line */
		{NULL,},
	}
};


#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1220 1220000
#define VOL_1000 1000000

/* GPIO Pin control*/
struct platform_device *cam_plt_dev;
struct pinctrl *camctrl;
struct pinctrl_state *cam0_pnd_h;	/* main cam */
struct pinctrl_state *cam0_pnd_l;
struct pinctrl_state *cam0_rst_h;
struct pinctrl_state *cam0_rst_l;
struct pinctrl_state *cam1_pnd_h;	/* sub cam */
struct pinctrl_state *cam1_pnd_l;
struct pinctrl_state *cam1_rst_h;
struct pinctrl_state *cam1_rst_l;
struct pinctrl_state *cam2_pnd_h;	/* main2 cam */
struct pinctrl_state *cam2_pnd_l;
struct pinctrl_state *cam2_rst_h;
struct pinctrl_state *cam2_rst_l;
struct pinctrl_state *cam_ldo_vcama_h;	/* for AVDD */
struct pinctrl_state *cam_ldo_vcama_l;
struct pinctrl_state *cam_ldo_vcamd_h;	/* for DVDD */
struct pinctrl_state *cam_ldo_vcamd_l;
struct pinctrl_state *cam_ldo_vcamio_h;	/* for DOVDD */
struct pinctrl_state *cam_ldo_vcamio_l;
struct pinctrl_state *cam_ldo_vcamaf_h;	/* for AFVDD */
struct pinctrl_state *cam_ldo_vcamaf_l;
struct pinctrl_state *cam_ldo_sub_vcamd_h;	/* for SUB_DVDD */
struct pinctrl_state *cam_ldo_sub_vcamd_l;
struct pinctrl_state *cam_ldo_main2_vcamd_h;	/* for MAIN2_DVDD */
struct pinctrl_state *cam_ldo_main2_vcamd_l;
struct pinctrl_state *cam_mipi_switch_en_h;	/* for mipi switch enable */
struct pinctrl_state *cam_mipi_switch_en_l;
struct pinctrl_state *cam_mipi_switch_sel_h;	/* for mipi switch select */
struct pinctrl_state *cam_mipi_switch_sel_l;
int has_mipi_switch;

/*GPIO expander pin control*/
static int mtkcam_cam_en;
static int mtkcam_cam_sel;
static int mtkcam_main_vddret;
static int mtkcam_main_rst;
static int mtkcam_main_vddio;
static int mtkcam_sub_rst;
static int mtkcam_main2_rst;
static int mtkcam_main2_vddad;
static int mtkcam_main2_vddaf;


/* for set expander GPIO */
static int gpioexp_set(int gpio_num, int value)
{
	int ret = 0;
	ret = gpio_request(gpio_num, NULL);
	if (ret < 0) {
				PK_ERR("%s : gpioexp_set err,GPIO_num %d, Val %d.\n", __func__,gpio_num, value);
				return ret;
			}
	PK_ERR("%s : gpioexp_set OK, gpio_num=%d.\n", __func__,gpio_num);
	ret = gpio_direction_output(gpio_num, value);
	gpio_set_value(gpio_num, value);
	gpio_free(gpio_num);

	return ret;
}

int mtkcam_gpio_init(struct platform_device *pdev)
{
	PK_ERR("%s : init gpio_expander\n", __func__);
	has_mipi_switch = 1;
	mtkcam_cam_en = of_get_named_gpio(pdev->dev.of_node, "mtkcam_cam_en", 0);
	mtkcam_cam_sel = of_get_named_gpio(pdev->dev.of_node, "mtkcam_cam_sel", 0);
	mtkcam_main_vddret = of_get_named_gpio(pdev->dev.of_node, "mtkcam_main_vddret", 0);
	mtkcam_main_vddio = of_get_named_gpio(pdev->dev.of_node, "mtkcam_main_vddio", 0);
	mtkcam_main_rst = of_get_named_gpio(pdev->dev.of_node, "mtkcam_main_rst", 0);
	mtkcam_sub_rst = of_get_named_gpio(pdev->dev.of_node, "mtkcam_sub_rst", 0);
	mtkcam_main2_rst = of_get_named_gpio(pdev->dev.of_node, "mtkcam_main2_rst", 0);
	mtkcam_main2_vddad = of_get_named_gpio(pdev->dev.of_node, "mtkcam_main2_vddad", 0);
	mtkcam_main2_vddaf = of_get_named_gpio(pdev->dev.of_node, "mtkcam_main2_vddaf", 0);
	return 0;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	static signed int m2DVDD_usercounter = 0;

	switch (PwrType) {
	case RST:
		if (PinIdx == 0) {
			if(Val ==0 || Val ==1)
				gpioexp_set(mtkcam_main_rst, Val);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx, Val);
		} else if (PinIdx == 1) {
			if(Val ==0 || Val ==1)
				gpioexp_set(mtkcam_sub_rst, Val);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx, Val);
		} else {
			if(Val ==0 || Val ==1)
				gpioexp_set(mtkcam_main2_rst, Val);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx, Val);
		}
		break;
	case MAIN2_AVDD:
	case MAIN2_DVDD:
		if (PinIdx == 2) {
			if (Val == 1) {
				gpioexp_set(mtkcam_main2_vddad, Val);
				m2DVDD_usercounter++;
			}
			else if (Val == 0) {
				m2DVDD_usercounter--;
				if (m2DVDD_usercounter <= 0)
					m2DVDD_usercounter = 0;
				gpioexp_set(mtkcam_main2_vddad, Val);
			}
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, ADVDD\n", __func__,PinIdx, Val);
		}
		break;
	case DOVDD:
		if (PinIdx == 0) {
			if(Val ==0 || Val ==1)
				gpioexp_set(mtkcam_main_vddio, Val);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, IOVDD\n", __func__,PinIdx, Val);
		}
		break;
	default:
		PK_ERR("%s: PwrType(%d) is invalid !!\n", __func__, PwrType);
		break;
	}

	return ret;
}

BOOL hwpoweron(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD) {
		if (pinSetIdx == 2) {
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Gpio_Pin ==
			    GPIO_UNSUPPORTED) {
				if (_hwPowerOn(MAIN2_AVDD, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable analog power2\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_AVDD,
						    PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Voltage)) {
					PK_INFO("[CAMERA CUST_MAIN2_AVDD] set gpio failed!!\n");
				}
			}
		} else if (pinSetIdx == 1) {
			if (PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerOn(SUB_AVDD, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable analog power1\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, SUB_AVDD,
						    PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_SUB_AVDD] set gpio failed!!\n");
				}
			}
		} else {
			if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerOn(pwInfo.PowerType, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable analog power0\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, AVDD,
						    PowerCustList.PowerCustInfo[CUST_AVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
	} else if (pwInfo.PowerType == DVDD) {
		if (pinSetIdx == 2) {
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Gpio_Pin ==
			    GPIO_UNSUPPORTED) {
				PK_DBG("[CAMERA SENSOR] MAIN2 camera VCAM_D power on");
				/*vcamd: unsupportable voltage range: 1500000-1210000uV */
				if (pwInfo.Voltage == Vol_1200) {
					pwInfo.Voltage = Vol_1220;
					/* PK_INFO("[CAMERA SENSOR] Main2 camera VCAM_D power 1.2V to 1.21V\n"); */
				}
				if (_hwPowerOn(MAIN2_DVDD, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power2\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_DVDD,
						    PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_MAIN2_DVDD] set gpio failed!!\n");
				}
			}
		} else if (pinSetIdx == 1) {
			if (PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power on");
				/*if (pwInfo.Voltage == Vol_1200) {
				 * pwInfo.Voltage = Vol_1220;
				 * PK_DBG("[CAMERA SENSOR] Sub camera VCAM_D power 1.2V to 1.21V\n");
				 * }
				 */
				if (_hwPowerOn(SUB_DVDD, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power1\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, SUB_DVDD,
						    PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
				}
			}
		} else {
			/*s5k2l2 needs DVDDRET power */
			gpioexp_set(mtkcam_main_vddret, 1);

			if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				PK_DBG("[CAMERA SENSOR] Main camera VCAM_D power on");
				/* if (pwInfo.Voltage == Vol_1200) { */
				/* pwInfo.Voltage = Vol_1220; */
				/* PK_INFO("[CAMERA SENSOR] Sub camera VCAM_D power 1.2V to 1.21V\n"); */
				/* } */
				if (_hwPowerOn(DVDD, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power0\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, DVDD,
						    PowerCustList.PowerCustInfo[CUST_DVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
				}
			}
		}
	} else if (pwInfo.PowerType == DOVDD) {
		if (pinSetIdx == 0) {
			/*C10 will use ldo DOVDD*/
			if (PowerCustList.PowerCustInfo[CUST_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerOn(DOVDD, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable analog power2\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, DOVDD,
						    PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage)) {
					PK_INFO("[CAMERA CUST_MAIN2_AVDD] set gpio failed!!\n");
				}
			}
		} else { /*sub/main2 use same DOVDD*/
			if (PowerCustList.PowerCustInfo[CUST_SUB_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerOn(pwInfo.PowerType, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable io power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, SUB_DOVDD, PowerCustList.PowerCustInfo[CUST_SUB_DOVDD].Voltage))
					PK_ERR("[CAMERA CUST_DOVDD] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
		/* PK_INFO("[CAMERA SENSOR] Skip AFVDD setting\n"); */
		if (pinSetIdx != 2) {
			if (PowerCustList.PowerCustInfo[CUST_AFVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerOn(pwInfo.PowerType, pwInfo.Voltage) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable af power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, AFVDD, PowerCustList.PowerCustInfo[CUST_AFVDD].Voltage))
					PK_ERR("[CAMERA CUST_AFVDD] set gpio failed!!\n");
			}
		} else { /*	MAIN2_AFVDD, use ldo	*/
			gpioexp_set(mtkcam_main2_vddaf, Vol_High);
		}
	} else if (pwInfo.PowerType == PDN) {
		/* PK_INFO("hwPowerOn: PDN %d\n", pwInfo.Voltage); */
		/*mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]); */
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		} else {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		}
	} else if (pwInfo.PowerType == RST) {
		/*mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]); */
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		} else {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		}

	} else if (pwInfo.PowerType == SensorMCLK) {
		if (pinSetIdx == 0) {
			/* PK_INFO("Sensor MCLK1 On"); */
			ISP_MCLK1_EN(TRUE);
		} else if (pinSetIdx == 1) {
			/* PK_INFO("Sensor MCLK2 On"); */
			ISP_MCLK2_EN(TRUE);
		} else if (pinSetIdx == 2) {
			/* PK_INFO("Sensor MCLK2 On"); */
			ISP_MCLK1_EN(TRUE);
		} else {
			/* PK_INFO("Sensor MCLK3 On"); */
			ISP_MCLK1_EN(TRUE);
		}
	} else {
	}
	if (pwInfo.Delay > 0)
		mdelay(pwInfo.Delay);
	return TRUE;
}



BOOL hwpowerdown(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD) {
		if (pinSetIdx == 2) {
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerDown(MAIN2_AVDD) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to diable analog power2\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_AVDD,
						    1 - PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		} else if (pinSetIdx == 1) {
			if (PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerDown(SUB_AVDD) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to diable analog power1\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, SUB_AVDD,
						    1 - PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		} else {
			if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerDown(AVDD) != TRUE) {
					PK_ERR("[CAMERA SENSOR]Fail to diable analog power0\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, AVDD,
						    1 - PowerCustList.PowerCustInfo[CUST_AVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}


	} else if (pwInfo.PowerType == DVDD) {
		if (pinSetIdx == 2) {
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Gpio_Pin ==
			    GPIO_UNSUPPORTED) {
				if (_hwPowerDown(MAIN2_DVDD) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to disable digital power2\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_DVDD,
						    1 - PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_MAIN2_DVDD] set gpio failed!!\n");
				}
			}
		} else if (pinSetIdx == 1) {
			if (PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerDown(SUB_DVDD) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to disable digital power1\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, SUB_DVDD,
						    1 - PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
				}
			}
		} else {
			if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerDown(DVDD) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to disable digital power0\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, DVDD,
						    1 - PowerCustList.PowerCustInfo[CUST_DVDD].Voltage)) {
					PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
				}
			}
			
			/*s5k2l2 needs DVDDRET power */
			gpioexp_set(mtkcam_main_vddret, 0);
		}
	} else if (pwInfo.PowerType == DOVDD) {
		if (pinSetIdx == 0) {
			/*C10 will use ldo DOVDD*/
			if (PowerCustList.PowerCustInfo[CUST_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerDown(DOVDD) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable analog power2\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, DOVDD,
						    1-PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage)) {
					PK_INFO("[CAMERA CUST_DOVDD] set gpio failed!!\n");
				}
			}
		} else { /*sub/main2 use same DOVDD*/
			if (PowerCustList.PowerCustInfo[CUST_SUB_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				if (_hwPowerDown(pwInfo.PowerType) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable io power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, SUB_DOVDD, 1-PowerCustList.PowerCustInfo[CUST_SUB_DOVDD].Voltage))
					PK_ERR("[CAMERA CUST_SUB_DOVDD] set gpio failed!!\n");
			}
		}
	} else if (pwInfo.PowerType == AFVDD) {
		if (pinSetIdx != 2) {
			if (PowerCustList.PowerCustInfo[CUST_AFVDD].Gpio_Pin == GPIO_UNSUPPORTED) {
				printk("[DHL] ****************** AFVDD Power OFF *****************\n");
				if (_hwPowerDown(pwInfo.PowerType) != TRUE) {
					PK_ERR("[CAMERA SENSOR] Fail to enable af power\n");
					return FALSE;
				}
			} else {
				if (mtkcam_gpio_set(pinSetIdx, AFVDD, 1 - PowerCustList.PowerCustInfo[CUST_AFVDD].Voltage))
					PK_ERR("[CAMERA CUST_AFVDD] set gpio failed!!\n");
			}
		} else { /*	MAIN2_AFVDD, use ldo	*/
			gpioexp_set(mtkcam_main2_vddaf, Vol_Low);
		}
	} else if (pwInfo.PowerType == PDN) {
		/*PK_INFO("hwPowerDown: PDN %d\n", pwInfo.Voltage); */
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		} else {
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		}
	} else if (pwInfo.PowerType == RST) {
		/*PK_INFO("hwPowerDown: RST %d\n", pwInfo.Voltage); */
		if (pwInfo.Voltage == Vol_High) {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		} else {
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			if (pwInfo.Delay > 0)
				mdelay(pwInfo.Delay);
		}

	} else if (pwInfo.PowerType == SensorMCLK) {
	if (mtkcam_gpio_set(pinSetIdx, SensorMCLK, 0))
			PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
		if (pinSetIdx == 0)
			ISP_MCLK1_EN(FALSE);
		else if (pinSetIdx == 1)
			ISP_MCLK2_EN(FALSE);
		else if (pinSetIdx == 2)
			ISP_MCLK1_EN(FALSE);
		else
			ISP_MCLK1_EN(FALSE);
	}
	return TRUE;
}




int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On,
		       char *mode_name)
{

	int pwListIdx, pwIdx;
	BOOL sensorInPowerList = KAL_FALSE;

	if (SensorIdx == DUAL_CAMERA_MAIN_SENSOR)
		pinSetIdx = 0;
	else if (SensorIdx == DUAL_CAMERA_SUB_SENSOR)
		pinSetIdx = 1;
	else if (SensorIdx == DUAL_CAMERA_MAIN_2_SENSOR)
		pinSetIdx = 2;

	if (currSensorName && (strcmp(currSensorName, "ov5670mipiraw") == 0)) {
		if (pinSetIdx == 1)
			goto _kdCISModulePowerOn_exit_;
	}

	if (currSensorName && (strcmp(currSensorName, "imx258mipiraw") == 0)) {
		if (pinSetIdx == 1)
			goto _kdCISModulePowerOn_exit_;
	}
	if (currSensorName && (strcmp(currSensorName, "imx258mipimono") == 0)) {
		if (pinSetIdx == 1)
			goto _kdCISModulePowerOn_exit_;
	}
	if (currSensorName && (strcmp(currSensorName, "imx258mipiraw") == 0)) {
		if (pinSetIdx == 2)
			goto _kdCISModulePowerOn_exit_;
	}

	/* power ON */
	if (On) {
		printk("[DHL]PowerOn:SensorName=%s, pinSetIdx=%d, sensorIdx:%d\n", currSensorName,
			pinSetIdx, SensorIdx);
		PK_INFO("PowerOn:SensorName=%s, pinSetIdx=%d, sensorIdx:%d\n", currSensorName,
			pinSetIdx, SensorIdx);
		/* MIPI SWITCH */
		if (has_mipi_switch) {
			if (SensorIdx == DUAL_CAMERA_SUB_SENSOR) {
				gpioexp_set(mtkcam_cam_en, 1);/*enable mipi switch*/
				gpioexp_set(mtkcam_cam_sel, 0);
				printk("[DHL]mipi switch for VTCAM\n");
			} else if (SensorIdx == DUAL_CAMERA_MAIN_2_SENSOR) {
				gpioexp_set(mtkcam_cam_en, 1);/*enable mipi switch*/
				gpioexp_set(mtkcam_cam_sel, 1);
				printk("[DHL]mipi switch for MONOCAM\n");
			}
		}


		for (pwListIdx = 0; pwListIdx < 16; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName, currSensorName) == 0)) {
				/* PK_INFO("sensorIdx:%d\n", SensorIdx); */

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 0; pwIdx < 10; pwIdx++) {
					PK_DBG("PowerType:%d\n",
					       PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);

					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == VDD_None) {
						PK_DBG("pwIdx=%d\n", pwIdx);
						break;
					} else if (hwpoweron(PowerOnList.PowerSeq[pwListIdx].
							     PowerInfo[pwIdx], mode_name) == FALSE) {
						goto _kdCISModulePowerOn_exit_;
					}
				}
				break;
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			}
		}
	} else {
		/* power OFF */
		PK_INFO("PowerOFF:pinSetIdx=%d, sensorIdx:%d\n", pinSetIdx, SensorIdx);
		if (has_mipi_switch) {
			if ((SensorIdx == DUAL_CAMERA_SUB_SENSOR)
			    || (SensorIdx == DUAL_CAMERA_MAIN_2_SENSOR)) {
				gpioexp_set(mtkcam_cam_en, 0);/*disable mipi switch*/
			}
		}

		for (pwListIdx = 0; pwListIdx < 16; pwListIdx++) {
			if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
			    && (strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName, currSensorName) == 0)) {
				/*PK_INFO("kdCISModulePowerOn get in---\n"); */
				/* PK_INFO("sensorIdx:%d\n", SensorIdx); */

				sensorInPowerList = KAL_TRUE;

				for (pwIdx = 9; pwIdx >= 0; pwIdx--) {
					PK_DBG("PowerType:%d\n",
					       PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);

					if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType == VDD_None) {
						PK_DBG("pwIdx=%d\n", pwIdx);
					} else if (hwpowerdown(PowerOnList.PowerSeq[pwListIdx].
							       PowerInfo[pwIdx], mode_name) == FALSE) {
						PK_ERR("PowerOFF:pinSetIdx=%d, sensorIdx:%d, PowerType:%d fail!\n", pinSetIdx, SensorIdx,
						PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
						if(pwIdx > 0)
						{
							PK_ERR("PowerOFF: need to turn off another power current index(%d)\n", pwIdx);
						}else
						{
							goto _kdCISModulePowerOn_exit_;
						}
					} else if ((pwIdx > 0) &&
						   (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx - 1].Delay > 0)) {
						mdelay(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx - 1].Delay);
					}
				}
			} else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL) {
				break;
			}
		}
	}			/*  */

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}
EXPORT_SYMBOL(kdCISModulePowerOn);

static int __init kd_camera_hw_init(void)
{
	return 0;
}
arch_initcall(kd_camera_hw_init);

/* !-- */
/*  */
