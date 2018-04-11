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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	   s5k2l2mipi_Sensor.c
 *
 * Project:
 * --------
 *	   ALPS
 *
 * Description:
 * ------------
 *	   Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k2l2mipiraw_Sensor.h"


/* #include "s5k2l2_otp.h" */
/*Enable PDAF function */
#define ENABLE_S5K2L2_PDAF_RAW

#define SENSOR_M1 /* open this define for m1, close this define for m2 */
#ifdef ENABLE_S5K2L2_PDAF_RAW
    #define MARK_HDR
#endif
/* #define TEST_PATTERN_EN */
/*WDR auto ration mode*/
/* #define ENABLE_WDR_AUTO_RATION */
/****************************Modify Following Strings for Debug****************************/
#define PFX "s5k2l2_camera_sensor"
#define LOG_1 LOG_INF("s5k2l2,MIPI 4LANE\n")
/****************************	Modify end	  *******************************************/
#define LOG_INF(fmt, args...)	pr_debug(PFX "[%s] " fmt, __func__, ##args)
static kal_uint32 chip_id;

static DEFINE_SPINLOCK(imgsensor_drv_lock);
#define ORIGINAL_VERSION 1

#define USE_MODE3 (1)

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K2L2_SENSOR_ID,		  /* record sensor id defined in Kd_imgsensor.h */
	.checksum_value = 0xafb5098f,	   /* checksum value for Camera Auto Test */

#if (USE_MODE3 == 1)
	/* using mode3 fullsize */
	.pre = {
		.pclk = 960000000,
		.linelength = 10160,
		.framelength = 3149,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 960000000,
		.linelength = 10160,
		.framelength = 3149,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 960000000,
		.linelength = 10160,
		.framelength = 3149,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
#else
	/* using mode1 fullsize */
	.pre = {
		.pclk = 960000000,				/* record different mode's pclk */
		.linelength = 10208,				/* record different mode's linelength */
		.framelength = 3134,
		.startx = 0,					/* record different mode's startx of grabwindow */
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width = 4032,		/* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */
		.grabwindow_height = 3024,		/* record different mode's height of grabwindow */
		.mipi_data_lp2hs_settle_dc = 85,  /* unit , ns */
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 960000000,
		.linelength = 10208,
		.framelength = 3134,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 960000000,
		.linelength = 10208,
		.framelength = 3134,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
#endif

#if 0
	.normal_video = {
		.pclk = 960000000,				/* record different mode's pclk */
		.linelength = 6496,				/* record different mode's linelength */
		.framelength = 4926,
		.startx = 0,					/* record different mode's startx of grabwindow */
		.starty = 0,					/* record different mode's starty of grabwindow */
		.grabwindow_width = 4032,		/* Dual PD: need to tg grab width / 2, p1 drv will * 2 itself */
		.grabwindow_height = 3024,		/* record different mode's height of grabwindow */
		.mipi_data_lp2hs_settle_dc = 85,  /* unit , ns */
		.max_framerate = 300,
	},
#endif
	.hs_video = {
		.pclk = 960000000,
		.linelength = 7776,
		.framelength = 1028,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1344,
		.grabwindow_height = 756,
		.mipi_data_lp2hs_settle_dc = 85,/* unit , ns */
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 960000000,
		.linelength = 20224,
		.framelength = 1579,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2016,
		.grabwindow_height = 1512,
		.mipi_data_lp2hs_settle_dc = 85,/* unit , ns */
		.max_framerate = 300,
	},
#if (USE_MODE3 == 1)
	.custom1 = {/* use capture setting mode3 */
		.pclk = 960000000,
		.linelength = 10160,
		.framelength = 3928, /*24fps*/
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
	},
#else
	.custom1 = {/* use capture setting mode1*/
		.pclk = 960000000,
		.linelength = 10208,
		.framelength = 3912, /*24fps*/
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4032,
		.grabwindow_height = 3024,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 240,
	},
#endif
	.custom2 = {/*mode2 binning 24fps*/
		.pclk = 960000000,
		.linelength = 20224,
		.framelength = 1974,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2016,
		.grabwindow_height = 1512,
		.mipi_data_lp2hs_settle_dc = 85,/* unit , ns */
		.max_framerate = 240,
	},
	.margin = 4,
	.min_shutter = 4,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  /* 1, support; 0,not support */
	.ihdr_le_firstline = 0,  /* 1,le first ; 0, se first */
	.sensor_mode_num = 7,	  /* support sensor mode num */
	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	.custom1_delay_frame = 2,	/* add new mode */
	.custom2_delay_frame = 2,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 1,/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = { 0x5A, 0x20, 0xFF},
	.i2c_speed = 300,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,					/* current shutter */
	.gain = 0x100,						/* current gain */
	.dummy_pixel = 0,					/* current dummypixel */
	.dummy_line = 0,					/* current dummyline */
	.current_fps = 0,  /* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/* current scenario id */
	.hdr_mode = KAL_FALSE, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x5A,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] = {
	{4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* Preview */
	{4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* capture */
	{4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* normal_video */
	{4032, 3024, 0, 378, 4032, 2268, 1344,  756, 0, 0, 1344,  756, 0, 0, 1344,  756}, /* hs_video */
	{4032, 3024, 0,   0, 4032, 3024, 2016, 1512, 0, 0, 2016, 1512, 0, 0, 2016, 1512}, /* slim_video */
	{4032, 3024, 0,   0, 4032, 3024, 4032, 3024, 0, 0, 4032, 3024, 0, 0, 4032, 3024}, /* custom1*/
	{4032, 3024, 0,   0, 4032, 3024, 2016, 1512, 0, 0, 2016, 1512, 0, 0, 2016, 1512}, /* custom2 */
};

/* #define USE_OIS */
#ifdef USE_OIS
#define OIS_I2C_WRITE_ID 0x48
#define OIS_I2C_READ_ID 0x49

#define RUMBA_OIS_CTRL	 0x0000
#define RUMBA_OIS_STATUS 0x0001
#define RUMBA_OIS_MODE	 0x0002
#define CENTERING_MODE	 0x05
#define RUMBA_OIS_OFF	 0x0030

#define RUMBA_OIS_SETTING_ADD 0x0002
#define RUMBA_OIS_PRE_SETTING 0x02
#define RUMBA_OIS_CAP_SETTING 0x01


#define RUMBA_OIS_PRE	 0
#define RUMBA_OIS_CAP	 1


static void OIS_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, OIS_I2C_WRITE_ID);
}

static kal_uint16 OIS_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, OIS_I2C_READ_ID);
	return get_byte;
}
static int OIS_on(int mode)
{
	int ret = 0;

	if (mode == RUMBA_OIS_PRE_SETTING)
		OIS_write_cmos_sensor(RUMBA_OIS_SETTING_ADD, RUMBA_OIS_PRE_SETTING);

	if (mode == RUMBA_OIS_CAP_SETTING)
		OIS_write_cmos_sensor(RUMBA_OIS_SETTING_ADD, RUMBA_OIS_CAP_SETTING);

	OIS_write_cmos_sensor(RUMBA_OIS_MODE, CENTERING_MODE);
	ret = OIS_read_cmos_sensor(RUMBA_OIS_MODE);
	LOG_INF("pangfei OIS ret=%d %s %d\n", ret, __func__, __LINE__);

	if (ret != CENTERING_MODE)
		/* return -1; */

	OIS_write_cmos_sensor(RUMBA_OIS_CTRL, 0x01);
	ret = OIS_read_cmos_sensor(RUMBA_OIS_CTRL);
	LOG_INF("pangfei OIS ret=%d %s %d\n", ret, __func__, __LINE__);
	if (ret != 0x01)
		/* return -1; */

	return ret;
}

static int OIS_off(void)
{
	int ret = 0;

	OIS_write_cmos_sensor(RUMBA_OIS_OFF, 0x01);
	ret = OIS_read_cmos_sensor(RUMBA_OIS_OFF);
	LOG_INF("pangfei OIS ret=%d %s %d\n", ret, __func__, __LINE__);
}
#endif
/* add for s5k2l2 pdaf */
static SET_PD_BLOCK_INFO_T imgsensor_pd_info = {

	.i4OffsetX = 0,
	.i4OffsetY = 0,
	.i4PitchX  = 0,
	.i4PitchY  = 0,
	.i4PairNum = 0,
	.i4SubBlkW = 0,
	.i4SubBlkH = 0,
	.i4PosL = {{0, 0} },
	.i4PosR = {{0, 0} },
	.i4BlockNumX = 0,
	.i4BlockNumY = 0,
	.i4LeFirst = 0,
	.i4Crop = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
	.iMirrorFlip = 3, /*0:IMAGE_NORMAL,1:IMAGE_H_MIRROR,2:IMAGE_V_MIRROR,3:IMAGE_HV_MIRROR*/
};

static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[7] = {
	/* Preview mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x31, 0x13b0, 0x02f4, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Capture mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x31, 0x13b0, 0x02f4, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Video mode setting */
	{
		0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1070, 0x0C30, 0x01, 0x00, 0x0000, 0x0000,
		0x01, 0x31, 0x13b0, 0x02f4, 0x03, 0x00, 0x0000, 0x0000
	},
	/* HS video mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x31, 0x0690, 0x00BD, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Slim video mode setting */
	/* not used, cause of mode2*/
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x0834, 0x0618, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x31, 0x09D8, 0x017a, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Custom1 mode setting */
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x1070, 0x0C30, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x31, 0x13b0, 0x02f4, 0x03, 0x00, 0x0000, 0x0000
	},
	/* Custom2 mode setting */
	/* not used, cause of mode2*/
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2b, 0x0834, 0x0618, 0x00, 0x00, 0x0280, 0x0001,
		0x01, 0x31, 0x09D8, 0x017a, 0x03, 0x00, 0x0000, 0x0000
	}
};

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	/* iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id); */
	iReadRegI2CTiming(pu_send_cmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);

	return get_byte;
}
static kal_uint16 read_cmos_sensor_twobyte(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char get_word[2];
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	/* iReadRegI2C(pu_send_cmd, 2, get_word, 2, imgsensor.i2c_write_id); */
	iReadRegI2CTiming(pu_send_cmd, 2, get_word, 2, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
	get_byte = (((int)get_word[0])<<8) | get_word[1];
	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	/* iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id); */
	iWriteRegI2CTiming(pu_send_cmd, 3, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
}

static void write_cmos_sensor_twobyte(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};
	/* LOG_INF("write_cmos_sensor_twobyte is %x,%x,%x,%x\n", pu_send_cmd[0], pu_send_cmd[1],
	*pu_send_cmd[2], pu_send_cmd[3]);
	*/
	/* iWriteRegI2C(pu_send_cmd, 4, imgsensor.i2c_write_id); */
	iWriteRegI2CTiming(pu_send_cmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
}

static void set_dummy(void)
{
#if 1
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* write_cmos_sensor(0x0104, 0x01); */
	/* write_cmos_sensor_twobyte(0x6028,0x4000); */
	/* write_cmos_sensor_twobyte(0x602A,0xC340 ); */
	write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);

	/* write_cmos_sensor_twobyte(0x602A,0xC342 ); */
	write_cmos_sensor_twobyte(0x0342, imgsensor.line_length);

	/* write_cmos_sensor(0x0104, 0x00); */
#endif
#if 0
	LOG_INF("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	write_cmos_sensor(0x0104, 0x01);
	write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);
	write_cmos_sensor_twobyte(0x0342, imgsensor.line_length);
	write_cmos_sensor(0x0104, 0x00);
#endif
}	 /*    set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/* kal_int16 dummy_line; */
	kal_uint32 frame_length = imgsensor.frame_length;
	/* unsigned long flags; */

	LOG_INF("framerate = %d, min framelength should enable = %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length)
				? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	 /*    set_max_framerate  */


/*************************************************************************
* FUNCTION
*	 set_shutter
*
* DESCRIPTION
*	 This function set e-shutter of sensor to change exposure time.
*	 The registers 0x3500 ,0x3501 and 0x3502 control exposure of s5k2l2.
*	 The exposure value is in number of Tline, where Tline is the time of sensor one line.
*
*	 Exposure = [reg 0x3500]<<12 + [reg 0x3501]<<4 + [reg 0x3502]>>4;
*	 The maximum exposure value is limited by VTS defined by register 0x380e and 0x380f.
	  Maximum Exposure <= VTS -4
*
* PARAMETERS
*	 iShutter : exposured lines
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	/* LOG_INF("enter xxxx  set_shutter, shutter =%d\n", shutter); */

	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	/* kal_uint32 frame_length = 0; */
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("set_shutter =%d\n", shutter);
	/* OV Recommend Solution */
	/* if shutter bigger than frame_length, should extend frame length first */
	if (!shutter)
		shutter = 1; /*avoid 0*/
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
					? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;


	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);
	} else {
	/* Frame length :4000 C340 */
		/* write_cmos_sensor_twobyte(0x6028,0x4000); */
		/* write_cmos_sensor_twobyte(0x602A,0xC340 ); */
		write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);
	}

/* Shutter reg : 4000 C202 */
	/* write_cmos_sensor_twobyte(0x6028,0x4000); */
	/* write_cmos_sensor_twobyte(0x602A,0xC202 ); */
	write_cmos_sensor_twobyte(0x0202, shutter);
	write_cmos_sensor_twobyte(0x0226, shutter);

}	 /*    set_shutter */

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/*Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	/*  */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/* Update Shutter */
	write_cmos_sensor_twobyte(0x0202, shutter);
	write_cmos_sensor_twobyte(0x0226, shutter);
	LOG_INF("Exit! shutter =%d, framelength =%d/%d, dummy_line=%d\n", shutter,
		imgsensor.frame_length, frame_length, dummy_line);

}				/* set_shutter_frame_length */


static void hdr_write_shutter(kal_uint16 le, kal_uint16 se)
{
	/* LOG_INF("enter xxxx  set_shutter, shutter =%d\n", shutter); */
	unsigned int iRation;

	unsigned long flags;
	/* kal_uint16 realtime_fps = 0; */
	/* kal_uint32 frame_length = 0; */
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = le;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	if (!le)
		le = 1; /*avoid 0*/

	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);

	le = (le < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : le;
	le = (le > (imgsensor_info.max_frame_length - imgsensor_info.margin))
				? (imgsensor_info.max_frame_length - imgsensor_info.margin) : le;

	/* Frame length :4000 C340 */
	/* write_cmos_sensor_twobyte(0x6028,0x4000); */
	/* write_cmos_sensor_twobyte(0x602A,0xC340 ); */
	write_cmos_sensor_twobyte(0x0340, imgsensor.frame_length);

	/* SET LE/SE ration */
	/* iRation = (((LE + SE/2)/SE) >> 1 ) << 1 ; */
	iRation = ((10 * le / se) + 5) / 10;
	if (iRation < 2)
		iRation = 1;
	else if (iRation < 4)
		iRation = 2;
	else if (iRation < 8)
		iRation = 4;
	else if (iRation < 16)
		iRation = 8;
	else if (iRation < 32)
		iRation = 16;
	else
		iRation = 1;

	/*set ration for auto */
	iRation = 0x100 * iRation;
#if defined(ENABLE_WDR_AUTO_RATION)
	/*LE / SE ration ,	0x218/0x21a =  LE Ration*/
	/*0x218 =0x400, 0x21a=0x100, LE/SE = 4x*/
	write_cmos_sensor_twobyte(0x0220, iRation);
	write_cmos_sensor_twobyte(0x0222, 0x100);
#endif

	/*Short exposure */
	write_cmos_sensor_twobyte(0x0202, se);
	/*Log exposure ratio*/
	write_cmos_sensor_twobyte(0x0226, le);

	LOG_INF("HDR set shutter LE=%d, SE=%d, iRation=0x%x\n", le, se, iRation);

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0;

	reg_gain = gain/2;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
 * FUNCTION
 *    set_dual_gain
 *
 * DESCRIPTION
 *    This function is to set dual gain to sensor.
 *
 * PARAMETERS
 *    iGain : sensor dual gain(base: 0x40)
 *
 * RETURNS
 *    the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_dual_gain(kal_uint16 gain1, kal_uint16 gain2)
{
	kal_uint16 reg_gain1, reg_gain2;

	if (gain1 < BASEGAIN || gain1 > 16 * BASEGAIN) {
		LOG_INF("Error gain1 setting");

		if (gain1 < BASEGAIN)
			gain1 = BASEGAIN;
		else if (gain1 > 16 * BASEGAIN)
			gain1 = 16 * BASEGAIN;
	}

	if (gain2 < BASEGAIN || gain2 > 16 * BASEGAIN) {
		LOG_INF("Error gain2 setting");

		if (gain2 < BASEGAIN)
			gain2 = BASEGAIN;
		else if (gain2 > 16 * BASEGAIN)
			gain2 = 16 * BASEGAIN;
	}

	reg_gain1 = gain2reg(gain1);
	reg_gain2 = gain2reg(gain2);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain1;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain1 = %d , reg_gain1 = 0x%x, gain2 = %d , reg_gain2 = 0x%x\n ", gain1, reg_gain1,
		gain2, reg_gain2);

	/* Global Analog Gain for Long expo */
	/*write_cmos_sensor_twobyte(0x0206, reg_gain1 & 0xFFFF);*/
	/* Global Analog Gain for Short expo */
	/*write_cmos_sensor_twobyte(0x0204, reg_gain2 & 0xFFFF);*/

	/*change gain to digital gain BASE(256) */
	reg_gain1 = reg_gain1<<3;
	reg_gain2 = reg_gain2<<3;
	write_cmos_sensor(0x306A, 0x00);
	/*Global Digital Gain for Long expo*/
	write_cmos_sensor_twobyte(0x3072, reg_gain1 & 0xFFFF);
	/*Global Digital Gain for Short expo*/
	write_cmos_sensor_twobyte(0x020E, reg_gain2 & 0xFFFF);


	return gain1;
}				/*    set_dual_gain  */


/*************************************************************************
* FUNCTION
*	 set_gain
*
* DESCRIPTION
*	 This function is to set global gain to sensor.
*
* PARAMETERS
*	 iGain : sensor global gain(base: 0x80)
*
* RETURNS
*	 the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{

	  kal_uint16 reg_gain;
	  kal_uint32 sensor_gain1 = 0;
	  kal_uint32 sensor_gain2 = 0;
	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X	*/
	/* [4:9] = M meams M X		 */
	/* Total gain = M + N /16 X   */

	if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 32 * BASEGAIN)
			gain = 32 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	/* LOG_INF("gain = %d , reg_gain = 0x%x ", gain, reg_gain); */

	/* Analog gain HW reg : 4000 C204 */

	write_cmos_sensor_twobyte(0x6028, 0x4000);
	write_cmos_sensor_twobyte(0x602A, 0x0204);
	write_cmos_sensor_twobyte(0x6F12, reg_gain);
	write_cmos_sensor_twobyte(0x6F12, reg_gain);


	write_cmos_sensor_twobyte(0x602C, 0x4000);
	write_cmos_sensor_twobyte(0x602E, 0x0204);
	sensor_gain1 = read_cmos_sensor_twobyte(0x6F12);

	write_cmos_sensor_twobyte(0x602C, 0x4000);
	write_cmos_sensor_twobyte(0x602E, 0x0206);
	sensor_gain2 = read_cmos_sensor_twobyte(0x6F12);
	LOG_INF("imgsensor.gain(0x%x), gain1(0x%x), gain2(0x%x)\n", imgsensor.gain, sensor_gain1, sensor_gain2);

	return gain;

}	 /*    set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	spin_lock(&imgsensor_drv_lock);
	imgsensor.mirror = image_mirror;
	spin_unlock(&imgsensor_drv_lock);

	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor(0x0101, 0x00);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor(0x0101, 0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor(0x0101, 0x02);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor(0x0101, 0x03);
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
	}
}

/*************************************************************************
* FUNCTION
*	 night_mode
*
* DESCRIPTION
*	 This function night mode of sensor.
*
* PARAMETERS
*	 bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	 /*    night_mode	 */


static void sensor_WDR_zhdr(void)
{
	if (imgsensor.hdr_mode == 9) {
		LOG_INF("sensor_WDR_zhdr On\n");
		/*it would write 0x21E = 0x1, 0x21F=0x00*/
		/*0x21E=1 , Enable WDR*/
		/*0x21F=0x00, Use Manual mode to set short /long exp */
#if defined(ENABLE_WDR_AUTO_RATION)
		write_cmos_sensor_twobyte(0x021E, 0x0101); /*For WDR auot ration*/
#else
		write_cmos_sensor_twobyte(0x021E, 0x0100); /*For WDR manual ration*/
#endif
		/*write_cmos_sensor_twobyte(0x0220, 0x0801);*/
		/*write_cmos_sensor(0x0222, 0x01);*/

	} else {
		write_cmos_sensor_twobyte(0x021E, 0x0000);
		write_cmos_sensor_twobyte(0x0220, 0x0801);
		LOG_INF("sensor_WDR_zhdr Off\n");
	}
	/*for LE/SE Test*/
	/* hdr_write_shutter(3460,800); */
}


static void check_stremoff(void)
{
	unsigned int i = 0, framecnt = 0;

	for (i = 0; i < 100; i++) {
		framecnt = read_cmos_sensor(0x0005);
		if (framecnt == 0xFF)
			return;
	}
	LOG_INF(" Stream Off Fail!\n");
	/*	return; */
}

static void sensor_init(void)
{
	/*s5k2l2_non-burst init setting*/
	LOG_INF("Enter s5k2l2 sensor_init.\n");
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0xBBF8);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6010, 0x0001);
	mDELAY(3);
	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x0136, 0x1800);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x87AC);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0449);
	write_cmos_sensor_twobyte(0x6F12, 0x0348);
	write_cmos_sensor_twobyte(0x6F12, 0x044A);
	write_cmos_sensor_twobyte(0x6F12, 0x4860);
	write_cmos_sensor_twobyte(0x6F12, 0x101A);
	write_cmos_sensor_twobyte(0x6F12, 0x0881);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xB2B9);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x8CB6);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x5C20);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0xA3D4);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x30B5);
	write_cmos_sensor_twobyte(0x6F12, 0xF64C);
	write_cmos_sensor_twobyte(0x6F12, 0xB0F8);
	write_cmos_sensor_twobyte(0x6F12, 0xE232);
	write_cmos_sensor_twobyte(0x6F12, 0x608C);
	write_cmos_sensor_twobyte(0x6F12, 0x081A);
	write_cmos_sensor_twobyte(0x6F12, 0xA18C);
	write_cmos_sensor_twobyte(0x6F12, 0x401E);
	write_cmos_sensor_twobyte(0x6F12, 0x1944);
	write_cmos_sensor_twobyte(0x6F12, 0x8142);
	write_cmos_sensor_twobyte(0x6F12, 0x01D9);
	write_cmos_sensor_twobyte(0x6F12, 0x0020);
	write_cmos_sensor_twobyte(0x6F12, 0x0346);
	write_cmos_sensor_twobyte(0x6F12, 0xF149);
	write_cmos_sensor_twobyte(0x6F12, 0x9340);
	write_cmos_sensor_twobyte(0x6F12, 0x8887);
	write_cmos_sensor_twobyte(0x6F12, 0x9040);
	write_cmos_sensor_twobyte(0x6F12, 0xF04A);
	write_cmos_sensor_twobyte(0x6F12, 0x99B2);
	write_cmos_sensor_twobyte(0x6F12, 0xD181);
	write_cmos_sensor_twobyte(0x6F12, 0x1082);
	write_cmos_sensor_twobyte(0x6F12, 0xEF4D);
	write_cmos_sensor_twobyte(0x6F12, 0xAD79);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x6F12, 0x06D1);
	write_cmos_sensor_twobyte(0x6F12, 0x33B1);
	write_cmos_sensor_twobyte(0x6F12, 0x94F8);
	write_cmos_sensor_twobyte(0x6F12, 0xB730);
	write_cmos_sensor_twobyte(0x6F12, 0x1BB1);
	write_cmos_sensor_twobyte(0x6F12, 0x5181);
	write_cmos_sensor_twobyte(0x6F12, 0x401E);
	write_cmos_sensor_twobyte(0x6F12, 0x9081);
	write_cmos_sensor_twobyte(0x6F12, 0x30BD);
	write_cmos_sensor_twobyte(0x6F12, 0x4FF6);
	write_cmos_sensor_twobyte(0x6F12, 0xFF70);
	write_cmos_sensor_twobyte(0x6F12, 0x5081);
	write_cmos_sensor_twobyte(0x6F12, 0xF9E7);
	write_cmos_sensor_twobyte(0x6F12, 0x2DE9);
	write_cmos_sensor_twobyte(0x6F12, 0xF041);
	write_cmos_sensor_twobyte(0x6F12, 0x0646);
	write_cmos_sensor_twobyte(0x6F12, 0xE348);
	write_cmos_sensor_twobyte(0x6F12, 0x0D46);
	write_cmos_sensor_twobyte(0x6F12, 0x2438);
	write_cmos_sensor_twobyte(0x6F12, 0x4268);
	write_cmos_sensor_twobyte(0x6F12, 0x140C);
	write_cmos_sensor_twobyte(0x6F12, 0x97B2);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0x3946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xE7F9);
	write_cmos_sensor_twobyte(0x6F12, 0x2946);
	write_cmos_sensor_twobyte(0x6F12, 0x3046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xE8F9);
	write_cmos_sensor_twobyte(0x6F12, 0xDC48);
	write_cmos_sensor_twobyte(0x6F12, 0xDE49);
	write_cmos_sensor_twobyte(0x6F12, 0x0122);
	write_cmos_sensor_twobyte(0x6F12, 0x808F);
	write_cmos_sensor_twobyte(0x6F12, 0xA1F8);
	write_cmos_sensor_twobyte(0x6F12, 0xBC07);
	write_cmos_sensor_twobyte(0x6F12, 0x3946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0xBDE8);
	write_cmos_sensor_twobyte(0x6F12, 0xF041);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xD7B9);
	write_cmos_sensor_twobyte(0x6F12, 0x70B5);
	write_cmos_sensor_twobyte(0x6F12, 0x0646);
	write_cmos_sensor_twobyte(0x6F12, 0xD548);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0x2438);
	write_cmos_sensor_twobyte(0x6F12, 0x8168);
	write_cmos_sensor_twobyte(0x6F12, 0x0C0C);
	write_cmos_sensor_twobyte(0x6F12, 0x8DB2);
	write_cmos_sensor_twobyte(0x6F12, 0x2946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xCBF9);
	write_cmos_sensor_twobyte(0x6F12, 0x3046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xD2F9);
	write_cmos_sensor_twobyte(0x6F12, 0xD249);
	write_cmos_sensor_twobyte(0x6F12, 0x96F8);
	write_cmos_sensor_twobyte(0x6F12, 0xB100);
	write_cmos_sensor_twobyte(0x6F12, 0x0880);
	write_cmos_sensor_twobyte(0x6F12, 0x2946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0xBDE8);
	write_cmos_sensor_twobyte(0x6F12, 0x7040);
	write_cmos_sensor_twobyte(0x6F12, 0x0122);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xBDB9);
	write_cmos_sensor_twobyte(0x6F12, 0x70B5);
	write_cmos_sensor_twobyte(0x6F12, 0x0646);
	write_cmos_sensor_twobyte(0x6F12, 0xC848);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0x2438);
	write_cmos_sensor_twobyte(0x6F12, 0xC168);
	write_cmos_sensor_twobyte(0x6F12, 0x0C0C);
	write_cmos_sensor_twobyte(0x6F12, 0x8DB2);
	write_cmos_sensor_twobyte(0x6F12, 0x2946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xB1F9);
	write_cmos_sensor_twobyte(0x6F12, 0x3046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xBDF9);
	write_cmos_sensor_twobyte(0x6F12, 0x26B3);
	write_cmos_sensor_twobyte(0x6F12, 0xC448);
	write_cmos_sensor_twobyte(0x6F12, 0xB0F8);
	write_cmos_sensor_twobyte(0x6F12, 0xA007);
	write_cmos_sensor_twobyte(0x6F12, 0x00B3);
	write_cmos_sensor_twobyte(0x6F12, 0xC248);
	write_cmos_sensor_twobyte(0x6F12, 0x6521);
	write_cmos_sensor_twobyte(0x6F12, 0x90F8);
	write_cmos_sensor_twobyte(0x6F12, 0xE407);
	write_cmos_sensor_twobyte(0x6F12, 0x4843);
	write_cmos_sensor_twobyte(0x6F12, 0xC149);
	write_cmos_sensor_twobyte(0x6F12, 0x01EB);
	write_cmos_sensor_twobyte(0x6F12, 0x8003);
	write_cmos_sensor_twobyte(0x6F12, 0x01F6);
	write_cmos_sensor_twobyte(0x6F12, 0x9C02);
	write_cmos_sensor_twobyte(0x6F12, 0xC049);
	write_cmos_sensor_twobyte(0x6F12, 0xB3F8);
	write_cmos_sensor_twobyte(0x6F12, 0x3202);
	write_cmos_sensor_twobyte(0x6F12, 0x91F8);
	write_cmos_sensor_twobyte(0x6F12, 0xF719);
	write_cmos_sensor_twobyte(0x6F12, 0x32F8);
	write_cmos_sensor_twobyte(0x6F12, 0x1110);
	write_cmos_sensor_twobyte(0x6F12, 0x0844);
	write_cmos_sensor_twobyte(0x6F12, 0x40F4);
	write_cmos_sensor_twobyte(0x6F12, 0x8051);
	write_cmos_sensor_twobyte(0x6F12, 0xBC48);
	write_cmos_sensor_twobyte(0x6F12, 0x0180);
	write_cmos_sensor_twobyte(0x6F12, 0xB3F8);
	write_cmos_sensor_twobyte(0x6F12, 0x3612);
	write_cmos_sensor_twobyte(0x6F12, 0xB94B);
	write_cmos_sensor_twobyte(0x6F12, 0x93F8);
	write_cmos_sensor_twobyte(0x6F12, 0xF939);
	write_cmos_sensor_twobyte(0x6F12, 0x32F8);
	write_cmos_sensor_twobyte(0x6F12, 0x1320);
	write_cmos_sensor_twobyte(0x6F12, 0x1144);
	write_cmos_sensor_twobyte(0x6F12, 0x41F4);
	write_cmos_sensor_twobyte(0x6F12, 0x8051);
	write_cmos_sensor_twobyte(0x6F12, 0x8180);
	write_cmos_sensor_twobyte(0x6F12, 0x2946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0xBDE8);
	write_cmos_sensor_twobyte(0x6F12, 0x7040);
	write_cmos_sensor_twobyte(0x6F12, 0x0122);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x81B9);
	write_cmos_sensor_twobyte(0x6F12, 0x70B5);
	write_cmos_sensor_twobyte(0x6F12, 0xB24D);
	write_cmos_sensor_twobyte(0x6F12, 0x41F2);
	write_cmos_sensor_twobyte(0x6F12, 0xF046);
	write_cmos_sensor_twobyte(0x6F12, 0x1544);
	write_cmos_sensor_twobyte(0x6F12, 0x049C);
	write_cmos_sensor_twobyte(0x6F12, 0xAD5D);
	write_cmos_sensor_twobyte(0x6F12, 0x0DB9);
	write_cmos_sensor_twobyte(0x6F12, 0x1B78);
	write_cmos_sensor_twobyte(0x6F12, 0x03B1);
	write_cmos_sensor_twobyte(0x6F12, 0x0123);
	write_cmos_sensor_twobyte(0x6F12, 0x2370);
	write_cmos_sensor_twobyte(0x6F12, 0xAE4B);
	write_cmos_sensor_twobyte(0x6F12, 0x03EB);
	write_cmos_sensor_twobyte(0x6F12, 0xC203);
	write_cmos_sensor_twobyte(0x6F12, 0xB3F8);
	write_cmos_sensor_twobyte(0x6F12, 0xD630);
	write_cmos_sensor_twobyte(0x6F12, 0x2381);
	write_cmos_sensor_twobyte(0x6F12, 0x90F8);
	write_cmos_sensor_twobyte(0x6F12, 0xCC50);
	write_cmos_sensor_twobyte(0x6F12, 0x1346);
	write_cmos_sensor_twobyte(0x6F12, 0x05B1);
	write_cmos_sensor_twobyte(0x6F12, 0x0223);
	write_cmos_sensor_twobyte(0x6F12, 0xA94D);
	write_cmos_sensor_twobyte(0x6F12, 0x03EB);
	write_cmos_sensor_twobyte(0x6F12, 0xC303);
	write_cmos_sensor_twobyte(0x6F12, 0x05EB);
	write_cmos_sensor_twobyte(0x6F12, 0x8303);
	write_cmos_sensor_twobyte(0x6F12, 0x1B79);
	write_cmos_sensor_twobyte(0x6F12, 0x53B1);
	write_cmos_sensor_twobyte(0x6F12, 0x9F4B);
	write_cmos_sensor_twobyte(0x6F12, 0xB3F8);
	write_cmos_sensor_twobyte(0x6F12, 0xB855);
	write_cmos_sensor_twobyte(0x6F12, 0x9A4B);
	write_cmos_sensor_twobyte(0x6F12, 0x1B78);
	write_cmos_sensor_twobyte(0x6F12, 0x03F5);
	write_cmos_sensor_twobyte(0x6F12, 0xFC63);
	write_cmos_sensor_twobyte(0x6F12, 0x9D42);
	write_cmos_sensor_twobyte(0x6F12, 0x01D9);
	write_cmos_sensor_twobyte(0x6F12, 0x0123);
	write_cmos_sensor_twobyte(0x6F12, 0x2370);
	write_cmos_sensor_twobyte(0x6F12, 0x90F8);
	write_cmos_sensor_twobyte(0x6F12, 0x2830);
	write_cmos_sensor_twobyte(0x6F12, 0x03B1);
	write_cmos_sensor_twobyte(0x6F12, 0x0123);
	write_cmos_sensor_twobyte(0x6F12, 0x6370);
	write_cmos_sensor_twobyte(0x6F12, 0x90F8);
	write_cmos_sensor_twobyte(0x6F12, 0x1833);
	write_cmos_sensor_twobyte(0x6F12, 0xE370);
	write_cmos_sensor_twobyte(0x6F12, 0x90F8);
	write_cmos_sensor_twobyte(0x6F12, 0x1703);
	write_cmos_sensor_twobyte(0x6F12, 0xA070);
	write_cmos_sensor_twobyte(0x6F12, 0x01EB);
	write_cmos_sensor_twobyte(0x6F12, 0x8200);
	write_cmos_sensor_twobyte(0x6F12, 0xD0F8);
	write_cmos_sensor_twobyte(0x6F12, 0x2C11);
	write_cmos_sensor_twobyte(0x6F12, 0xC1F3);
	write_cmos_sensor_twobyte(0x6F12, 0x8F11);
	write_cmos_sensor_twobyte(0x6F12, 0xA180);
	write_cmos_sensor_twobyte(0x6F12, 0xD0F8);
	write_cmos_sensor_twobyte(0x6F12, 0x3401);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F3);
	write_cmos_sensor_twobyte(0x6F12, 0x8F10);
	write_cmos_sensor_twobyte(0x6F12, 0xE080);
	write_cmos_sensor_twobyte(0x6F12, 0x70BD);
	write_cmos_sensor_twobyte(0x6F12, 0x0346);
	write_cmos_sensor_twobyte(0x6F12, 0x30B5);
	write_cmos_sensor_twobyte(0x6F12, 0x8C48);
	write_cmos_sensor_twobyte(0x6F12, 0xB0F8);
	write_cmos_sensor_twobyte(0x6F12, 0xD007);
	write_cmos_sensor_twobyte(0x6F12, 0x0844);
	write_cmos_sensor_twobyte(0x6F12, 0x8949);
	write_cmos_sensor_twobyte(0x6F12, 0xB1F8);
	write_cmos_sensor_twobyte(0x6F12, 0x6E40);
	write_cmos_sensor_twobyte(0x6F12, 0x001B);
	write_cmos_sensor_twobyte(0x6F12, 0x854C);
	write_cmos_sensor_twobyte(0x6F12, 0x00B2);
	write_cmos_sensor_twobyte(0x6F12, 0xE087);
	write_cmos_sensor_twobyte(0x6F12, 0x01EB);
	write_cmos_sensor_twobyte(0x6F12, 0x4304);
	write_cmos_sensor_twobyte(0x6F12, 0x101A);
	write_cmos_sensor_twobyte(0x6F12, 0xB4F9);
	write_cmos_sensor_twobyte(0x6F12, 0x7E40);
	write_cmos_sensor_twobyte(0x6F12, 0x2044);
	write_cmos_sensor_twobyte(0x6F12, 0x864C);
	write_cmos_sensor_twobyte(0x6F12, 0x9840);
	write_cmos_sensor_twobyte(0x6F12, 0xD4F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0459);
	write_cmos_sensor_twobyte(0x6F12, 0x0524);
	write_cmos_sensor_twobyte(0x6F12, 0x9C40);
	write_cmos_sensor_twobyte(0x6F12, 0x2C44);
	write_cmos_sensor_twobyte(0x6F12, 0x8442);
	write_cmos_sensor_twobyte(0x6F12, 0x04DD);
	write_cmos_sensor_twobyte(0x6F12, 0x91F8);
	write_cmos_sensor_twobyte(0x6F12, 0x7010);
	write_cmos_sensor_twobyte(0x6F12, 0x9A40);
	write_cmos_sensor_twobyte(0x6F12, 0x09B1);
	write_cmos_sensor_twobyte(0x6F12, 0x1044);
	write_cmos_sensor_twobyte(0x6F12, 0x30BD);
	write_cmos_sensor_twobyte(0x6F12, 0x1046);
	write_cmos_sensor_twobyte(0x6F12, 0x30BD);
	write_cmos_sensor_twobyte(0x6F12, 0xF0B5);
	write_cmos_sensor_twobyte(0x6F12, 0x7B4C);
	write_cmos_sensor_twobyte(0x6F12, 0x03EB);
	write_cmos_sensor_twobyte(0x6F12, 0x400C);
	write_cmos_sensor_twobyte(0x6F12, 0xB4F8);
	write_cmos_sensor_twobyte(0x6F12, 0x5049);
	write_cmos_sensor_twobyte(0x6F12, 0x8C42);
	write_cmos_sensor_twobyte(0x6F12, 0x784C);
	write_cmos_sensor_twobyte(0x6F12, 0x02D1);
	write_cmos_sensor_twobyte(0x6F12, 0xD4F8);
	write_cmos_sensor_twobyte(0x6F12, 0x5449);
	write_cmos_sensor_twobyte(0x6F12, 0x01E0);
	write_cmos_sensor_twobyte(0x6F12, 0xD4F8);
	write_cmos_sensor_twobyte(0x6F12, 0x7049);
	write_cmos_sensor_twobyte(0x6F12, 0xDFF8);
	write_cmos_sensor_twobyte(0x6F12, 0xC4E1);
	write_cmos_sensor_twobyte(0x6F12, 0x1702);
	write_cmos_sensor_twobyte(0x6F12, 0x9EF8);
	write_cmos_sensor_twobyte(0x6F12, 0x0250);
	write_cmos_sensor_twobyte(0x6F12, 0x0DB1);
	write_cmos_sensor_twobyte(0x6F12, 0x2646);
	write_cmos_sensor_twobyte(0x6F12, 0x01E0);
	write_cmos_sensor_twobyte(0x6F12, 0x4FF4);
	write_cmos_sensor_twobyte(0x6F12, 0x8056);
	write_cmos_sensor_twobyte(0x6F12, 0xB7FB);
	write_cmos_sensor_twobyte(0x6F12, 0xF6F7);
	write_cmos_sensor_twobyte(0x6F12, 0x03EB);
	write_cmos_sensor_twobyte(0x6F12, 0x4000);
	write_cmos_sensor_twobyte(0x6F12, 0xBEF8);
	write_cmos_sensor_twobyte(0x6F12, 0x0460);
	write_cmos_sensor_twobyte(0x6F12, 0xB0F9);
	write_cmos_sensor_twobyte(0x6F12, 0x8200);
	write_cmos_sensor_twobyte(0x6F12, 0xA1F5);
	write_cmos_sensor_twobyte(0x6F12, 0x8071);
	write_cmos_sensor_twobyte(0x6F12, 0x4843);
	write_cmos_sensor_twobyte(0x6F12, 0xB742);
	write_cmos_sensor_twobyte(0x6F12, 0x18D9);
	write_cmos_sensor_twobyte(0x6F12, 0xBCF9);
	write_cmos_sensor_twobyte(0x6F12, 0x0230);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x7043);
	write_cmos_sensor_twobyte(0x6F12, 0x5943);
	write_cmos_sensor_twobyte(0x6F12, 0xC6EB);
	write_cmos_sensor_twobyte(0x6F12, 0x1216);
	write_cmos_sensor_twobyte(0x6F12, 0x0912);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x4E43);
	write_cmos_sensor_twobyte(0x6F12, 0x310A);
	write_cmos_sensor_twobyte(0x6F12, 0x25B1);
	write_cmos_sensor_twobyte(0x6F12, 0x6043);
	write_cmos_sensor_twobyte(0x6F12, 0xB0FB);
	write_cmos_sensor_twobyte(0x6F12, 0xF2F0);
	write_cmos_sensor_twobyte(0x6F12, 0x6143);
	write_cmos_sensor_twobyte(0x6F12, 0x03E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0003);
	write_cmos_sensor_twobyte(0x6F12, 0xB0FB);
	write_cmos_sensor_twobyte(0x6F12, 0xF2F0);
	write_cmos_sensor_twobyte(0x6F12, 0x0903);
	write_cmos_sensor_twobyte(0x6F12, 0xB1FB);
	write_cmos_sensor_twobyte(0x6F12, 0xF2F1);
	write_cmos_sensor_twobyte(0x6F12, 0x0844);
	write_cmos_sensor_twobyte(0x6F12, 0xF0BD);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0xF0BD);
	write_cmos_sensor_twobyte(0x6F12, 0x70B5);
	write_cmos_sensor_twobyte(0x6F12, 0x0446);
	write_cmos_sensor_twobyte(0x6F12, 0x5748);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0x2438);
	write_cmos_sensor_twobyte(0x6F12, 0xC069);
	write_cmos_sensor_twobyte(0x6F12, 0x86B2);
	write_cmos_sensor_twobyte(0x6F12, 0x050C);
	write_cmos_sensor_twobyte(0x6F12, 0x3146);
	write_cmos_sensor_twobyte(0x6F12, 0x2846);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xD0F8);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xE1F8);
	write_cmos_sensor_twobyte(0x6F12, 0x94F8);
	write_cmos_sensor_twobyte(0x6F12, 0x2500);
	write_cmos_sensor_twobyte(0x6F12, 0x8107);
	write_cmos_sensor_twobyte(0x6F12, 0x4F48);
	write_cmos_sensor_twobyte(0x6F12, 0x04D5);
	write_cmos_sensor_twobyte(0x6F12, 0x6188);
	write_cmos_sensor_twobyte(0x6F12, 0x4078);
	write_cmos_sensor_twobyte(0x6F12, 0x0844);
	write_cmos_sensor_twobyte(0x6F12, 0x6080);
	write_cmos_sensor_twobyte(0x6F12, 0x03E0);
	write_cmos_sensor_twobyte(0x6F12, 0xE188);
	write_cmos_sensor_twobyte(0x6F12, 0x4078);
	write_cmos_sensor_twobyte(0x6F12, 0x081A);
	write_cmos_sensor_twobyte(0x6F12, 0xE080);
	write_cmos_sensor_twobyte(0x6F12, 0x3146);
	write_cmos_sensor_twobyte(0x6F12, 0x2846);
	write_cmos_sensor_twobyte(0x6F12, 0xBDE8);
	write_cmos_sensor_twobyte(0x6F12, 0x7040);
	write_cmos_sensor_twobyte(0x6F12, 0x0122);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xB8B8);
	write_cmos_sensor_twobyte(0x6F12, 0x2DE9);
	write_cmos_sensor_twobyte(0x6F12, 0xF041);
	write_cmos_sensor_twobyte(0x6F12, 0x0646);
	write_cmos_sensor_twobyte(0x6F12, 0x4548);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0x2438);
	write_cmos_sensor_twobyte(0x6F12, 0x006A);
	write_cmos_sensor_twobyte(0x6F12, 0x85B2);
	write_cmos_sensor_twobyte(0x6F12, 0x040C);
	write_cmos_sensor_twobyte(0x6F12, 0x2946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xABF8);
	write_cmos_sensor_twobyte(0x6F12, 0x3046);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0xC1F8);
	write_cmos_sensor_twobyte(0x6F12, 0x4948);
	write_cmos_sensor_twobyte(0x6F12, 0x0068);
	write_cmos_sensor_twobyte(0x6F12, 0x0088);
	write_cmos_sensor_twobyte(0x6F12, 0xA0F5);
	write_cmos_sensor_twobyte(0x6F12, 0x0051);
	write_cmos_sensor_twobyte(0x6F12, 0xC239);
	write_cmos_sensor_twobyte(0x6F12, 0x29D1);
	write_cmos_sensor_twobyte(0x6F12, 0x3E49);
	write_cmos_sensor_twobyte(0x6F12, 0xF008);
	write_cmos_sensor_twobyte(0x6F12, 0x3A4E);
	write_cmos_sensor_twobyte(0x6F12, 0xD1F8);
	write_cmos_sensor_twobyte(0x6F12, 0xE810);
	write_cmos_sensor_twobyte(0x6F12, 0x4843);
	write_cmos_sensor_twobyte(0x6F12, 0xF188);
	write_cmos_sensor_twobyte(0x6F12, 0x400B);
	write_cmos_sensor_twobyte(0x6F12, 0x01B3);
	write_cmos_sensor_twobyte(0x6F12, 0x3189);
	write_cmos_sensor_twobyte(0x6F12, 0x8142);
	write_cmos_sensor_twobyte(0x6F12, 0x01D3);
	write_cmos_sensor_twobyte(0x6F12, 0x0020);
	write_cmos_sensor_twobyte(0x6F12, 0x03E0);
	write_cmos_sensor_twobyte(0x6F12, 0x7189);
	write_cmos_sensor_twobyte(0x6F12, 0x8142);
	write_cmos_sensor_twobyte(0x6F12, 0x02D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0220);
	write_cmos_sensor_twobyte(0x6F12, 0x86F8);
	write_cmos_sensor_twobyte(0x6F12, 0x4000);
	write_cmos_sensor_twobyte(0x6F12, 0x96F8);
	write_cmos_sensor_twobyte(0x6F12, 0x4000);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0x0C27);
	write_cmos_sensor_twobyte(0x6F12, 0x8119);
	write_cmos_sensor_twobyte(0x6F12, 0x08E0);
	write_cmos_sensor_twobyte(0x6F12, 0x07EB);
	write_cmos_sensor_twobyte(0x6F12, 0x4000);
	write_cmos_sensor_twobyte(0x6F12, 0x03F1);
	write_cmos_sensor_twobyte(0x6F12, 0x8043);
	write_cmos_sensor_twobyte(0x6F12, 0x085A);
	write_cmos_sensor_twobyte(0x6F12, 0x1880);
	write_cmos_sensor_twobyte(0x6F12, 0x521C);
	write_cmos_sensor_twobyte(0x6F12, 0x082A);
	write_cmos_sensor_twobyte(0x6F12, 0x06D2);
	write_cmos_sensor_twobyte(0x6F12, 0x02EB);
	write_cmos_sensor_twobyte(0x6F12, 0x4200);
	write_cmos_sensor_twobyte(0x6F12, 0x06EB);
	write_cmos_sensor_twobyte(0x6F12, 0x4003);
	write_cmos_sensor_twobyte(0x6F12, 0x1B8A);
	write_cmos_sensor_twobyte(0x6F12, 0x002B);
	write_cmos_sensor_twobyte(0x6F12, 0xEFD1);
	write_cmos_sensor_twobyte(0x6F12, 0x2946);
	write_cmos_sensor_twobyte(0x6F12, 0x2046);
	write_cmos_sensor_twobyte(0x6F12, 0xBDE8);
	write_cmos_sensor_twobyte(0x6F12, 0xF041);
	write_cmos_sensor_twobyte(0x6F12, 0x0122);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x70B8);
	write_cmos_sensor_twobyte(0x6F12, 0x10B5);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0x5B31);
	write_cmos_sensor_twobyte(0x6F12, 0x2C48);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x87F8);
	write_cmos_sensor_twobyte(0x6F12, 0x1F4C);
	write_cmos_sensor_twobyte(0x6F12, 0x243C);
	write_cmos_sensor_twobyte(0x6F12, 0x0122);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0x2131);
	write_cmos_sensor_twobyte(0x6F12, 0x2060);
	write_cmos_sensor_twobyte(0x6F12, 0x2948);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x7EF8);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0xF121);
	write_cmos_sensor_twobyte(0x6F12, 0x6060);
	write_cmos_sensor_twobyte(0x6F12, 0x2648);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x77F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0xCD21);
	write_cmos_sensor_twobyte(0x6F12, 0xA060);
	write_cmos_sensor_twobyte(0x6F12, 0x2448);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x70F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0x6121);
	write_cmos_sensor_twobyte(0x6F12, 0xE060);
	write_cmos_sensor_twobyte(0x6F12, 0x2148);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x69F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0xEF11);
	write_cmos_sensor_twobyte(0x6F12, 0x2061);
	write_cmos_sensor_twobyte(0x6F12, 0x1F48);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x62F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0xB311);
	write_cmos_sensor_twobyte(0x6F12, 0x6061);
	write_cmos_sensor_twobyte(0x6F12, 0x1C48);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x5BF8);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0x4311);
	write_cmos_sensor_twobyte(0x6F12, 0xA061);
	write_cmos_sensor_twobyte(0x6F12, 0x1A48);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x54F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0022);
	write_cmos_sensor_twobyte(0x6F12, 0xAFF2);
	write_cmos_sensor_twobyte(0x6F12, 0x0711);
	write_cmos_sensor_twobyte(0x6F12, 0xE061);
	write_cmos_sensor_twobyte(0x6F12, 0x1748);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x6F12, 0x4DF8);
	write_cmos_sensor_twobyte(0x6F12, 0x2062);
	write_cmos_sensor_twobyte(0x6F12, 0x10BD);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x0AE0);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x4000);
	write_cmos_sensor_twobyte(0x6F12, 0xF000);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x0A40);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x5D00);
	write_cmos_sensor_twobyte(0x6F12, 0x4000);
	write_cmos_sensor_twobyte(0x6F12, 0x6B56);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x68F0);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x0BB0);
	write_cmos_sensor_twobyte(0x6F12, 0x4000);
	write_cmos_sensor_twobyte(0x6F12, 0xF66E);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x7220);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x5900);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x8720);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x6F12, 0x0300);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x6F12, 0x14E7);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x5059);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x512F);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0xFB2B);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0xD02D);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x724F);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0xE959);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x287F);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x6F12, 0x0631);
	write_cmos_sensor_twobyte(0x6F12, 0x45F6);
	write_cmos_sensor_twobyte(0x6F12, 0x576C);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F2);
	write_cmos_sensor_twobyte(0x6F12, 0x000C);
	write_cmos_sensor_twobyte(0x6F12, 0x6047);
	write_cmos_sensor_twobyte(0x6F12, 0x45F2);
	write_cmos_sensor_twobyte(0x6F12, 0x590C);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F2);
	write_cmos_sensor_twobyte(0x6F12, 0x000C);
	write_cmos_sensor_twobyte(0x6F12, 0x6047);
	write_cmos_sensor_twobyte(0x6F12, 0x45F2);
	write_cmos_sensor_twobyte(0x6F12, 0x2F1C);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F2);
	write_cmos_sensor_twobyte(0x6F12, 0x000C);
	write_cmos_sensor_twobyte(0x6F12, 0x6047);
	write_cmos_sensor_twobyte(0x6F12, 0x4FF6);
	write_cmos_sensor_twobyte(0x6F12, 0x2B3C);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F2);
	write_cmos_sensor_twobyte(0x6F12, 0x000C);
	write_cmos_sensor_twobyte(0x6F12, 0x6047);
	write_cmos_sensor_twobyte(0x6F12, 0x42F6);
	write_cmos_sensor_twobyte(0x6F12, 0x7F0C);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F2);
	write_cmos_sensor_twobyte(0x6F12, 0x000C);
	write_cmos_sensor_twobyte(0x6F12, 0x6047);
	write_cmos_sensor_twobyte(0x6F12, 0x40F2);
	write_cmos_sensor_twobyte(0x6F12, 0x316C);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F2);
	write_cmos_sensor_twobyte(0x6F12, 0x010C);
	write_cmos_sensor_twobyte(0x6F12, 0x6047);
	write_cmos_sensor_twobyte(0x6F12, 0x43F2);
	write_cmos_sensor_twobyte(0x6F12, 0x7B5C);
	write_cmos_sensor_twobyte(0x6F12, 0xC0F2);
	write_cmos_sensor_twobyte(0x6F12, 0x000C);
	write_cmos_sensor_twobyte(0x6F12, 0x6047);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x0208, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x16B0);
	write_cmos_sensor_twobyte(0x6F12, 0x004E);
	write_cmos_sensor_twobyte(0x6F12, 0x004E);
	write_cmos_sensor_twobyte(0x602A, 0x167E);
	write_cmos_sensor_twobyte(0x6F12, 0x0018);
	write_cmos_sensor_twobyte(0x602A, 0x1682);
	write_cmos_sensor_twobyte(0x6F12, 0x0010);
	write_cmos_sensor_twobyte(0x602A, 0x16A0);
	write_cmos_sensor_twobyte(0x6F12, 0x2101);
	write_cmos_sensor_twobyte(0x602A, 0x1664);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x2A9A);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x2AC2);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x2AEA);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x2B12);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x2B3A);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x2B62);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x2B8A);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x2BB2);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x6F12, 0x0FD0);
	write_cmos_sensor_twobyte(0x602A, 0x3568);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x402C);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x4AF0);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x35C4);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x35F6);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x35BA);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x35EC);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x35B0);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x35E2);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x35A6);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x35D8);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x4088);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x40BA);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x407E);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x40B0);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x4074);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x40A6);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x406A);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x409C);
	write_cmos_sensor_twobyte(0x6F12, 0xFFD8);
	write_cmos_sensor_twobyte(0x602A, 0x4B4C);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x602A, 0x4B7E);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x602A, 0x4B42);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x602A, 0x4B74);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x602A, 0x4B38);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x602A, 0x4B6A);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x602A, 0x4B2E);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x602A, 0x4B60);
	write_cmos_sensor_twobyte(0x6F12, 0xFFCD);
	write_cmos_sensor_twobyte(0x0200, 0x0100);
	write_cmos_sensor_twobyte(0x0202, 0x0100);
	write_cmos_sensor_twobyte(0x0224, 0x0100);
	write_cmos_sensor_twobyte(0x0226, 0x0100);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B52);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1686);
	write_cmos_sensor_twobyte(0x6F12, 0x0328);
	write_cmos_sensor_twobyte(0x602A, 0x4C54);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x4C58);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x4C5C);
	write_cmos_sensor_twobyte(0x6F12, 0x010F);
	write_cmos_sensor_twobyte(0x602A, 0x09A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x09AE);
	write_cmos_sensor_twobyte(0x6F12, 0xF408);
	write_cmos_sensor_twobyte(0x602A, 0x8C7C);
	write_cmos_sensor_twobyte(0x6F12, 0x0103);
	write_cmos_sensor_twobyte(0x6F12, 0x0103);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x6F12, 0xD004);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x6F12, 0xD804);
	write_cmos_sensor_twobyte(0x6F12, 0x0011);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0xD0D0);
	write_cmos_sensor_twobyte(0x6F12, 0x0011);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0xD8D0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x1668);
	write_cmos_sensor_twobyte(0x6F12, 0x0183);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x602A, 0x16B4);
	write_cmos_sensor_twobyte(0x6F12, 0x54C2);
	write_cmos_sensor_twobyte(0x602A, 0x168E);
	write_cmos_sensor_twobyte(0x6F12, 0x0030);
	write_cmos_sensor_twobyte(0x6F12, 0x0C18);
	write_cmos_sensor_twobyte(0x6F12, 0x0C18);
	write_cmos_sensor_twobyte(0x602A, 0x169C);
	write_cmos_sensor_twobyte(0x6F12, 0x0C30);
	write_cmos_sensor_twobyte(0x6F12, 0x0C30);
	write_cmos_sensor_twobyte(0x602A, 0x168A);
	write_cmos_sensor_twobyte(0x6F12, 0x001B);
	write_cmos_sensor_twobyte(0x602A, 0x1640);
	write_cmos_sensor_twobyte(0x6F12, 0x0505);
	write_cmos_sensor_twobyte(0x6F12, 0x070E);
	write_cmos_sensor_twobyte(0x602A, 0x1648);
	write_cmos_sensor_twobyte(0x6F12, 0x0505);
	write_cmos_sensor_twobyte(0x6F12, 0x070E);
	write_cmos_sensor_twobyte(0x602A, 0x1650);
	write_cmos_sensor_twobyte(0x6F12, 0x0A0A);
	write_cmos_sensor_twobyte(0x6F12, 0x0A0A);
	write_cmos_sensor_twobyte(0x602A, 0x1658);
	write_cmos_sensor_twobyte(0x6F12, 0x0A0A);
	write_cmos_sensor_twobyte(0x6F12, 0x0A0A);
	write_cmos_sensor_twobyte(0x602A, 0x1630);
	write_cmos_sensor_twobyte(0x6F12, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x1C1C);
	write_cmos_sensor_twobyte(0x602A, 0x1638);
	write_cmos_sensor_twobyte(0x6F12, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x1C1C);
	write_cmos_sensor_twobyte(0x602A, 0x15E4);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x6F12, 0x0202);
	write_cmos_sensor_twobyte(0x6F12, 0x0302);
	write_cmos_sensor_twobyte(0x602A, 0x0BCA);
	write_cmos_sensor_twobyte(0x6F12, 0x0420);
	write_cmos_sensor_twobyte(0x6F12, 0x0420);
	write_cmos_sensor_twobyte(0x6F12, 0x0418);
	write_cmos_sensor_twobyte(0x602A, 0x0BD8);
	write_cmos_sensor_twobyte(0x6F12, 0x000A);
	write_cmos_sensor_twobyte(0x6F12, 0x0006);
	write_cmos_sensor_twobyte(0x6F12, 0x0006);
	write_cmos_sensor_twobyte(0x6F12, 0x0004);
	write_cmos_sensor_twobyte(0x602A, 0x16BC);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x0A10);
	write_cmos_sensor_twobyte(0x6F12, 0x0064);
	write_cmos_sensor_twobyte(0x602A, 0x0C16);
	write_cmos_sensor_twobyte(0x6F12, 0x01D8);
	write_cmos_sensor_twobyte(0x602A, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x019D);
	write_cmos_sensor_twobyte(0x6F12, 0x0114);
	write_cmos_sensor_twobyte(0x6F12, 0x0107);
	write_cmos_sensor_twobyte(0x6F12, 0x00BF);
	write_cmos_sensor_twobyte(0x6F12, 0x01D5);
	write_cmos_sensor_twobyte(0x6F12, 0x014C);
	write_cmos_sensor_twobyte(0x6F12, 0x013F);
	write_cmos_sensor_twobyte(0x6F12, 0x00F7);
	write_cmos_sensor_twobyte(0x6F12, 0x019C);
	write_cmos_sensor_twobyte(0x6F12, 0x0113);
	write_cmos_sensor_twobyte(0x6F12, 0x0106);
	write_cmos_sensor_twobyte(0x6F12, 0x00BE);
	write_cmos_sensor_twobyte(0x6F12, 0x01D6);
	write_cmos_sensor_twobyte(0x6F12, 0x014D);
	write_cmos_sensor_twobyte(0x6F12, 0x0140);
	write_cmos_sensor_twobyte(0x6F12, 0x00F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0094);
	write_cmos_sensor_twobyte(0x6F12, 0x0072);
	write_cmos_sensor_twobyte(0x6F12, 0x0086);
	write_cmos_sensor_twobyte(0x6F12, 0x010F);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x6F12, 0x005D);
	write_cmos_sensor_twobyte(0x6F12, 0x0071);
	write_cmos_sensor_twobyte(0x6F12, 0x00FA);
	write_cmos_sensor_twobyte(0x6F12, 0x0069);
	write_cmos_sensor_twobyte(0x6F12, 0x005B);
	write_cmos_sensor_twobyte(0x6F12, 0x005B);
	write_cmos_sensor_twobyte(0x6F12, 0x005B);
	write_cmos_sensor_twobyte(0x602A, 0x0C68);
	write_cmos_sensor_twobyte(0x6F12, 0x008D);
	write_cmos_sensor_twobyte(0x6F12, 0x006B);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x6F12, 0x0108);
	write_cmos_sensor_twobyte(0x6F12, 0x0086);
	write_cmos_sensor_twobyte(0x6F12, 0x0064);
	write_cmos_sensor_twobyte(0x6F12, 0x0078);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x6F12, 0x0062);
	write_cmos_sensor_twobyte(0x6F12, 0x0054);
	write_cmos_sensor_twobyte(0x6F12, 0x0054);
	write_cmos_sensor_twobyte(0x6F12, 0x0054);
	write_cmos_sensor_twobyte(0x602A, 0x0C88);
	write_cmos_sensor_twobyte(0x6F12, 0x0063);
	write_cmos_sensor_twobyte(0x6F12, 0x0055);
	write_cmos_sensor_twobyte(0x6F12, 0x0055);
	write_cmos_sensor_twobyte(0x6F12, 0x0055);
	write_cmos_sensor_twobyte(0x602A, 0x0C98);
	write_cmos_sensor_twobyte(0x6F12, 0x008D);
	write_cmos_sensor_twobyte(0x6F12, 0x006B);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x6F12, 0x0108);
	write_cmos_sensor_twobyte(0x6F12, 0x0086);
	write_cmos_sensor_twobyte(0x6F12, 0x0064);
	write_cmos_sensor_twobyte(0x6F12, 0x0078);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x6F12, 0x0062);
	write_cmos_sensor_twobyte(0x6F12, 0x0054);
	write_cmos_sensor_twobyte(0x6F12, 0x0054);
	write_cmos_sensor_twobyte(0x6F12, 0x0054);
	write_cmos_sensor_twobyte(0x602A, 0x0CC0);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00BD);
	write_cmos_sensor_twobyte(0x602A, 0x0CCA);
	write_cmos_sensor_twobyte(0x6F12, 0x0282);
	write_cmos_sensor_twobyte(0x6F12, 0x0261);
	write_cmos_sensor_twobyte(0x6F12, 0x01D5);
	write_cmos_sensor_twobyte(0x6F12, 0x019D);
	write_cmos_sensor_twobyte(0x6F12, 0x0114);
	write_cmos_sensor_twobyte(0x6F12, 0x0107);
	write_cmos_sensor_twobyte(0x6F12, 0x00BF);
	write_cmos_sensor_twobyte(0x602A, 0x0CDA);
	write_cmos_sensor_twobyte(0x6F12, 0x0280);
	write_cmos_sensor_twobyte(0x6F12, 0x025F);
	write_cmos_sensor_twobyte(0x6F12, 0x01D3);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00BD);
	write_cmos_sensor_twobyte(0x6F12, 0x01F3);
	write_cmos_sensor_twobyte(0x6F12, 0x016A);
	write_cmos_sensor_twobyte(0x6F12, 0x015D);
	write_cmos_sensor_twobyte(0x6F12, 0x0113);
	write_cmos_sensor_twobyte(0x602A, 0x0CF8);
	write_cmos_sensor_twobyte(0x6F12, 0x00DF);
	write_cmos_sensor_twobyte(0x6F12, 0x005A);
	write_cmos_sensor_twobyte(0x602A, 0x0D08);
	write_cmos_sensor_twobyte(0x6F12, 0x00D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0051);
	write_cmos_sensor_twobyte(0x602A, 0x0D18);
	write_cmos_sensor_twobyte(0x6F12, 0x00DD);
	write_cmos_sensor_twobyte(0x6F12, 0x0058);
	write_cmos_sensor_twobyte(0x602A, 0x0D20);
	write_cmos_sensor_twobyte(0x6F12, 0x0018);
	write_cmos_sensor_twobyte(0x6F12, 0x0018);
	write_cmos_sensor_twobyte(0x6F12, 0x0018);
	write_cmos_sensor_twobyte(0x6F12, 0x0018);
	write_cmos_sensor_twobyte(0x6F12, 0x0193);
	write_cmos_sensor_twobyte(0x6F12, 0x010A);
	write_cmos_sensor_twobyte(0x6F12, 0x00FD);
	write_cmos_sensor_twobyte(0x6F12, 0x00BA);
	write_cmos_sensor_twobyte(0x602A, 0x0D38);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0010);
	write_cmos_sensor_twobyte(0x6F12, 0x01D5);
	write_cmos_sensor_twobyte(0x6F12, 0x014C);
	write_cmos_sensor_twobyte(0x6F12, 0x013F);
	write_cmos_sensor_twobyte(0x6F12, 0x00F7);
	write_cmos_sensor_twobyte(0x6F12, 0x01EB);
	write_cmos_sensor_twobyte(0x6F12, 0x0162);
	write_cmos_sensor_twobyte(0x6F12, 0x0155);
	write_cmos_sensor_twobyte(0x6F12, 0x010B);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x0D6E);
	write_cmos_sensor_twobyte(0x6F12, 0x01D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x602A, 0x0D7C);
	write_cmos_sensor_twobyte(0x6F12, 0x0007);
	write_cmos_sensor_twobyte(0x602A, 0x0D8E);
	write_cmos_sensor_twobyte(0x6F12, 0x01D8);
	write_cmos_sensor_twobyte(0x602A, 0x0D9E);
	write_cmos_sensor_twobyte(0x6F12, 0x01D8);
	write_cmos_sensor_twobyte(0x602A, 0x0DAA);
	write_cmos_sensor_twobyte(0x6F12, 0x0282);
	write_cmos_sensor_twobyte(0x6F12, 0x0261);
	write_cmos_sensor_twobyte(0x6F12, 0x01D5);
	write_cmos_sensor_twobyte(0x602A, 0x0DBA);
	write_cmos_sensor_twobyte(0x6F12, 0x0282);
	write_cmos_sensor_twobyte(0x6F12, 0x0261);
	write_cmos_sensor_twobyte(0x6F12, 0x01D5);
	write_cmos_sensor_twobyte(0x6F12, 0x015F);
	write_cmos_sensor_twobyte(0x6F12, 0x00DA);
	write_cmos_sensor_twobyte(0x6F12, 0x00CD);
	write_cmos_sensor_twobyte(0x6F12, 0x008D);
	write_cmos_sensor_twobyte(0x6F12, 0x0193);
	write_cmos_sensor_twobyte(0x6F12, 0x010A);
	write_cmos_sensor_twobyte(0x6F12, 0x00FD);
	write_cmos_sensor_twobyte(0x6F12, 0x00BB);
	write_cmos_sensor_twobyte(0x6F12, 0x026D);
	write_cmos_sensor_twobyte(0x6F12, 0x01E4);
	write_cmos_sensor_twobyte(0x6F12, 0x01C3);
	write_cmos_sensor_twobyte(0x6F12, 0x013A);
	write_cmos_sensor_twobyte(0x602A, 0x0DDA);
	write_cmos_sensor_twobyte(0x6F12, 0x027E);
	write_cmos_sensor_twobyte(0x6F12, 0x025D);
	write_cmos_sensor_twobyte(0x6F12, 0x01D1);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00C2);
	write_cmos_sensor_twobyte(0x6F12, 0x01AD);
	write_cmos_sensor_twobyte(0x6F12, 0x0124);
	write_cmos_sensor_twobyte(0x6F12, 0x0117);
	write_cmos_sensor_twobyte(0x6F12, 0x00D4);
	write_cmos_sensor_twobyte(0x6F12, 0x01A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0117);
	write_cmos_sensor_twobyte(0x6F12, 0x010A);
	write_cmos_sensor_twobyte(0x6F12, 0x00C7);
	write_cmos_sensor_twobyte(0x6F12, 0x01B2);
	write_cmos_sensor_twobyte(0x6F12, 0x0129);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x00D9);
	write_cmos_sensor_twobyte(0x6F12, 0x01A5);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x010F);
	write_cmos_sensor_twobyte(0x6F12, 0x00CC);
	write_cmos_sensor_twobyte(0x6F12, 0x01B2);
	write_cmos_sensor_twobyte(0x6F12, 0x0129);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x00D9);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00C2);
	write_cmos_sensor_twobyte(0x6F12, 0x019E);
	write_cmos_sensor_twobyte(0x6F12, 0x0115);
	write_cmos_sensor_twobyte(0x6F12, 0x0108);
	write_cmos_sensor_twobyte(0x6F12, 0x00C5);
	write_cmos_sensor_twobyte(0x602A, 0x0E30);
	write_cmos_sensor_twobyte(0x6F12, 0x01A3);
	write_cmos_sensor_twobyte(0x6F12, 0x011A);
	write_cmos_sensor_twobyte(0x6F12, 0x010D);
	write_cmos_sensor_twobyte(0x6F12, 0x00CA);
	write_cmos_sensor_twobyte(0x6F12, 0x01B2);
	write_cmos_sensor_twobyte(0x6F12, 0x0129);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x00D9);
	write_cmos_sensor_twobyte(0x602A, 0x0E50);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00C2);
	write_cmos_sensor_twobyte(0x6F12, 0x019E);
	write_cmos_sensor_twobyte(0x6F12, 0x0115);
	write_cmos_sensor_twobyte(0x6F12, 0x0108);
	write_cmos_sensor_twobyte(0x6F12, 0x00C5);
	write_cmos_sensor_twobyte(0x6F12, 0x00FC);
	write_cmos_sensor_twobyte(0x6F12, 0x0077);
	write_cmos_sensor_twobyte(0x6F12, 0x0074);
	write_cmos_sensor_twobyte(0x6F12, 0x0074);
	write_cmos_sensor_twobyte(0x6F12, 0x0196);
	write_cmos_sensor_twobyte(0x6F12, 0x010D);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x6F12, 0x00BD);
	write_cmos_sensor_twobyte(0x6F12, 0x026B);
	write_cmos_sensor_twobyte(0x6F12, 0x01E2);
	write_cmos_sensor_twobyte(0x6F12, 0x01C1);
	write_cmos_sensor_twobyte(0x6F12, 0x0138);
	write_cmos_sensor_twobyte(0x602A, 0x0E7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0280);
	write_cmos_sensor_twobyte(0x6F12, 0x025F);
	write_cmos_sensor_twobyte(0x6F12, 0x01D2);
	write_cmos_sensor_twobyte(0x602A, 0x0E90);
	write_cmos_sensor_twobyte(0x6F12, 0x019A);
	write_cmos_sensor_twobyte(0x6F12, 0x0111);
	write_cmos_sensor_twobyte(0x6F12, 0x0104);
	write_cmos_sensor_twobyte(0x6F12, 0x00C1);
	write_cmos_sensor_twobyte(0x6F12, 0x01B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0127);
	write_cmos_sensor_twobyte(0x6F12, 0x011A);
	write_cmos_sensor_twobyte(0x6F12, 0x00D6);
	write_cmos_sensor_twobyte(0x6F12, 0x01B2);
	write_cmos_sensor_twobyte(0x6F12, 0x0129);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x00D8);
	write_cmos_sensor_twobyte(0x6F12, 0x01C8);
	write_cmos_sensor_twobyte(0x6F12, 0x013F);
	write_cmos_sensor_twobyte(0x6F12, 0x0132);
	write_cmos_sensor_twobyte(0x6F12, 0x00ED);
	write_cmos_sensor_twobyte(0x6F12, 0x01CA);
	write_cmos_sensor_twobyte(0x6F12, 0x0141);
	write_cmos_sensor_twobyte(0x6F12, 0x0134);
	write_cmos_sensor_twobyte(0x6F12, 0x00EF);
	write_cmos_sensor_twobyte(0x6F12, 0x01E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0157);
	write_cmos_sensor_twobyte(0x6F12, 0x014A);
	write_cmos_sensor_twobyte(0x6F12, 0x0104);
	write_cmos_sensor_twobyte(0x6F12, 0x01E3);
	write_cmos_sensor_twobyte(0x6F12, 0x015A);
	write_cmos_sensor_twobyte(0x6F12, 0x014D);
	write_cmos_sensor_twobyte(0x6F12, 0x0107);
	write_cmos_sensor_twobyte(0x6F12, 0x01F9);
	write_cmos_sensor_twobyte(0x6F12, 0x0170);
	write_cmos_sensor_twobyte(0x602A, 0x0ECE);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x01FB);
	write_cmos_sensor_twobyte(0x6F12, 0x0172);
	write_cmos_sensor_twobyte(0x602A, 0x0ED6);
	write_cmos_sensor_twobyte(0x6F12, 0x011E);
	write_cmos_sensor_twobyte(0x6F12, 0x0211);
	write_cmos_sensor_twobyte(0x6F12, 0x0188);
	write_cmos_sensor_twobyte(0x6F12, 0x017B);
	write_cmos_sensor_twobyte(0x6F12, 0x0133);
	write_cmos_sensor_twobyte(0x6F12, 0x0213);
	write_cmos_sensor_twobyte(0x6F12, 0x018A);
	write_cmos_sensor_twobyte(0x6F12, 0x017D);
	write_cmos_sensor_twobyte(0x6F12, 0x0135);
	write_cmos_sensor_twobyte(0x6F12, 0x0229);
	write_cmos_sensor_twobyte(0x6F12, 0x01A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0193);
	write_cmos_sensor_twobyte(0x6F12, 0x014A);
	write_cmos_sensor_twobyte(0x6F12, 0x022B);
	write_cmos_sensor_twobyte(0x6F12, 0x01A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0195);
	write_cmos_sensor_twobyte(0x6F12, 0x014C);
	write_cmos_sensor_twobyte(0x6F12, 0x0241);
	write_cmos_sensor_twobyte(0x6F12, 0x01B8);
	write_cmos_sensor_twobyte(0x6F12, 0x01AB);
	write_cmos_sensor_twobyte(0x6F12, 0x0161);
	write_cmos_sensor_twobyte(0x6F12, 0x0243);
	write_cmos_sensor_twobyte(0x6F12, 0x01BA);
	write_cmos_sensor_twobyte(0x6F12, 0x01AD);
	write_cmos_sensor_twobyte(0x6F12, 0x0163);
	write_cmos_sensor_twobyte(0x6F12, 0x0259);
	write_cmos_sensor_twobyte(0x6F12, 0x01D0);
	write_cmos_sensor_twobyte(0x6F12, 0x01C3);
	write_cmos_sensor_twobyte(0x6F12, 0x0178);
	write_cmos_sensor_twobyte(0x602A, 0x0F12);
	write_cmos_sensor_twobyte(0x6F12, 0x0237);
	write_cmos_sensor_twobyte(0x6F12, 0x0217);
	write_cmos_sensor_twobyte(0x6F12, 0x018B);
	write_cmos_sensor_twobyte(0x602A, 0x0F1A);
	write_cmos_sensor_twobyte(0x6F12, 0x024D);
	write_cmos_sensor_twobyte(0x6F12, 0x022D);
	write_cmos_sensor_twobyte(0x6F12, 0x01A0);
	write_cmos_sensor_twobyte(0x602A, 0x0F22);
	write_cmos_sensor_twobyte(0x6F12, 0x024F);
	write_cmos_sensor_twobyte(0x6F12, 0x022F);
	write_cmos_sensor_twobyte(0x6F12, 0x01A2);
	write_cmos_sensor_twobyte(0x602A, 0x0F2A);
	write_cmos_sensor_twobyte(0x6F12, 0x0265);
	write_cmos_sensor_twobyte(0x6F12, 0x0245);
	write_cmos_sensor_twobyte(0x6F12, 0x01B7);
	write_cmos_sensor_twobyte(0x602A, 0x0F32);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x6F12, 0x0247);
	write_cmos_sensor_twobyte(0x6F12, 0x01B9);
	write_cmos_sensor_twobyte(0x602A, 0x0F3A);
	write_cmos_sensor_twobyte(0x6F12, 0x027D);
	write_cmos_sensor_twobyte(0x6F12, 0x025D);
	write_cmos_sensor_twobyte(0x6F12, 0x01CE);
	write_cmos_sensor_twobyte(0x602A, 0x0FB0);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00C2);
	write_cmos_sensor_twobyte(0x6F12, 0x01B2);
	write_cmos_sensor_twobyte(0x6F12, 0x0129);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x00D9);
	write_cmos_sensor_twobyte(0x602A, 0x0FD0);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00C2);
	write_cmos_sensor_twobyte(0x6F12, 0x019E);
	write_cmos_sensor_twobyte(0x6F12, 0x0115);
	write_cmos_sensor_twobyte(0x6F12, 0x0108);
	write_cmos_sensor_twobyte(0x6F12, 0x00C5);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x602A, 0x0FEA);
	write_cmos_sensor_twobyte(0x6F12, 0x0284);
	write_cmos_sensor_twobyte(0x6F12, 0x0263);
	write_cmos_sensor_twobyte(0x6F12, 0x01D7);
	write_cmos_sensor_twobyte(0x6F12, 0x00F7);
	write_cmos_sensor_twobyte(0x6F12, 0x0072);
	write_cmos_sensor_twobyte(0x6F12, 0x006F);
	write_cmos_sensor_twobyte(0x6F12, 0x006F);
	write_cmos_sensor_twobyte(0x602A, 0x1008);
	write_cmos_sensor_twobyte(0x6F12, 0x00FA);
	write_cmos_sensor_twobyte(0x6F12, 0x0075);
	write_cmos_sensor_twobyte(0x6F12, 0x0072);
	write_cmos_sensor_twobyte(0x6F12, 0x0072);
	write_cmos_sensor_twobyte(0x6F12, 0x019A);
	write_cmos_sensor_twobyte(0x6F12, 0x0111);
	write_cmos_sensor_twobyte(0x6F12, 0x0104);
	write_cmos_sensor_twobyte(0x6F12, 0x00C1);
	write_cmos_sensor_twobyte(0x6F12, 0x019D);
	write_cmos_sensor_twobyte(0x6F12, 0x0114);
	write_cmos_sensor_twobyte(0x6F12, 0x0107);
	write_cmos_sensor_twobyte(0x6F12, 0x00C4);
	write_cmos_sensor_twobyte(0x602A, 0x1022);
	write_cmos_sensor_twobyte(0x6F12, 0x027F);
	write_cmos_sensor_twobyte(0x6F12, 0x025E);
	write_cmos_sensor_twobyte(0x6F12, 0x01D2);
	write_cmos_sensor_twobyte(0x602A, 0x102A);
	write_cmos_sensor_twobyte(0x6F12, 0x0282);
	write_cmos_sensor_twobyte(0x6F12, 0x0261);
	write_cmos_sensor_twobyte(0x6F12, 0x01D5);
	write_cmos_sensor_twobyte(0x6F12, 0x019A);
	write_cmos_sensor_twobyte(0x6F12, 0x0111);
	write_cmos_sensor_twobyte(0x6F12, 0x0104);
	write_cmos_sensor_twobyte(0x6F12, 0x00C1);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x019F);
	write_cmos_sensor_twobyte(0x6F12, 0x0116);
	write_cmos_sensor_twobyte(0x6F12, 0x0109);
	write_cmos_sensor_twobyte(0x6F12, 0x00C6);
	write_cmos_sensor_twobyte(0x602A, 0x104C);
	write_cmos_sensor_twobyte(0x6F12, 0x0002);
	write_cmos_sensor_twobyte(0x602A, 0x1054);
	write_cmos_sensor_twobyte(0x6F12, 0x01EC);
	write_cmos_sensor_twobyte(0x602A, 0x1058);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x602A, 0x1062);
	write_cmos_sensor_twobyte(0x6F12, 0x0280);
	write_cmos_sensor_twobyte(0x6F12, 0x025F);
	write_cmos_sensor_twobyte(0x6F12, 0x01D3);
	write_cmos_sensor_twobyte(0x602A, 0x1098);
	write_cmos_sensor_twobyte(0x6F12, 0x01BC);
	write_cmos_sensor_twobyte(0x6F12, 0x0133);
	write_cmos_sensor_twobyte(0x6F12, 0x0126);
	write_cmos_sensor_twobyte(0x6F12, 0x00E3);
	write_cmos_sensor_twobyte(0x6F12, 0x01C6);
	write_cmos_sensor_twobyte(0x6F12, 0x013D);
	write_cmos_sensor_twobyte(0x6F12, 0x0130);
	write_cmos_sensor_twobyte(0x6F12, 0x00ED);
	write_cmos_sensor_twobyte(0x602A, 0x10D8);
	write_cmos_sensor_twobyte(0x6F12, 0x019B);
	write_cmos_sensor_twobyte(0x6F12, 0x0112);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x00C2);
	write_cmos_sensor_twobyte(0x6F12, 0x01AD);
	write_cmos_sensor_twobyte(0x6F12, 0x0124);
	write_cmos_sensor_twobyte(0x6F12, 0x0117);
	write_cmos_sensor_twobyte(0x6F12, 0x00D4);
	write_cmos_sensor_twobyte(0x602A, 0x1118);
	write_cmos_sensor_twobyte(0x6F12, 0x01D3);
	write_cmos_sensor_twobyte(0x6F12, 0x014A);
	write_cmos_sensor_twobyte(0x6F12, 0x013D);
	write_cmos_sensor_twobyte(0x6F12, 0x00F5);
	write_cmos_sensor_twobyte(0x602A, 0x115A);
	write_cmos_sensor_twobyte(0x6F12, 0x027F);
	write_cmos_sensor_twobyte(0x6F12, 0x025E);
	write_cmos_sensor_twobyte(0x6F12, 0x01D2);
	write_cmos_sensor_twobyte(0x602A, 0x1198);
	write_cmos_sensor_twobyte(0x6F12, 0x01A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0117);
	write_cmos_sensor_twobyte(0x6F12, 0x010A);
	write_cmos_sensor_twobyte(0x6F12, 0x00C7);
	write_cmos_sensor_twobyte(0x6F12, 0x01B2);
	write_cmos_sensor_twobyte(0x6F12, 0x0129);
	write_cmos_sensor_twobyte(0x6F12, 0x011C);
	write_cmos_sensor_twobyte(0x6F12, 0x00D9);
	write_cmos_sensor_twobyte(0x602A, 0x11B8);
	write_cmos_sensor_twobyte(0x6F12, 0x0014);
	write_cmos_sensor_twobyte(0x6F12, 0x0014);
	write_cmos_sensor_twobyte(0x6F12, 0x0014);
	write_cmos_sensor_twobyte(0x6F12, 0x0014);
	write_cmos_sensor_twobyte(0x6F12, 0x0198);
	write_cmos_sensor_twobyte(0x6F12, 0x010F);
	write_cmos_sensor_twobyte(0x6F12, 0x0102);
	write_cmos_sensor_twobyte(0x6F12, 0x00BF);
	write_cmos_sensor_twobyte(0x6F12, 0x0275);
	write_cmos_sensor_twobyte(0x6F12, 0x01EC);
	write_cmos_sensor_twobyte(0x6F12, 0x01E9);
	write_cmos_sensor_twobyte(0x6F12, 0x0142);
	write_cmos_sensor_twobyte(0x6F12, 0x02C1);
	write_cmos_sensor_twobyte(0x6F12, 0x0258);
	write_cmos_sensor_twobyte(0x6F12, 0x0214);
	write_cmos_sensor_twobyte(0x6F12, 0x018A);
	write_cmos_sensor_twobyte(0x602A, 0x1258);
	write_cmos_sensor_twobyte(0x6F12, 0x000B);
	write_cmos_sensor_twobyte(0x6F12, 0x000B);
	write_cmos_sensor_twobyte(0x6F12, 0x000B);
	write_cmos_sensor_twobyte(0x6F12, 0x000B);
	write_cmos_sensor_twobyte(0x602A, 0x1268);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x00C1);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0x0013);
	write_cmos_sensor_twobyte(0x6F12, 0x00CB);
	write_cmos_sensor_twobyte(0x6F12, 0x01B5);
	write_cmos_sensor_twobyte(0x6F12, 0x01B5);
	write_cmos_sensor_twobyte(0x6F12, 0x0107);
	write_cmos_sensor_twobyte(0x6F12, 0x00D5);
	write_cmos_sensor_twobyte(0x6F12, 0x01BF);
	write_cmos_sensor_twobyte(0x6F12, 0x01BF);
	write_cmos_sensor_twobyte(0x6F12, 0x0111);
	write_cmos_sensor_twobyte(0x6F12, 0x00DF);
	write_cmos_sensor_twobyte(0x6F12, 0x01C9);
	write_cmos_sensor_twobyte(0x6F12, 0x01C9);
	write_cmos_sensor_twobyte(0x6F12, 0x011B);
	write_cmos_sensor_twobyte(0x6F12, 0x00E9);
	write_cmos_sensor_twobyte(0x6F12, 0x01D3);
	write_cmos_sensor_twobyte(0x6F12, 0x01D3);
	write_cmos_sensor_twobyte(0x6F12, 0x0125);
	write_cmos_sensor_twobyte(0x6F12, 0x00F3);
	write_cmos_sensor_twobyte(0x6F12, 0x01DD);
	write_cmos_sensor_twobyte(0x6F12, 0x01DD);
	write_cmos_sensor_twobyte(0x6F12, 0x012F);
	write_cmos_sensor_twobyte(0x6F12, 0x00FD);
	write_cmos_sensor_twobyte(0x6F12, 0x01E7);
	write_cmos_sensor_twobyte(0x6F12, 0x01E7);
	write_cmos_sensor_twobyte(0x6F12, 0x0139);
	write_cmos_sensor_twobyte(0x6F12, 0x0107);
	write_cmos_sensor_twobyte(0x6F12, 0x01F1);
	write_cmos_sensor_twobyte(0x6F12, 0x01F1);
	write_cmos_sensor_twobyte(0x6F12, 0x0143);
	write_cmos_sensor_twobyte(0x6F12, 0x0111);
	write_cmos_sensor_twobyte(0x6F12, 0x01FB);
	write_cmos_sensor_twobyte(0x6F12, 0x01FB);
	write_cmos_sensor_twobyte(0x6F12, 0x014D);
	write_cmos_sensor_twobyte(0x6F12, 0x011B);
	write_cmos_sensor_twobyte(0x6F12, 0x0205);
	write_cmos_sensor_twobyte(0x6F12, 0x0205);
	write_cmos_sensor_twobyte(0x6F12, 0x0157);
	write_cmos_sensor_twobyte(0x6F12, 0x0125);
	write_cmos_sensor_twobyte(0x6F12, 0x020F);
	write_cmos_sensor_twobyte(0x6F12, 0x020F);
	write_cmos_sensor_twobyte(0x6F12, 0x0161);
	write_cmos_sensor_twobyte(0x6F12, 0x012F);
	write_cmos_sensor_twobyte(0x6F12, 0x0219);
	write_cmos_sensor_twobyte(0x6F12, 0x0219);
	write_cmos_sensor_twobyte(0x6F12, 0x016B);
	write_cmos_sensor_twobyte(0x6F12, 0x0139);
	write_cmos_sensor_twobyte(0x6F12, 0x0223);
	write_cmos_sensor_twobyte(0x6F12, 0x0223);
	write_cmos_sensor_twobyte(0x602A, 0x12D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0143);
	write_cmos_sensor_twobyte(0x6F12, 0x022D);
	write_cmos_sensor_twobyte(0x6F12, 0x022D);
	write_cmos_sensor_twobyte(0x602A, 0x12DE);
	write_cmos_sensor_twobyte(0x6F12, 0x014D);
	write_cmos_sensor_twobyte(0x6F12, 0x0237);
	write_cmos_sensor_twobyte(0x6F12, 0x0237);
	write_cmos_sensor_twobyte(0x6F12, 0x0189);
	write_cmos_sensor_twobyte(0x6F12, 0x0157);
	write_cmos_sensor_twobyte(0x6F12, 0x0241);
	write_cmos_sensor_twobyte(0x6F12, 0x0241);
	write_cmos_sensor_twobyte(0x6F12, 0x0193);
	write_cmos_sensor_twobyte(0x6F12, 0x0161);
	write_cmos_sensor_twobyte(0x6F12, 0x024B);
	write_cmos_sensor_twobyte(0x6F12, 0x024B);
	write_cmos_sensor_twobyte(0x6F12, 0x019D);
	write_cmos_sensor_twobyte(0x6F12, 0x016B);
	write_cmos_sensor_twobyte(0x6F12, 0x0255);
	write_cmos_sensor_twobyte(0x6F12, 0x0255);
	write_cmos_sensor_twobyte(0x6F12, 0x01A7);
	write_cmos_sensor_twobyte(0x6F12, 0x0175);
	write_cmos_sensor_twobyte(0x6F12, 0x025F);
	write_cmos_sensor_twobyte(0x6F12, 0x025F);
	write_cmos_sensor_twobyte(0x6F12, 0x01B1);
	write_cmos_sensor_twobyte(0x6F12, 0x017F);
	write_cmos_sensor_twobyte(0x6F12, 0x0269);
	write_cmos_sensor_twobyte(0x6F12, 0x0269);
	write_cmos_sensor_twobyte(0x6F12, 0x01BB);
	write_cmos_sensor_twobyte(0x6F12, 0x0189);
	write_cmos_sensor_twobyte(0x6F12, 0x0273);
	write_cmos_sensor_twobyte(0x6F12, 0x0273);
	write_cmos_sensor_twobyte(0x6F12, 0x01C5);
	write_cmos_sensor_twobyte(0x6F12, 0x0193);
	write_cmos_sensor_twobyte(0x6F12, 0x027D);
	write_cmos_sensor_twobyte(0x6F12, 0x027D);
	write_cmos_sensor_twobyte(0x6F12, 0x01CF);
	write_cmos_sensor_twobyte(0x6F12, 0x019D);
	write_cmos_sensor_twobyte(0x6F12, 0x0287);
	write_cmos_sensor_twobyte(0x6F12, 0x0287);
	write_cmos_sensor_twobyte(0x6F12, 0x01D9);
	write_cmos_sensor_twobyte(0x6F12, 0x01A7);
	write_cmos_sensor_twobyte(0x6F12, 0x0291);
	write_cmos_sensor_twobyte(0x6F12, 0x0291);
	write_cmos_sensor_twobyte(0x6F12, 0x0215);
	write_cmos_sensor_twobyte(0x6F12, 0x01B1);
	write_cmos_sensor_twobyte(0x6F12, 0x029B);
	write_cmos_sensor_twobyte(0x6F12, 0x029B);
	write_cmos_sensor_twobyte(0x6F12, 0x0251);
	write_cmos_sensor_twobyte(0x6F12, 0x01BB);
	write_cmos_sensor_twobyte(0x6F12, 0x01B0);
	write_cmos_sensor_twobyte(0x6F12, 0x01B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0102);
	write_cmos_sensor_twobyte(0x6F12, 0x00D0);
	write_cmos_sensor_twobyte(0x6F12, 0x01C4);
	write_cmos_sensor_twobyte(0x6F12, 0x01C4);
	write_cmos_sensor_twobyte(0x6F12, 0x0116);
	write_cmos_sensor_twobyte(0x6F12, 0x00E4);
	write_cmos_sensor_twobyte(0x6F12, 0x01D8);
	write_cmos_sensor_twobyte(0x6F12, 0x01D8);
	write_cmos_sensor_twobyte(0x6F12, 0x012A);
	write_cmos_sensor_twobyte(0x6F12, 0x00F8);
	write_cmos_sensor_twobyte(0x6F12, 0x01EC);
	write_cmos_sensor_twobyte(0x6F12, 0x01EC);
	write_cmos_sensor_twobyte(0x6F12, 0x013E);
	write_cmos_sensor_twobyte(0x6F12, 0x010C);
	write_cmos_sensor_twobyte(0x6F12, 0x0200);
	write_cmos_sensor_twobyte(0x6F12, 0x0200);
	write_cmos_sensor_twobyte(0x6F12, 0x0152);
	write_cmos_sensor_twobyte(0x6F12, 0x0120);
	write_cmos_sensor_twobyte(0x6F12, 0x0214);
	write_cmos_sensor_twobyte(0x6F12, 0x0214);
	write_cmos_sensor_twobyte(0x6F12, 0x0166);
	write_cmos_sensor_twobyte(0x6F12, 0x0134);
	write_cmos_sensor_twobyte(0x6F12, 0x0228);
	write_cmos_sensor_twobyte(0x6F12, 0x0228);
	write_cmos_sensor_twobyte(0x602A, 0x136E);
	write_cmos_sensor_twobyte(0x6F12, 0x0148);
	write_cmos_sensor_twobyte(0x6F12, 0x023C);
	write_cmos_sensor_twobyte(0x6F12, 0x023C);
	write_cmos_sensor_twobyte(0x6F12, 0x018E);
	write_cmos_sensor_twobyte(0x6F12, 0x015C);
	write_cmos_sensor_twobyte(0x6F12, 0x0250);
	write_cmos_sensor_twobyte(0x6F12, 0x0250);
	write_cmos_sensor_twobyte(0x6F12, 0x01A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0170);
	write_cmos_sensor_twobyte(0x6F12, 0x0264);
	write_cmos_sensor_twobyte(0x6F12, 0x0264);
	write_cmos_sensor_twobyte(0x6F12, 0x01B6);
	write_cmos_sensor_twobyte(0x6F12, 0x0184);
	write_cmos_sensor_twobyte(0x6F12, 0x0278);
	write_cmos_sensor_twobyte(0x6F12, 0x0278);
	write_cmos_sensor_twobyte(0x6F12, 0x01CA);
	write_cmos_sensor_twobyte(0x6F12, 0x0198);
	write_cmos_sensor_twobyte(0x6F12, 0x028C);
	write_cmos_sensor_twobyte(0x6F12, 0x028C);
	write_cmos_sensor_twobyte(0x6F12, 0x01DE);
	write_cmos_sensor_twobyte(0x6F12, 0x01AC);
	write_cmos_sensor_twobyte(0x6F12, 0x02A0);
	write_cmos_sensor_twobyte(0x6F12, 0x02A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0256);
	write_cmos_sensor_twobyte(0x6F12, 0x01C0);
	write_cmos_sensor_twobyte(0x602A, 0x14BC);
	write_cmos_sensor_twobyte(0x6F12, 0x0094);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x6F12, 0x0024);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0x0024);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x602A, 0x14D0);
	write_cmos_sensor_twobyte(0x6F12, 0x019A);
	write_cmos_sensor_twobyte(0x6F12, 0x0111);
	write_cmos_sensor_twobyte(0x6F12, 0x0104);
	write_cmos_sensor_twobyte(0x6F12, 0x00C1);
	write_cmos_sensor_twobyte(0x6F12, 0x0087);
	write_cmos_sensor_twobyte(0x6F12, 0x0065);
	write_cmos_sensor_twobyte(0x6F12, 0x0079);
	write_cmos_sensor_twobyte(0x6F12, 0x0102);
	write_cmos_sensor_twobyte(0x602A, 0x14E2);
	write_cmos_sensor_twobyte(0x6F12, 0x028B);
	write_cmos_sensor_twobyte(0x6F12, 0x0268);
	write_cmos_sensor_twobyte(0x6F12, 0x01D9);
	write_cmos_sensor_twobyte(0x6F12, 0x001A);
	write_cmos_sensor_twobyte(0x6F12, 0x001A);
	write_cmos_sensor_twobyte(0x6F12, 0x001A);
	write_cmos_sensor_twobyte(0x6F12, 0x001A);
	write_cmos_sensor_twobyte(0x602A, 0x14B8);
	write_cmos_sensor_twobyte(0x6F12, 0x00BE);
	write_cmos_sensor_twobyte(0x602A, 0x1508);
	write_cmos_sensor_twobyte(0x6F12, 0x0603);
	write_cmos_sensor_twobyte(0x602A, 0x1550);
	write_cmos_sensor_twobyte(0x6F12, 0x0306);
	write_cmos_sensor_twobyte(0x6F12, 0x0606);
	write_cmos_sensor_twobyte(0x6F12, 0x0606);
	write_cmos_sensor_twobyte(0x6F12, 0x0603);
	write_cmos_sensor_twobyte(0x602A, 0x15DE);
	write_cmos_sensor_twobyte(0x6F12, 0x0107);
	write_cmos_sensor_twobyte(0x3064, 0x0020);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B4A);
	write_cmos_sensor_twobyte(0x6F12, 0x0018);
	write_cmos_sensor_twobyte(0x0110, 0x1002);
	write_cmos_sensor_twobyte(0x0114, 0x0300);
	write_cmos_sensor_twobyte(0x0380, 0x0001);
	write_cmos_sensor_twobyte(0x0384, 0x0001);
	write_cmos_sensor_twobyte(0x0400, 0x0000);
	write_cmos_sensor_twobyte(0x0404, 0x0010);
	write_cmos_sensor_twobyte(0x0302, 0x0001);
	write_cmos_sensor_twobyte(0x0312, 0x0000);
	write_cmos_sensor_twobyte(0x030E, 0x0004);
	write_cmos_sensor_twobyte(0x030A, 0x0001);
	write_cmos_sensor_twobyte(0x0308, 0x0008);
	write_cmos_sensor_twobyte(0x30C0, 0x0001);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x2800);
	write_cmos_sensor_twobyte(0x6F12, 0x0245);
	write_cmos_sensor_twobyte(0x6F12, 0x0105);
	write_cmos_sensor_twobyte(0x602A, 0x2816);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0180);
	write_cmos_sensor_twobyte(0x602A, 0x2824);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x2814);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x0998);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x55BE);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x30C6, 0x0100);
	write_cmos_sensor_twobyte(0x30CA, 0x0300);
	write_cmos_sensor_twobyte(0x30C8, 0x05DC);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x51D0);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x51E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x51F0);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x0B04, 0x0101);
	write_cmos_sensor_twobyte(0x3094, 0x2800);
	write_cmos_sensor_twobyte(0x3096, 0x5400);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0A80);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x51D2);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x51E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0800);
	write_cmos_sensor_twobyte(0x602A, 0x51F2);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x602A, 0x0A50);
	write_cmos_sensor_twobyte(0x6F12, 0x2100);
	write_cmos_sensor_twobyte(0x602A, 0x5150);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x8C76);
	write_cmos_sensor_twobyte(0x6F12, 0x0189);
	write_cmos_sensor_twobyte(0x6F12, 0xFFFF);
	write_cmos_sensor_twobyte(0x602A, 0x51D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0032);
	write_cmos_sensor_twobyte(0x602A, 0x51E8);
	write_cmos_sensor_twobyte(0x6F12, 0x0032);
	write_cmos_sensor_twobyte(0x602A, 0x51F8);
	write_cmos_sensor_twobyte(0x6F12, 0x0032);
	write_cmos_sensor_twobyte(0x602A, 0x5688);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x6F12, 0x010A);
	write_cmos_sensor_twobyte(0x6F12, 0x0132);
	write_cmos_sensor_twobyte(0x6F12, 0x0259);
	write_cmos_sensor_twobyte(0x6F12, 0x0075);
	write_cmos_sensor_twobyte(0x6F12, 0x03FF);
	write_cmos_sensor_twobyte(0x602A, 0x56A4);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x602A, 0x56B2);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x602A, 0x56B8);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x602A, 0x56BC);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x0FFF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x0FFF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x602A, 0x56CC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x6F12, 0x010A);
	write_cmos_sensor_twobyte(0x6F12, 0x0132);
	write_cmos_sensor_twobyte(0x6F12, 0x0259);
	write_cmos_sensor_twobyte(0x6F12, 0x0075);
	write_cmos_sensor_twobyte(0x6F12, 0x03FF);
	write_cmos_sensor_twobyte(0x602A, 0x56E8);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x6F12, 0x001F);
	write_cmos_sensor_twobyte(0x602A, 0x56F6);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x602A, 0x56FC);
	write_cmos_sensor_twobyte(0x6F12, 0x002D);
	write_cmos_sensor_twobyte(0x602A, 0x5700);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x0FFF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x0FFF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x602A, 0x0AF4);
	write_cmos_sensor_twobyte(0x6F12, 0x0003);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0X6028, 0X4000);	// 0x4000 page
	write_cmos_sensor_twobyte(0X30BE, 0X0000);	// ULPM Disable
	LOG_INF("End s5k2l2 sensor_init.\n");
}

#ifndef MARK_HDR
static void capture_setting_WDR(kal_uint16 currefps)
{
	write_cmos_sensor_twobyte(0x0100, 0x0000);
	LOG_INF("Capture WDR(fps = %d)", currefps);
	while (1) {
	if (read_cmos_sensor(0x0005) == 0xFF)
		break;
}

	#ifdef SENSOR_M1
write_cmos_sensor_twobyte(0X6028, 0X4000);
write_cmos_sensor_twobyte(0X6214, 0X7970);
write_cmos_sensor_twobyte(0X6218, 0X7150);
write_cmos_sensor_twobyte(0X0344, 0X0000);
write_cmos_sensor_twobyte(0X0346, 0X0000);
write_cmos_sensor_twobyte(0X0348, 0X1F7F);
write_cmos_sensor_twobyte(0X034A, 0X0BDF);
write_cmos_sensor_twobyte(0X034C, 0X1F80);
write_cmos_sensor_twobyte(0X034E, 0X0BD0);
write_cmos_sensor_twobyte(0X0408, 0X0000);
write_cmos_sensor_twobyte(0X040A, 0X0008);
write_cmos_sensor_twobyte(0X0900, 0X0011);
write_cmos_sensor_twobyte(0X0380, 0X0001);
write_cmos_sensor_twobyte(0X0382, 0X0001);
write_cmos_sensor_twobyte(0X0384, 0X0001);
write_cmos_sensor_twobyte(0X0386, 0X0001);
write_cmos_sensor_twobyte(0X0400, 0X0000);
write_cmos_sensor_twobyte(0X0404, 0X0010);
write_cmos_sensor_twobyte(0X3060, 0X0100);
write_cmos_sensor_twobyte(0X0114, 0X0300);
write_cmos_sensor_twobyte(0X0110, 0X1002);
write_cmos_sensor_twobyte(0X0136, 0X1800);
write_cmos_sensor_twobyte(0X0304, 0X0006);
write_cmos_sensor_twobyte(0X0306, 0X01E0);
write_cmos_sensor_twobyte(0X0302, 0X0001);
write_cmos_sensor_twobyte(0X0300, 0X0004);
write_cmos_sensor_twobyte(0X030C, 0X0001);
write_cmos_sensor_twobyte(0X030E, 0X0004);
write_cmos_sensor_twobyte(0X0310, 0X0153);
write_cmos_sensor_twobyte(0X0312, 0X0000);
write_cmos_sensor_twobyte(0X030A, 0X0001);
write_cmos_sensor_twobyte(0X0308, 0X0008);
write_cmos_sensor_twobyte(0X0342, 0X27E0);
write_cmos_sensor_twobyte(0X0340, 0x0C3E);
write_cmos_sensor_twobyte(0X021E, 0X0000);
write_cmos_sensor_twobyte(0X3098, 0X0400);
write_cmos_sensor_twobyte(0X309A, 0X0002);
write_cmos_sensor_twobyte(0X30BC, 0X0031);
write_cmos_sensor_twobyte(0X30A8, 0X0000);
write_cmos_sensor_twobyte(0X30AC, 0X0000);
write_cmos_sensor_twobyte(0X30A0, 0X0000);
write_cmos_sensor_twobyte(0X30A4, 0X0000);
write_cmos_sensor_twobyte(0XF41E, 0X2180);
write_cmos_sensor_twobyte(0X6028, 0X2000);
write_cmos_sensor_twobyte(0X602A, 0X0990);
write_cmos_sensor_twobyte(0X6F12, 0X0020);
write_cmos_sensor_twobyte(0X602A, 0X0AF8);
write_cmos_sensor_twobyte(0X6F12, 0X0004);
write_cmos_sensor_twobyte(0X602A, 0X27A8);
write_cmos_sensor_twobyte(0X6F12, 0X0100);
write_cmos_sensor_twobyte(0X602A, 0X09AA);
write_cmos_sensor_twobyte(0X6F12, 0X1E7F);
write_cmos_sensor_twobyte(0X6F12, 0X1E7F);
write_cmos_sensor_twobyte(0X602A, 0X16B6);
write_cmos_sensor_twobyte(0X6F12, 0X122F);
write_cmos_sensor_twobyte(0X6F12, 0X4328);
write_cmos_sensor_twobyte(0X602A, 0X1688);
write_cmos_sensor_twobyte(0X6F12, 0X00A2);
write_cmos_sensor_twobyte(0X602A, 0X168C);
write_cmos_sensor_twobyte(0X6F12, 0X0028);
write_cmos_sensor_twobyte(0X6F12, 0X0030);
write_cmos_sensor_twobyte(0X6F12, 0X0C18);
write_cmos_sensor_twobyte(0X6F12, 0X0C18);
write_cmos_sensor_twobyte(0X6F12, 0X0C28);
write_cmos_sensor_twobyte(0X6F12, 0X0C28);
write_cmos_sensor_twobyte(0X6F12, 0X0C20);
write_cmos_sensor_twobyte(0X6F12, 0X0C20);
write_cmos_sensor_twobyte(0X6F12, 0X0C30);
write_cmos_sensor_twobyte(0X6F12, 0X0C30);
write_cmos_sensor_twobyte(0X602A, 0X15F4);
write_cmos_sensor_twobyte(0X6F12, 0X0004);
write_cmos_sensor_twobyte(0X602A, 0X16BE);
write_cmos_sensor_twobyte(0X6F12, 0X06C0);
write_cmos_sensor_twobyte(0X6F12, 0X0101);
write_cmos_sensor_twobyte(0X602A, 0X0B16);
write_cmos_sensor_twobyte(0X6F12, 0X0200);
write_cmos_sensor_twobyte(0X602A, 0X11D2);
write_cmos_sensor_twobyte(0X6F12, 0X0258);
write_cmos_sensor_twobyte(0X602A, 0X127A);
write_cmos_sensor_twobyte(0X6F12, 0X01B5);
write_cmos_sensor_twobyte(0X602A, 0X1282);
write_cmos_sensor_twobyte(0X6F12, 0X01BF);
write_cmos_sensor_twobyte(0X602A, 0X128A);
write_cmos_sensor_twobyte(0X6F12, 0X01C9);
write_cmos_sensor_twobyte(0X602A, 0X1292);
write_cmos_sensor_twobyte(0X6F12, 0X01D3);
write_cmos_sensor_twobyte(0X602A, 0X129A);
write_cmos_sensor_twobyte(0X6F12, 0X01DD);
write_cmos_sensor_twobyte(0X602A, 0X12A2);
write_cmos_sensor_twobyte(0X6F12, 0X01E7);
write_cmos_sensor_twobyte(0X602A, 0X12AA);
write_cmos_sensor_twobyte(0X6F12, 0X01F1);
write_cmos_sensor_twobyte(0X602A, 0X12B2);
write_cmos_sensor_twobyte(0X6F12, 0X01FB);
write_cmos_sensor_twobyte(0X602A, 0X12BA);
write_cmos_sensor_twobyte(0X6F12, 0X0205);
write_cmos_sensor_twobyte(0X602A, 0X12C2);
write_cmos_sensor_twobyte(0X6F12, 0X020F);
write_cmos_sensor_twobyte(0X602A, 0X12CA);
write_cmos_sensor_twobyte(0X6F12, 0X0219);
write_cmos_sensor_twobyte(0X602A, 0X12D2);
write_cmos_sensor_twobyte(0X6F12, 0X0223);
write_cmos_sensor_twobyte(0X602A, 0X12DA);
write_cmos_sensor_twobyte(0X6F12, 0X022D);
write_cmos_sensor_twobyte(0X602A, 0X12E2);
write_cmos_sensor_twobyte(0X6F12, 0X0237);
write_cmos_sensor_twobyte(0X602A, 0X12EA);
write_cmos_sensor_twobyte(0X6F12, 0X0241);
write_cmos_sensor_twobyte(0X602A, 0X12F2);
write_cmos_sensor_twobyte(0X6F12, 0X024B);
write_cmos_sensor_twobyte(0X602A, 0X12FA);
write_cmos_sensor_twobyte(0X6F12, 0X0255);
write_cmos_sensor_twobyte(0X602A, 0X1302);
write_cmos_sensor_twobyte(0X6F12, 0X025F);
write_cmos_sensor_twobyte(0X602A, 0X130A);
write_cmos_sensor_twobyte(0X6F12, 0X0269);
write_cmos_sensor_twobyte(0X602A, 0X1312);
write_cmos_sensor_twobyte(0X6F12, 0X0273);
write_cmos_sensor_twobyte(0X602A, 0X131A);
write_cmos_sensor_twobyte(0X6F12, 0X027D);
write_cmos_sensor_twobyte(0X602A, 0X1322);
write_cmos_sensor_twobyte(0X6F12, 0X0287);
write_cmos_sensor_twobyte(0X602A, 0X132A);
write_cmos_sensor_twobyte(0X6F12, 0X0291);
write_cmos_sensor_twobyte(0X602A, 0X1332);
write_cmos_sensor_twobyte(0X6F12, 0X029B);
write_cmos_sensor_twobyte(0X602A, 0X133A);
write_cmos_sensor_twobyte(0X6F12, 0X01B0);
write_cmos_sensor_twobyte(0X602A, 0X1342);
write_cmos_sensor_twobyte(0X6F12, 0X01C4);
write_cmos_sensor_twobyte(0X602A, 0X134A);
write_cmos_sensor_twobyte(0X6F12, 0X01D8);
write_cmos_sensor_twobyte(0X602A, 0X1352);
write_cmos_sensor_twobyte(0X6F12, 0X01EC);
write_cmos_sensor_twobyte(0X602A, 0X135A);
write_cmos_sensor_twobyte(0X6F12, 0X0200);
write_cmos_sensor_twobyte(0X602A, 0X1362);
write_cmos_sensor_twobyte(0X6F12, 0X0214);
write_cmos_sensor_twobyte(0X602A, 0X136A);
write_cmos_sensor_twobyte(0X6F12, 0X0228);
write_cmos_sensor_twobyte(0X602A, 0X1372);
write_cmos_sensor_twobyte(0X6F12, 0X023C);
write_cmos_sensor_twobyte(0X602A, 0X137A);
write_cmos_sensor_twobyte(0X6F12, 0X0250);
write_cmos_sensor_twobyte(0X602A, 0X1382);
write_cmos_sensor_twobyte(0X6F12, 0X0264);
write_cmos_sensor_twobyte(0X602A, 0X138A);
write_cmos_sensor_twobyte(0X6F12, 0X0278);
write_cmos_sensor_twobyte(0X602A, 0X1392);
write_cmos_sensor_twobyte(0X6F12, 0X028C);
write_cmos_sensor_twobyte(0X602A, 0X139A);
write_cmos_sensor_twobyte(0X6F12, 0X02A0);
write_cmos_sensor_twobyte(0X602A, 0X14BA);
write_cmos_sensor_twobyte(0X6F12, 0X0096);
write_cmos_sensor_twobyte(0X602A, 0X0C22);
write_cmos_sensor_twobyte(0X6F12, 0X000A);
write_cmos_sensor_twobyte(0X602A, 0X15DC);
write_cmos_sensor_twobyte(0X6F12, 0X0001);
write_cmos_sensor_twobyte(0X602A, 0X0A94);
write_cmos_sensor_twobyte(0X6F12, 0X2000);
write_cmos_sensor_twobyte(0X602A, 0X0B62);
write_cmos_sensor_twobyte(0X6F12, 0X0146);
write_cmos_sensor_twobyte(0X602A, 0X8B2C);
write_cmos_sensor_twobyte(0X6F12, 0X0008);
write_cmos_sensor_twobyte(0X602A, 0X5150);
write_cmos_sensor_twobyte(0X6F12, 0X0000);
write_cmos_sensor_twobyte(0X602A, 0X0AE6);
write_cmos_sensor_twobyte(0X6F12, 0X0BE0);
write_cmos_sensor_twobyte(0X602A, 0X29D8);
write_cmos_sensor_twobyte(0X6F12, 0X0BE0);
write_cmos_sensor_twobyte(0X602A, 0X29E2);
write_cmos_sensor_twobyte(0X6F12, 0X0BE0);
write_cmos_sensor_twobyte(0X602A, 0X2958);
write_cmos_sensor_twobyte(0X6F12, 0X0BE0);
write_cmos_sensor_twobyte(0X602A, 0X2998);
write_cmos_sensor_twobyte(0X6F12, 0X0BE0);
write_cmos_sensor_twobyte(0X602A, 0X2962);
write_cmos_sensor_twobyte(0X6F12, 0X0BE0);
write_cmos_sensor_twobyte(0X602A, 0X29A2);
write_cmos_sensor_twobyte(0X6F12, 0X0BE0);
write_cmos_sensor_twobyte(0X6028, 0X4000);
write_cmos_sensor_twobyte(0X6214, 0X79F0);
write_cmos_sensor_twobyte(0X6218, 0X79F0);



#else
/* M2_full_size_setting */
/* TBD */
#endif
	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);

	mDELAY(10);
}
#endif

static void capture_setting_mode1(void)
{
	LOG_INF("enter Capture_setting_mode1");
	/* Stream Off*/
	write_cmos_sensor(0x0100, 0x00);
	check_stremoff();
	sensor_WDR_zhdr();
	/* ues Mode1 setting; fullsize */
	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AA0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x602A, 0x2A98);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0B02);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x09A2);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09A6);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09AA);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x602A, 0x09B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x8C7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1666);
	write_cmos_sensor_twobyte(0x6F12, 0x054C);
	write_cmos_sensor_twobyte(0x602A, 0x16B6);
	write_cmos_sensor_twobyte(0x6F12, 0x122F);
	write_cmos_sensor_twobyte(0x6F12, 0x4328);
	write_cmos_sensor_twobyte(0x602A, 0x1688);
	write_cmos_sensor_twobyte(0x6F12, 0x00A2);
	write_cmos_sensor_twobyte(0x602A, 0x16AA);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x16AE);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x1620);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x1628);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x168C);
	write_cmos_sensor_twobyte(0x6F12, 0x0028);
	write_cmos_sensor_twobyte(0x602A, 0x1694);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x602A, 0x15F4);
	write_cmos_sensor_twobyte(0x6F12, 0x0004);
	write_cmos_sensor_twobyte(0x602A, 0x0BC8);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x602A, 0x16BE);
	write_cmos_sensor_twobyte(0x6F12, 0x06C0);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x602A, 0x0A12);
	write_cmos_sensor_twobyte(0x6F12, 0x0690);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x6F12, 0x06DE);
	write_cmos_sensor_twobyte(0x602A, 0x0B16);
	write_cmos_sensor_twobyte(0x6F12, 0x0200);
	write_cmos_sensor_twobyte(0x602A, 0x15FE);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D6A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D8A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D9A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x14BA);
	write_cmos_sensor_twobyte(0x6F12, 0x0096);
	write_cmos_sensor_twobyte(0x602A, 0x0C10);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0CC8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0CD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0D68);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D88);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D98);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0DA8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DB8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0351);
	write_cmos_sensor_twobyte(0x602A, 0x0E78);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0F10);
	write_cmos_sensor_twobyte(0x6F12, 0x030E);
	write_cmos_sensor_twobyte(0x602A, 0x0F18);
	write_cmos_sensor_twobyte(0x6F12, 0x0324);
	write_cmos_sensor_twobyte(0x602A, 0x0F20);
	write_cmos_sensor_twobyte(0x6F12, 0x0326);
	write_cmos_sensor_twobyte(0x602A, 0x0F28);
	write_cmos_sensor_twobyte(0x6F12, 0x033C);
	write_cmos_sensor_twobyte(0x602A, 0x0F30);
	write_cmos_sensor_twobyte(0x6F12, 0x033E);
	write_cmos_sensor_twobyte(0x602A, 0x0F38);
	write_cmos_sensor_twobyte(0x6F12, 0x0354);
	write_cmos_sensor_twobyte(0x602A, 0x0FE8);
	write_cmos_sensor_twobyte(0x6F12, 0x0357);
	write_cmos_sensor_twobyte(0x602A, 0x1020);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x1028);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x1060);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x1158);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x14E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0362);
	write_cmos_sensor_twobyte(0x602A, 0x15DC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0AEC);
	write_cmos_sensor_twobyte(0x6F12, 0x0207);
	write_cmos_sensor_twobyte(0x602A, 0x0A94);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AE4);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2956);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2996);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2960);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x0AE6);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2958);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2998);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2962);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x0408, 0x0026);
	write_cmos_sensor_twobyte(0x040A, 0x0009);
	write_cmos_sensor_twobyte(0x0344, 0x0000);
	write_cmos_sensor_twobyte(0x0346, 0x0004);
	write_cmos_sensor_twobyte(0x0348, 0x1FC7);
	write_cmos_sensor_twobyte(0x034A, 0x0BDF);
	write_cmos_sensor_twobyte(0x034C, 0x1F80);
	write_cmos_sensor_twobyte(0x034E, 0x0BD0);
	write_cmos_sensor_twobyte(0x0382, 0x0001);
	write_cmos_sensor_twobyte(0x0386, 0x0001);
	write_cmos_sensor_twobyte(0x0900, 0x0011);
	write_cmos_sensor_twobyte(0x3060, 0x0100);
	write_cmos_sensor_twobyte(0x0304, 0x0006);
	write_cmos_sensor_twobyte(0x030C, 0x0001);
	write_cmos_sensor_twobyte(0x0306, 0x01E0);
	write_cmos_sensor_twobyte(0x0300, 0x0004);
	write_cmos_sensor_twobyte(0x0310, 0x0153);
	write_cmos_sensor_twobyte(0x0342, 0x27E0);/*0x27B0);*/
	write_cmos_sensor_twobyte(0x0340, 0x0C3E);/*0x0C4D);*/
	write_cmos_sensor_twobyte(0x30A2, 0x0000);
	write_cmos_sensor_twobyte(0x30A4, 0x0000);
	write_cmos_sensor_twobyte(0x021E, 0x0100);
	write_cmos_sensor_twobyte(0x3098, 0x0400);
	write_cmos_sensor_twobyte(0x309A, 0x0002);
	write_cmos_sensor_twobyte(0x30BC, 0x0031);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B62);
	write_cmos_sensor_twobyte(0x6F12, 0x0102);
	write_cmos_sensor_twobyte(0x30AC, 0x0000);
	write_cmos_sensor_twobyte(0x30AA, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x5840);
	write_cmos_sensor_twobyte(0x6F12, 0x0008);
	write_cmos_sensor_twobyte(0x602A, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0xF466, 0x000E);
	write_cmos_sensor_twobyte(0xF468, 0x000F);
	write_cmos_sensor_twobyte(0xF41E, 0x0180);
	write_cmos_sensor_twobyte(0xF488, 0x0008);
	write_cmos_sensor_twobyte(0xF414, 0x0007);
	write_cmos_sensor_twobyte(0xF416, 0x0004);
	write_cmos_sensor_twobyte(0x6B36, 0x5200);
	write_cmos_sensor_twobyte(0x6B38, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0x0100, 0x0103);

	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);
	mDELAY(5);
	LOG_INF("End setCapture_setting_mode1");
}

static void capture_setting_mode3(void)
{
	LOG_INF("enter capture_setting_mode3");
	/* Stream Off*/
	write_cmos_sensor(0x0100, 0x00);
	check_stremoff();
	/* ues Mode3 setting; fullsize */
	sensor_WDR_zhdr();
	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AA0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x602A, 0x2A98);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0B02);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x09A2);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09A6);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09AA);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x602A, 0x09B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x8C7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1666);
	write_cmos_sensor_twobyte(0x6F12, 0x054C);
	write_cmos_sensor_twobyte(0x602A, 0x16B6);
	write_cmos_sensor_twobyte(0x6F12, 0x122F);
	write_cmos_sensor_twobyte(0x6F12, 0x4328);
	write_cmos_sensor_twobyte(0x602A, 0x1688);
	write_cmos_sensor_twobyte(0x6F12, 0x00A2);
	write_cmos_sensor_twobyte(0x602A, 0x16AA);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x16AE);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x1620);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x1628);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x168C);
	write_cmos_sensor_twobyte(0x6F12, 0x0028);
	write_cmos_sensor_twobyte(0x602A, 0x1694);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x602A, 0x15F4);
	write_cmos_sensor_twobyte(0x6F12, 0x0004);
	write_cmos_sensor_twobyte(0x602A, 0x0BC8);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x602A, 0x16BE);
	write_cmos_sensor_twobyte(0x6F12, 0x06C0);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x602A, 0x0A12);
	write_cmos_sensor_twobyte(0x6F12, 0x0690);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x6F12, 0x06DE);
	write_cmos_sensor_twobyte(0x602A, 0x0B16);
	write_cmos_sensor_twobyte(0x6F12, 0x0200);
	write_cmos_sensor_twobyte(0x602A, 0x15FE);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D6A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D8A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D9A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x14BA);
	write_cmos_sensor_twobyte(0x6F12, 0x0096);
	write_cmos_sensor_twobyte(0x602A, 0x0C10);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0CC8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0CD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0D68);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D88);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D98);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0DA8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DB8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0351);
	write_cmos_sensor_twobyte(0x602A, 0x0E78);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0F10);
	write_cmos_sensor_twobyte(0x6F12, 0x030E);
	write_cmos_sensor_twobyte(0x602A, 0x0F18);
	write_cmos_sensor_twobyte(0x6F12, 0x0324);
	write_cmos_sensor_twobyte(0x602A, 0x0F20);
	write_cmos_sensor_twobyte(0x6F12, 0x0326);
	write_cmos_sensor_twobyte(0x602A, 0x0F28);
	write_cmos_sensor_twobyte(0x6F12, 0x033C);
	write_cmos_sensor_twobyte(0x602A, 0x0F30);
	write_cmos_sensor_twobyte(0x6F12, 0x033E);
	write_cmos_sensor_twobyte(0x602A, 0x0F38);
	write_cmos_sensor_twobyte(0x6F12, 0x0354);
	write_cmos_sensor_twobyte(0x602A, 0x0FE8);
	write_cmos_sensor_twobyte(0x6F12, 0x0357);
	write_cmos_sensor_twobyte(0x602A, 0x1020);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x1028);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x1060);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x1158);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x14E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0362);
	write_cmos_sensor_twobyte(0x602A, 0x15DC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0AEC);
	write_cmos_sensor_twobyte(0x6F12, 0x0207);
	write_cmos_sensor_twobyte(0x602A, 0x0A94);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AE4);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2956);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2996);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2960);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x0AE6);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2958);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2998);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2962);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x0408, 0x0013);
	write_cmos_sensor_twobyte(0x040A, 0x0009);
	write_cmos_sensor_twobyte(0x0344, 0x0000);
	write_cmos_sensor_twobyte(0x0346, 0x0004);
	write_cmos_sensor_twobyte(0x0348, 0x1FC7);
	write_cmos_sensor_twobyte(0x034A, 0x0BDF);
	write_cmos_sensor_twobyte(0x034C, 0x0FC0);
	write_cmos_sensor_twobyte(0x034E, 0x0BD0);
	write_cmos_sensor_twobyte(0x0382, 0x0001);
	write_cmos_sensor_twobyte(0x0386, 0x0001);
	write_cmos_sensor_twobyte(0x0900, 0x0011);
	write_cmos_sensor_twobyte(0x3060, 0x0100);
	write_cmos_sensor_twobyte(0x0304, 0x0006);
	write_cmos_sensor_twobyte(0x030C, 0x0001);
	write_cmos_sensor_twobyte(0x0306, 0x01E0);
	write_cmos_sensor_twobyte(0x0300, 0x0004);
	write_cmos_sensor_twobyte(0x0310, 0x0153);
	write_cmos_sensor_twobyte(0x0342, 0x27B0);/*0x27E0/27B0);*/
	write_cmos_sensor_twobyte(0x0340, 0x0C4D);/*0x0C3E/0C4D);*/
	write_cmos_sensor_twobyte(0x30A2, 0x0FC0);
	write_cmos_sensor_twobyte(0x30A4, 0x02F4);
	write_cmos_sensor_twobyte(0x021E, 0x0100);
	write_cmos_sensor_twobyte(0x3098, 0x0300);
	write_cmos_sensor_twobyte(0x309A, 0x0102);
	write_cmos_sensor_twobyte(0x30BC, 0x012B);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B62);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x30AC, 0x0000);
	write_cmos_sensor_twobyte(0x30AA, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x5840);
	write_cmos_sensor_twobyte(0x6F12, 0x0008);
	write_cmos_sensor_twobyte(0x602A, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0xF466, 0x000E);
	write_cmos_sensor_twobyte(0xF468, 0x000F);
	write_cmos_sensor_twobyte(0xF41E, 0x0180);
	write_cmos_sensor_twobyte(0xF488, 0x0008);
	write_cmos_sensor_twobyte(0xF414, 0x0007);
	write_cmos_sensor_twobyte(0xF416, 0x0004);
	write_cmos_sensor_twobyte(0x6B36, 0x5200);
	write_cmos_sensor_twobyte(0x6B38, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0x0100, 0x0103);

	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);
	mDELAY(5);
	LOG_INF("End set Capture_setting_mode3");
}

#if 0
/* this is mode2 fullsize setting */
static void capture_setting_mode2(void)
{
	LOG_INF("enter normal_video_setting");
	/* Stream Off*/
	write_cmos_sensor(0x0100, 0x00);
	check_stremoff();
	/* ues Mode2 setting; fullsize */
	sensor_WDR_zhdr();
	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AA0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x602A, 0x2A98);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x0B02);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x09A2);
	write_cmos_sensor_twobyte(0x6F12, 0x003F);
	write_cmos_sensor_twobyte(0x602A, 0x09A6);
	write_cmos_sensor_twobyte(0x6F12, 0x003F);
	write_cmos_sensor_twobyte(0x602A, 0x09AA);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x602A, 0x09B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0xF482);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0xF484);
	write_cmos_sensor_twobyte(0x602A, 0x8C7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1666);
	write_cmos_sensor_twobyte(0x6F12, 0x054C);
	write_cmos_sensor_twobyte(0x602A, 0x16B6);
	write_cmos_sensor_twobyte(0x6F12, 0x122F);
	write_cmos_sensor_twobyte(0x6F12, 0x4328);
	write_cmos_sensor_twobyte(0x602A, 0x1688);
	write_cmos_sensor_twobyte(0x6F12, 0x0082);
	write_cmos_sensor_twobyte(0x602A, 0x16AA);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x16AE);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x1620);
	write_cmos_sensor_twobyte(0x6F12, 0x0808);
	write_cmos_sensor_twobyte(0x602A, 0x1628);
	write_cmos_sensor_twobyte(0x6F12, 0x0808);
	write_cmos_sensor_twobyte(0x602A, 0x168C);
	write_cmos_sensor_twobyte(0x6F12, 0x0028);
	write_cmos_sensor_twobyte(0x602A, 0x1694);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x602A, 0x15F4);
	write_cmos_sensor_twobyte(0x6F12, 0x0004);
	write_cmos_sensor_twobyte(0x602A, 0x0BC8);
	write_cmos_sensor_twobyte(0x6F12, 0x0420);
	write_cmos_sensor_twobyte(0x602A, 0x16BE);
	write_cmos_sensor_twobyte(0x6F12, 0x0800);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x0A12);
	write_cmos_sensor_twobyte(0x6F12, 0x0690);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x6F12, 0x06DE);
	write_cmos_sensor_twobyte(0x602A, 0x0B16);
	write_cmos_sensor_twobyte(0x6F12, 0x0200);
	write_cmos_sensor_twobyte(0x602A, 0x15FE);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D6A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D8A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D9A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x14BA);
	write_cmos_sensor_twobyte(0x6F12, 0x0096);
	write_cmos_sensor_twobyte(0x602A, 0x0C10);
	write_cmos_sensor_twobyte(0x6F12, 0x0359);
	write_cmos_sensor_twobyte(0x602A, 0x0CC8);
	write_cmos_sensor_twobyte(0x6F12, 0x030F);
	write_cmos_sensor_twobyte(0x602A, 0x0CD8);
	write_cmos_sensor_twobyte(0x6F12, 0x030D);
	write_cmos_sensor_twobyte(0x602A, 0x0D68);
	write_cmos_sensor_twobyte(0x6F12, 0x0359);
	write_cmos_sensor_twobyte(0x602A, 0x0D88);
	write_cmos_sensor_twobyte(0x6F12, 0x0359);
	write_cmos_sensor_twobyte(0x602A, 0x0D98);
	write_cmos_sensor_twobyte(0x6F12, 0x0359);
	write_cmos_sensor_twobyte(0x602A, 0x0DA8);
	write_cmos_sensor_twobyte(0x6F12, 0x030F);
	write_cmos_sensor_twobyte(0x602A, 0x0DB8);
	write_cmos_sensor_twobyte(0x6F12, 0x030F);
	write_cmos_sensor_twobyte(0x602A, 0x0DD8);
	write_cmos_sensor_twobyte(0x6F12, 0x030B);
	write_cmos_sensor_twobyte(0x602A, 0x0E78);
	write_cmos_sensor_twobyte(0x6F12, 0x030D);
	write_cmos_sensor_twobyte(0x602A, 0x0F10);
	write_cmos_sensor_twobyte(0x6F12, 0x02C8);
	write_cmos_sensor_twobyte(0x602A, 0x0F18);
	write_cmos_sensor_twobyte(0x6F12, 0x02DE);
	write_cmos_sensor_twobyte(0x602A, 0x0F20);
	write_cmos_sensor_twobyte(0x6F12, 0x02E0);
	write_cmos_sensor_twobyte(0x602A, 0x0F28);
	write_cmos_sensor_twobyte(0x6F12, 0x02F6);
	write_cmos_sensor_twobyte(0x602A, 0x0F30);
	write_cmos_sensor_twobyte(0x6F12, 0x02F8);
	write_cmos_sensor_twobyte(0x602A, 0x0F38);
	write_cmos_sensor_twobyte(0x6F12, 0x030E);
	write_cmos_sensor_twobyte(0x602A, 0x0FE8);
	write_cmos_sensor_twobyte(0x6F12, 0x0311);
	write_cmos_sensor_twobyte(0x602A, 0x1020);
	write_cmos_sensor_twobyte(0x6F12, 0x030C);
	write_cmos_sensor_twobyte(0x602A, 0x1028);
	write_cmos_sensor_twobyte(0x6F12, 0x030F);
	write_cmos_sensor_twobyte(0x602A, 0x1060);
	write_cmos_sensor_twobyte(0x6F12, 0x030D);
	write_cmos_sensor_twobyte(0x602A, 0x1158);
	write_cmos_sensor_twobyte(0x6F12, 0x030C);
	write_cmos_sensor_twobyte(0x602A, 0x14E0);
	write_cmos_sensor_twobyte(0x6F12, 0x031C);
	write_cmos_sensor_twobyte(0x602A, 0x15DC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0AEC);
	write_cmos_sensor_twobyte(0x6F12, 0x0207);
	write_cmos_sensor_twobyte(0x602A, 0x0A94);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AE4);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2956);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2996);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2960);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x0AE6);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2958);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2998);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2962);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x0408, 0x0013);
	write_cmos_sensor_twobyte(0x040A, 0x0009);
	write_cmos_sensor_twobyte(0x0344, 0x0000);
	write_cmos_sensor_twobyte(0x0346, 0x0004);
	write_cmos_sensor_twobyte(0x0348, 0x1FCF);
	write_cmos_sensor_twobyte(0x034A, 0x0BDF);
	write_cmos_sensor_twobyte(0x034C, 0x0FC0);
	write_cmos_sensor_twobyte(0x034E, 0x0BD0);
	write_cmos_sensor_twobyte(0x0382, 0x0001);
	write_cmos_sensor_twobyte(0x0386, 0x0001);
	write_cmos_sensor_twobyte(0x0900, 0x0221);
	write_cmos_sensor_twobyte(0x3060, 0x0100);
	write_cmos_sensor_twobyte(0x0304, 0x0006);
	write_cmos_sensor_twobyte(0x030C, 0x0001);
	write_cmos_sensor_twobyte(0x0306, 0x01E0);
	write_cmos_sensor_twobyte(0x0300, 0x0004);
	write_cmos_sensor_twobyte(0x0310, 0x0157);
	write_cmos_sensor_twobyte(0x0342, 0x1960);
	write_cmos_sensor_twobyte(0x0340, 0x133E);
	write_cmos_sensor_twobyte(0x30A2, 0x0000);
	write_cmos_sensor_twobyte(0x30A4, 0x0000);
	write_cmos_sensor_twobyte(0x021E, 0x0000);
	write_cmos_sensor_twobyte(0x3098, 0x0100);
	write_cmos_sensor_twobyte(0x309A, 0x0002);
	write_cmos_sensor_twobyte(0x30BC, 0x0031);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B62);
	write_cmos_sensor_twobyte(0x6F12, 0x0040);
	write_cmos_sensor_twobyte(0x30AC, 0x0000);
	write_cmos_sensor_twobyte(0x30AA, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x5840);
	write_cmos_sensor_twobyte(0x6F12, 0x0008);
	write_cmos_sensor_twobyte(0x602A, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0xF466, 0x000E);
	write_cmos_sensor_twobyte(0xF468, 0x000F);
	write_cmos_sensor_twobyte(0xF41E, 0x2100);
	write_cmos_sensor_twobyte(0xF488, 0x0008);
	write_cmos_sensor_twobyte(0xF414, 0x0007);
	write_cmos_sensor_twobyte(0xF416, 0x0004);
	write_cmos_sensor_twobyte(0x6B36, 0x5200);
	write_cmos_sensor_twobyte(0x6B38, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0x0100, 0x0103);

	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);
	mDELAY(10);
}
#endif

static void capture_setting(void)
{
	if (USE_MODE3 == 1)
		capture_setting_mode3();
	else
		capture_setting_mode1();
}

static void preview_setting(void)
{
	LOG_INF("enter preview setting using capture_setting.\n");
	capture_setting();
}

static void normal_video_setting(void)
{
	LOG_INF("enter normal_video setting using capture_setting.\n");
	capture_setting();
}

static void hs_video_setting_11(void)
{
	LOG_INF("hs_video_setting_11");

/* Stream Off */
write_cmos_sensor(0x0100, 0x00);
while (1) {
	if (read_cmos_sensor(0x0005) == 0xFF)
		break;
}

write_cmos_sensor_twobyte(0X6028, 0X4000);
write_cmos_sensor_twobyte(0X6214, 0X7970);
write_cmos_sensor_twobyte(0X6218, 0X7150);
write_cmos_sensor_twobyte(0X0344, 0X0000);
write_cmos_sensor_twobyte(0X0346, 0X016E);
write_cmos_sensor_twobyte(0X0348, 0X1F7F);
write_cmos_sensor_twobyte(0X034A, 0X0A61);
write_cmos_sensor_twobyte(0X034C, 0X0540);
write_cmos_sensor_twobyte(0X034E, 0X02F4);
write_cmos_sensor_twobyte(0X0408, 0X0000);
write_cmos_sensor_twobyte(0X040A, 0X0004);
write_cmos_sensor_twobyte(0X0900, 0X0113);
write_cmos_sensor_twobyte(0X0380, 0X0001);
write_cmos_sensor_twobyte(0X0382, 0X0001);
write_cmos_sensor_twobyte(0X0384, 0X0001);
write_cmos_sensor_twobyte(0X0386, 0X0005);
write_cmos_sensor_twobyte(0X0400, 0X0000);
write_cmos_sensor_twobyte(0X0404, 0X0010);
write_cmos_sensor_twobyte(0X3060, 0X0103);
write_cmos_sensor_twobyte(0X0114, 0X0300);
write_cmos_sensor_twobyte(0X0110, 0X1002);
write_cmos_sensor_twobyte(0X0136, 0X1800);
write_cmos_sensor_twobyte(0X0304, 0X0006);
write_cmos_sensor_twobyte(0X0306, 0X01E0);
write_cmos_sensor_twobyte(0X0302, 0X0001);
write_cmos_sensor_twobyte(0X0300, 0X0004);
write_cmos_sensor_twobyte(0X030C, 0X0001);
write_cmos_sensor_twobyte(0X030E, 0X0004);
write_cmos_sensor_twobyte(0X0310, 0X012C);
write_cmos_sensor_twobyte(0X0312, 0X0001);
write_cmos_sensor_twobyte(0X030A, 0X0001);
write_cmos_sensor_twobyte(0X0308, 0X0008);
write_cmos_sensor_twobyte(0X0342, 0X1E60);
write_cmos_sensor_twobyte(0X0340, 0X0404);
write_cmos_sensor_twobyte(0X021E, 0X0000);
write_cmos_sensor_twobyte(0X3098, 0X0300);
write_cmos_sensor_twobyte(0X309A, 0X0002);
write_cmos_sensor_twobyte(0X30BC, 0X0031);
write_cmos_sensor_twobyte(0X30A8, 0X0000);
write_cmos_sensor_twobyte(0X30AC, 0X0000);
write_cmos_sensor_twobyte(0X30A0, 0X0000);
write_cmos_sensor_twobyte(0X30A4, 0X0000);
write_cmos_sensor_twobyte(0XF41E, 0X2100);
write_cmos_sensor_twobyte(0X6028, 0X2000);
write_cmos_sensor_twobyte(0X602A, 0X0990);
write_cmos_sensor_twobyte(0X6F12, 0X0040);
write_cmos_sensor_twobyte(0X602A, 0X0AF8);
write_cmos_sensor_twobyte(0X6F12, 0X000C);
write_cmos_sensor_twobyte(0X602A, 0X27A8);
write_cmos_sensor_twobyte(0X6F12, 0X0000);
write_cmos_sensor_twobyte(0X602A, 0X09AA);
write_cmos_sensor_twobyte(0X6F12, 0X0F7F);
write_cmos_sensor_twobyte(0X6F12, 0X0F7F);
write_cmos_sensor_twobyte(0X602A, 0X16B6);
write_cmos_sensor_twobyte(0X6F12, 0X122F);
write_cmos_sensor_twobyte(0X6F12, 0X0328);
write_cmos_sensor_twobyte(0X602A, 0X1688);
write_cmos_sensor_twobyte(0X6F12, 0X00A0);
write_cmos_sensor_twobyte(0X602A, 0X168C);
write_cmos_sensor_twobyte(0X6F12, 0X0028);
write_cmos_sensor_twobyte(0X6F12, 0X0034);
write_cmos_sensor_twobyte(0X6F12, 0X0C16);
write_cmos_sensor_twobyte(0X6F12, 0X0C16);
write_cmos_sensor_twobyte(0X6F12, 0X0C1C);
write_cmos_sensor_twobyte(0X6F12, 0X0C1C);
write_cmos_sensor_twobyte(0X6F12, 0X0C22);
write_cmos_sensor_twobyte(0X6F12, 0X0C22);
write_cmos_sensor_twobyte(0X6F12, 0X0C28);
write_cmos_sensor_twobyte(0X6F12, 0X0C28);
write_cmos_sensor_twobyte(0X602A, 0X15F4);
write_cmos_sensor_twobyte(0X6F12, 0X0104);
write_cmos_sensor_twobyte(0X602A, 0X16BE);
write_cmos_sensor_twobyte(0X6F12, 0X0800);
write_cmos_sensor_twobyte(0X6F12, 0X0201);
write_cmos_sensor_twobyte(0X602A, 0X0B16);
write_cmos_sensor_twobyte(0X6F12, 0X0100);
write_cmos_sensor_twobyte(0X602A, 0X11D2);
write_cmos_sensor_twobyte(0X6F12, 0X022C);
write_cmos_sensor_twobyte(0X602A, 0X127A);
write_cmos_sensor_twobyte(0X6F12, 0X0129);
write_cmos_sensor_twobyte(0X602A, 0X1282);
write_cmos_sensor_twobyte(0X6F12, 0X0133);
write_cmos_sensor_twobyte(0X602A, 0X128A);
write_cmos_sensor_twobyte(0X6F12, 0X013D);
write_cmos_sensor_twobyte(0X602A, 0X1292);
write_cmos_sensor_twobyte(0X6F12, 0X0147);
write_cmos_sensor_twobyte(0X602A, 0X129A);
write_cmos_sensor_twobyte(0X6F12, 0X0151);
write_cmos_sensor_twobyte(0X602A, 0X12A2);
write_cmos_sensor_twobyte(0X6F12, 0X015B);
write_cmos_sensor_twobyte(0X602A, 0X12AA);
write_cmos_sensor_twobyte(0X6F12, 0X0165);
write_cmos_sensor_twobyte(0X602A, 0X12B2);
write_cmos_sensor_twobyte(0X6F12, 0X016F);
write_cmos_sensor_twobyte(0X602A, 0X12BA);
write_cmos_sensor_twobyte(0X6F12, 0X0179);
write_cmos_sensor_twobyte(0X602A, 0X12C2);
write_cmos_sensor_twobyte(0X6F12, 0X0183);
write_cmos_sensor_twobyte(0X602A, 0X12CA);
write_cmos_sensor_twobyte(0X6F12, 0X018D);
write_cmos_sensor_twobyte(0X602A, 0X12D2);
write_cmos_sensor_twobyte(0X6F12, 0X0197);
write_cmos_sensor_twobyte(0X602A, 0X12DA);
write_cmos_sensor_twobyte(0X6F12, 0X01A1);
write_cmos_sensor_twobyte(0X602A, 0X12E2);
write_cmos_sensor_twobyte(0X6F12, 0X01AB);
write_cmos_sensor_twobyte(0X602A, 0X12EA);
write_cmos_sensor_twobyte(0X6F12, 0X01B5);
write_cmos_sensor_twobyte(0X602A, 0X12F2);
write_cmos_sensor_twobyte(0X6F12, 0X01BF);
write_cmos_sensor_twobyte(0X602A, 0X12FA);
write_cmos_sensor_twobyte(0X6F12, 0X01C9);
write_cmos_sensor_twobyte(0X602A, 0X1302);
write_cmos_sensor_twobyte(0X6F12, 0X01D3);
write_cmos_sensor_twobyte(0X602A, 0X130A);
write_cmos_sensor_twobyte(0X6F12, 0X01DD);
write_cmos_sensor_twobyte(0X602A, 0X1312);
write_cmos_sensor_twobyte(0X6F12, 0X01E7);
write_cmos_sensor_twobyte(0X602A, 0X131A);
write_cmos_sensor_twobyte(0X6F12, 0X01F1);
write_cmos_sensor_twobyte(0X602A, 0X1322);
write_cmos_sensor_twobyte(0X6F12, 0X01FB);
write_cmos_sensor_twobyte(0X602A, 0X132A);
write_cmos_sensor_twobyte(0X6F12, 0X0205);
write_cmos_sensor_twobyte(0X602A, 0X1332);
write_cmos_sensor_twobyte(0X6F12, 0X020F);
write_cmos_sensor_twobyte(0X602A, 0X133A);
write_cmos_sensor_twobyte(0X6F12, 0X0124);
write_cmos_sensor_twobyte(0X602A, 0X1342);
write_cmos_sensor_twobyte(0X6F12, 0X0138);
write_cmos_sensor_twobyte(0X602A, 0X134A);
write_cmos_sensor_twobyte(0X6F12, 0X014C);
write_cmos_sensor_twobyte(0X602A, 0X1352);
write_cmos_sensor_twobyte(0X6F12, 0X0160);
write_cmos_sensor_twobyte(0X602A, 0X135A);
write_cmos_sensor_twobyte(0X6F12, 0X0174);
write_cmos_sensor_twobyte(0X602A, 0X1362);
write_cmos_sensor_twobyte(0X6F12, 0X0188);
write_cmos_sensor_twobyte(0X602A, 0X136A);
write_cmos_sensor_twobyte(0X6F12, 0X019C);
write_cmos_sensor_twobyte(0X602A, 0X1372);
write_cmos_sensor_twobyte(0X6F12, 0X01B0);
write_cmos_sensor_twobyte(0X602A, 0X137A);
write_cmos_sensor_twobyte(0X6F12, 0X01C4);
write_cmos_sensor_twobyte(0X602A, 0X1382);
write_cmos_sensor_twobyte(0X6F12, 0X01D8);
write_cmos_sensor_twobyte(0X602A, 0X138A);
write_cmos_sensor_twobyte(0X6F12, 0X01EC);
write_cmos_sensor_twobyte(0X602A, 0X1392);
write_cmos_sensor_twobyte(0X6F12, 0X0200);
write_cmos_sensor_twobyte(0X602A, 0X139A);
write_cmos_sensor_twobyte(0X6F12, 0X0214);
write_cmos_sensor_twobyte(0X602A, 0X14BA);
write_cmos_sensor_twobyte(0X6F12, 0X027C);
write_cmos_sensor_twobyte(0X602A, 0X0C22);
write_cmos_sensor_twobyte(0X6F12, 0X0009);
write_cmos_sensor_twobyte(0X602A, 0X15DC);
write_cmos_sensor_twobyte(0X6F12, 0X0006);
write_cmos_sensor_twobyte(0X602A, 0X0A94);
write_cmos_sensor_twobyte(0X6F12, 0X0800);
write_cmos_sensor_twobyte(0X602A, 0X0B62);
write_cmos_sensor_twobyte(0X6F12, 0X0126);
write_cmos_sensor_twobyte(0X602A, 0X8B2C);
write_cmos_sensor_twobyte(0X6F12, 0X000C);
write_cmos_sensor_twobyte(0X602A, 0X5150);
write_cmos_sensor_twobyte(0X6F12, 0X0100);
write_cmos_sensor_twobyte(0X602A, 0X0AE6);
write_cmos_sensor_twobyte(0X6F12, 0X0BD0);
write_cmos_sensor_twobyte(0X602A, 0X29D8);
write_cmos_sensor_twobyte(0X6F12, 0X0BD0);
write_cmos_sensor_twobyte(0X602A, 0X29E2);
write_cmos_sensor_twobyte(0X6F12, 0X0BD0);
write_cmos_sensor_twobyte(0X602A, 0X2958);
write_cmos_sensor_twobyte(0X6F12, 0X0BD0);
write_cmos_sensor_twobyte(0X602A, 0X2998);
write_cmos_sensor_twobyte(0X6F12, 0X0BD0);
write_cmos_sensor_twobyte(0X602A, 0X2962);
write_cmos_sensor_twobyte(0X6F12, 0X0BD0);
write_cmos_sensor_twobyte(0X602A, 0X29A2);
write_cmos_sensor_twobyte(0X6F12, 0X0BD0);
write_cmos_sensor_twobyte(0X6028, 0X4000);
write_cmos_sensor_twobyte(0X6214, 0X79F0);
write_cmos_sensor_twobyte(0X6218, 0X79F0);
/* Stream On */
write_cmos_sensor(0x0100, 0x01);



mDELAY(10);

}


static void custom1_24fps_setting_mode1(void)
{
	LOG_INF("enter custom1_24fps_setting_mode1");
	/* Stream Off*/
	write_cmos_sensor(0x0100, 0x00);
	check_stremoff();
	sensor_WDR_zhdr();
	/* ues Mode1 setting; fullsize */
	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AA0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x602A, 0x2A98);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0B02);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x09A2);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09A6);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09AA);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x602A, 0x09B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x8C7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1666);
	write_cmos_sensor_twobyte(0x6F12, 0x054C);
	write_cmos_sensor_twobyte(0x602A, 0x16B6);
	write_cmos_sensor_twobyte(0x6F12, 0x122F);
	write_cmos_sensor_twobyte(0x6F12, 0x4328);
	write_cmos_sensor_twobyte(0x602A, 0x1688);
	write_cmos_sensor_twobyte(0x6F12, 0x00A2);
	write_cmos_sensor_twobyte(0x602A, 0x16AA);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x16AE);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x1620);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x1628);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x168C);
	write_cmos_sensor_twobyte(0x6F12, 0x0028);
	write_cmos_sensor_twobyte(0x602A, 0x1694);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x602A, 0x15F4);
	write_cmos_sensor_twobyte(0x6F12, 0x0004);
	write_cmos_sensor_twobyte(0x602A, 0x0BC8);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x602A, 0x16BE);
	write_cmos_sensor_twobyte(0x6F12, 0x06C0);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x602A, 0x0A12);
	write_cmos_sensor_twobyte(0x6F12, 0x0690);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x6F12, 0x06DE);
	write_cmos_sensor_twobyte(0x602A, 0x0B16);
	write_cmos_sensor_twobyte(0x6F12, 0x0200);
	write_cmos_sensor_twobyte(0x602A, 0x15FE);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D6A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D8A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D9A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x14BA);
	write_cmos_sensor_twobyte(0x6F12, 0x0096);
	write_cmos_sensor_twobyte(0x602A, 0x0C10);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0CC8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0CD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0D68);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D88);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D98);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0DA8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DB8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0351);
	write_cmos_sensor_twobyte(0x602A, 0x0E78);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0F10);
	write_cmos_sensor_twobyte(0x6F12, 0x030E);
	write_cmos_sensor_twobyte(0x602A, 0x0F18);
	write_cmos_sensor_twobyte(0x6F12, 0x0324);
	write_cmos_sensor_twobyte(0x602A, 0x0F20);
	write_cmos_sensor_twobyte(0x6F12, 0x0326);
	write_cmos_sensor_twobyte(0x602A, 0x0F28);
	write_cmos_sensor_twobyte(0x6F12, 0x033C);
	write_cmos_sensor_twobyte(0x602A, 0x0F30);
	write_cmos_sensor_twobyte(0x6F12, 0x033E);
	write_cmos_sensor_twobyte(0x602A, 0x0F38);
	write_cmos_sensor_twobyte(0x6F12, 0x0354);
	write_cmos_sensor_twobyte(0x602A, 0x0FE8);
	write_cmos_sensor_twobyte(0x6F12, 0x0357);
	write_cmos_sensor_twobyte(0x602A, 0x1020);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x1028);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x1060);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x1158);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x14E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0362);
	write_cmos_sensor_twobyte(0x602A, 0x15DC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0AEC);
	write_cmos_sensor_twobyte(0x6F12, 0x0207);
	write_cmos_sensor_twobyte(0x602A, 0x0A94);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AE4);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2956);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2996);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2960);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x0AE6);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2958);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2998);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2962);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x0408, 0x0026);
	write_cmos_sensor_twobyte(0x040A, 0x0009);
	write_cmos_sensor_twobyte(0x0344, 0x0000);
	write_cmos_sensor_twobyte(0x0346, 0x0004);
	write_cmos_sensor_twobyte(0x0348, 0x1FC7);
	write_cmos_sensor_twobyte(0x034A, 0x0BDF);
	write_cmos_sensor_twobyte(0x034C, 0x1F80);
	write_cmos_sensor_twobyte(0x034E, 0x0BD0);
	write_cmos_sensor_twobyte(0x0382, 0x0001);
	write_cmos_sensor_twobyte(0x0386, 0x0001);
	write_cmos_sensor_twobyte(0x0900, 0x0011);
	write_cmos_sensor_twobyte(0x3060, 0x0100);
	write_cmos_sensor_twobyte(0x0304, 0x0006);
	write_cmos_sensor_twobyte(0x030C, 0x0001);
	write_cmos_sensor_twobyte(0x0306, 0x01E0);
	write_cmos_sensor_twobyte(0x0300, 0x0004);
	write_cmos_sensor_twobyte(0x0310, 0x0153);
	write_cmos_sensor_twobyte(0x0342, 0x27E0);
	write_cmos_sensor_twobyte(0x0340, 0x0F48);/*24fps*/
	write_cmos_sensor_twobyte(0x30A2, 0x0000);
	write_cmos_sensor_twobyte(0x30A4, 0x0000);
	write_cmos_sensor_twobyte(0x021E, 0x0100);
	write_cmos_sensor_twobyte(0x3098, 0x0400);
	write_cmos_sensor_twobyte(0x309A, 0x0002);
	write_cmos_sensor_twobyte(0x30BC, 0x0031);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B62);
	write_cmos_sensor_twobyte(0x6F12, 0x0102);
	write_cmos_sensor_twobyte(0x30AC, 0x0000);
	write_cmos_sensor_twobyte(0x30AA, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x5840);
	write_cmos_sensor_twobyte(0x6F12, 0x0008);
	write_cmos_sensor_twobyte(0x602A, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0xF466, 0x000E);
	write_cmos_sensor_twobyte(0xF468, 0x000F);
	write_cmos_sensor_twobyte(0xF41E, 0x0180);
	write_cmos_sensor_twobyte(0xF488, 0x0008);
	write_cmos_sensor_twobyte(0xF414, 0x0007);
	write_cmos_sensor_twobyte(0xF416, 0x0004);
	write_cmos_sensor_twobyte(0x6B36, 0x5200);
	write_cmos_sensor_twobyte(0x6B38, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0x0100, 0x0103);

	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);
	mDELAY(5);
	LOG_INF("End setCapture_setting_mode1");
}

static void custom1_24fps_setting_mode3(void)
{
	LOG_INF("enter custom1_24fps_setting_mode3");
	/* Stream Off*/
	write_cmos_sensor(0x0100, 0x00);
	check_stremoff();
	/* ues Mode3 setting; fullsize */
	sensor_WDR_zhdr();
	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AA0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x602A, 0x2A98);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0B02);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x09A2);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09A6);
	write_cmos_sensor_twobyte(0x6F12, 0x007F);
	write_cmos_sensor_twobyte(0x602A, 0x09AA);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x6F12, 0x1E7F);
	write_cmos_sensor_twobyte(0x602A, 0x09B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x8C7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1666);
	write_cmos_sensor_twobyte(0x6F12, 0x054C);
	write_cmos_sensor_twobyte(0x602A, 0x16B6);
	write_cmos_sensor_twobyte(0x6F12, 0x122F);
	write_cmos_sensor_twobyte(0x6F12, 0x4328);
	write_cmos_sensor_twobyte(0x602A, 0x1688);
	write_cmos_sensor_twobyte(0x6F12, 0x00A2);
	write_cmos_sensor_twobyte(0x602A, 0x16AA);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x16AE);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x1620);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x1628);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x168C);
	write_cmos_sensor_twobyte(0x6F12, 0x0028);
	write_cmos_sensor_twobyte(0x602A, 0x1694);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x602A, 0x15F4);
	write_cmos_sensor_twobyte(0x6F12, 0x0004);
	write_cmos_sensor_twobyte(0x602A, 0x0BC8);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x602A, 0x16BE);
	write_cmos_sensor_twobyte(0x6F12, 0x06C0);
	write_cmos_sensor_twobyte(0x6F12, 0x0101);
	write_cmos_sensor_twobyte(0x602A, 0x0A12);
	write_cmos_sensor_twobyte(0x6F12, 0x0690);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x6F12, 0x06DE);
	write_cmos_sensor_twobyte(0x602A, 0x0B16);
	write_cmos_sensor_twobyte(0x6F12, 0x0200);
	write_cmos_sensor_twobyte(0x602A, 0x15FE);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D6A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D8A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D9A);
	write_cmos_sensor_twobyte(0x6F12, 0x028A);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x14BA);
	write_cmos_sensor_twobyte(0x6F12, 0x0096);
	write_cmos_sensor_twobyte(0x602A, 0x0C10);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0CC8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0CD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0D68);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D88);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D98);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0DA8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DB8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0351);
	write_cmos_sensor_twobyte(0x602A, 0x0E78);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0F10);
	write_cmos_sensor_twobyte(0x6F12, 0x030E);
	write_cmos_sensor_twobyte(0x602A, 0x0F18);
	write_cmos_sensor_twobyte(0x6F12, 0x0324);
	write_cmos_sensor_twobyte(0x602A, 0x0F20);
	write_cmos_sensor_twobyte(0x6F12, 0x0326);
	write_cmos_sensor_twobyte(0x602A, 0x0F28);
	write_cmos_sensor_twobyte(0x6F12, 0x033C);
	write_cmos_sensor_twobyte(0x602A, 0x0F30);
	write_cmos_sensor_twobyte(0x6F12, 0x033E);
	write_cmos_sensor_twobyte(0x602A, 0x0F38);
	write_cmos_sensor_twobyte(0x6F12, 0x0354);
	write_cmos_sensor_twobyte(0x602A, 0x0FE8);
	write_cmos_sensor_twobyte(0x6F12, 0x0357);
	write_cmos_sensor_twobyte(0x602A, 0x1020);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x1028);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x1060);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x1158);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x14E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0362);
	write_cmos_sensor_twobyte(0x602A, 0x15DC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0AEC);
	write_cmos_sensor_twobyte(0x6F12, 0x0207);
	write_cmos_sensor_twobyte(0x602A, 0x0A94);
	write_cmos_sensor_twobyte(0x6F12, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AE4);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2956);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2996);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2960);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x0AE6);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2958);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2998);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x2962);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE8);
	write_cmos_sensor_twobyte(0x0408, 0x0013);
	write_cmos_sensor_twobyte(0x040A, 0x0009);
	write_cmos_sensor_twobyte(0x0344, 0x0000);
	write_cmos_sensor_twobyte(0x0346, 0x0004);
	write_cmos_sensor_twobyte(0x0348, 0x1FC7);
	write_cmos_sensor_twobyte(0x034A, 0x0BDF);
	write_cmos_sensor_twobyte(0x034C, 0x0FC0);
	write_cmos_sensor_twobyte(0x034E, 0x0BD0);
	write_cmos_sensor_twobyte(0x0382, 0x0001);
	write_cmos_sensor_twobyte(0x0386, 0x0001);
	write_cmos_sensor_twobyte(0x0900, 0x0011);
	write_cmos_sensor_twobyte(0x3060, 0x0100);
	write_cmos_sensor_twobyte(0x0304, 0x0006);
	write_cmos_sensor_twobyte(0x030C, 0x0001);
	write_cmos_sensor_twobyte(0x0306, 0x01E0);
	write_cmos_sensor_twobyte(0x0300, 0x0004);
	write_cmos_sensor_twobyte(0x0310, 0x0153);
	write_cmos_sensor_twobyte(0x0342, 0x27B0);
	write_cmos_sensor_twobyte(0x0340, 0x0F58);/*24fps*/
	write_cmos_sensor_twobyte(0x30A2, 0x0FC0);
	write_cmos_sensor_twobyte(0x30A4, 0x02F4);
	write_cmos_sensor_twobyte(0x021E, 0x0100);
	write_cmos_sensor_twobyte(0x3098, 0x0300);
	write_cmos_sensor_twobyte(0x309A, 0x0102);
	write_cmos_sensor_twobyte(0x30BC, 0x012B);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B62);
	write_cmos_sensor_twobyte(0x6F12, 0x00F0);
	write_cmos_sensor_twobyte(0x30AC, 0x0000);
	write_cmos_sensor_twobyte(0x30AA, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x5840);
	write_cmos_sensor_twobyte(0x6F12, 0x0008);
	write_cmos_sensor_twobyte(0x602A, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0xF466, 0x000E);
	write_cmos_sensor_twobyte(0xF468, 0x000F);
	write_cmos_sensor_twobyte(0xF41E, 0x0180);
	write_cmos_sensor_twobyte(0xF488, 0x0008);
	write_cmos_sensor_twobyte(0xF414, 0x0007);
	write_cmos_sensor_twobyte(0xF416, 0x0004);
	write_cmos_sensor_twobyte(0x6B36, 0x5200);
	write_cmos_sensor_twobyte(0x6B38, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0x0100, 0x0103);

	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);
	mDELAY(5);
	LOG_INF("End set Capture_setting_mode3");
}

static void custom1_setting(void)
{
	if (USE_MODE3 == 1)
		custom1_24fps_setting_mode3();
	else
		custom1_24fps_setting_mode1();
}

static void slim_video_setting(void)
{

	LOG_INF("enter slim_video_setting.\n");
	/* Stream Off*/
	write_cmos_sensor(0x0100, 0x00);
	check_stremoff();
	/* ues Mode2 setting; fullsize */

	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AA0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x602A, 0x2A98);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x0B02);
	write_cmos_sensor_twobyte(0x6F12, 0x0180);
	write_cmos_sensor_twobyte(0x602A, 0x09A2);
	write_cmos_sensor_twobyte(0x6F12, 0x003F);
	write_cmos_sensor_twobyte(0x602A, 0x09A6);
	write_cmos_sensor_twobyte(0x6F12, 0x003F);
	write_cmos_sensor_twobyte(0x602A, 0x09AA);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x602A, 0x09B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0xF482);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0xF484);
	write_cmos_sensor_twobyte(0x602A, 0x8C7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1666);
	write_cmos_sensor_twobyte(0x6F12, 0x044C);
	write_cmos_sensor_twobyte(0x602A, 0x16B6);
	write_cmos_sensor_twobyte(0x6F12, 0x12AF);
	write_cmos_sensor_twobyte(0x6F12, 0x0328);
	write_cmos_sensor_twobyte(0x602A, 0x1688);
	write_cmos_sensor_twobyte(0x6F12, 0x0080);
	write_cmos_sensor_twobyte(0x602A, 0x16AA);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x16AE);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x1620);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x1628);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x168C);
	write_cmos_sensor_twobyte(0x6F12, 0x0028);
	write_cmos_sensor_twobyte(0x602A, 0x1694);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x602A, 0x15F4);
	write_cmos_sensor_twobyte(0x6F12, 0x0104);
	write_cmos_sensor_twobyte(0x602A, 0x0BC8);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x602A, 0x16BE);
	write_cmos_sensor_twobyte(0x6F12, 0x0800);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x0A12);
	write_cmos_sensor_twobyte(0x6F12, 0x0690);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x6F12, 0x06DE);
	write_cmos_sensor_twobyte(0x602A, 0x0B16);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x15FE);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D6A);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D8A);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D9A);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x14BA);
	write_cmos_sensor_twobyte(0x6F12, 0x0096);
	write_cmos_sensor_twobyte(0x602A, 0x0C10);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0CC8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0CD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0D68);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D88);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D98);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0DA8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DB8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0351);
	write_cmos_sensor_twobyte(0x602A, 0x0E78);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0F10);
	write_cmos_sensor_twobyte(0x6F12, 0x030E);
	write_cmos_sensor_twobyte(0x602A, 0x0F18);
	write_cmos_sensor_twobyte(0x6F12, 0x0324);
	write_cmos_sensor_twobyte(0x602A, 0x0F20);
	write_cmos_sensor_twobyte(0x6F12, 0x0326);
	write_cmos_sensor_twobyte(0x602A, 0x0F28);
	write_cmos_sensor_twobyte(0x6F12, 0x033C);
	write_cmos_sensor_twobyte(0x602A, 0x0F30);
	write_cmos_sensor_twobyte(0x6F12, 0x033E);
	write_cmos_sensor_twobyte(0x602A, 0x0F38);
	write_cmos_sensor_twobyte(0x6F12, 0x0354);
	write_cmos_sensor_twobyte(0x602A, 0x0FE8);
	write_cmos_sensor_twobyte(0x6F12, 0x0357);
	write_cmos_sensor_twobyte(0x602A, 0x1020);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x1028);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x1060);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x1158);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x14E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0362);
	write_cmos_sensor_twobyte(0x602A, 0x15DC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0AEC);
	write_cmos_sensor_twobyte(0x6F12, 0x0208);
	write_cmos_sensor_twobyte(0x602A, 0x0A94);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x602A, 0x0AE4);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2956);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2996);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2960);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x0AE6);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x29D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x29E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x2958);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x2998);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x2962);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x29A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x0408, 0x0009);
	write_cmos_sensor_twobyte(0x040A, 0x0003);
	write_cmos_sensor_twobyte(0x0344, 0x0000);
	write_cmos_sensor_twobyte(0x0346, 0x0000);
	write_cmos_sensor_twobyte(0x0348, 0x1FCF);
	write_cmos_sensor_twobyte(0x034A, 0x0BD7);
	write_cmos_sensor_twobyte(0x034C, 0x07E0);
	write_cmos_sensor_twobyte(0x034E, 0x05E8);
	write_cmos_sensor_twobyte(0x0382, 0x0003);
	write_cmos_sensor_twobyte(0x0386, 0x0003);
	write_cmos_sensor_twobyte(0x0900, 0x0242);
	write_cmos_sensor_twobyte(0x3060, 0x0100);
	write_cmos_sensor_twobyte(0x0304, 0x0006);
	write_cmos_sensor_twobyte(0x030C, 0x0001);
	write_cmos_sensor_twobyte(0x0306, 0x01E0);
	write_cmos_sensor_twobyte(0x0300, 0x0004);
	write_cmos_sensor_twobyte(0x0310, 0x00B6);
	write_cmos_sensor_twobyte(0x0342, 0x4F00);
	write_cmos_sensor_twobyte(0x0340, 0x062B);
	write_cmos_sensor_twobyte(0x30A2, 0x0000);
	write_cmos_sensor_twobyte(0x30A4, 0x0000);
	write_cmos_sensor_twobyte(0x021E, 0x0000);
	write_cmos_sensor_twobyte(0x3098, 0x0100);
	write_cmos_sensor_twobyte(0x309A, 0x0002);
	write_cmos_sensor_twobyte(0x30BC, 0x0031);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B62);
	write_cmos_sensor_twobyte(0x6F12, 0x00D0);
	write_cmos_sensor_twobyte(0x30AC, 0x0000);
	write_cmos_sensor_twobyte(0x30AA, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x5840);
	write_cmos_sensor_twobyte(0x6F12, 0x0008);
	write_cmos_sensor_twobyte(0x602A, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x0400);
	write_cmos_sensor_twobyte(0xF466, 0x000E);
	write_cmos_sensor_twobyte(0xF468, 0x000F);
	write_cmos_sensor_twobyte(0xF41E, 0x2100);
	write_cmos_sensor_twobyte(0xF488, 0x0008);
	write_cmos_sensor_twobyte(0xF414, 0x0007);
	write_cmos_sensor_twobyte(0xF416, 0x0004);
	write_cmos_sensor_twobyte(0x6B36, 0x5200);
	write_cmos_sensor_twobyte(0x6B38, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0x0100, 0x0103);

	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);
	mDELAY(10);
}

static void custom2_setting(void)
{

	LOG_INF("enter custom2_setting/24fps.\n");
	/* Stream Off*/
	write_cmos_sensor(0x0100, 0x00);
	check_stremoff();
	/* ues Mode2 setting; fullsize */

	write_cmos_sensor_twobyte(0xFCFC, 0x4000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0AA0);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x7970);
	write_cmos_sensor_twobyte(0x6218, 0x7150);
	write_cmos_sensor_twobyte(0x602A, 0x2A98);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x0B02);
	write_cmos_sensor_twobyte(0x6F12, 0x0180);
	write_cmos_sensor_twobyte(0x602A, 0x09A2);
	write_cmos_sensor_twobyte(0x6F12, 0x003F);
	write_cmos_sensor_twobyte(0x602A, 0x09A6);
	write_cmos_sensor_twobyte(0x6F12, 0x003F);
	write_cmos_sensor_twobyte(0x602A, 0x09AA);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x6F12, 0x07FF);
	write_cmos_sensor_twobyte(0x602A, 0x09B0);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0xF482);
	write_cmos_sensor_twobyte(0x6F12, 0x0009);
	write_cmos_sensor_twobyte(0x6F12, 0x0012);
	write_cmos_sensor_twobyte(0x6F12, 0xF484);
	write_cmos_sensor_twobyte(0x602A, 0x8C7A);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x1666);
	write_cmos_sensor_twobyte(0x6F12, 0x044C);
	write_cmos_sensor_twobyte(0x602A, 0x16B6);
	write_cmos_sensor_twobyte(0x6F12, 0x12AF);
	write_cmos_sensor_twobyte(0x6F12, 0x0328);
	write_cmos_sensor_twobyte(0x602A, 0x1688);
	write_cmos_sensor_twobyte(0x6F12, 0x0080);
	write_cmos_sensor_twobyte(0x602A, 0x16AA);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x16AE);
	write_cmos_sensor_twobyte(0x6F12, 0x2608);
	write_cmos_sensor_twobyte(0x602A, 0x1620);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x1628);
	write_cmos_sensor_twobyte(0x6F12, 0x0408);
	write_cmos_sensor_twobyte(0x602A, 0x168C);
	write_cmos_sensor_twobyte(0x6F12, 0x0028);
	write_cmos_sensor_twobyte(0x602A, 0x1694);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C28);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x6F12, 0x0C20);
	write_cmos_sensor_twobyte(0x602A, 0x15F4);
	write_cmos_sensor_twobyte(0x6F12, 0x0104);
	write_cmos_sensor_twobyte(0x602A, 0x0BC8);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x602A, 0x16BE);
	write_cmos_sensor_twobyte(0x6F12, 0x0800);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x0A12);
	write_cmos_sensor_twobyte(0x6F12, 0x0690);
	write_cmos_sensor_twobyte(0x6F12, 0x06E0);
	write_cmos_sensor_twobyte(0x6F12, 0x06DE);
	write_cmos_sensor_twobyte(0x602A, 0x0B16);
	write_cmos_sensor_twobyte(0x6F12, 0x0100);
	write_cmos_sensor_twobyte(0x602A, 0x15FE);
	write_cmos_sensor_twobyte(0x6F12, 0x0000);
	write_cmos_sensor_twobyte(0x602A, 0x0C12);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D6A);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D8A);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x0D9A);
	write_cmos_sensor_twobyte(0x6F12, 0x0B2D);
	write_cmos_sensor_twobyte(0x6F12, 0x0267);
	write_cmos_sensor_twobyte(0x602A, 0x14BA);
	write_cmos_sensor_twobyte(0x6F12, 0x0096);
	write_cmos_sensor_twobyte(0x602A, 0x0C10);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0CC8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0CD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0D68);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D88);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0D98);
	write_cmos_sensor_twobyte(0x6F12, 0x0361);
	write_cmos_sensor_twobyte(0x602A, 0x0DA8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DB8);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x0DD8);
	write_cmos_sensor_twobyte(0x6F12, 0x0351);
	write_cmos_sensor_twobyte(0x602A, 0x0E78);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x0F10);
	write_cmos_sensor_twobyte(0x6F12, 0x030E);
	write_cmos_sensor_twobyte(0x602A, 0x0F18);
	write_cmos_sensor_twobyte(0x6F12, 0x0324);
	write_cmos_sensor_twobyte(0x602A, 0x0F20);
	write_cmos_sensor_twobyte(0x6F12, 0x0326);
	write_cmos_sensor_twobyte(0x602A, 0x0F28);
	write_cmos_sensor_twobyte(0x6F12, 0x033C);
	write_cmos_sensor_twobyte(0x602A, 0x0F30);
	write_cmos_sensor_twobyte(0x6F12, 0x033E);
	write_cmos_sensor_twobyte(0x602A, 0x0F38);
	write_cmos_sensor_twobyte(0x6F12, 0x0354);
	write_cmos_sensor_twobyte(0x602A, 0x0FE8);
	write_cmos_sensor_twobyte(0x6F12, 0x0357);
	write_cmos_sensor_twobyte(0x602A, 0x1020);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x1028);
	write_cmos_sensor_twobyte(0x6F12, 0x0355);
	write_cmos_sensor_twobyte(0x602A, 0x1060);
	write_cmos_sensor_twobyte(0x6F12, 0x0353);
	write_cmos_sensor_twobyte(0x602A, 0x1158);
	write_cmos_sensor_twobyte(0x6F12, 0x0352);
	write_cmos_sensor_twobyte(0x602A, 0x14E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0362);
	write_cmos_sensor_twobyte(0x602A, 0x15DC);
	write_cmos_sensor_twobyte(0x6F12, 0x0001);
	write_cmos_sensor_twobyte(0x602A, 0x0AEC);
	write_cmos_sensor_twobyte(0x6F12, 0x0208);
	write_cmos_sensor_twobyte(0x602A, 0x0A94);
	write_cmos_sensor_twobyte(0x6F12, 0x1000);
	write_cmos_sensor_twobyte(0x602A, 0x0AE4);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29D6);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29E0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2956);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2996);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x2960);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x29A0);
	write_cmos_sensor_twobyte(0x6F12, 0x0FE8);
	write_cmos_sensor_twobyte(0x602A, 0x0AE6);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x29D8);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x29E2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x2958);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x2998);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x2962);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x602A, 0x29A2);
	write_cmos_sensor_twobyte(0x6F12, 0x0BE0);
	write_cmos_sensor_twobyte(0x0408, 0x0009);
	write_cmos_sensor_twobyte(0x040A, 0x0003);
	write_cmos_sensor_twobyte(0x0344, 0x0000);
	write_cmos_sensor_twobyte(0x0346, 0x0000);
	write_cmos_sensor_twobyte(0x0348, 0x1FCF);
	write_cmos_sensor_twobyte(0x034A, 0x0BD7);
	write_cmos_sensor_twobyte(0x034C, 0x07E0);
	write_cmos_sensor_twobyte(0x034E, 0x05E8);
	write_cmos_sensor_twobyte(0x0382, 0x0003);
	write_cmos_sensor_twobyte(0x0386, 0x0003);
	write_cmos_sensor_twobyte(0x0900, 0x0242);
	write_cmos_sensor_twobyte(0x3060, 0x0100);
	write_cmos_sensor_twobyte(0x0304, 0x0006);
	write_cmos_sensor_twobyte(0x030C, 0x0001);
	write_cmos_sensor_twobyte(0x0306, 0x01E0);
	write_cmos_sensor_twobyte(0x0300, 0x0004);
	write_cmos_sensor_twobyte(0x0310, 0x00B6);
	write_cmos_sensor_twobyte(0x0342, 0x4F00);
	write_cmos_sensor_twobyte(0x0340, 0x07B6);/*24fps*/
	write_cmos_sensor_twobyte(0x30A2, 0x0000);
	write_cmos_sensor_twobyte(0x30A4, 0x0000);
	write_cmos_sensor_twobyte(0x021E, 0x0000);
	write_cmos_sensor_twobyte(0x3098, 0x0100);
	write_cmos_sensor_twobyte(0x309A, 0x0002);
	write_cmos_sensor_twobyte(0x30BC, 0x0031);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x0B62);
	write_cmos_sensor_twobyte(0x6F12, 0x00D0);
	write_cmos_sensor_twobyte(0x30AC, 0x0000);
	write_cmos_sensor_twobyte(0x30AA, 0x0000);
	write_cmos_sensor_twobyte(0x6028, 0x2000);
	write_cmos_sensor_twobyte(0x602A, 0x5840);
	write_cmos_sensor_twobyte(0x6F12, 0x0008);
	write_cmos_sensor_twobyte(0x602A, 0x8C74);
	write_cmos_sensor_twobyte(0x6F12, 0x0400);
	write_cmos_sensor_twobyte(0xF466, 0x000E);
	write_cmos_sensor_twobyte(0xF468, 0x000F);
	write_cmos_sensor_twobyte(0xF41E, 0x2100);
	write_cmos_sensor_twobyte(0xF488, 0x0008);
	write_cmos_sensor_twobyte(0xF414, 0x0007);
	write_cmos_sensor_twobyte(0xF416, 0x0004);
	write_cmos_sensor_twobyte(0x6B36, 0x5200);
	write_cmos_sensor_twobyte(0x6B38, 0x0000);
	write_cmos_sensor_twobyte(0x6214, 0x79F0);
	write_cmos_sensor_twobyte(0x6218, 0x79F0);
	write_cmos_sensor_twobyte(0x0100, 0x0103);

	/* Stream On */
	write_cmos_sensor(0x0100, 0x01);
	mDELAY(10);
}


static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	/********************************************************
	*0x5040[7]: 1 enable,  0 disable
	*0x5040[3:2]; color bar style 00 standard color bar
	*0x5040[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
	********************************************************/

	if (enable)
		write_cmos_sensor(0x0600, 0x000C); /* Grayscale */
	else
		write_cmos_sensor(0x0600, 0x0000); /* Off */

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	 get_imgsensor_id
*
* DESCRIPTION
*	 This function get the sensor ID
*
* PARAMETERS
*	 *sensorID : return the sensor ID
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 5;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {

			*sensor_id = read_cmos_sensor_twobyte(0x0000);
			if (*sensor_id == imgsensor_info.sensor_id || *sensor_id == 0x20C1) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);

				return ERROR_NONE;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id && *sensor_id != 0x20C1) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	 open
*
* DESCRIPTION
*	 This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	 None
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 5;
	kal_uint32 sensor_id = 0;

	LOG_1;
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			write_cmos_sensor_twobyte(0x602C, 0x4000);
			write_cmos_sensor_twobyte(0x602E, 0x0000);
			sensor_id = read_cmos_sensor_twobyte(0x6F12);

			write_cmos_sensor_twobyte(0x602C, 0x4000);
			write_cmos_sensor_twobyte(0x602E, 0x001A);
			chip_id = read_cmos_sensor_twobyte(0x6F12);

			if (sensor_id == imgsensor_info.sensor_id  || sensor_id == 0x20C1) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x, chip_id (0x%x)\n",
							imgsensor.i2c_write_id, sensor_id, chip_id);
				break;
			}
			LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id || sensor_id == 0x20C1)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id && 0x20C1 != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	sensor_init();

#ifdef	USE_OIS
	/* OIS_on(RUMBA_OIS_CAP_SETTING);//pangfei OIS */
	LOG_INF("pangfei capture OIS setting\n");
	OIS_write_cmos_sensor(0x0002, 0x05);
	OIS_write_cmos_sensor(0x0002, 0x00);
	OIS_write_cmos_sensor(0x0000, 0x01);
#endif

	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.hdr_mode = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	 /*    open  */



/*************************************************************************
* FUNCTION
*	 close
*
* DESCRIPTION
*
*
* PARAMETERS
*	 None
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/

	return ERROR_NONE;
}	 /*    close  */


static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	if (imgsensor.current_fps == imgsensor_info.custom1.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	/*PIP24fps_capture_setting(imgsensor.current_fps,imgsensor.pdaf_mode); */
	/* custom1_setting();*/
	custom1_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	if (imgsensor.current_fps == imgsensor_info.custom2.max_framerate) {
		/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.custom2.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	 This function start the sensor preview.
*
* PARAMETERS
*	 *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	/* imgsensor.video_mode = KAL_FALSE; */
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);


#ifdef USE_OIS
	/* OIS_on(RUMBA_OIS_PRE_SETTING);	//pangfei OIS */
	LOG_INF("pangfei preview OIS setting\n");
	OIS_write_cmos_sensor(0x0002, 0x05);
	OIS_write_cmos_sensor(0x0002, 0x00);
	OIS_write_cmos_sensor(0x0000, 0x01);
#endif
	return ERROR_NONE;
}	 /*    preview	 */

/*************************************************************************
* FUNCTION
*	 capture
*
* DESCRIPTION
*	 This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	 None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	/* Mark PIP case for dual pd */
	if (0) {/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
					imgsensor.current_fps, imgsensor_info.cap.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);

#ifdef	USE_OIS
	/* OIS_on(RUMBA_OIS_CAP_SETTING);//pangfei OIS */
	LOG_INF("pangfei capture OIS setting\n");
	OIS_write_cmos_sensor(0x0002, 0x05);
	OIS_write_cmos_sensor(0x0002, 0x00);
	OIS_write_cmos_sensor(0x0000, 0x01);
#endif
	return ERROR_NONE;
	}	 /* capture() */


static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);

	return ERROR_NONE;
}	 /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	if (chip_id == 0x022C)
		hs_video_setting_11();
	else
		hs_video_setting_11();

	set_mirror_flip(IMAGE_HV_MIRROR);
	return ERROR_NONE;
}	 /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_HV_MIRROR);

	return ERROR_NONE;
}	 /*    slim_video	  */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	  = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	= imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;

	return ERROR_NONE;
}	 /*    get_resolution	 */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
#if defined(ENABLE_S5K2L2_PDAF_RAW)
	#if (USE_MODE3 == 1)
		sensor_info->PDAF_Support = 5;
	#else
		sensor_info->PDAF_Support = 4;
	#endif
		/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode(Full), 3:PDAF VC mode(Binning), */
		/*4: PDAF DualPD Raw Data mode, 5: PDAF DualPD VC mode*/
#else
		sensor_info->PDAF_Support = 0;
#endif
	sensor_info->HDR_Support = 3; /*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR*/

	/*0: no support, 1: G0,R0.B0, 2: G0,R0.B1, 3: G0,R1.B0, 4: G0,R1.B1*/
	/*					  5: G1,R0.B0, 6: G1,R0.B1, 7: G1,R1.B0, 8: G1,R1.B1*/
	sensor_info->ZHDR_Mode = 5;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	 /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
					imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
					imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
					imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
					imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
					imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
					imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}	 /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_err("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	 /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{/* This Function not used after ROME */
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
						? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk / framerate * 10
						/ imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength)
							? (frame_length - imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength)
						? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength)
					? (frame_length - imgsensor_info.hs_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength)
					? (frame_length - imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		break;
	default:  /* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength)
					? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT32 sensor_id;
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para=(unsigned long long *) feature_para; */

	SET_PD_BLOCK_INFO_T *PDAFinfo;
	SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	SENSOR_VC_INFO_STRUCT *pvcinfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*feature_id = SENSOR_FEATURE_SET_ESHUTTER(0x3004)&SENSOR_FEATURE_SET_GAIN(0x3006)*/
	if ((feature_id != 0x3004) || (feature_id != 0x3006))
		LOG_INF("feature_id = %d\n", feature_id);

	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
						imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		night_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN:
		set_dual_gain((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data),
						(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("current fps :%d\n", imgsensor.current_fps);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("hdr mode :%d\n", (BOOL)*feature_data);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.hdr_mode = (BOOL)*feature_data;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:/*0x3080*/
		wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[6], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo = (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info, sizeof(SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM2:
		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) (*feature_data));
		pvcinfo = (SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		if (USE_MODE3 == 1) {
			switch (*feature_data_32) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				       sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				       sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3],
				       sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
			case MSDK_SCENARIO_ID_CUSTOM2:
				memset((void *)pvcinfo, 0, sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[5],
				       sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				       sizeof(SENSOR_VC_INFO_STRUCT));
				break;
			}
		} else {
			memset((void *)pvcinfo, 0, sizeof(SENSOR_VC_INFO_STRUCT));
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu\n", *feature_data);

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x09;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM2:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/* PDAF capacity enable or not, s5k2l2 only full size support PDAF */
		switch (*feature_data) {
#if defined(ENABLE_S5K2L2_PDAF_RAW)
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		break;
#else
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
		break;
#endif
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM2:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		get_imgsensor_id(&sensor_id);
		LOG_INF("s5k2l2_read_otp_pdaf_data %x\n", sensor_id);
		s5k2l2_read_otp_pdaf_data((kal_uint16)(*feature_data),
				(char *)(uintptr_t)(*(feature_data+1)), (kal_uint32)(*(feature_data+2)), sensor_id);
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16)*feature_data, (UINT16)*(feature_data + 1));
		break;
#if 0
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
#endif
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
						(UINT16)*feature_data, (UINT16)*(feature_data+1));
		hdr_write_shutter((UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_TWO_PIXEL_SETTING:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1; /*force to set 2pixel mode*/
			LOG_INF("custom1: two pixel mode enable\n");
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
		break;
	default:
		break;
	}

return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K2L2_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	 /*    s5k2l2_MIPI_RAW_SensorInit	 */
