/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include "ssp.h"
#include "alsps.h"
#include "accel.h"
#include "gyroscope.h"
#include "mag.h"

static int dump_sensor_data;
static struct ssp_data *local_data;


#define NUM_SENSOR_INFO_1 20

static void init_sensorlist_1(struct ssp_data *data)
{
	struct sensor_info sensorinfo[NUM_SENSOR_INFO_1] = {
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_ACCELEROMETER,
		SENSOR_INFO_GEOMAGNETIC_FIELD,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_GYRO,
		SENSOR_INFO_LIGHT,
		SENSOR_INFO_PRESSURE,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_PROXIMITY,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_ROTATION_VECTOR,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_MAGNETIC_FIELD_UNCALIBRATED,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_GYRO_UNCALIBRATED,
		SENSOR_INFO_SIGNIFICANT_MOTION,
		SENSOR_INFO_STEP_DETECTOR,
		SENSOR_INFO_STEP_COUNTER,
	};

	memcpy(&data->info, sensorinfo, sizeof(sensorinfo));
}

static void init_sensorlist_2(struct ssp_data *data)
{
	struct sensor_info sensorinfo[SSP_SENSOR_TYPE_MAX-NUM_SENSOR_INFO_1] = {
		SENSOR_INFO_GEOMAGNETIC_ROTATION_VECTOR,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_TILT_DETECTOR,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_PICK_UP_GESTURE,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_META,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_PROXIMITY_RAW,
		SENSOR_INFO_GEOMAGNETIC_POWER,
		SENSOR_INFO_INTERRUPT_GYRO,
	};

	memcpy(&(data->info[NUM_SENSOR_INFO_1]), sensorinfo, sizeof(sensorinfo));
}

static void init_sensorlist(struct ssp_data *data)
{
	init_sensorlist_1(data);
	init_sensorlist_2(data);
	if (data->prox_type != PROX_LIGHT_CM36657) {
		struct sensor_info prox_info = SENSOR_INFO_PROXIMITY_1BYTE;
		struct sensor_info prox_raw_info = SENSOR_INFO_PROXIMITY_RAW_1BYTE;

		memcpy(&(data->info[SSP_SENSOR_TYPE_PROXIMITY]), &prox_info, sizeof(struct sensor_info));
		memcpy(&(data->info[SSP_SENSOR_TYPE_PROXIMITY_RAW]), &prox_raw_info, sizeof(struct sensor_info));
	}
}

void set_dump_sensor_data(int enable)
{
	dump_sensor_data = enable;
}
int get_dump_sensor_data(void)
{
	return dump_sensor_data;
}

static void sensor_data_dump(int type, struct sensor_value *event)
{
	switch (type) {
	case SSP_SENSOR_TYPE_ACCELEROMETER:
	case SSP_SENSOR_TYPE_GYROSCOPE:
		pr_err("%s: type=%d, x=%hd, y=%hd, z=%hd, gyro_dps=%u\n",
			__func__, type, event->x, event->y, event->z, event->gyro_dps);
		break;
	case SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD:
	case SSP_SENSOR_TYPE_ORIENTATION:
		pr_err("%s: type=%d, x=%hd, y=%hd, z=%hd, accuracy=%hu\n",
			__func__, type, event->cal_x, event->cal_y, event->cal_z, event->accuracy);
		break;
	case SSP_SENSOR_TYPE_LIGHT:
		pr_err("%s: type=%d,lux=%u,cct=%d,r=%hu,g=%hu,b=%hu,w=%hu,a_time=%hu,a_gain=%hu\n",
			__func__, type, event->lux, event->cct, event->r, event->g,
			event->b, event->w, event->a_time, event->a_gain);
		break;
	case SSP_SENSOR_TYPE_PROXIMITY:
		pr_err("%s: type=%d, prox=%hu, prox_ex=%hu\n",
			__func__, type, event->prox, event->prox_ex);
		break;
	case SSP_SENSOR_TYPE_PROXIMITY_RAW:
		pr_err("%s: type=%d, prox_raw=%hu,%hu,%hu,%hu\n",
			__func__, type, event->prox_raw[0],
			event->prox_raw[1], event->prox_raw[2], event->prox_raw[3]);
		break;
	default:
		break;
	}
}
static void report_prox_raw_data(struct ssp_data *data, int type,
	struct sensor_value *proxrawdata)
{
	if (data->uFactoryProxAvg[0]++ >= PROX_AVG_READ_NUM) {
		data->uFactoryProxAvg[2] /= PROX_AVG_READ_NUM;
		data->buf[type].prox_raw[1] = (u16)data->uFactoryProxAvg[1];
		data->buf[type].prox_raw[2] = (u16)data->uFactoryProxAvg[2];
		data->buf[type].prox_raw[3] = (u16)data->uFactoryProxAvg[3];

		data->uFactoryProxAvg[0] = 0;
		data->uFactoryProxAvg[1] = 0;
		data->uFactoryProxAvg[2] = 0;
		data->uFactoryProxAvg[3] = 0;
	} else {
		data->uFactoryProxAvg[2] += proxrawdata->prox_raw[0];

		if (data->uFactoryProxAvg[0] == 1)
			data->uFactoryProxAvg[1] = proxrawdata->prox_raw[0];
		else if (proxrawdata->prox_raw[0] < data->uFactoryProxAvg[1])
			data->uFactoryProxAvg[1] = proxrawdata->prox_raw[0];

		if (proxrawdata->prox_raw[0] > data->uFactoryProxAvg[3])
			data->uFactoryProxAvg[3] = proxrawdata->prox_raw[0];
	}

	data->buf[type].prox_raw[0] = proxrawdata->prox_raw[0];
}
void report_sensor_data(struct ssp_data *data, int type,
			struct sensor_value *event)
{
	int res = 0;

	if (dump_sensor_data)
		sensor_data_dump(type, &data->buf[type]);

	if (type == SSP_SENSOR_TYPE_PROXIMITY) {
		pr_info("Proximity Sensor Detect : %u, raw : %u",
			event->prox, event->prox_ex);
		res = ps_report_interrupt_data(event->prox);
	} else if (type == SSP_SENSOR_TYPE_PROXIMITY_RAW) {
		report_prox_raw_data(data, type, event);
		return;

	} else if (type == SSP_SENSOR_TYPE_LIGHT) {
		event->a_gain &= 0x03;
		if (data->light_log_cnt < 3) {
			pr_info("Light Sensor : r=%d g=%d b=%d c=%d atime=%d again=%d",
				data->buf[SSP_SENSOR_TYPE_LIGHT].r,
				data->buf[SSP_SENSOR_TYPE_LIGHT].g,
				data->buf[SSP_SENSOR_TYPE_LIGHT].b,
				data->buf[SSP_SENSOR_TYPE_LIGHT].w,
				data->buf[SSP_SENSOR_TYPE_LIGHT].a_time,
				data->buf[SSP_SENSOR_TYPE_LIGHT].a_gain);
			data->light_log_cnt++;
		}
	} else if (type == SSP_SENSOR_TYPE_STEP_COUNTER) {
		data->buf[type].step_total += event->step_diff;
	}
	memcpy(&data->buf[type], (char *)event, data->info[type].get_data_len);

#if 0
	ssp_iio_push_buffers(data->indio_devs[type], event->timestamp,
			(char *)&data->buf[type], data->info[type].report_data_len);

	/* wake-up sensor */
	if (type == SSP_SENSOR_TYPE_PROXIMITY || type == SSP_SENSOR_TYPE_SIGNIFICANT_MOTION
		|| type == SSP_SENSOR_TYPE_TILT_DETECTOR || type == SSP_SENSOR_TYPE_PICK_UP_GESTURE) {
		wake_lock_timeout(&data->ssp_wake_lock, 0.3 * HZ);
	}
#endif
}

void report_meta_data(struct ssp_data *data, int type, struct sensor_value *s)
{
	pr_info("what: %d, sensor: %d",
		s->meta_data.what, s->meta_data.sensor);
	return;
#if 0
	if ((s->meta_data.sensor == SSP_SENSOR_TYPE_ACCELEROMETER)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_GYROSCOPE)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_PRESSURE)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_ROTATION_VECTOR)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_GAME_ROTATION_VECTOR)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_STEP_DETECTOR)
		|| (s->meta_data.sensor == SSP_SENSOR_TYPE_INTERRUPT_GYRO)) {
		char *meta_event
			= kzalloc(data->info[s->meta_data.sensor].report_data_len,
					GFP_KERNEL);
		if (!meta_event) {
			ssp_errf("fail to allocate memory for meta event");
			return;
		}

		memset(meta_event, META_EVENT,
			data->info[s->meta_data.sensor].report_data_len);
		ssp_iio_push_buffers(data->indio_devs[s->meta_data.sensor],
				META_TIMESTAMP, meta_event,
				data->info[s->meta_data.sensor].report_data_len);
		kfree(meta_event);
	} else {
		ssp_iio_push_buffers(data->indio_devs[type],
				META_TIMESTAMP, (char *)&s->meta_data,
				sizeof(s->meta_data));
	}
#endif
}

static inline int ssp_mtk_open_report_data(int open)
{
    return 0;
}

static int ssp_mtk_acc_enable_nodata(int en)
{
	if (local_data == NULL)
		return -EFAULT;
	return ssp_mtk_sensors_enable(local_data, SSP_SENSOR_TYPE_ACCELEROMETER, en);
}

static int ssp_mtk_acc_set_delay(u64 ns)
{
	uint64_t value = 0;

	if (local_data == NULL)
		return -EFAULT;
	value = (uint64_t)ns/1000/1000;
	return ssp_mtk_sensors_set_delay(local_data, SSP_SENSOR_TYPE_ACCELEROMETER, value);
}

static int ssp_mtk_acc_get_data(int *x, int *y, int *z, int *status)
{
	if (local_data == NULL)
		return -EFAULT;
	*x = local_data->buf[SSP_SENSOR_TYPE_ACCELEROMETER].x * SSP_ACC_SENSITIVE;
	*y = local_data->buf[SSP_SENSOR_TYPE_ACCELEROMETER].y * SSP_ACC_SENSITIVE;
	*z = local_data->buf[SSP_SENSOR_TYPE_ACCELEROMETER].z * SSP_ACC_SENSITIVE;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
static int ssp_mtk_acc_get_raw_data(int *x, int *y, int *z)
{
	if (local_data == NULL)
		return -EFAULT;
	*x = local_data->buf[SSP_SENSOR_TYPE_ACCELEROMETER].x;
	*y = local_data->buf[SSP_SENSOR_TYPE_ACCELEROMETER].y;
	*z = local_data->buf[SSP_SENSOR_TYPE_ACCELEROMETER].z;

	return 0;
}
static int ssp_mtk_mag_enable(int en)
{
	if (local_data == NULL)
		return -EFAULT;
    return ssp_mtk_sensors_enable(local_data, SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD, en);
}

static int ssp_mtk_mag_set_delay(u64 ns)
{
    uint64_t value = 0;

	if (local_data == NULL)
		return -EFAULT;
    value = (uint64_t)ns/1000/1000;
    return ssp_mtk_sensors_set_delay(local_data, SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD, value);
}

static int ssp_mtk_mag_get_data(int *x, int *y, int *z, int *status)
{
	if (local_data == NULL)
		return -EFAULT;
	*x = (int)local_data->buf[SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_x * SSP_MAG_SENSITIVE;
	*y = (int)local_data->buf[SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_y * SSP_MAG_SENSITIVE;
	*z = (int)local_data->buf[SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_z * SSP_MAG_SENSITIVE;
	*status = local_data->buf[SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD].accuracy;

	return 0;
}
static int ssp_mtk_mag_get_raw_data(int *x, int *y, int *z)
{
	if (local_data == NULL)
		return -EFAULT;
	*x = (int)local_data->buf[SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_x;/* * CONVERT_M; */
	*y = (int)local_data->buf[SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_y;/* * CONVERT_M; */
	*z = (int)local_data->buf[SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_z;/* * CONVERT_M; */

	return 0;
}

static int ssp_mtk_ori_enable(int en)
{
	if (local_data == NULL)
		return -EFAULT;
	return ssp_mtk_sensors_enable(local_data, SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD, en);
}

static int ssp_mtk_ori_set_delay(u64 ns)
{
    uint64_t value = 0;

	if (local_data == NULL)
		return -EFAULT;
    value = (uint64_t)ns/1000/1000;
    return ssp_mtk_sensors_set_delay(local_data, SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD, value);
}

static int ssp_mtk_ori_get_data(int *x, int *y, int *z, int *status)
{
	return 0;
}

static int ssp_mtk_gyro_enable_nodata(int en)
{
	if (local_data == NULL)
		return -EFAULT;
	return ssp_mtk_sensors_enable(local_data, SSP_SENSOR_TYPE_GYROSCOPE, en);
}

static int ssp_mtk_gyro_set_delay(u64 ns)
{
	uint64_t value = 0;

	if (local_data == NULL)
		return -EFAULT;
	value = (uint64_t)ns/1000/1000;
	return ssp_mtk_sensors_set_delay(local_data, SSP_SENSOR_TYPE_GYROSCOPE, value);
}

static int ssp_mtk_gyro_get_data(int *x, int *y, int *z, int *status)
{
	if (local_data == NULL)
		return -EFAULT;
	*x = (int)local_data->buf[SSP_SENSOR_TYPE_GYROSCOPE].x * SSP_GYRO_SENSITIVE;
	*y = (int)local_data->buf[SSP_SENSOR_TYPE_GYROSCOPE].y * SSP_GYRO_SENSITIVE;
	*z = (int)local_data->buf[SSP_SENSOR_TYPE_GYROSCOPE].z * SSP_GYRO_SENSITIVE;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
static int ssp_mtk_gyro_get_raw_data(int *x, int *y, int *z)
{
	if (local_data == NULL)
		return -EFAULT;
	*x = (int)local_data->buf[SSP_SENSOR_TYPE_GYROSCOPE].x * 100;
	*y = (int)local_data->buf[SSP_SENSOR_TYPE_GYROSCOPE].y * 100;
	*z = (int)local_data->buf[SSP_SENSOR_TYPE_GYROSCOPE].z * 100;

	return 0;
}
static int ssp_mtk_als_enable_nodata(int en)
{
	if (local_data == NULL)
		return -EFAULT;
	return ssp_mtk_sensors_enable(local_data, SSP_SENSOR_TYPE_LIGHT, en);
}

static int ssp_mtk_als_set_delay(u64 ns)
{
	return 0;
}

static int ssp_mtk_als_get_data(int *value, int *status)
{
	if (local_data == NULL)
		return -EFAULT;
	*value = local_data->buf[SSP_SENSOR_TYPE_LIGHT].lux;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int ssp_mtk_ps_enable_nodata(int en)
{
	/*int ret = 0;*/

	if (local_data == NULL)
		return -EFAULT;
	/*ret = ssp_mtk_sensors_enable(local_data, SSP_SENSOR_TYPE_PROXIMITY_RAW, en);*/
	return ssp_mtk_sensors_enable(local_data, SSP_SENSOR_TYPE_PROXIMITY, en);
}

static int ssp_mtk_ps_set_delay(u64 ns)
{
	return 0;
}

static int ssp_mtk_ps_get_data(int *value, int *status)
{
	if (local_data == NULL)
		return -EFAULT;
	*value = local_data->buf[SSP_SENSOR_TYPE_PROXIMITY].prox;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

static int ssp_mtk_acc_init(void)
{
	int err = 0;

	struct acc_control_path ctl_path = { 0 };
	struct acc_data_path data_path = { 0 };

	ctl_path.is_use_common_factory = true;
	ctl_path.open_report_data = ssp_mtk_open_report_data;
	ctl_path.enable_nodata = ssp_mtk_acc_enable_nodata;
	ctl_path.set_delay  = ssp_mtk_acc_set_delay;
	ctl_path.is_report_input_direct = false;
	ctl_path.is_support_batch = false;
	err = acc_register_control_path(&ctl_path);
	if (err) {
		pr_err("%s:register acc control path err,err = %d\n", __func__, err);
		return -EFAULT;
	}

	data_path.get_data = ssp_mtk_acc_get_data;
	data_path.get_raw_data = ssp_mtk_acc_get_raw_data;
	data_path.vender_div = SSP_ACC_DIV;
	err = acc_register_data_path(&data_path);
	if (err) {
		pr_err("%s:acc_register_data_path err,err = %d\n", __func__, err);
		return -EFAULT;
	}

	return err;
}
static int ssp_mtk_mag_init(void)
{
	int err = 0;

	struct mag_control_path ctl = {0};
	struct mag_data_path mag_data = {0};

	ctl.is_use_common_factory = true;
	ctl.m_enable = ssp_mtk_mag_enable;
	ctl.m_set_delay  = ssp_mtk_mag_set_delay;
	ctl.m_open_report_data = ssp_mtk_open_report_data;
	ctl.o_enable = ssp_mtk_ori_enable;
	ctl.o_set_delay  = ssp_mtk_ori_set_delay;
	ctl.o_open_report_data = ssp_mtk_open_report_data;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;

	err = mag_register_control_path(&ctl);
	if (err) {
		pr_err("%s:mag_register_control_path err,err = %d\n", __func__, err);
		return -EFAULT;
	}

	mag_data.div_m = SSP_MAG_DIV;/* CONVERT_M_DIV; */
	mag_data.div_o = SSP_MAG_DIV;/* CONVERT_O_DIV; */
	mag_data.get_data_o = ssp_mtk_ori_get_data;
	mag_data.get_data_m = ssp_mtk_mag_get_data;
	mag_data.get_raw_data = ssp_mtk_mag_get_raw_data;
	err = mag_register_data_path(&mag_data);
	if (err) {
		pr_err("%s:mag_register_data_path,err = %d\n", __func__, err);
		return -EFAULT;
	}

	return err;
}
static int ssp_mtk_gyro_init(void)
{
	int err = 0;

	struct gyro_control_path ctl = { 0 };
    struct gyro_data_path data = { 0 };

	ctl.is_use_common_factory = true;
	ctl.open_report_data = ssp_mtk_open_report_data;
	ctl.enable_nodata = ssp_mtk_gyro_enable_nodata;
	ctl.set_delay = ssp_mtk_gyro_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;

	err = gyro_register_control_path(&ctl);
	if (err) {
		pr_err("%s:gyro_register_control_path,err = %d\n", __func__, err);
		return -EFAULT;
	}

	data.get_data = ssp_mtk_gyro_get_data;
	data.get_raw_data = ssp_mtk_gyro_get_raw_data;
	data.vender_div = DEGREE_TO_RAD;
	err = gyro_register_data_path(&data);
	if (err) {
		pr_err("%s:gyro_register_data_path,err = %d\n", __func__, err);
		return -EFAULT;
	}

	return err;
}
static int ssp_mtk_alsps_init(void)
{
	int err = 0;

	struct als_control_path als_ctl = { 0 };
	struct als_data_path als_data = { 0 };
	struct ps_control_path ps_ctl = { 0 };
	struct ps_data_path ps_data = { 0 };

	als_ctl.is_use_common_factory = true;
	als_ctl.open_report_data = ssp_mtk_open_report_data;
	als_ctl.enable_nodata = ssp_mtk_als_enable_nodata;
	als_ctl.set_delay  = ssp_mtk_als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if (err) {
		pr_err("%s:als_register_control_path,err = %d\n", __func__, err);
		return -EFAULT;
	}

	als_data.get_data = ssp_mtk_als_get_data;
	als_data.vender_div = SSP_ALS_DIV;
	err = als_register_data_path(&als_data);
	if (err) {
		pr_err("%s:als_register_data_path,err = %d\n", __func__, err);
		return -EFAULT;
	}

	ps_ctl.is_use_common_factory = true;
	ps_ctl.open_report_data = ssp_mtk_open_report_data;
	ps_ctl.enable_nodata = ssp_mtk_ps_enable_nodata;
	ps_ctl.set_delay  = ssp_mtk_ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;

	err = ps_register_control_path(&ps_ctl);
	if (err) {
		pr_err("%s:ps_register_control_path,err = %d\n", __func__, err);
		return -EFAULT;
	}

	ps_data.get_data = ssp_mtk_ps_get_data;
	ps_data.vender_div = SSP_PS_DIV;
	err = ps_register_data_path(&ps_data);
	if (err) {
		pr_err("%s:ps_register_data_path,err = %d\n", __func__, err);
		return -EFAULT;
	}
	return err;
}
int initialize_indio_dev(struct ssp_data *data)
{
	if (data == NULL) {
		pr_err("%s data is null\n", __func__);
		return ERROR;
	}

	local_data = data;
	init_sensorlist(data);
	return SUCCESS;
}

void remove_indio_dev(struct ssp_data *data)
{
	local_data = NULL;
}
static int ssp_mtk_local_uninit(void)
{
    return 0;
}
static int ssp_mtk_gyro_local_init(struct platform_device *pdev)
{
	return ssp_mtk_gyro_init();
}
static struct mag_init_info ssp_mtk_mag_init_info = {
	.name = "ssp_mtk_mag",
	.init = ssp_mtk_mag_init,
	.uninit = ssp_mtk_local_uninit,
};
static struct acc_init_info  ssp_mtk_acc_init_info = {
	.name   = "ssp_mtk_accel",
	.init   = ssp_mtk_acc_init,
	.uninit = ssp_mtk_local_uninit,
};
static struct gyro_init_info  ssp_mtk_gyro_init_info = {

    .name   = "ssp_mtk_gyro",
    .init   = ssp_mtk_gyro_local_init,
    .uninit = ssp_mtk_local_uninit,
};
static struct alsps_init_info ssp_mtk_alsps_init_info = {
	.name = "ssp_mtk_alsps",
	.init = ssp_mtk_alsps_init,
	.uninit = ssp_mtk_local_uninit,

};
static int __init ssp_mtk_init(void)
{
	acc_driver_add(&ssp_mtk_acc_init_info);
	mag_driver_add(&ssp_mtk_mag_init_info);
	gyro_driver_add(&ssp_mtk_gyro_init_info);
	alsps_driver_add(&ssp_mtk_alsps_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ssp_mtk__exit(void)
{
	pr_info("%s\n", __func__);
}
/*----------------------------------------------------------------------------*/
module_init(ssp_mtk_init);
module_exit(ssp_mtk__exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mediatek sensor for stm32");
MODULE_AUTHOR("Xj.wang@mediatek.com");
