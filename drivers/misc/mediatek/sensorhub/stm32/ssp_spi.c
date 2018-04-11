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

#include "ssp_spi.h"

static int ssp_spi_read(struct spi_device *spi,
	char *buffer, size_t len)
{
	int ret;
	static u8 tx_buf[255] = {0x00, };

	struct spi_message m;
	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = buffer,
		.len	= len,
	};

	spi_message_init(&m);
	spi_message_add_tail(&xfer, &m);

	ret = spi_sync(spi, &m);

	if (ret < 0) {
		ssp_err("Error ssp_spi_read() ret = %d", ret);
		return ret;
	}

	return ret;
}

#if 0
static int ssp_spi_write(struct spi_device *spi,
	struct ssp_msg *msg, ssize_t len)
{
	int ret;
	static u8 rx_buf[255] = {0, };
	/* static u8 tx_buf[STM_MAX_BUFFER_SIZE] = {0, }; */
	struct spi_message m;

	struct spi_transfer	xfer = {
		.tx_buf		= msg->buffer,
		.rx_buf		= rx_buf,
		.len		= (unsigned int)len,
		.bits_per_word = 8,
	};
	/* memcpy(tx_buf, buffer, len); */
	/* t.tx_buf = tx_buf; */

	spi_message_init(&m);
	spi_message_add_tail(&xfer, &m);

	ret = spi_sync(spi, &m);

	printk("%s tx_buf = %p rx_buf = %p len = %d\n", __func__,
		xfer.tx_buf, xfer.rx_buf, xfer.len);

	if (ret < 0) {
		ssp_err("Error ssp_spi_write() ret = %d", ret);
		return ret;
	}

	return len;
}
#endif

static void clean_msg(struct ssp_msg *msg)
{
	if (msg->free_buffer)
		kfree(msg->buffer);

	kfree(msg);
}
/*extern int mt_get_gpio_in(unsigned long pin);*/
static int do_transfer(struct ssp_data *data, struct ssp_msg *msg,
		struct completion *done, int timeout) {
	int status = 0;
	int iDelaycnt = 0;
	bool msg_dead = false, ssp_down = false;
	bool use_no_irq = msg->length == 0;

	msg->dead_hook = &msg_dead;
	msg->dead = false;
	msg->done = done;

	mutex_lock(&data->comm_mutex);

	gpio_set_value_cansleep(data->ap_int, 0);
	while (gpio_get_value_cansleep(data->mcu_int2)) {
		usleep_range(2900, 3000);
		ssp_down = data->is_ssp_shutdown;
		if (ssp_down || iDelaycnt++ > 500) {
			ssp_errf("exit1 - Time out!!");
			gpio_set_value_cansleep(data->ap_int, 1);
			status = -1;
			goto exit;
		}
	}

	status = spi_write(data->spi, msg, 9) >= 0;

	if (status == 0) {
		ssp_errf("spi_write fail!!");
		gpio_set_value_cansleep(data->ap_int, 1);
		status = -1;
		goto exit;
	}

	if (!use_no_irq) {
		mutex_lock(&data->pending_mutex);
		list_add_tail(&msg->list, &data->pending_list);
		mutex_unlock(&data->pending_mutex);
	}

	iDelaycnt = 0;
	gpio_set_value_cansleep(data->ap_int, 1);
	while (!gpio_get_value_cansleep(data->mcu_int2)) {
		usleep_range(2900, 3000);
		ssp_down = data->is_ssp_shutdown;
/*		ssp_errf("ssp_down=%d,data->mcu_int2 value=%d\n", ssp_down, gpio_get_value_cansleep(data->mcu_int2));
*		ssp_errf("mt_get_gpio_in(115) = %d\n", mt_get_gpio_in(115 | 0x80000000));
*/
		if (ssp_down || iDelaycnt++ > 500) {
			ssp_errf("exit2 - Time out!!");
			status = -2;
			goto exit;
		}
	}

exit:
	mutex_unlock(&data->comm_mutex);

	if (ssp_down)
		ssp_errf("ssp down");

	if (status == -1) {
		data->cnt_timeout += ssp_down ? 0 : 1;
		clean_msg(msg);
		return status;
	}

	if (status == 1 && done != NULL) {
		int ret = wait_for_completion_timeout(done,
				msecs_to_jiffies(timeout));
		if (!ret) {
			status = -2;
			ssp_err("mcu_int1 level: %d",
				gpio_get_value(data->mcu_int1));
		}
	}
	mutex_lock(&data->pending_mutex);
	if (!msg_dead) {
		msg->done = NULL;
		msg->dead_hook = NULL;

		if (status != 1)
			msg->dead = true;
		if (status == -2)
			data->cnt_timeout += ssp_down ? 0 : 1;
	}
	mutex_unlock(&data->pending_mutex);

	if (use_no_irq)
		clean_msg(msg);

	return status;
}

int ssp_spi_async(struct ssp_data *data, struct ssp_msg *msg)
{
	int status = 0;

	status = do_transfer(data, msg, NULL, 0);

	return status;
}

int ssp_spi_sync(struct ssp_data *data, struct ssp_msg *msg, int timeout)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status = 0;

	if (msg->length == 0) {
		ssp_errf("length must not be 0");
		clean_msg(msg);
		return status;
	}

	status = do_transfer(data, msg, &done, timeout);

	return status;
}

int select_irq_msg(struct ssp_data *data)
{
	struct ssp_msg *msg, *n;
	bool found = false;
	u16 chLength = 0, msg_options = 0;
	u8 msg_type = 0;
	int ret = 0;
#define CHTEMPBUF_LEN 4
	/* buffer aligned */
	/*static char chTempBuf[4] __aligned(4);*/
	static char *chTempBuf;

	static char *buffer __aligned(4);
	static char *hub2ap_buffer __aligned(4);
	static char *ap2hub_buffer __aligned(4);

	if (chTempBuf == NULL) {
		chTempBuf = kzalloc(CHTEMPBUF_LEN, GFP_KERNEL | GFP_DMA);
		if (chTempBuf == NULL) {
			ssp_errf("chTempBuf, kzalloc failed\n");
			return -ENOMEM;
		}
	}
	/*memset(chTempBuf, 0, sizeof(chTempBuf));*/
	buffer = NULL;
	hub2ap_buffer = NULL;
	ap2hub_buffer = NULL;

	/* change spi read func */
	/* ret = spi_read(data->spi, chTempBuf, sizeof(chTempBuf)); */
	ret = ssp_spi_read(data->spi, chTempBuf, CHTEMPBUF_LEN);
	if (ret < 0) {
		if (chTempBuf != NULL) {
			kfree(chTempBuf);
			chTempBuf = NULL;
		}
		ssp_errf("spi_read fail, ret = %d", ret);
		return ret;
	}

	memcpy(&msg_options, &chTempBuf[0], 2);
	msg_type = msg_options & SSP_SPI_MASK;
	memcpy(&chLength, &chTempBuf[2], 2);

	if (chLength == 0)
		ssp_errf("lengh is 0 - buf:0x%x 0x%x msg_type:%d",
			chTempBuf[0], chTempBuf[1], msg_type);

	switch (msg_type) {
	case AP2HUB_READ:
	case AP2HUB_WRITE:
		mutex_lock(&data->pending_mutex);
		if (!list_empty(&data->pending_list)) {
			list_for_each_entry_safe(msg, n,
				&data->pending_list, list) {
				if (msg->options == msg_options) {
					list_del(&msg->list);
					found = true;
					break;
				}
			}

			if (!found) {
				ssp_errf("%d - Not match error", msg_options);
				goto exit;
			}

			if (msg->dead && !msg->free_buffer) {
				/* buffer aligned */
				ap2hub_buffer = kzalloc(msg->length + 4, GFP_KERNEL | GFP_DMA);
				{
					int temp = (uint64_t)ap2hub_buffer & (uint64_t)0x03;

					msg->buffer = ap2hub_buffer + temp;
				}
				msg->free_buffer = 0;
			} /* For dead msg, make a temporary buffer to read */

			if (msg_type == AP2HUB_READ)
				/* change spi read func */
				/* ret = spi_read(data->spi, msg->buffer, msg->length); */
				ret = ssp_spi_read(data->spi, msg->buffer, msg->length);
			if (msg_type == AP2HUB_WRITE) {
				ret = spi_write(data->spi,
						msg->buffer, msg->length);

				/* There is no use it now */
				if (msg_options & AP2HUB_RETURN) {
					msg->options = AP2HUB_READ
							| AP2HUB_RETURN;
					msg->length = 1;
					list_add_tail(&msg->list,
							&data->pending_list);
					goto exit;
				}
			}

			if (msg->done != NULL && !completion_done(msg->done))
				complete(msg->done);
			if (msg->dead_hook != NULL)
				*(msg->dead_hook) = true;

			/* buffer aligned */
			if (ap2hub_buffer != NULL)
				kfree(ap2hub_buffer);

			clean_msg(msg);
		} else
			ssp_err("List empty error(%d)", msg_type);
exit:
		mutex_unlock(&data->pending_mutex);
		break;
	case HUB2AP_WRITE:
		if (chLength == 0) {
			ssp_errf("HUB2AP_WRITE length is 0");
			ret = -EINVAL;
			break;
		}

		/* buffer aligned */
		hub2ap_buffer = kzalloc(chLength + 4, GFP_KERNEL | GFP_DMA);
	{
	    int temp = (uint64_t)hub2ap_buffer & (uint64_t)0x3;

	    buffer = (char *)(hub2ap_buffer + temp);
	}

		if (buffer == NULL) {
			ssp_errf("failed to alloc memory for buffer");
			ret = -ENOMEM;
			break;
		}

		/* change spi read func */
		/* ret = spi_read(data->spi, buffer, chLength); */
		ret = ssp_spi_read(data->spi, buffer, chLength);

		if (ret < 0)
			ssp_errf("spi_read fail");
		else
			parse_dataframe(data, buffer, chLength);

		/* buffer aligned */
		if (hub2ap_buffer != NULL)
			kfree(hub2ap_buffer);

		break;
	default:
		ssp_err("No type error(%d)", msg_type);
		break;
	}

	if (ret < 0) {
		ssp_errf("MSG2SSP_SSD error %d", ret);
		return ERROR;
	}

	return SUCCESS;
}

void clean_pending_list(struct ssp_data *data)
{
	struct ssp_msg *msg, *n;

	ssp_dbgf(" IN");

	mutex_lock(&data->pending_mutex);
	list_for_each_entry_safe(msg, n, &data->pending_list, list) {
		list_del(&msg->list);
		if (msg->done != NULL && !completion_done(msg->done))
			complete(msg->done);
		if (msg->dead_hook != NULL)
			*(msg->dead_hook) = true;

		clean_msg(msg);
	}
	mutex_unlock(&data->pending_mutex);
	ssp_dbgf(" OUT");

}

int ssp_send_cmd(struct ssp_data *data, char command, int arg)
{
	int ret = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = command;
	msg->length = 0;
	msg->options = AP2HUB_WRITE;
	msg->data = arg;
	msg->free_buffer = 0;

	ret = ssp_spi_async(data, msg);
	if (ret != SUCCESS) {
		ssp_errf("command 0x%x failed %d", command, ret);
		return ERROR;
	}

	ssp_infof("command 0x%x %d", command, arg);

	return SUCCESS;
}

int send_instruction(struct ssp_data *data, u8 uInst,
	u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command;
	int ret = 0;
	struct ssp_msg *msg;


	static char buffer[10] __aligned(4) = { 0, };

	memset(buffer, 0, sizeof(buffer));

	if (data->fw_dl_state == FW_DL_STATE_DOWNLOADING) {
		ssp_errf("Skip Inst! DL state = %d", data->fw_dl_state);
		return SUCCESS;
	} else if ((!(data->uSensorState & (1 << uSensorType)))
		&& (uInst <= CHANGE_DELAY)) {
		ssp_errf("Bypass Inst Skip! - %u", uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_REMOVE;
		break;
	case ADD_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_ADD;
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		break;
	case CHANGE_DELAY:
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		command = MSG2SSP_INST_CHANGE_DELAY;
		break;
	case GO_SLEEP:
		command = MSG2SSP_AP_STATUS_SLEEP;
		data->uLastAPState = MSG2SSP_AP_STATUS_SLEEP;
		break;
	case REMOVE_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_REMOVE;
		break;
	case ADD_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_ADD;
		break;
	default:
		command = uInst;
		break;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		ret = -ENOMEM;
		return ret;
	}
	msg->cmd = command;
	msg->length = uLength + 1;
	msg->options = AP2HUB_WRITE;
	/* msg->buffer = kzalloc(uLength + 1, GFP_KERNEL); */
	/* msg->free_buffer = 1; */

	msg->buffer = buffer;
	msg->free_buffer = 0;

	msg->buffer[0] = uSensorType;
	memcpy(&msg->buffer[1], uSendBuf, uLength);

	ssp_infof("Inst = 0x%x, Sensor Type = 0x%x, data = %u",
			command, uSensorType, msg->buffer[1]);

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		ssp_errf("Instruction CMD Fail %d", ret);
		return ERROR;
	}

	return ret;
}

int send_instruction_sync(struct ssp_data *data, u8 uInst,
	u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command;
	int ret = 0;

	/* buffer aligned */
	static char buffer[10] __aligned(4) = { 0, };
	struct ssp_msg *msg;

	memset(buffer, 0, sizeof(buffer));

	if (data->fw_dl_state == FW_DL_STATE_DOWNLOADING) {
		ssp_errf("Skip Inst! DL state = %d", data->fw_dl_state);
		return SUCCESS;
	} else if ((!(data->uSensorState & (1 << uSensorType)))
		&& (uInst <= CHANGE_DELAY)) {
		ssp_errf("Bypass Inst Skip! - %u", uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = MSG2SSP_INST_BYPASS_SENSOR_REMOVE;
		break;
	case ADD_SENSOR:
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		command = MSG2SSP_INST_BYPASS_SENSOR_ADD;
		break;
	case CHANGE_DELAY:
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		command = MSG2SSP_INST_CHANGE_DELAY;
		break;
	case GO_SLEEP:
		command = MSG2SSP_AP_STATUS_SLEEP;
		data->uLastAPState = MSG2SSP_AP_STATUS_SLEEP;
		break;
	case REMOVE_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_REMOVE;
		break;
	case ADD_LIBRARY:
		command = MSG2SSP_INST_LIBRARY_ADD;
		break;
	default:
		command = uInst;
		break;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = command;
	msg->length = uLength + 1;
	msg->options = AP2HUB_WRITE | AP2HUB_RETURN;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	msg->buffer[0] = uSensorType;
	memcpy(&msg->buffer[1], uSendBuf, uLength);

	ssp_infof("Inst Sync = 0x%x, Sensor Type = %u, data = %u",
			command, uSensorType, msg->buffer[0]);

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret != SUCCESS) {
		ssp_errf("Instruction CMD Fail %d", ret);
		return ERROR;
	}

	return buffer[0];
}

static int ssp_readwrite_data(struct ssp_data *data, char command,
			unsigned short option, char *buffer, int len,
			int timeout, unsigned char free_buffer)
{
	int ret = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}

	msg->cmd = command;
	msg->length = len;
	msg->options = option;
	msg->buffer = buffer;
	msg->free_buffer = free_buffer;

	if (timeout > 0)
		ret = ssp_spi_sync(data, msg, timeout);
	else
		ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		ssp_errf("ssp_readwrite_data 0x%x failed %d", command, ret);
		return ERROR;
	}

	return SUCCESS;
}

int flush(struct ssp_data *data, u8 uSensorType)
{
	int ret = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	/* buffer aligned */
	static char buffer __aligned(4);
	buffer = 0;

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_BATCH_FLUSH;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->data = uSensorType;
	msg->buffer = &buffer;

	msg->free_buffer = 0;

	ssp_infof("Sensor Type = 0x%x, data = %u", uSensorType, buffer);

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret != SUCCESS) {
		ssp_errf("fail %d", ret);
		return ERROR;
	}

	return buffer ? 0 : -1;
}

int get_batch_count(struct ssp_data *data, u8 uSensorType)
{
	int ret = 0;
	s32 result = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	/* buffer aligned */
	static char buffer[4] __aligned(4) = { 0, };

	memset(buffer, 0, sizeof(buffer));

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_BATCH_COUNT;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->data = uSensorType;
	msg->buffer = buffer;

	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret != SUCCESS) {
		ssp_errf("fail %d", ret);
		return ERROR;
	}

	memcpy(&result, buffer, 4);

	ssp_infof("Sensor Type = 0x%x, data = %u", uSensorType, result);

	return result;
}

int get_chipid(struct ssp_data *data)
{
	int ret, reties = 0;
	struct ssp_msg *msg;

	/* buffer aligned */
	static char buffer __aligned(4);
	buffer = 0;

retries:
	msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_WHOAMI;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = &buffer;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);

	if (buffer != DEVICE_ID && reties++ < 2) {
		usleep_range(5000, 5500);
		ssp_errf("get chip ID retry");
		goto retries;
	}

	if (ret == SUCCESS) {
		ssp_dbgf("%d (%d)", buffer, ret);
		return buffer;
	}

	ssp_errf("get chip ID failed %d", ret);
	return ERROR;
}

int set_sensor_position(struct ssp_data *data)
{
	int ret = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	/* buffer aligned */
	static char buffer[3] __aligned(4) = { 0, };

	memset(buffer, 0, sizeof(buffer));

#if 1 /** DEBUG **/
	ssp_infof();
#endif
	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_FORMATION;
	msg->length = 3;
	msg->options = AP2HUB_WRITE;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	msg->buffer[0] = data->accel_position;
	msg->buffer[1] = data->accel_position;
	msg->buffer[2] = data->mag_position;

	ret = ssp_spi_async(data, msg);

	ssp_info("Sensor Posision A : %u, G : %u, M: %u, P: 0",
		data->accel_position, data->accel_position, data->mag_position);

	if (ret != SUCCESS) {
		ssp_errf("fail to set_sensor_position %d", ret);
		ret = ERROR;
	}

	return ret;
}

int get_6axis_type(struct ssp_data *data)
{
	char acc_type = -1;
	int ret = ssp_readwrite_data(data, MSG2SSP_AP_WHOAMI_6AXIS,
			AP2HUB_READ, &acc_type, sizeof(acc_type),
			0, 0);

	if (ret != SUCCESS) {
		ssp_errf("fail to get_6axis_type %d", ret);
		return ERROR;
	}

	ssp_infof("6axis type from mcu: %d", acc_type);

	if (acc_type < SIX_AXIS_MPU6500 || acc_type >= SIX_AXIS_MAX)
		ssp_errf("wrong 6axis type from mcu");

	return (int)acc_type;
}

int set_6axis_dot(struct ssp_data *data)
{
	char accel_dot = data->accel_dot;
	int ret = ssp_readwrite_data(data, MSG2SSP_AP_SET_6AXIS_PIN,
			AP2HUB_WRITE, &accel_dot, sizeof(accel_dot),
			0, 0);

	ssp_info("6axis sensor dot: %u", data->accel_dot);

	if (ret != SUCCESS) {
		ssp_errf("fail to set_6axis_dot %d", ret);
		ret = ERROR;
	}

	return ret;
}

void set_proximity_threshold(struct ssp_data *data)
{
	int ret = 0;

	struct ssp_msg *msg;

	/* buffer aligned */
	static char buffer[8] __aligned(4) = { 0, };

	memset(buffer, 0, sizeof(buffer));

	if (!(data->uSensorState & (1 << SSP_SENSOR_TYPE_PROXIMITY))) {
		ssp_infof("Skip this function!, proximity sensor is not connected(0x%llx)",
			data->uSensorState);
		return;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	if (msg == NULL)
		return;

	msg->cmd = MSG2SSP_AP_SENSOR_PROXTHRESHOLD;
	msg->options = AP2HUB_WRITE;
	msg->free_buffer = 0;

	if (data->prox_type == PROX_LIGHT_CM36657) {
		msg->buffer = buffer;
		msg->length = 8;

		msg->buffer[0] = ((char) (data->uProxHiThresh >> 8) & 0xff);
		msg->buffer[1] = (char) data->uProxHiThresh;
		msg->buffer[2] = ((char) (data->uProxLoThresh >> 8) & 0xff);
		msg->buffer[3] = (char) data->uProxLoThresh;
		msg->buffer[4] = ((char) (data->uProxHiThresh_detect >> 8) & 0xff);
		msg->buffer[5] = (char) data->uProxHiThresh_detect;
		msg->buffer[6] = ((char) (data->uProxLoThresh_detect >> 8) & 0xff);
		msg->buffer[7] = (char) data->uProxLoThresh_detect;
	} else {
		msg->buffer = buffer;
		msg->length = 4;

		msg->buffer[0] = (char) data->uProxHiThresh;
		msg->buffer[1] = (char) data->uProxLoThresh;
		msg->buffer[2] = (char) data->uProxHiThresh_detect;
		msg->buffer[3] = (char) data->uProxLoThresh_detect;
	}

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		ssp_err("SENSOR_PROXTHRESHOLD CMD fail %d", ret);
		return;
	}

	ssp_info("Proximity Threshold - %u, %u, %u, %u", data->uProxHiThresh, data->uProxLoThresh,
		data->uProxHiThresh_detect, data->uProxLoThresh_detect);
}

void set_light_coef(struct ssp_data *data)
{
	int iRet = 0;
	struct ssp_msg *msg;

	/* buffer aligned */
	static int buffer[7] __aligned(4) = { 0, };

	if (data->prox_type != PROX_LIGHT_CM36657) {
		memset(buffer, 0, sizeof(buffer));

		if (!(data->uSensorState & (1 << SSP_SENSOR_TYPE_LIGHT))) {
			pr_info("[SSP]: %s - Skip this function!!!,"\
				"light sensor is not connected(0x%llx)\n",
				__func__, data->uSensorState);
			return;
		}

		msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
		if (msg == NULL) {
			pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
				__func__);
			return;
		}

		msg->cmd = MSG2SSP_AP_SET_LIGHT_COEF;
		msg->length = sizeof(data->light_coef);
		msg->options = AP2HUB_WRITE;
		msg->buffer = (char *)buffer;
		msg->free_buffer = 0;

		memcpy(msg->buffer, data->light_coef, sizeof(data->light_coef));

		iRet = ssp_spi_async(data, msg);

		if (iRet != SUCCESS) {
			pr_err("[SSP]: %s - MSG2SSP_AP_SET_LIGHT_COEF CMD fail %d\n",
				__func__, iRet);
			return;
		}

		pr_info("[SSP]: %s - %d %d %d %d %d %d %d\n", __func__,
			data->light_coef[0], data->light_coef[1], data->light_coef[2],
			data->light_coef[3], data->light_coef[4], data->light_coef[5], data->light_coef[6]);
	}
}


void set_proximity_barcode_enable(struct ssp_data *data, bool bEnable)
{
	int ret = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	/* buffer aligned */
	static char buffer[1] __aligned(4) = {0 };

	memset(buffer, 0, sizeof(buffer));

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_BARCODE_EMUL;
	msg->length = 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	data->is_barcode_enabled = bEnable;
	msg->buffer[0] = bEnable;

	ret = ssp_spi_async(data, msg);
	if (ret != SUCCESS) {
		ssp_errf("SENSOR_BARCODE_EMUL CMD fail %d", ret);
		return;
	}

	ssp_info("Proximity Barcode En : %u", bEnable);
}

void set_gesture_current(struct ssp_data *data, unsigned char uData1)
{
	int ret = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_GESTURE_CURRENT;
	msg->length = 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = kzalloc(1, GFP_KERNEL);
	msg->free_buffer = 1;

	msg->buffer[0] = uData1;
	ret = ssp_spi_async(data, msg);
	if (ret != SUCCESS) {
		ssp_errf("SENSOR_GESTURE_CURRENT CMD fail %d", ret);
		return;
	}

	ssp_info("Gesture Current Setting - %u", uData1);
}

uint64_t get_sensor_scanning_info(struct ssp_data *data)
{
	int ret = 0, z = 0;

	/* buffer aligned */
	static uint64_t result __aligned(4);

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	char sensor_scanning_state[SSP_SENSOR_TYPE_MAX + 1];

	/* buffer aligned */
	result = 0;

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_SENSOR_SCANNING;
	msg->length = sizeof(result);
	msg->options = AP2HUB_READ;
	msg->buffer = (char *)&result;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);
	if (ret < 0)
		ssp_errf("MSG2SSP_AP_SENSOR_SCANNING fail %d", ret);

	sensor_scanning_state[SSP_SENSOR_TYPE_MAX] = '\0';
	for (z = 0; z < SSP_SENSOR_TYPE_MAX; z++)
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - z]
			= (result &(1 << z)) ? '1' : '0';

	ssp_info("state: %s", sensor_scanning_state);

	/*change state format same as flagship*/
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX] = '\0';
	for (z = 0; z < BIG_DATA_SENSOR_TYPE_MAX; z++)
		data->sensor_state[z] = '0';

	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_ACCELEROMETER] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_ACCELEROMETER];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_GYROSCOPE] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_GYROSCOPE];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_GEOMAGNETIC_POWER] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_GEOMAGNETIC_POWER];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_GEOMAGNETIC_FIELD] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_GEOMAGNETIC_FIELD];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_PRESSURE] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_PRESSURE];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_PROXIMITY] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_PROXIMITY];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_LIGHT] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_LIGHT];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX-1-BIG_DATA_SENSOR_TYPE_PROXIMITY_RAW] =
		sensor_scanning_state[SSP_SENSOR_TYPE_MAX - 1 - SSP_SENSOR_TYPE_PROXIMITY_RAW];

	ssp_info("state2: %s", data->sensor_state);

	return result;
}

unsigned int get_firmware_rev(struct ssp_data *data)
{
	int ret;

	/* buffer aligned */
	static u32 result __aligned(4);
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	/* buffer aligned */
	result = SSP_INVALID_REVISION;

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_FIRMWARE_REV;
	msg->length = 4;
	msg->options = AP2HUB_READ;
	msg->buffer = (char *)&result;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);
	if (ret != SUCCESS)
		ssp_errf("transfer fail %d", ret);

	return result;
}

int set_big_data_start(struct ssp_data *data, u8 type, u32 length)
{
	int ret = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);

	/* buffer aligned */
	static char buffer[5] __aligned(4) = {0, };

	memset(buffer, 0, sizeof(buffer));

	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_START_BIG_DATA;
	msg->length = 5;
	msg->options = AP2HUB_WRITE;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	msg->buffer[0] = type;
	memcpy(&msg->buffer[1], &length, 4);

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		ssp_errf("spi fail %d", ret);
		ret = ERROR;
	}

	return ret;
}

int set_time(struct ssp_data *data)
{
	int ret;
	struct ssp_msg *msg;
	struct timespec ts;
	struct rtc_time tm;

	/* buffer aligned */
	static char buffer[12] __aligned(4) = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	memset(buffer, 0, sizeof(buffer));

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	ssp_infof("%d-%02d-%02d %02d:%02d:%02d.%09lu UTC",
		tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min,
		tm.tm_sec, ts.tv_nsec);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_SET_TIME;
	msg->length = 12;
	msg->options = AP2HUB_WRITE;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	msg->buffer[0] = tm.tm_hour;
	msg->buffer[1] = tm.tm_min;
	msg->buffer[2] = tm.tm_sec;
	msg->buffer[3] = tm.tm_hour > 11 ? 64 : 0;
	msg->buffer[4] = tm.tm_wday;
	msg->buffer[5] = tm.tm_mon + 1;
	msg->buffer[6] = tm.tm_mday;
	msg->buffer[7] = tm.tm_year % 100;
	memcpy(&msg->buffer[8], &ts.tv_nsec, 4);

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		ssp_errf("spi fail %d", ret);
		ret = ERROR;
	}

	return ret;
}

int get_time(struct ssp_data *data)
{
	int ret;
	struct ssp_msg *msg;
	struct timespec ts;
	struct rtc_time tm;

	/* buffer aligned */
	static char buffer[12] __aligned(4) = { 0, };

	memset(buffer, 0, sizeof(buffer));

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	ssp_infof("ap %d-%02d-%02d %02d:%02d:%02d.%09lu UTC",
		tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min,
		tm.tm_sec, ts.tv_nsec);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	if (msg == NULL) {
		ssp_errf("failed to alloc memory for ssp_msg");
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_MCU_GET_TIME;
	msg->length = 12;
	msg->options = AP2HUB_READ;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret != SUCCESS) {
		ssp_err("spi failed %d", ret);
		return 0;
	}

	tm.tm_hour = buffer[0];
	tm.tm_min = buffer[1];
	tm.tm_sec = buffer[2];
	tm.tm_mon = msg->buffer[5] - 1;
	tm.tm_mday = buffer[6];
	tm.tm_year = buffer[7] + 100;
	rtc_tm_to_time(&tm, &ts.tv_sec);
	memcpy(&ts.tv_nsec, &msg->buffer[8], 4);

	rtc_time_to_tm(ts.tv_sec, &tm);
	ssp_infof("mcu %d-%02d-%02d %02d:%02d:%02d.%09lu UTC",
		tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min,
		tm.tm_sec, ts.tv_nsec);

	return ret;
}

void set_gyro_cal_lib_enable(struct ssp_data *data, bool bEnable)
{
	int ret = 0;
	u8 cmd;
	struct ssp_msg *msg;

	/* buffer aligned */
	static char buffer __aligned(4);
	buffer = 0;

	pr_info("[SSP] %s - enable %d(cur %d)\n", __func__, bEnable, data->gyro_lib_state);

	if (bEnable)
		cmd = SH_MSG2AP_GYRO_CALIBRATION_START;
	else
		cmd = SH_MSG2AP_GYRO_CALIBRATION_STOP;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL | GFP_DMA);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return;
	}
	msg->cmd = cmd;
	msg->length = 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = &buffer;
	msg->free_buffer = 0;

	msg->buffer[0] = bEnable;

	ret = ssp_spi_async(data, msg);

	if (ret == SUCCESS) {
		if (bEnable)
			data->gyro_lib_state = GYRO_CALIBRATION_STATE_REGISTERED;
		else
			data->gyro_lib_state = GYRO_CALIBRATION_STATE_DONE;
	} else
		pr_err("[SSP] %s - gyro lib enable cmd fail\n", __func__);

}
