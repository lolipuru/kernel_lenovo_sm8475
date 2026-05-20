/*
 * aw8680x.c   aw8680x touch key module
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 *  Author: JiaHuanQing <jiahuanqing@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/input/touch_event_notify.h>
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#endif
#endif
#include "aw_protocol_config.h"
#include "aw_protocol_map.h"
#include "aw_com_protocol.h"
#include "aw_protocol_fun.h"
#include "aw_soc_protocol_interface.h"
#include "aw_bin_parse.h"
#include "aw8680x.h"
/******************************************************
*
* Marco
*
******************************************************/
#define AW8680X_I2C_NAME "aw8680x_touch"
#define AW8680X_NAME "ndt"
#define AW8680X_DRIVER_VERSION "v0.1.9"
#define AW8680X_I2C_RETRIES 3
#define AW8680X_BIN_INIT_BOOT_DELAY 5000
#define AW8680X_BIN_INIT_ADB_DELAY 10
#define AW8680X_ACTIVE_POLLING_DELAY 10
#define AW8680X_DILE_POLLING_DELAY 100

struct aw8680x *g_aw8680x;
/* The name of the bin file that needs to be
 * obtained from /system/vendor/firmware
 */

#if defined(CONFIG_DRM)
static struct drm_panel *active_panel;
#endif
static char *aw8680x_flash_bin = "aw8680x_flash.bin";
static char *aw8680x_sram_bin = "aw8680x_sram.bin";
#if 0
uint32_t aw8680x_input_x;
uint32_t aw8680x_input_y;
#endif
static int aw8680x_drm_suspend(struct aw8680x *aw8680x);
static int aw8680x_drm_resume(struct aw8680x *aw8680x);

/******************************************************
 *
 * aw8680x i2c write/read
 *
 ******************************************************/
static int i2c_writes(struct aw8680x *aw8680x)
{
	int ret = -1;

	struct i2c_msg msgs[] = {
		{
			.addr = aw8680x->i2c->addr,
			.flags = 0,
			.len = aw8680x->p_gui_data_s.soc_data_len,
			.buf = aw8680x->p_protocol_tx_data,
		},
	};

	ret = i2c_transfer(aw8680x->i2c->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s: i2c read error: %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int i2c_reads(struct aw8680x *aw8680x)
{
	int ret = -1;

	struct i2c_msg msgs[] = {
		{
			.addr = aw8680x->i2c->addr,
			.flags = I2C_M_RD,
			.len = aw8680x->p_gui_data_s.ui_rd_data_len,
			.buf = aw8680x->p_protocol_rx_data,
		},
	};

	ret = i2c_transfer(aw8680x->i2c->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s: i2c read error: %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int aw8680x_i2c_writes(struct aw8680x *aw8680x)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW8680X_I2C_RETRIES) {
		ret = i2c_writes(aw8680x);
		if (ret < 0) {
			pr_err("%s: i2c_writes cnt=%d error=%d\n", __func__,
			       cnt, ret);
		} else {
			break;
		}
		cnt++;
	}

	return ret;
}

static int aw8680x_i2c_reads(struct aw8680x *aw8680x)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW8680X_I2C_RETRIES) {
		ret = i2c_reads(aw8680x);
		if (ret < 0) {
			pr_err("%s: i2c_reads cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
	}

	return ret;
}

static int aw8680x_i2c_write(struct aw8680x *aw8680x,
			     unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW8680X_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw8680x->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		msleep(1);
	}

	return ret;
}

static int aw8680x_i2c_read(struct aw8680x *aw8680x,
			    unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW8680X_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8680x->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(1);
	}

	return ret;
}

static int aw8680x_i2c_reads_data(struct aw8680x *aw8680x,
				  unsigned char reg_addr, unsigned int len)
{
	int ret = -1;

	struct i2c_msg msgs[] = {
		{
			.addr = aw8680x->i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg_addr,
		},
		{
			.addr = aw8680x->i2c->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = aw8680x->read_data,
		},
	};

	ret = i2c_transfer(aw8680x->i2c->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s: i2c read error: %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int aw8680x_i2c_writes_data(struct aw8680x *aw8680x,
				   unsigned char reg_addr, unsigned char *buf,
				   unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw8680x->i2c, data, len + 1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);
	kfree(data);

	return ret;
}
/******************************************************
 *
 * get touch data
 *
 *****************************************************/
static int aw8680x_get_key_data(struct aw8680x *aw8680x)
{
	int ret = -1;
	unsigned char key_val = 0;

	ret = aw8680x_i2c_read(aw8680x, KEY_DATA_ADDR, &key_val);
	if (ret < 0) {
		dev_err(aw8680x->dev,
			"%s: failed to read KEY_DATA_ADDR : %d\n",
			__func__, ret);
		return -EIO;
	}
	aw8680x->info.key_data = key_val;
	pr_debug("aw8680x->info.key_data = 0x%x\n", aw8680x->info.key_data);

	return 0;
}

int aw8680x_get_adc_data(struct aw8680x *aw8680x)
{
	int ret = -1;
	int i = 0;

	ret = aw8680x_i2c_reads_data(aw8680x, ADC_DATA_ADDR, ADC_DATA_LEN);
	if (ret < 0) {
		dev_err(aw8680x->dev,
			"%s: failed to read ADC_DATA_ADDR, ret is : %d\n",
			__func__, ret);
		return -EIO;
	}
	memcpy(&(aw8680x->info.adc_data), aw8680x->read_data, ADC_DATA_LEN);
	for (i = 0; i < ADC_DATA_NUM; i++) {
		pr_info("aw8680x->info.adc_data[%d] = 0x%x\n", i,
						aw8680x->info.adc_data[i]);
	}

	return 0;
}

int aw8680x_get_raw_data(struct aw8680x *aw8680x)
{
	int ret = -1;
	int i = 0;

	ret = aw8680x_i2c_reads_data(aw8680x, RAW_DATA_ADDR, RAW_DATA_LEN);
	if (ret < 0) {
		dev_err(aw8680x->dev,
			"%s: failed to read RAW_DATA_ADDR, ret is : %d\n",
			__func__, ret);
		return -EIO;
	}
	memcpy(&(aw8680x->info.raw_data), aw8680x->read_data, RAW_DATA_LEN);
	for (i = 0; i < RAW_DATA_NUM; i++) {
		pr_info("aw8680x->info.raw_data[%d] = 0x%x\n", i,
						aw8680x->info.raw_data[i]);
	}

	return 0;
}

int aw8680x_get_force_data(struct aw8680x *aw8680x)
{
	int ret = -1;
	int i = 0;

	ret = aw8680x_i2c_reads_data(aw8680x, FORCE_DATA_ADDR, FORCE_DATA_LEN);
	if (ret < 0) {
		dev_err(aw8680x->dev,
			"%s: failed to read FORCE_DATA_ADDR, ret is : %d\n",
			__func__, ret);
		return -EIO;
	}
	memcpy(&(aw8680x->info.force_data), aw8680x->read_data, FORCE_DATA_LEN);
	for (i = 0; i < FORCE_DATA_NUM; i++) {
		pr_info("aw8680x->info.force_data[%d] = 0x%x\n", i,
						aw8680x->info.force_data[i]);
	}

	return 0;
}

int aw8680x_get_base_data(struct aw8680x *aw8680x)
{
	int ret = -1;
	int i = 0;

	ret = aw8680x_i2c_reads_data(aw8680x, BASE_DATA_ADDR, BASE_DATA_LEN);
	if (ret < 0) {
		dev_err(aw8680x->dev,
			"%s: failed to read BASE_DATA_ADDR, ret is : %d\n",
			__func__, ret);
		return -EIO;
	}
	memcpy(&(aw8680x->info.base_data), aw8680x->read_data, BASE_DATA_LEN);
	for (i = 0; i < BASE_DATA_NUM; i++) {
		pr_info("aw8680x->info.base_data[%d] = 0x%x\n", i,
						aw8680x->info.base_data[i]);
	}

	return 0;
}

int aw8680x_get_diff_data(struct aw8680x *aw8680x)
{
	int ret = -1;
	int i = 0;

	ret = aw8680x_i2c_reads_data(aw8680x, DIFF_DATA_ADDR, DIFF_DATA_LEN);
	if (ret < 0) {
		dev_err(aw8680x->dev,
			"%s: failed to read DIFF_DATA_ADDR, ret is : %d\n",
			__func__, ret);
		return -EIO;
	}
	memcpy(&(aw8680x->info.diff_data), aw8680x->read_data, DIFF_DATA_LEN);
	for (i = 0; i < DIFF_DATA_NUM; i++) {
		pr_info("aw8680x->info.diff_data[%d] = 0x%x\n", i,
						aw8680x->info.diff_data[i]);
	}

	return 0;
}

int aw8680x_get_press_threshold(struct aw8680x *aw8680x)
{
	int ret = -1;
	int i = 0;

	ret = aw8680x_i2c_reads_data(aw8680x, PRESS_THRESHOLD_ADDR,
							PRESS_THRESHOLD_LEN);
	if (ret < 0) {
		dev_err(aw8680x->dev,
			"%s: failed to read PRESS_THRESHOLD_ADDR, ret is : %d\n",
			__func__, ret);
		return -EIO;
	}
	memcpy(&(aw8680x->info.press_threshold), aw8680x->read_data,
							PRESS_THRESHOLD_LEN);
	for (i = 0; i < PRESS_THRESHOLD_LEN; i++) {
		pr_info("aw8680x->info.press_threshold[%d] = 0x%x\n", i,
					aw8680x->info.press_threshold[i]);
	}

	return 0;
}

int aw8680x_ori_mode = 0;
#ifdef CONFIG_QGKI
void aw8680x_get_ori_mode(int rotation)
{
	pr_info("aw8680x: rotation = %d\n", rotation);
	switch (rotation) {
	case 0:
	case 1:
	case 2:
		aw8680x_ori_mode = rotation;
		break;
	default:
		aw8680x_ori_mode = 0;
	}
	return;
}
EXPORT_SYMBOL(aw8680x_get_ori_mode);
#endif

#define LCD_VERTICAL	0
#define LCD_TRANSVERSE	1
#define LCD_REVERSE		2
static void aw8680x_tp_select(int half_screen[], int *input_xy, int status)
{
	if (status == LCD_VERTICAL || status == LCD_TRANSVERSE) {
		if (half_screen[0] > input_xy[0]) {
			half_screen[0] = input_xy[0];
			half_screen[1] = input_xy[1];
			half_screen[2] = input_xy[2];
		}
	} else if (status == LCD_REVERSE) {
		if (half_screen[0] < input_xy[0]) {
			half_screen[0] = input_xy[0];
			half_screen[1] = input_xy[1];
			half_screen[2] = input_xy[2];
		}
	}
}

#define MAX_PIXEL 5000
#define RESET_BASELINE 0x05
static void aw8680x_tp_process(struct aw8680x *aw8680x, int top_screen[], int lower_screen[])
{
	static int top_id = -1;
	static int lower_id = -1;
	unsigned char buf[1] = {0};

	/* top half screen coordinate processing */
	if (top_screen[0] == MAX_PIXEL) {
		aw8680x->top_screen_x = -1;
		aw8680x->top_screen_y = -1;
	} else if (top_id != top_screen[2]) {
		aw8680x->top_screen_x = top_screen[0];
		aw8680x->top_screen_y = top_screen[1];
		//reser top half screen baseline
		if (aw8680x->last_status[0] == 1) {
			buf[0] = 0;
			aw8680x_i2c_writes_data(aw8680x, RESET_BASELINE, buf, 1);
		}
	} else {
		aw8680x->top_screen_x = top_screen[0];
		aw8680x->top_screen_y = top_screen[1];
	}
	top_id = top_screen[2];

	/* lower half screen coordinate processing */
	if (lower_screen[0] == MAX_PIXEL) {
		aw8680x->lower_screen_x = -1;
		aw8680x->lower_screen_y = -1;
	} else if (lower_id != lower_screen[2]) {
		aw8680x->lower_screen_x = lower_screen[0];
		aw8680x->lower_screen_y = lower_screen[1];
		//reser lower half screen baseline
		if (aw8680x->last_status[1] == 1) {
			buf[0] = 1;
			aw8680x_i2c_writes_data(aw8680x, RESET_BASELINE, buf, 1);
		}
	} else {
		aw8680x->lower_screen_x = lower_screen[0];
		aw8680x->lower_screen_y = lower_screen[1];
	}
	lower_id = lower_screen[2];
}

int input_xy[10][3];
unsigned int point_num = 0;
/* input_xy[num][0] = x, input_xy[num][1] = y, input_xy[num][2] = id */
void aw8680x_get_tp_data(struct aw8680x *aw8680x, int input_xy[10][3], int point_num)
{
	int ret = 0, i;
	char tp_data[10] = {0};
	int top_screen[3] = {0};
	int lower_screen[3] = {0};
	int check_sum = 0;
	int status = aw8680x_ori_mode;
	static int last_point[2][2] = {0};

	if (status == LCD_VERTICAL ||status == LCD_TRANSVERSE ) {
		for (i = 0; i < 2; i++) {
			top_screen[i] = MAX_PIXEL;
			lower_screen[i] = MAX_PIXEL;
		}
	} else if (status == LCD_REVERSE) {
		for (i = 0; i < 2; i++) {
			top_screen[i] = -1;
			lower_screen[i] = -1;
		}
	}
	top_screen[2] = -1;
	lower_screen[2] = -1;

	for (i = 0; i < point_num; i++) {
		pr_debug("%s x=%d, y=%d\n", __func__, input_xy[i][0], input_xy[i][1]);
		if ((input_xy[i][0] >= aw8680x->screen_xrange[0]) &&
			(input_xy[i][0] <= aw8680x->screen_xrange[1]) &&
			(input_xy[i][1] >= aw8680x->screen_yrange[0]) &&
			(input_xy[i][1] <= aw8680x->screen_yrange[1])) {
			aw8680x_tp_select(top_screen, &input_xy[i][0], status);
		} else if ((input_xy[i][0] >= aw8680x->screen_xrange[2]) &&
				   (input_xy[i][0] <= aw8680x->screen_xrange[3]) &&
				   (input_xy[i][1] >= aw8680x->screen_yrange[2]) &&
				   (input_xy[i][1] <= aw8680x->screen_yrange[3])) {
			aw8680x_tp_select(lower_screen, &input_xy[i][0], status);
		}
	}

	aw8680x_tp_process(aw8680x, top_screen, lower_screen);
	if ((last_point[0][0] == aw8680x->top_screen_x) && (last_point[0][1] == aw8680x->top_screen_y) &&
		(last_point[1][0] == aw8680x->lower_screen_x) && (last_point[1][1] == aw8680x->lower_screen_y)) {
		pr_debug("%s point not update\n", __func__);
		return;
	}

	check_sum = -(aw8680x->top_screen_x + aw8680x->top_screen_y +
				aw8680x->lower_screen_x + aw8680x->lower_screen_y);
	tp_data[0] = aw8680x->top_screen_x;
	tp_data[1] = aw8680x->top_screen_x >> 8;
	tp_data[2] = aw8680x->top_screen_y;
	tp_data[3] = aw8680x->top_screen_y >> 8;
	tp_data[4] = aw8680x->lower_screen_x;
	tp_data[5] = aw8680x->lower_screen_x >> 8;
	tp_data[6] = aw8680x->lower_screen_y;
	tp_data[7] = aw8680x->lower_screen_y >> 8;
	tp_data[8] = check_sum;
	tp_data[9] = check_sum >> 8;
	pr_debug("%s x0=%d, y0=%d, x1=%d, y1=%d\n", __func__, aw8680x->top_screen_x,
		aw8680x->top_screen_y, aw8680x->lower_screen_x, aw8680x->lower_screen_y);
	for( i = 0; i < TP_DATA_LEN; i++) {
		pr_debug("%s tp_data[%d] = %d\n", __func__, i, tp_data[i]);
	}
	ret = aw8680x_i2c_writes_data(aw8680x, TP_DATA_ADDR, tp_data, TP_DATA_LEN);
	if(ret < 0)
		pr_err("aw8680x Press down tp data send mcu fail\n");

	last_point[0][0] = aw8680x->top_screen_x;
	last_point[0][1] = aw8680x->top_screen_y;
	last_point[1][0] = aw8680x->lower_screen_x;
	last_point[1][1] = aw8680x->lower_screen_y;
}

/*
static void aw8680x_get_leave_tp_data(struct aw8680x *aw8680x)
{
	int i = 0;
	int ret = 0;
	char tp_data[10] = {0};

	aw8680x->top_screen_x = -1;
	aw8680x->top_screen_y = -1;
	aw8680x->lower_screen_x = -1;
	aw8680x->lower_screen_y = -1;
	for( i = 0; i < TP_DATA_LEN - 2; i++) {
		tp_data[i] = 0xff;
	}
	tp_data[8] = 0x4;
	tp_data[9] = 0x0;
	pr_debug("%s \n", __func__);
	ret = aw8680x_i2c_writes_data(g_aw8680x, TP_DATA_ADDR, tp_data, TP_DATA_LEN);
	if(ret < 0)
		pr_err("aw8680x Lift up tp data send mcu fail\n");
}
*/

static void aw8680x_report_tp_work(struct work_struct *work)
{
    struct aw8680x *aw8680x = container_of(work, struct aw8680x,
                                  report_tp_work);

    aw8680x_get_tp_data(aw8680x, input_xy, point_num);
}

static int aw8680x_fb_notifier_callback_tp(struct notifier_block *self, unsigned long event, void *data)
{
	struct aw8680x *aw8680x = container_of(self, struct aw8680x, fb_notif);
	struct touch_event *tp_event = (struct touch_event *)data;
	int i;

	if (aw8680x->suspend_mode == true)
		return 0;

	point_num = (unsigned int)event;
	pr_debug("%s: tp event = %c, n=%d\n", __func__, tp_event[0].type, point_num);
	if (tp_event[0].type == 'D') {
		for (i = 0; i< point_num; i++) {
			input_xy[i][0] =  tp_event[i].x;
			input_xy[i][1] =  tp_event[i].y;
			input_xy[i][2] =  tp_event[i].fid;
		}
	} else if(tp_event[0].type == 'U'){
		point_num = 0;
	}
	queue_work(aw8680x->tp_workqueue, &aw8680x->report_tp_work);
/*
	if (tp_event->type == 'D') {
		aw8680x_get_press_tp_data(aw8680x, tp_event->x, tp_event->y);
	} else if(tp_event->type == 'U'){
		aw8680x_get_leave_tp_data(aw8680x);
	}
*/
	return 0;
}
/*****************************************************
 *
 * aw8680x rst
 *
 *****************************************************/
static void aw8680x_hw_reset(struct aw8680x *aw8680x)
{
	pr_info("%s enter\n", __func__);

	gpio_set_value_cansleep(aw8680x->reset_gpio, 1);
	udelay(150);
	gpio_set_value_cansleep(aw8680x->reset_gpio, 0);
	mdelay(RESET_INIT_TIME);
}

/*****************************************************
 *
 * aw8680x scan mode select
 *
 *****************************************************/
static void aw8680x_wake_pin(struct aw8680x *aw8680x)
{
	pr_debug("%s enter\n", __func__);

	gpio_set_value_cansleep(aw8680x->wake_gpio, 1);
	mdelay(1);
	gpio_set_value_cansleep(aw8680x->wake_gpio, 0);
}

static void aw8680x_active_mode_set(struct aw8680x *aw8680x)
{
	pr_debug("%s enter\n", __func__);

	aw8680x_wake_pin(aw8680x);
	aw8680x_i2c_write(aw8680x, REG_SCAN_MODE_SWITCH_ADDR, HIGH_SPEED);
	aw8680x->polling_cycle = AW8680X_ACTIVE_POLLING_DELAY;
}

static void aw8680x_idle_mode_set(struct aw8680x *aw8680x)
{
	pr_debug("%s enter\n", __func__);

	aw8680x_wake_pin(aw8680x);
	aw8680x_i2c_write(aw8680x, REG_SCAN_MODE_SWITCH_ADDR, LOW_SPEED);
	aw8680x->polling_cycle = AW8680X_DILE_POLLING_DELAY;
}

static void aw8680x_sleep_mode_set(struct aw8680x *aw8680x)
{
	pr_debug("%s enter\n", __func__);

	aw8680x_i2c_write(aw8680x, REG_SCAN_MODE_SWITCH_ADDR, POWER_OFF);
}

/****************************************************************
 *
 * attribute  function
 *
 ****************************************************************/
static void aw8680x_set_common_info(struct aw8680x *aw8680x,
				    unsigned short soc_data_len,
				    unsigned char module_id,
				    unsigned char event_id)
{
	/* Configure some command information
	 * Effective data length
	 */
	aw8680x->p_gui_data_s.soc_data_len = soc_data_len;
	aw8680x->p_gui_data_s.module_id = module_id; /* module id */
	aw8680x->p_gui_data_s.event_id = event_id; /* evnet id */
	aw8680x->module_id = module_id;
}

/* pack data and i2c_write data to mcu */
static int aw8680x_send(struct aw8680x *aw8680x)
{
	int ret = -1;

	/* Pack instruction data according to soc protocol */
	ret = aw_soc_protocol_pack_interface(&(aw8680x->p_gui_data_s),
						aw8680x->p_protocol_tx_data);
	if (ret != 0) {
		pr_err("%s soc data update pack fail!", __func__);
		return ret;
	}
	/* Send the packaged data to the MCU through the i2c write interface */
	ret = aw8680x_i2c_writes(aw8680x);
	if (ret < 0)
		return ret;

	return 0;
}

/* i2c_read ack_data from IC and unpack ack_data */
static int aw8680x_ack(struct aw8680x *aw8680x)
{
	int ret = -1;
	/* Read the information processed by the instruction
	 * through the i2c read interface
	 */
	ret = aw8680x_i2c_reads(aw8680x);
	if (ret < 0)
		return ret;
	/* Unpack instruction data according to soc protocol */
	ret = aw_soc_protocol_unpack_interface(&(aw8680x->p_gui_data_s),
						aw8680x->p_protocol_rx_data);
	if ((aw8680x->p_gui_data_s.module_id == aw8680x->module_id) &&
	    (ret == 0) &&
	    (aw8680x->p_gui_data_s.err_flag == 0) &&
	    (aw8680x->p_protocol_rx_data[7] == 1))
		return 0;

	return -ERR_FLAG;
}

/* Get information in ack data area */
static void aw8680x_get_info(struct aw8680x *aw8680x)
{
	/* connect */
	if ((aw8680x->p_gui_data_s.module_id == HANDSHAKE_ID)
	    && (aw8680x->p_gui_data_s.event_id == CONNECT_ACK_ID))
		aw8680x->app_current_addr = aw8680x->p_gui_data_s.soc_data[0] |
				(aw8680x->p_gui_data_s.soc_data[1] << 8) |
				(aw8680x->p_gui_data_s.soc_data[2] << 16) |
				(aw8680x->p_gui_data_s.soc_data[3] << 24);
	/* flash read */
	if ((aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(aw8680x->p_gui_data_s.event_id == FLASH_READ_ACK_ID))
		memcpy(aw8680x->read_data, aw8680x->p_gui_data_s.soc_data,
							aw8680x->read_len);
}

static void aw8680x_send_delay(struct aw8680x *aw8680x)
{
	/* erase sector delay */
	if ((aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(aw8680x->p_gui_data_s.event_id == FLASH_ERASE_ID)) {
		mdelay(aw8680x->p_gui_data_s.read_len * 11);
	/* write flash delay */
	} else if ((aw8680x->p_gui_data_s.module_id == FLASH_ID) &&
		(aw8680x->p_gui_data_s.event_id == FLASH_WRITE_ID)) {
		mdelay(2);
	/* write sram delay */
	} else if ((aw8680x->p_gui_data_s.module_id == RAM_ID) &&
		(aw8680x->p_gui_data_s.event_id == RAM_WRITE_ID)) {
		mdelay(2);
	/* other delay */
	} else {
		udelay(80);
	}
}

static void aw8680x_ack_delay(struct aw8680x *aw8680x)
{
	int delay_count = 0;

	/* jump delay */
	if (aw8680x->p_gui_data_s.module_id == END_ID)
		mdelay(JUMP_INIT_TIME);
	/* Flash option failure handling mechanism */
	while ((aw8680x->p_gui_data_s.module_id == FLASH_ID)
	       && (aw8680x->p_gui_data_s.event_id == FLASH_ERASE_ACK_ID)
	       && (aw8680x->ack_flag != 0)) {
		mdelay(5);
		delay_count += 1;
		aw8680x->ack_flag = aw8680x_ack(aw8680x);
		if (delay_count == 5)
			break;
	}
}

static int aw8680x_soc_protocol_set(struct aw8680x *aw8680x,
				    unsigned int addr, unsigned short read_len)
{
	int ret = -1;

	/* Configure some command information */
	aw8680x->p_gui_data_s.addr = addr; /* Read or write addr */
	aw8680x->p_gui_data_s.read_len = read_len; /* Read addr data length */

	ret = aw8680x_send(aw8680x);
	if (ret != 0)
		return ret;
	aw8680x_send_delay(aw8680x);
	/* Flash erase failure handling mechanism */
	aw8680x->ack_flag = aw8680x_ack(aw8680x);
	aw8680x_ack_delay(aw8680x);
	if (aw8680x->ack_flag != 0)
		return aw8680x->ack_flag;
	/* get ack information */
	aw8680x_get_info(aw8680x);

	return 0;
}
/***********************************************************
 *
 * ap instruction to ic
 *
 ***********************************************************/
/* Instruction function function, according to different function
 * configuration instructions
 */
static int aw8680x_connect(struct aw8680x *aw8680x)
{
	aw8680x_set_common_info(aw8680x, SOC_DATA_LEN, HANDSHAKE_ID,
				CONNECT_ID);
	return aw8680x_soc_protocol_set(aw8680x, SOC_ADDR, SOC_READ_LEN);
}

static int aw8680x_flash_read(struct aw8680x *aw8680x, unsigned int addr,
			      unsigned int read_len)
{
	aw8680x->read_len = (unsigned short)read_len;
	aw8680x_set_common_info(aw8680x, SOC_DATA_LEN, FLASH_ID, FLASH_READ_ID);
	return aw8680x_soc_protocol_set(aw8680x, addr, aw8680x->read_len);
}

static int aw8680x_jump_flash(struct aw8680x *aw8680x, unsigned int jump_addr)
{
	aw8680x_set_common_info(aw8680x, SOC_DATA_LEN, END_ID, FLASH_JUMP_ID);
	return aw8680x_soc_protocol_set(aw8680x, jump_addr, SOC_READ_LEN);
}

static int aw8680x_jump_sram(struct aw8680x *aw8680x, unsigned int jump_addr)
{
	aw8680x_set_common_info(aw8680x, SOC_DATA_LEN, END_ID, RAM_JUMP_ID);
	return aw8680x_soc_protocol_set(aw8680x, jump_addr, SOC_READ_LEN);
}

static int aw8680x_erase_sector(struct aw8680x *aw8680x, unsigned int addr,
				unsigned int sector_num)
{
	aw8680x_set_common_info(aw8680x, SOC_DATA_LEN, FLASH_ID,
				FLASH_ERASE_ID);
	return aw8680x_soc_protocol_set(aw8680x, addr, sector_num);
}

static int aw8680x_jump_uboot(struct aw8680x *aw8680x)
{
	int ret = -1;
	int connect_count = 5;

	while (connect_count--) {
		aw8680x_hw_reset(aw8680x);
		aw8680x_i2c_write(aw8680x, REG_ADDR, 1);
		mdelay(CHIP_INIT_TIME);
		ret = aw8680x_connect(aw8680x);
		if ((ret == 0) &&
		    ((aw8680x->app_current_addr == CPU_IN_UBOOT) ||
		     (aw8680x->app_current_addr == CPU_IN_FLASH_BOOT)))
			break;
	}
	if ((ret == 0) &&
	    ((aw8680x->app_current_addr == CPU_IN_UBOOT) ||
	     (aw8680x->app_current_addr == CPU_IN_FLASH_BOOT))) {
		pr_info("aw8680x jump uboot success!\n");
		return 0;
	}
	pr_err("aw8680x jump uboot fail! ret is %d\n", ret);

	return -ERR_FLAG;
}

static int aw8680x_flash_cycle_write(struct aw8680x *aw8680x, int cycle_num,
								int effect_len)
{
	memcpy(aw8680x->p_gui_data_s.soc_data, &(aw8680x->flash_bin->info.
		data[aw8680x->flash_bin->header_info[0].valid_data_addr +
				cycle_num * WRITE_FLASH_MAX]), effect_len);
	aw8680x_set_common_info(aw8680x, effect_len, FLASH_ID, FLASH_WRITE_ID);
	return aw8680x_soc_protocol_set(aw8680x, aw8680x->flash_bin->
				header_info[0].download_addr + WRITE_FLASH_MAX *
				cycle_num, SOC_READ_LEN);
}

static int aw8680x_sram_cycle_write(struct aw8680x *aw8680x, int cycle_num,
								int effect_len)
{
	memcpy(aw8680x->p_gui_data_s.soc_data, &(aw8680x->sram_bin->info.
		data[aw8680x->sram_bin->header_info[0].valid_data_addr +
				cycle_num * WRITE_SRAM_MAX]), effect_len);
	aw8680x_set_common_info(aw8680x, effect_len, RAM_ID, RAM_WRITE_ID);
	return aw8680x_soc_protocol_set(aw8680x, aw8680x->sram_bin->
				header_info[0].download_addr + WRITE_SRAM_MAX *
				cycle_num, SOC_READ_LEN);
}

static int aw8680x_flash_write_bin(struct aw8680x *aw8680x)
{
	int i = 0;
	int ret = -1;
	int flash_write_count = 0;
	unsigned short flash_write_last = 0;

	pr_info("flash write data len is %d\n", aw8680x->flash_bin->
						header_info[0].valid_data_len);
	if (aw8680x->flash_bin->header_info[0].valid_data_len >
							WRITE_FLASH_MAX) {
		flash_write_count = aw8680x->flash_bin->header_info[0].
					valid_data_len / WRITE_FLASH_MAX;
		flash_write_last = aw8680x->flash_bin->header_info[0].
					valid_data_len % WRITE_FLASH_MAX;
		pr_info("flash_write_count is %d\n", flash_write_count);
		pr_info("flash_write_last is %d\n", flash_write_last);
		for (i = 0; i < flash_write_count; i++) {
			ret = aw8680x_flash_cycle_write(aw8680x, i,
							WRITE_FLASH_MAX);
			if (ret != 0) {
				pr_info("aw8680x_flash_cycle_write fail!\n");
				return ret;
			}
		}
		if (flash_write_last != 0) {
			ret = aw8680x_flash_cycle_write(aw8680x,
					flash_write_count, flash_write_last);
			if (ret != 0) {
				pr_info("aw8680x_flash_cycle_write last fail!\n");
				return ret;
			}
		}
	} else {
		ret = aw8680x_flash_cycle_write(aw8680x, 0, aw8680x->flash_bin->
						header_info[0].valid_data_len);
		if (ret != 0) {
			pr_info("aw8680x_flash_cycle_write fail!\n");
			return ret;
		}
	}
	pr_info("aw8680x Successfully write all data to flash!!!\n");

	return 0;
}

static int aw8680x_sram_write_bin(struct aw8680x *aw8680x)
{
	int i = 0;
	int ret = -1;
	int sram_write_count = 0;
	unsigned short sram_write_last = 0;

	pr_info("sram write data len is %d\n",
		aw8680x->sram_bin->header_info[0].valid_data_len);
	if (aw8680x->sram_bin->header_info[0].valid_data_len > WRITE_SRAM_MAX) {
		sram_write_count = aw8680x->sram_bin->header_info[0].
					valid_data_len / WRITE_SRAM_MAX;
		sram_write_last = aw8680x->sram_bin->header_info[0].
					valid_data_len % WRITE_SRAM_MAX;
		pr_info("sram_write_count is %d\n", sram_write_count);
		pr_info("sram_write_last is %d\n", sram_write_last);
		for (i = 0; i < sram_write_count; i++) {
			ret = aw8680x_sram_cycle_write(aw8680x, i,
								WRITE_SRAM_MAX);
			if (ret != 0) {
				pr_info("aw8680x_sram_cycle_write fail!\n");
				return ret;
			}
		}
		if (sram_write_last != 0) {
			ret = aw8680x_sram_cycle_write(aw8680x,
					sram_write_count, sram_write_last);
			if (ret != 0) {
				pr_info("aw8680x_sram_cycle_write fail!\n");
				return ret;
			}
		}
	} else {
		ret = aw8680x_sram_cycle_write(aw8680x, 0, aw8680x->sram_bin->
						header_info[0].valid_data_len);
		if (ret != 0) {
			pr_info("aw8680x_sram_cycle_write fail!\n");
			return ret;
		}
	}
	pr_info("aw8680x Successfully write all data to sram!!!\n");
	return 0;
}

/* Erase flash data whose size is the size of the app data to be written */
static int aw8680x_erase_bin_data(struct aw8680x *aw8680x)
{
	int ret = -1;
	int erase_sector_num = 0;
	int erase_count = 5;

	/* First erase the sectors according to the size of
	 * the algorithm program to be written,
	 * the size of each sector is 512,
	 * get the number of sectors to be erased,
	 * and call the aw8680x_erase_sector instruction function to erase
	 */
	if (aw8680x->flash_bin->header_info[0].valid_data_len >
							ERASE_BYTE_MAX) {
		if ((aw8680x->flash_bin->header_info[0].valid_data_len %
							ERASE_BYTE_MAX) == 0)
			erase_sector_num = aw8680x->flash_bin->header_info[0].
						valid_data_len / ERASE_BYTE_MAX;
		else
			erase_sector_num = aw8680x->flash_bin->header_info[0].
					valid_data_len / ERASE_BYTE_MAX + 1;
	} else {
		erase_sector_num = 1;
	}
	pr_info("erase_sector_num is %d\n", erase_sector_num);
	while (erase_count--) {
		ret = aw8680x_erase_sector(aw8680x, aw8680x->flash_bin->
				header_info[0].download_addr, erase_sector_num);
		if (ret == 0)
			break;
	}
	if (ret != 0) {
		pr_err("aw8680x erase_sector of the flash fail\n");
		return ret;
	}
	pr_info("aw8680x erase_sector of the flash success\n");
	return 0;
}

static int aw8680x_update_flash_algorithm(struct aw8680x *aw8680x)
{
	int ret = -1;
	int update_count = 5;
	int connect_count = 5;

	while (update_count--) {
		ret = aw8680x_erase_bin_data(aw8680x);
		if (ret != 0)
			break;
		ret = aw8680x_flash_write_bin(aw8680x);
		if (ret == 0)
			break;
	}
	if (ret != 0) {
		pr_err("aw8680x update flash fail\n");
		return -ERR_FLAG;
	}
	pr_info("aw8680x update flash success\n");
	ret = aw8680x_jump_flash(aw8680x, aw8680x->flash_app_addr);
	if (ret != 0) {
		pr_err("aw8680x jump flash fail\n");
		return -ERR_FLAG;
	}
	pr_info("aw8680x jump flash success\n");
	while (connect_count--) {
		ret = aw8680x_connect(aw8680x);
		if (ret == 0)
			break;
	}
	if (ret != 0) {
		pr_err("after update connect fail!\n");
		return -ERR_FLAG;
	}
	aw8680x->cpu_in_flash = aw8680x->app_current_addr;
	aw8680x->ic_status = true;
	pr_info("after update connect success : aw8680x->cpu_in_flash is 0x%x\n",
							aw8680x->cpu_in_flash);

	return 0;
}

static int aw8680x_sram_update_init(struct aw8680x *aw8680x)
{
	int ret = -1;

	pr_info("%s enter\n", __func__);
	ret = aw8680x_sram_write_bin(aw8680x);
	if (ret != 0) {
		pr_err("%s sram write error\n", __func__);
		return ret;
	}
	ret = aw8680x_jump_sram(aw8680x, aw8680x->sram_addr);
	if (ret != 0)
		pr_err("%s sram jump error\n", __func__);

	return ret;
}

static int aw8680x_sram_update_cycle(struct aw8680x *aw8680x)
{
	int i = 0;
	int ret = -1;

	for (i = 0; i < 5; ++i) {
		ret = aw8680x_sram_update_init(aw8680x);
		if (ret == 0)
			return ret;
		udelay(5);
	}
	pr_err("%s error\n", __func__);

	return ret;
}

static void aw8680x_soc_update_init(struct aw8680x *aw8680x)
{
	int ret = -1;

	ret = aw8680x_jump_uboot(aw8680x);
	if (ret != 0)
		return;
	ret = aw8680x_sram_update_cycle(aw8680x);
	if (ret != 0)
		return;
	ret = aw8680x_update_flash_algorithm(aw8680x);
	if (ret == 0)
		pr_info("%s update success\n", __func__);
}

static void aw8680x_update_init_loaded(const struct firmware *cont,
								void *context)
{
	int ret = 0;
	struct aw8680x *aw8680x = context;

	/* If the bin file is not found,
	 * the algorithm program in the aw8680x_flash_update array
	 * included in the driver is written to flash
	 */
	if (!cont) {
		pr_err("%s: Can't find the bin file! %s\n", __func__,
							aw8680x_sram_bin);
		release_firmware(cont);
		/* If the bin file is found,
		 * copy the found bin file data to cont (this operation is
		 * completed in request_firmware_nowait),
		 * we need to do to copy the data
		 * in cont to our own device structure bin, and release cont
		 */
	} else {
		pr_info("%s: Find the bin file! %s\n", __func__,
							aw8680x_sram_bin);

		aw8680x->sram_bin = devm_kzalloc(aw8680x->dev, cont->size +
					sizeof(struct aw_bin), GFP_KERNEL);
		if (!(aw8680x->sram_bin)) {
			pr_err("%s: failed to allcating memory!\n", __func__);
			return;
		}
		aw8680x->sram_bin->info.len = cont->size;
		memcpy(aw8680x->sram_bin->info.data, cont->data, cont->size);
		release_firmware(cont);
		/* Call the bin parsing interface to parse the frame header of
		 * the bin file to obtain algorithm program data
		 */
		ret = aw_parsing_bin_file(aw8680x->sram_bin);
		if (ret < 0) {
			pr_err("aw8680x parse bin fail!\n");
			return;
		} else if (ret == 0) {
			aw8680x->sram_addr = aw8680x->sram_bin->header_info[0].
								download_addr;
			if (aw8680x->sram_bin->header_info[0].bin_data_type !=
							SOC_APP_DATA_TYPE) {
				pr_err("aw8680x SOC_APP_DATA_TYPE error!\n");
				return;
			} else if ((aw8680x->sram_addr < SRAM_BASE_ADDR) ||
					(aw8680x->sram_addr > SRAM_MAX_ADDR)) {
				pr_err("aw8680x update flash address err,flash download_addr is 0x%x\n",
					aw8680x->sram_bin->header_info[0].
								download_addr);
				return;
			}
			pr_info("aw8680x parse bin success!\n");
			pr_info("parse_chip_type is %s\n", aw8680x->sram_bin->
						header_info[0].chip_type);
			aw8680x_soc_update_init(aw8680x);
		}
	}
}

static int aw8680x_update_init(struct aw8680x *aw8680x)
{
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					aw8680x_sram_bin, &aw8680x->i2c->dev,
					GFP_KERNEL, aw8680x,
					aw8680x_update_init_loaded);
}

static int aw8680x_process_software_version(struct aw8680x *aw8680x)
{
	int ret = -1;
	int software_count = 5;

	while (software_count--) {
		ret = aw8680x_connect(aw8680x);
		if (ret == 0)
			break;
	}
	if (ret != 0) {
		pr_err("aw8680x get software version fail!\n");
		pr_err("Incomplete app data in flash!\n");
		aw8680x_update_init(aw8680x);
		return 0;
	}
	pr_info("aw8680x->flash_bin->header_info[0].app_version is 0x%x\n",
				aw8680x->flash_bin->header_info[0].app_version);
	pr_info("aw8680x->app_current_addr is 0x%x\n",
						aw8680x->app_current_addr);
	/* Compare the existing software version number in flash with the
	 * software version number after bin analysis
	 */
	if (aw8680x->flash_bin->header_info[0].app_version >
						aw8680x->app_current_addr) {
	/*if (1) {*/
		pr_info("aw8680x The parsed software version is higher than the existing software version\n");
		pr_info("aw8680x The algorithm program in flash needs to be updated!\n");
		/* If the existing software version number in flash is
		 * less than the software version number after bin analysis,
		 * you need to update it, first jump to uboot, and then
		 * call the flash algorithm program processing function
		 */
		aw8680x_update_init(aw8680x);
	} else {
		pr_info("aw8680x The parsed software version is lower than or equal to the existing software version\n");
		pr_info("aw8680x The algorithm program in flash does not need to be updated!\n");
		aw8680x->cpu_in_flash = aw8680x->app_current_addr;
		pr_info("not need update : aw8680x->cpu_in_flash is 0x%x\n",
							aw8680x->cpu_in_flash);
		aw8680x->ic_status = true;
		return -NOT_NEED_UPDATE;
	}
	return 0;
}

static int aw8680x_process_current_addr(struct aw8680x *aw8680x)
{
	int ret = -1;

	if (aw8680x->adb_update_flag == true) {
		pr_info("enter adb update\n");
		aw8680x_update_init(aw8680x);
		return 0;
	}
	/* If the location of the CPU after booting is in the uboot, Directly
	 * call Flash algorithm program processing function
	 */
	if ((aw8680x->app_current_addr == CPU_IN_UBOOT) ||
		(aw8680x->app_current_addr == CPU_IN_FLASH_BOOT)) {
		pr_info("cpu boot in uboot!!!\n");
		pr_info("aw8680x->app_current_addr is 0x%x\n",
						aw8680x->app_current_addr);
		ret = aw8680x_jump_flash(aw8680x, aw8680x->flash_app_addr);
		if (ret != 0) {
			pr_info("after jump flash , cpu also in uboot!!!\n");
			pr_info("No program in flash!!!\n");
			aw8680x_update_init(aw8680x);
		} else {
			pr_info("after jump flash , cpu in flash!!!\n");
			pr_info("There are programs in flash!!!\n");
			ret = aw8680x_process_software_version(aw8680x);
			if (ret != 0)
				return ret;
		}
	/* If the location of the CPU after booting is in flash */
	} else {
		pr_info("cpu boot in flash!!!\n");
		/* Get the software version number of the existing
		 * algorithm program in flash
		 */
		ret = aw8680x_process_software_version(aw8680x);
		if (ret != 0)
			return ret;
	}

	return 0;
}

static void aw8680x_flash_init(struct aw8680x *aw8680x)
{
	int ret = -1;

	ret = aw8680x_process_current_addr(aw8680x);
	if (ret == 0) {
	} else if (ret == -NOT_NEED_UPDATE) {
		pr_info("flash no need update!\n");
	} else {
		pr_err("aw8680x flash update fail!\n");
		return;
	}
}

/******************************************************
 *
 * Get and parse the bin file in firmware
 *
 ******************************************************/
static void aw8680x_cfg_loaded(const struct firmware *cont, void *context)
{
	int ret = 0;
	struct aw8680x *aw8680x = context;

	/* If the bin file is not found,
	 * the algorithm program in the aw8680x_flash_update array
	 * included in the driver is written to flash
	 */
	if (!cont) {
		pr_err("%s: Can't find the bin file! %s\n", __func__,
							aw8680x_flash_bin);
		release_firmware(cont);
		/* If the bin file is found,
		 * copy the found bin file data to cont (this operation is
		 * completed in request_firmware_nowait),
		 * we need to do to copy the data
		 * in cont to our own device structure bin, and release cont
		 */
	} else {
		pr_info("%s: Find the bin file! %s\n", __func__,
							aw8680x_flash_bin);
		aw8680x->flash_bin = devm_kzalloc(aw8680x->dev, cont->size +
					sizeof(struct aw_bin), GFP_KERNEL);
		if (!(aw8680x->flash_bin)) {
			pr_err("%s: failed to allcating memory!\n", __func__);
			return;
		}
		aw8680x->flash_bin->info.len = cont->size;
		memcpy(aw8680x->flash_bin->info.data, cont->data, cont->size);
		release_firmware(cont);
		/* Call the bin parsing interface to parse the frame header of
		 * the bin file to obtain algorithm program data
		 */
		ret = aw_parsing_bin_file(aw8680x->flash_bin);
		if (ret < 0) {
			pr_err("aw8680x parse bin fail!\n");
			return;
		} else if (ret == 0) {
			aw8680x->flash_app_addr = aw8680x->flash_bin->
						header_info[0].download_addr;
			if (aw8680x->flash_bin->header_info[0].bin_data_type !=
							SOC_APP_DATA_TYPE) {
				pr_err("aw8680x SOC_APP_DATA_TYPE error!\n");
				return;
			} else if ((aw8680x->flash_app_addr < FLASH_BASE_ADDR)
				|| (aw8680x->flash_app_addr > FLASH_MAX_ADDR)) {
				pr_err("aw8680x update flash address err,flash download_addr is 0x%x\n",
					aw8680x->flash_bin->header_info[0].
								download_addr);
				return;
			}
			pr_info("aw8680x parse bin success!\n");
			pr_info("parse_chip_type is %s\n", aw8680x->flash_bin->
						header_info[0].chip_type);
			aw8680x_flash_init(aw8680x);
		}
	}
}

static int aw8680x_bin_update(struct aw8680x *aw8680x)
{
	/* Go to the /system/vendor/firmware/ directory and
	 * search for the bin file with the name aw8680x_flash_bin,
	 * and then call the aw8680x_cfg_loaded function
	 */
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				aw8680x_flash_bin, &aw8680x->i2c->dev,
				GFP_KERNEL, aw8680x, aw8680x_cfg_loaded);
}

static void aw8680x_bin_work_routine(struct work_struct *work)
{
	ssize_t ret_fir = 0;
	/* Find the position of the structure according to
	 * the position of the members in the structure
	 */
	struct aw8680x *aw8680x = container_of(work, struct aw8680x,
								bin_work.work);

	ret_fir = aw8680x_bin_update(aw8680x);
	if (ret_fir != 0)
		pr_err("%s: request_firmware failed with read %s\n", __func__,
							aw8680x_flash_bin);
}

static void aw8680x_bin_init(struct aw8680x *aw8680x, int cfg_timer_val)
{
	/* Use the work queue function in the kernel,
	 * and call the aw8680x_bin_work_routine function
	 * in the work queue bin_work 5 seconds
	 * after entering the entry function
	 */
	INIT_DELAYED_WORK(&aw8680x->bin_work, aw8680x_bin_work_routine);
	schedule_delayed_work(&aw8680x->bin_work,
					msecs_to_jiffies(cfg_timer_val));
}

/*****************************************************
 *
 * aw8680x polling interface
 *
 *****************************************************/
static int aw8680x_sensor_data(struct aw8680x *aw8680x)
{
	int ret = -1;

	ret = aw8680x_get_adc_data(aw8680x);
	if (ret != 0) {
		pr_err("aw8680x adc data read fail!\n");
		return ret;
	}

	return 0;
}

static int aw8680x_thread_proc(void *data)
{
	struct aw8680x *aw8680x = data;

	while (1) {
		if (kthread_should_stop()) {
			break;
		} else if (aw8680x->polling_cycle ==
			   AW8680X_ACTIVE_POLLING_DELAY) {
			aw8680x_sensor_data(aw8680x);
			mdelay(10);
		} else if (aw8680x->polling_cycle ==
						AW8680X_DILE_POLLING_DELAY) {
			aw8680x_sensor_data(aw8680x);
			mdelay(100);
		} else {
			pr_info("Mode not supported\n");
		}
	}

	return 0;
}

static int aw8680x_polling_start(struct aw8680x *aw8680x)
{
	int err = 0;

	pr_info("%s: enter\n", __func__);
	pr_info("aw8680x->polling_cycle is %d\n", aw8680x->polling_cycle);
	aw8680x_wake_pin(aw8680x);
	aw8680x->thread = kthread_create(aw8680x_thread_proc, (void *)aw8680x,
							"polling_mode");
	if (IS_ERR(aw8680x->thread)) {
		err = PTR_ERR(aw8680x->thread);
		pr_info("kthread_create fail.\n");
		return err;
	}
	wake_up_process(aw8680x->thread);
	pr_info("thread create.\n");

	return 0;
}

static void aw8680x_polling_stop(struct aw8680x *aw8680x)
{
	pr_info("%s: enter\n", __func__);

	if (aw8680x->thread) {
		kthread_stop(aw8680x->thread);
		aw8680x->thread = NULL;
		pr_info("thread stop.\n");
	}

	pr_info("exit thread.\n");
}
/******************************************************
 *
 * attribute
 *
 ******************************************************/
/* Used when debugging adb */
static ssize_t aw8680x_connect_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int connect_count = 5;
	int ret = -1;

	aw8680x_wake_pin(aw8680x);
	while (connect_count--) {
		ret = aw8680x_connect(aw8680x);
		if (ret == 0)
			break;
	}
	if (ret == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len,
							"connect success!\n");
		len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x->app_current_addr is 0x%x\n",
					aw8680x->app_current_addr);
		if ((aw8680x->app_current_addr == CPU_IN_UBOOT)
		    || (aw8680x->app_current_addr == CPU_IN_FLASH_BOOT)) {
			len += snprintf(buf + len, PAGE_SIZE - len,
							"cpu in uboot!\n");
		} else if (aw8680x->app_current_addr == aw8680x->cpu_in_flash) {
			len += snprintf(buf + len, PAGE_SIZE - len,
							"cpu in flash!\n");
		} else {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"cpu is not in uboot or flash!!!\n");
		}
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "connect fail!\n");
	}

	return len;
}

static ssize_t aw8680x_update_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1) {
		pr_info("enter adb update\n");
		aw8680x->adb_update_flag = true;
		aw8680x_bin_init(aw8680x, AW8680X_BIN_INIT_ADB_DELAY);
	}

	return count;
}

static ssize_t aw8680x_jump_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	int jump_count = 5;
	int ret = -1;
	char databuf[10] = { 0 };

	if (sscanf(buf, "%s", databuf) == 1) {
		if ((strcmp(databuf, "flash") == 0)
		    && (aw8680x->app_current_addr != aw8680x->cpu_in_flash)) {
			pr_info("aw8680x databuf is flash\n");
			while (jump_count--) {
				ret = aw8680x_jump_flash(aw8680x,
						aw8680x->flash_app_addr);
				if (ret == 0)
					break;
			}
		} else if ((strcmp(databuf, "uboot") == 0) &&
			(aw8680x->app_current_addr != CPU_IN_UBOOT) &&
			(aw8680x->app_current_addr != CPU_IN_FLASH_BOOT)) {
			pr_info("aw8680x databuf is uboot\n");
			aw8680x_hw_reset(aw8680x);
			aw8680x_i2c_write(aw8680x, REG_ADDR, 1);
			mdelay(CHIP_INIT_TIME);
		} else {
			pr_info("aw8680x databuf is not support!\n");
		}
	}

	return count;
}

static ssize_t aw8680x_flash_read_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	int i = 0;
	ssize_t len = 0;

	if ((aw8680x->app_current_addr == CPU_IN_UBOOT) ||
	    (aw8680x->app_current_addr == CPU_IN_FLASH_BOOT)) {
		if (aw8680x->read_len <= READ_FLASH_MAX) {
			if (aw8680x->ack_flag == 0) {
				len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x flash read success!\n");
				for (i = 0; i < aw8680x->read_len; i++) {
					len += snprintf(buf + len,
						PAGE_SIZE - len,
						"aw8680x->read_data[%d] is 0x%x\n",
						i, aw8680x->read_data[i]);
				}
			} else {
				len += snprintf(buf + len, PAGE_SIZE - len,
						"aw8680x flash read fail!\n");
			}
		} else {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"aw8680x flash read_len more than the READ_FLASH_MAX!\n");
		}
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x cpu is not in uboot!\n");
	}

	return len;
}

static ssize_t aw8680x_flash_read_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int databuf[2] = { 0, 0 };
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	int read_count = 5;

	if (sscanf(buf, "%x %d", &databuf[0], &databuf[1]) == 2) {
		aw8680x->read_len = databuf[1];
		if (((aw8680x->app_current_addr == CPU_IN_UBOOT)
		     || (aw8680x->app_current_addr == CPU_IN_FLASH_BOOT))
		    && (databuf[1] <= READ_FLASH_MAX)) {
			while (read_count--) {
				aw8680x->ack_flag = aw8680x_flash_read(aw8680x,
							databuf[0], databuf[1]);
				if (aw8680x->ack_flag == 0)
					break;
			}
			if (aw8680x->ack_flag == 0)
				pr_info("flash read success!\n");
			else
				pr_info("flash read fail! aw8680x->ack_flag is %d\n",
							aw8680x->ack_flag);
		}
	}

	return count;
}

static ssize_t aw8680x_reset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1) {
		pr_info("enter adb reset\n");
		aw8680x_hw_reset(aw8680x);
		aw8680x_i2c_write(aw8680x, REG_ADDR, 1);
		mdelay(CHIP_INIT_TIME);
	}

	return count;
}

static ssize_t aw8680x_wake_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val == 1) {
		pr_info("enter adb wake\n");
		aw8680x_wake_pin(aw8680x);
	}

	return count;
}

static ssize_t aw8680x_scan_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	char databuf[10] = { 0 };
	struct aw8680x *aw8680x = dev_get_drvdata(dev);

	if (sscanf(buf, "%s", databuf) == 1) {
		if (strcmp(databuf, "active") == 0) {
			pr_info("enter adb active mode\n");
			aw8680x_active_mode_set(aw8680x);
		} else if (strcmp(databuf, "idle") == 0) {
			pr_info("enter adb idle mode\n");
			aw8680x_idle_mode_set(aw8680x);
		} else if (strcmp(databuf, "sleep") == 0) {
			pr_info("enter adb sleep mode\n");
			aw8680x_sleep_mode_set(aw8680x);
		} else {
			pr_info("The currently entered mode is not supported!\n");
		}
	}

	return count;
}

static ssize_t aw8680x_adc_data_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_adc_data(aw8680x);
	if (ret != 0) {
		pr_err("aw8680x adc data read fail!\n");
	} else {
		for (i = 0; i < ADC_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x->info.adc_data[%d] = 0x%x\n",
					i, aw8680x->info.adc_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_raw_data_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_raw_data(aw8680x);
	if (ret != 0) {
		pr_err("aw8680x raw data read fail!\n");
	} else {
		for (i = 0; i < RAW_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x->info.raw_data[%d] = 0x%x\n",
					i, aw8680x->info.raw_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_force_data_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_force_data(aw8680x);
	if (ret != 0) {
		pr_err("aw8680x force data read fail!\n");
	} else {
		for (i = 0; i < FORCE_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x->info.force_data[%d] = 0x%x\n",
					i, aw8680x->info.force_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_base_data_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_base_data(aw8680x);
	if (ret != 0) {
		pr_err("aw8680x base data read fail!\n");
	} else {
		for (i = 0; i < BASE_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x->info.base_data[%d] = 0x%x\n",
					i, aw8680x->info.base_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_diff_data_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_diff_data(aw8680x);
	if (ret != 0) {
		pr_err("aw8680x diff data read fail!\n");
	} else {
		for (i = 0; i < DIFF_DATA_NUM; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x->info.diff_data[%d] = 0x%x\n",
					i, aw8680x->info.diff_data[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_sensor_status_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	unsigned char data_val = 0;
	ssize_t len = 0;
	int read_len = 0;
	int i = 0;
	int read_count = 5;

	aw8680x_i2c_write(aw8680x, 0xf6, 0xa4);
	aw8680x_i2c_write(aw8680x, 0xf7, 0x0);
	mdelay(15);
	while (read_count--) {
		aw8680x_i2c_read(aw8680x, 0xf7, &data_val);
		if (data_val != 0)
			break;
		mdelay(15);
	}
	read_len = data_val;
	len += snprintf(buf + len, PAGE_SIZE - len, "read_len = %d\n",
								read_len);
	aw8680x_i2c_reads_data(aw8680x, 0xf8, read_len);
	for (i = 0; i < read_len; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
					"aw8680x->read_data[%d] = %d\n",
					i, aw8680x->read_data[i]);
	}
	return len;
}

static ssize_t aw8680x_ic_status_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;

	len +=
		snprintf(buf + len, PAGE_SIZE - len, "%s\n",
								aw8680x->ic_status? "Succeed" : "Failed");
	return len;
}

static ssize_t aw8680x_polling_set_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val != 0) {
		pr_info("enter adb polling start\n");
		aw8680x_polling_start(aw8680x);
	} else {
		pr_info("enter adb polling stop\n");
		aw8680x_polling_stop(aw8680x);
	}

	return count;
}

static ssize_t aw8680x_press_threshold_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i = 0;
	int ret = -1;

	ret = aw8680x_get_press_threshold(aw8680x);
	if (ret != 0) {
		pr_err("aw8680x press threshold read fail!\n");
	} else {
		for (i = 0; i < PRESS_THRESHOLD_LEN; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"aw8680x->info.press_threshold[%d] = 0x%x\n",
				i, aw8680x->info.press_threshold[i]);
		}
	}

	return len;
}

static ssize_t aw8680x_press_threshold_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	char handshake[2] = { 0 };
	unsigned char press[PRESS_THRESHOLD_LEN] = { 0 };
	unsigned int databuf[PRESS_THRESHOLD_LEN] = { 0 };
	int i = 0;
	int ret = -1;

	if (sscanf(buf, "%x %x %x %x %x %x %x %x", &databuf[0], &databuf[1],
			&databuf[2], &databuf[3], &databuf[4], &databuf[5],
			&databuf[6], &databuf[7]) == PRESS_THRESHOLD_LEN) {
		for (i = 0; i < PRESS_THRESHOLD_LEN; i++) {
			press[i] = (unsigned char)databuf[i];
		}
		ret = aw8680x_i2c_writes_data(aw8680x, PRESS_THRESHOLD_ADDR,
						press, PRESS_THRESHOLD_LEN);
		if (ret < 0) {
			dev_err(aw8680x->dev, "%s: failed to write data, ret is : %d\n",
								__func__, ret);
			return -1;
		}
		handshake[0] = 0x5b;
		handshake[1] = 0xa5;
		ret = aw8680x_i2c_writes_data(aw8680x, 0x01, handshake, 2);
		if (ret < 0) {
			dev_err(aw8680x->dev, "%s: failed to hand shake, ret is : %d\n",
								__func__, ret);
			return -1;
		}
	}

	return count;
}

static ssize_t aw8680x_touch_callback_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	int ret = -1, rc = 0;
	int val = 0, enable = 0;
	static int last_status = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

/*	enable = !!val;
	pr_info("aw8680x register fb_notifier: %d\n", enable);
	if (enable != 0 && (enable != last_status)) {
		ret = touch_event_register_notifier(&aw8680x->fb_notif);
		if (ret)
			pr_info("Unable to register fb_notifier: %d\n", ret);
	} else if (enable == 0 && (enable != last_status))
		ret = touch_event_unregister_notifier(&aw8680x->fb_notif);
*/
	pr_info("aw8680x register fb_notifier: %d\n", enable);
	if (enable == 0xED && (enable != last_status)) {
		ret = touch_event_register_notifier(&aw8680x->fb_notif);
		if (ret)
			pr_info("Unable to register fb_notifier: %d\n", ret);
	} else if (enable == 0xEE && (enable != last_status))
		ret = touch_event_unregister_notifier(&aw8680x->fb_notif);
	last_status = enable;

	return count;
}

static ssize_t aw8680x_top_range_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
		"Top Screen Range: point0(%d,%d), point1(%d,%d)\n",
		aw8680x->screen_xrange[0], aw8680x->screen_yrange[0],
		aw8680x->screen_xrange[1], aw8680x->screen_yrange[1]);

	return len;
}

static ssize_t aw8680x_top_range_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	unsigned int databuf[4] = { 0 };

	if (sscanf(buf, "%d %d %d %d", &databuf[0], &databuf[1],
			&databuf[2], &databuf[3]) == 4) {
		aw8680x->screen_xrange[0] = databuf[0]; /* top x0*/
		aw8680x->screen_xrange[1] = databuf[2]; /* top x1*/
		aw8680x->screen_yrange[0] = databuf[1]; /* top y0*/
		aw8680x->screen_yrange[1] = databuf[3]; /* top y1*/

		dev_info(aw8680x->dev, "Top Screen: point0(%d,%d), point1(%d,%d)\n",
			aw8680x->screen_xrange[0], aw8680x->screen_yrange[0],
			aw8680x->screen_xrange[1], aw8680x->screen_yrange[1]);
	}

	return count;
}

static ssize_t aw8680x_bottom_range_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
		"Bottom Screen Range: point0(%d,%d), point1(%d,%d)\n",
		aw8680x->screen_xrange[2], aw8680x->screen_yrange[2],
		aw8680x->screen_xrange[3], aw8680x->screen_yrange[3]);

	return len;
}

static ssize_t aw8680x_bottom_range_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct aw8680x *aw8680x = dev_get_drvdata(dev);
	unsigned int databuf[4] = { 0 };

	if (sscanf(buf, "%d %d %d %d", &databuf[0], &databuf[1],
			&databuf[2], &databuf[3]) == 4) {
		aw8680x->screen_xrange[2] = databuf[0]; /* bottom x0*/
		aw8680x->screen_xrange[3] = databuf[2]; /* bottom x1*/
		aw8680x->screen_yrange[2] = databuf[1]; /* bottom y0*/
		aw8680x->screen_yrange[3] = databuf[3]; /* bottom y1*/

		dev_info(aw8680x->dev, "Bottom Screen: point0(%d,%d), point1(%d,%d)\n",
			aw8680x->screen_xrange[2], aw8680x->screen_yrange[2],
			aw8680x->screen_xrange[3], aw8680x->screen_yrange[3]);
	}

	return count;
}

static ssize_t aw8680x_ftp_enable_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	int databuf = 0;
	struct aw8680x *aw8680x = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &databuf) == 1) {
		if (databuf == 1) {
			aw8680x->ftp_enable=true;
			aw8680x_drm_resume(aw8680x);
		} else if (databuf == 0) {
			aw8680x->ftp_enable=false;
			aw8680x_drm_suspend(aw8680x);
			cancel_work_sync(&aw8680x->report_tp_work);
		} else {
			pr_info("aw8680x: The currently entered mode is not supported!\n");
		}
	}

	return count;
}

static DEVICE_ATTR(connect, S_IWUSR | S_IRUGO, aw8680x_connect_show, NULL);
static DEVICE_ATTR(update, S_IWUSR | S_IRUGO, NULL, aw8680x_update_store);
static DEVICE_ATTR(jump, S_IWUSR | S_IRUGO, NULL, aw8680x_jump_store);
static DEVICE_ATTR(flash_read, S_IWUSR | S_IRUGO, aw8680x_flash_read_show,
		   aw8680x_flash_read_store);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, NULL, aw8680x_reset_store);
static DEVICE_ATTR(wake, S_IWUSR | S_IRUGO, NULL, aw8680x_wake_store);
static DEVICE_ATTR(scan_mode, S_IWUSR | S_IRUGO, NULL, aw8680x_scan_mode_store);
static DEVICE_ATTR(adc_data, S_IWUSR | S_IRUGO, aw8680x_adc_data_show, NULL);
static DEVICE_ATTR(raw_data, S_IWUSR | S_IRUGO, aw8680x_raw_data_show, NULL);
static DEVICE_ATTR(force_data, S_IWUSR | S_IRUGO, aw8680x_force_data_show,
									NULL);
static DEVICE_ATTR(base_data, S_IWUSR | S_IRUGO, aw8680x_base_data_show, NULL);
static DEVICE_ATTR(diff_data, S_IWUSR | S_IRUGO, aw8680x_diff_data_show, NULL);
static DEVICE_ATTR(polling_set, S_IWUSR | S_IRUGO, NULL,
		   aw8680x_polling_set_store);
static DEVICE_ATTR(sensor_status, S_IWUSR | S_IRUGO,
		   aw8680x_sensor_status_show, NULL);
static DEVICE_ATTR(press_threshold, S_IWUSR | S_IRUGO,
		   aw8680x_press_threshold_show, aw8680x_press_threshold_store);
static DEVICE_ATTR(touch_callback, S_IWUSR | S_IRUGO,
		   NULL, aw8680x_touch_callback_store);
static DEVICE_ATTR(ic_status, S_IWUSR | S_IRUGO,
		   aw8680x_ic_status_show, NULL);
static DEVICE_ATTR(top_range, S_IWUSR | S_IRUGO,
		    aw8680x_top_range_show, aw8680x_top_range_store);
static DEVICE_ATTR(bottom_range, S_IWUSR | S_IRUGO,
		    aw8680x_bottom_range_show, aw8680x_bottom_range_store);
static DEVICE_ATTR(ftp_enable, S_IWUSR | S_IRUGO, NULL, aw8680x_ftp_enable_store);

static struct attribute *aw8680x_attributes[] = {
	&dev_attr_connect.attr,
	&dev_attr_update.attr,
	&dev_attr_jump.attr,
	&dev_attr_flash_read.attr,
	&dev_attr_reset.attr,
	&dev_attr_wake.attr,
	&dev_attr_scan_mode.attr,
	&dev_attr_adc_data.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_force_data.attr,
	&dev_attr_base_data.attr,
	&dev_attr_diff_data.attr,
	&dev_attr_polling_set.attr,
	&dev_attr_sensor_status.attr,
	&dev_attr_press_threshold.attr,
	&dev_attr_touch_callback.attr,
	&dev_attr_ic_status.attr,
	&dev_attr_top_range.attr,
	&dev_attr_bottom_range.attr,
	&dev_attr_ftp_enable.attr,
	NULL
};

static struct attribute_group aw8680x_attribute_group = {
	.attrs = aw8680x_attributes
};

/*****************************************************
 *
 * aw8680x apk interface
 *
 *****************************************************/
static int aw8680x_file_open(struct inode *inode, struct file *filp)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	filp->private_data = (void *)g_aw8680x;

	return 0;
}

static int aw8680x_file_release(struct inode *inode, struct file *filp)
{
	filp->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static ssize_t aw8680x_file_read(struct file *filp, char *buff, size_t len,
				 loff_t *offset)
{
	struct aw8680x *aw8680x = (struct aw8680x *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;
	unsigned char reg_addr = 0;

	pr_debug("enter %s\n", __func__);
	if (aw8680x->suspend_mode == true) {
		pr_info("aw8680x in suspend mode!\n");
		return len;
	}

	if (len > 256)
		return len;
	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw8680x->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	/* get reg addr */
	if (copy_from_user(&reg_addr, buff, 1)) {
		kfree(pbuff);
		return len;
	}
	/* pr_info("reg_addr is 0x%x\n", reg_addr);
	pr_info("read_len is %d\n", len); */
	ret = aw8680x_i2c_reads_data(aw8680x, reg_addr, len);
	if (ret < 0) {
		dev_err(aw8680x->dev, "%s: failed to read data, ret is : %d\n",
								__func__, ret);
		kfree(pbuff);
		return len;
	}
	for (i = 0; i < len; i++) {
		pbuff[i] = aw8680x->read_data[i];
		/* pr_info("%s: pbuff[%d] = 0x%02x\n", __func__, i, pbuff[i]); */
	}
	ret = copy_to_user(buff + 1, pbuff, len);
	if (ret) {
		dev_err(aw8680x->dev, "%s: copy to user fail\n", __func__);
		kfree(pbuff);
		return len;
	}

	kfree(pbuff);
	return len;
}

static ssize_t aw8680x_file_write(struct file *filp, const char *buff,
				  size_t len, loff_t *off)
{
	struct aw8680x *aw8680x = (struct aw8680x *)filp->private_data;
	//int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;
	unsigned char reg_addr = 0;

	pr_debug("%s enter\n", __func__);
	if (aw8680x->suspend_mode == true) {
		pr_info("aw8680x in suspend mode!\n");
		return len;
	}

	if (len > 256)
		return len;
	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw8680x->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	/* get reg addr */
	ret = copy_from_user(&reg_addr, buff, 1);
	if (ret) {
		dev_err(aw8680x->dev, "%s: copy from user reg_addr fail\n",
								__func__);
		kfree(pbuff);
		return len;
	}
	/* pr_info("reg_addr is 0x%x\n", reg_addr);
	pr_info("write_len is %d\n", len); */
	/* get reg data */
	ret = copy_from_user(pbuff, buff + 1, len);
	if (ret) {
		dev_err(aw8680x->dev, "%s: copy from user reg_data fail\n",
								__func__);
		kfree(pbuff);
		return len;
	}
	/* for (i = 0; i < len; i++)
		pr_info("%s: pbuff[%d] = 0x%02x\n", __func__, i, pbuff[i]); */
	ret = aw8680x_i2c_writes_data(aw8680x, reg_addr, pbuff, len);
	if (ret < 0) {
		dev_err(aw8680x->dev, "%s: failed to write data, ret is : %d\n",
								__func__, ret);
		kfree(pbuff);
		return len;
	}

	kfree(pbuff);
	return len;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw8680x_file_read,
	.write = aw8680x_file_write,
	.open = aw8680x_file_open,
	.release = aw8680x_file_release,
};

static struct miscdevice aw8680x_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8680X_NAME,
	.fops = &fops,
};

static int aw8680x_file_init(struct aw8680x *aw8680x)
{
	int ret = 0;

	ret = misc_register(&aw8680x_misc);
	if (ret) {
		dev_err(aw8680x->dev, "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}
	return 0;
}

static void aw8680x_report_event(struct aw8680x *aw8680x, int key, int value)
{
	input_report_abs(aw8680x->input, ABS_MT_TOOL_X, 0);
	input_report_abs(aw8680x->input, ABS_MT_TOOL_Y, 0);
	input_report_abs(aw8680x->input, ABS_MT_TRACKING_ID, 0);
	//key
	input_report_abs(aw8680x->input, ABS_MT_WIDTH_MAJOR, key);
	//value
	input_report_abs(aw8680x->input, ABS_MT_WIDTH_MINOR, value);

	input_report_abs(aw8680x->input, ABS_MT_BLOB_ID, 0);
	input_report_abs(aw8680x->input, ABS_MT_TOOL_TYPE, 0x146);
	input_report_abs(aw8680x->input, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(aw8680x->input, ABS_MT_TOUCH_MINOR, 0);

	//input_event(get_input_device(), EV_SYN, SYN_REPORT, 9);
	input_sync(aw8680x->input);
}
/*****************************************************
 *
 * aw8680x irq
 *
 *****************************************************/
static void aw8680x_top_screen(struct aw8680x *aw8680x)
{
	pr_debug("%s: last[0] = %d\n", __func__, aw8680x->last_status[0]);
	aw8680x->current_status[0] = aw8680x->info.key_data & 0x1;
	pr_debug("%s: current[0] = %d\n", __func__, aw8680x->current_status[0]);

	if (aw8680x->current_status[0] != aw8680x->last_status[0]) {
		if (aw8680x->current_status[0] == 1) {
			if (aw8680x->top_screen_x >= aw8680x->screen_xrange[0] &&
				aw8680x->top_screen_x <= aw8680x->screen_xrange[1] &&
				aw8680x->top_screen_y >= aw8680x->screen_yrange[0] &&
				aw8680x->top_screen_y <= aw8680x->screen_yrange[1]) {
				pr_info("%s: top approach\n", __func__);
				//input_report_key(aw8680x->input, KEY_FTOUCH_LEFT, 1);
				//input_sync(aw8680x->input);
				aw8680x_report_event(aw8680x, KEY_FTOUCH_LEFT, 1);
			}
		} else if (aw8680x->current_status[0] == 0) {
			pr_info("%s: top far\n", __func__);
			/* input_report_key(aw8680x->input, KEY_FTOUCH_LEFT, 0);
			input_sync(aw8680x->input); */
			aw8680x_report_event(aw8680x, KEY_FTOUCH_LEFT, 0);
		}
	}
	aw8680x->last_status[0] = aw8680x->current_status[0];
}

static void aw8680x_lower_screen(struct aw8680x *aw8680x)
{
	pr_debug("%s: last[1] = %d\n", __func__, aw8680x->last_status[1]);
	aw8680x->current_status[1] = (aw8680x->info.key_data >> 1) & 0x1;
	pr_debug("%s: current [1] = %d\n", __func__, aw8680x->current_status[1]);

	if (aw8680x->current_status[1] != aw8680x->last_status[1]) {
		pr_debug("%s: curr1 = %d\n", __func__, aw8680x->current_status[1]);
		if (aw8680x->current_status[1] == 1) {
			if (aw8680x->lower_screen_x >= aw8680x->screen_xrange[2] &&
				aw8680x->lower_screen_x <= aw8680x->screen_xrange[3] &&
				aw8680x->lower_screen_y >= aw8680x->screen_yrange[2] &&
				aw8680x->lower_screen_y <= aw8680x->screen_yrange[3]) {
				pr_info("%s: lower approach\n", __func__);
				/* input_report_key(aw8680x->input, KEY_FTOUCH_RIGHT, 1);
				input_sync(aw8680x->input); */
				aw8680x_report_event(aw8680x, KEY_FTOUCH_RIGHT, 1);

			}
		} else if (aw8680x->current_status[1] == 0) {
			pr_info("%s: lower far\n", __func__);
			/* input_report_key(aw8680x->input, KEY_FTOUCH_RIGHT, 0);
			input_sync(aw8680x->input); */
			aw8680x_report_event(aw8680x, KEY_FTOUCH_RIGHT, 0);
		}
	}
	aw8680x->last_status[1] = aw8680x->current_status[1];
}

static irqreturn_t aw8680x_irq(int irq, void *data)
{
	struct aw8680x *aw8680x = data;
	int ret = -1;

	pr_info("irq aw8680x->app_current_addr == 0x%x && aw8680x->cpu_in_flash == 0x%x\n",
			aw8680x->app_current_addr, aw8680x->cpu_in_flash);
	if (aw8680x->suspend_mode == true) {
		pr_info("aw8680x in suspend mode!\n");
		return IRQ_HANDLED;
	}

	if (aw8680x->app_current_addr != aw8680x->cpu_in_flash)
		return ret;

	/* top half of screen */
	ret = aw8680x_get_key_data(aw8680x);
	if (ret != 0)
		return ret;

	/* top half of screen */
	aw8680x_top_screen(aw8680x);

	/* Bottom half of screen */
	aw8680x_lower_screen(aw8680x);

	return IRQ_HANDLED;
}
/*****************************************************
 *
 * device tree
 *
 *****************************************************/
/* Get the irq port number in the device tree */
static int aw8680x_parse_dt(struct device *dev, struct aw8680x *aw8680x,
			    struct device_node *np)
{
	uint32_t val = 0;
	int32_t i = 0;

	aw8680x->irq_mode = of_property_read_bool(np, "irq-mode");
	if (aw8680x->irq_mode == IRQ_MODE_SET) {
		dev_info(dev, "%s driver use irq_mode\n", __func__);
		aw8680x->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
		if (gpio_is_valid(aw8680x->irq_gpio)) {
			dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
		} else {
			dev_err(dev, "%s: no irq gpio provided.\n", __func__);
			return -1;
		}
	} else if (aw8680x->irq_mode == NOT_IRQ_MODE_SET) {
		dev_info(dev, "%s driver not use irq_mode\n", __func__);
	} else {
		dev_info(dev, "%s driver not use any mode\n", __func__);
	}
	aw8680x->wake_mode = of_property_read_bool(np, "wake-mode");
	if (aw8680x->wake_mode == WAKE_MODE_SET) {
		dev_info(dev, "%s driver use wake_mode\n", __func__);
		aw8680x->wake_gpio = of_get_named_gpio(np, "wake-gpio", 0);
		if (gpio_is_valid(aw8680x->wake_gpio)) {
			dev_info(dev, "%s: wake gpio provided ok.\n", __func__);
		} else {
			dev_err(dev, "%s: no wake gpio provided.\n", __func__);
			return -1;
		}
	} else if (aw8680x->wake_mode == NOT_WAKE_MODE_SET) {
		dev_info(dev, "%s driver not use wake_mode\n", __func__);
	} else {
		dev_info(dev, "%s driver not use any mode\n", __func__);
	}
	aw8680x->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(aw8680x->reset_gpio)) {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	} else {
		dev_err(dev, "%s: no reset gpio provided\n", __func__);
		return -1;
	}
	val = of_property_read_u32_array(np, "screen_x_range",
		aw8680x->screen_xrange, ARRAY_SIZE(aw8680x->screen_xrange));
	if (val != 0) {
		pr_info("%s screen xrange get failed\n", __func__);
		aw8680x->screen_xrange[0] = LSCREEN_XRANGE_LEFT;
		aw8680x->screen_xrange[1] = LSCREEN_XRANGE_RIGHT;
		aw8680x->screen_xrange[2] = RSCREEN_XRANGE_LEFT;
		aw8680x->screen_xrange[3] = RSCREEN_XRANGE_RIGHT;
	}
	for (i = 0; i < ARRAY_SIZE(aw8680x->screen_xrange); i++)
		pr_info("%s screen xrange %d = %d\n", __func__, i,
						aw8680x->screen_xrange[i]);

	val = of_property_read_u32_array(np, "screen_y_range",
		aw8680x->screen_yrange, ARRAY_SIZE(aw8680x->screen_yrange));
	if (val != 0) {
		pr_info("%s screen xrange get failed\n", __func__);
		aw8680x->screen_yrange[0] = TOPSCREEN_YRANGE_BOTTOM;
		aw8680x->screen_yrange[1] = TOPSCREEN_YRANGE_TOP;
		aw8680x->screen_yrange[2] = BOTTOMSCREEN_YRANGE_BOTTOM;
		aw8680x->screen_yrange[3] = BOTTOMSCREEN_YRANGE_TOP;
	}
	for (i = 0; i < ARRAY_SIZE(aw8680x->screen_yrange); i++)
		pr_info("%s screen yrange%d = %d\n", __func__, i,
						aw8680x->screen_yrange[i]);

	return 0;
}

#define FTOUCH_RESET_ACTIVE    "pmx_ftouch_reset_active"
#define FTOUCH_INT   "pmx_ftouch_int"
#define FTOUCH_WAKEUP   "pmx_ftouch_wakeup"

static int aw8680x_i2c_pinctrl_init(struct aw8680x *aw8680x)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	aw8680x->active_pinctrl = devm_pinctrl_get(&aw8680x->i2c->dev);
	if (IS_ERR_OR_NULL(aw8680x->active_pinctrl)) {
		retval = PTR_ERR(aw8680x->active_pinctrl);
		pr_info("aw8680x: does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	aw8680x->pinctrl_reset_state
		= pinctrl_lookup_state(aw8680x->active_pinctrl, FTOUCH_RESET_ACTIVE);
	if (IS_ERR_OR_NULL(aw8680x->pinctrl_reset_state)) {
		retval = PTR_ERR(aw8680x->pinctrl_reset_state);
		pr_info("aw8680x: Can not lookup %s pinstate %d\n",
					FTOUCH_RESET_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}
/*
	aw8680x->pinctrl_int_state
		= pinctrl_lookup_state(aw8680x->active_pinctrl, FTOUCH_INT);
	if (IS_ERR_OR_NULL(aw8680x->pinctrl_int_state)) {
		retval = PTR_ERR(aw8680x->pinctrl_int_state);
		pr_info("aw8680x: Can not lookup %s pinstate %d\n",
					FTOUCH_INT, retval);
		goto err_pinctrl_lookup;
	}

	aw8680x->pinctrl_wakeup_state
		= pinctrl_lookup_state(aw8680x->active_pinctrl, FTOUCH_WAKEUP);
	if (IS_ERR_OR_NULL(aw8680x->pinctrl_wakeup_state)) {
		retval = PTR_ERR(aw8680x->pinctrl_wakeup_state);
		pr_info("aw8680x: Can not lookup %s pinstate %d\n",
					FTOUCH_WAKEUP, retval);
		goto err_pinctrl_lookup;
	}
*/
	if (aw8680x->active_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		retval = pinctrl_select_state(aw8680x->active_pinctrl,
						aw8680x->pinctrl_reset_state);
		if (retval < 0) {
			pr_info("%s: Failed to select %s pinstate %d\n",
				__func__, FTOUCH_RESET_ACTIVE, retval);
		}
/*		retval = pinctrl_select_state(aw8680x->active_pinctrl,
						aw8680x->pinctrl_int_state);
		if (retval < 0) {
			pr_info("%s: Failed to select %s pinstate %d\n",
				__func__, FTOUCH_INT, retval);
		}
		retval = pinctrl_select_state(aw8680x->active_pinctrl,
						aw8680x->pinctrl_wakeup_state);
		if (retval < 0) {
			pr_info("%s: Failed to select %s pinstate %d\n",
				__func__, FTOUCH_WAKEUP, retval);
		} */
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(aw8680x->active_pinctrl);
err_pinctrl_get:
	aw8680x->active_pinctrl = NULL;
	return retval;
}


static int aw8680x_gpio_set(struct aw8680x *aw8680x,
						struct i2c_client *i2c)
{
	int ret = 0;
	//int irq_flags;

	pr_info("%s enter\n", __func__);
	/* If the irq port number is valid, apply for the port number */
	ret = devm_gpio_request_one(&i2c->dev, aw8680x->reset_gpio,
					GPIOF_OUT_INIT_LOW, "aw8680x_rst");
	if (ret) {
		dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
		return -RST_REGISTER_ERR;
	}
	/* choice of interrupt trigger or active wakeup */
	if (aw8680x->irq_mode == IRQ_MODE_SET) {
		ret = devm_gpio_request_one(&i2c->dev, aw8680x->irq_gpio,
						GPIOF_DIR_IN, "aw8680x_int");
		if (ret) {
			dev_err(&i2c->dev, "%s:int request failed\n", __func__);
			return -IRQ_REGISTER_ERR;
		}
/*		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev, gpio_to_irq
						(aw8680x->irq_gpio), NULL,
						aw8680x_irq, irq_flags,
						"aw8680x", aw8680x); 
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw8680x->irq_gpio), ret);
			return -IRQ_THREAD_ERR;
		} */
	}
	if (aw8680x->wake_mode == WAKE_MODE_SET) {
		ret = devm_gpio_request_one(&i2c->dev, aw8680x->wake_gpio,
					GPIOF_OUT_INIT_LOW, "aw8680x_wake");
		if (ret) {
			dev_err(&i2c->dev, "%s: wake request failed\n",
								__func__);
			return -WAKE_REGISTER_ERR;
		}
	}

	return 0;
}

static int aw8680x_gpio_set_irq(struct aw8680x *aw8680x,
						struct i2c_client *i2c)
{
	int ret = 0;
	int irq_flags;

	irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	ret = devm_request_threaded_irq(&i2c->dev, gpio_to_irq
					(aw8680x->irq_gpio), NULL,
					aw8680x_irq, irq_flags,
					"aw8680x", aw8680x);
	if (ret != 0) {
		dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
			__func__, gpio_to_irq(aw8680x->irq_gpio), ret);
		return -IRQ_THREAD_ERR;
	}
	return 0;
}

static int aw8680x_drm_suspend(struct aw8680x *aw8680x)
{
	if (aw8680x->suspend_mode == true) {
		pr_info("drm:  aw8680x in sleep mode\n");
		return 0;
	}

	pr_info("drm: enter aw8680x sleep\n");
	aw8680x_sleep_mode_set(aw8680x);
	aw8680x->suspend_mode = true;
	return 0;
}

static int aw8680x_drm_resume(struct aw8680x *aw8680x)
{
	unsigned char data_val = 0;

	if (aw8680x->ftp_enable== false || aw8680x->suspend_mode == false) {
		pr_info("drm:  aw8680x keep in sleep mode\n");
		return 0;
	}

	aw8680x_active_mode_set(aw8680x);
	aw8680x_i2c_read(aw8680x, REG_SCAN_MODE_SWITCH_ADDR, &data_val);
	aw8680x->suspend_mode = false;
	pr_info("drm: enter aw8680x active, 0x56=%d\n", data_val);
	return 0;
}

static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct drm_panel_notifier *evdata = data;
    int *blank = NULL;
    struct aw8680x *aw8680x = container_of(self, struct aw8680x,
                                  fb_notif_drm);

    if (!evdata) {
        pr_info("evdata is null");
        return 0;
    }

    if (!((event == DRM_PANEL_EARLY_EVENT_BLANK )
          || (event == DRM_PANEL_EVENT_BLANK))) {
        pr_info("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    pr_debug("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case DRM_PANEL_BLANK_UNBLANK:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
		pr_debug("resume: event = %lu, not care\n", event);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
		aw8680x_drm_resume(aw8680x);
        }
        break;
    case DRM_PANEL_BLANK_POWERDOWN:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
		aw8680x_drm_suspend(aw8680x);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            pr_debug("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        pr_info("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}

static int aw8680x_drm_register_notif(struct aw8680x *aw8680x)
{
#if defined(CONFIG_DRM)

	aw8680x->fb_notif_drm.notifier_call = drm_notifier_callback;
	pr_info("aw8680x: %s, %d!\n", __func__, __LINE__);

	if (active_panel &&
		drm_panel_notifier_register(active_panel,
			&aw8680x->fb_notif_drm) < 0)
		pr_info("aw8680x: register notifier failed!\n");
#endif
	return 0;
}

static int aw8680x_drm_check_dt(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;
	pr_info("aw8680x: %s, %d!\n", __func__, __LINE__);

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			return 0;
		}
	}
	pr_info("aw8680x: %s, %d!\n", __func__, __LINE__);

	return PTR_ERR(panel);
}
/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
/* In this function can do a series of initialization work */
static int aw8680x_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	int ret = 0;
	struct aw8680x *aw8680x;
	struct device_node *np = i2c->dev.of_node;
	struct input_dev *input_dev;

	pr_info("%s enter\n", __func__);
	/* Determining the ability of the adapter */
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}
	/* Apply for memory for device structures */
	aw8680x = devm_kzalloc(&i2c->dev, sizeof(struct aw8680x), GFP_KERNEL);
	if (aw8680x == NULL)
		return -ENOMEM;
	aw8680x->dev = &i2c->dev;
	aw8680x->i2c = i2c;
	/* Save this structure in i2c, when we need to use the data
	 * in the structure, it can be easily found
	 */
	i2c_set_clientdata(i2c, aw8680x);
	/* Get the irq port number in the device tree */
	if (np) {
		ret = aw8680x_parse_dt(&i2c->dev, aw8680x, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_dt;
		}
		aw8680x_drm_check_dt(np);
	}
	aw8680x_i2c_pinctrl_init(aw8680x);
	ret = aw8680x_gpio_set(aw8680x, i2c);
	if (ret == -RST_REGISTER_ERR)
		goto err_rst_register;
	else if (ret == -WAKE_REGISTER_ERR)
		goto err_wake_register;
	else if (ret == -IRQ_REGISTER_ERR)
		goto err_irq_register;
	else if (ret == -IRQ_THREAD_ERR)
		goto err_irq_thread;
	/* connect */
	ret = aw8680x_connect(aw8680x);
	if (ret == 0) {
		pr_info("%s boot connect success!\n", __func__);
	} else {
		ret = aw8680x_jump_uboot(aw8680x);
		if (ret != 0) {
			pr_err("%s boot connect fail!\n", __func__);
			goto err_connect;
		}
		pr_info("%s boot connect success!\n", __func__);
	}
	/* The dev_set_drvdata function is used to set the private data
	 * of the device,The dev_get_drvdata function is used to
	 * obtain the private data of the device.
	 */
	dev_set_drvdata(&i2c->dev, aw8680x);
	g_aw8680x = aw8680x;
	/* input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		dev_err(&i2c->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	input_dev->name = AW8680X_I2C_NAME;
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);

	__set_bit(KEY_UP, input_dev->keybit);
	__set_bit(KEY_DOWN, input_dev->keybit);
	__set_bit(KEY_LEFT, input_dev->keybit);
	__set_bit(KEY_RIGHT, input_dev->keybit);
	__set_bit(KEY_FTOUCH_LEFT, input_dev->keybit);
	__set_bit(KEY_FTOUCH_RIGHT, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, 1023, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 1023, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 1023, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_BLOB_ID, 0, 65535, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOOL_X, 0, 1080, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOOL_Y, 0, 2460, 0, 0);
	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&i2c->dev, "%s: failed to register input device: %s\n",
						__func__, dev_name(&i2c->dev));
		goto exit_input_register_device_failed;
	}
	aw8680x->input = input_dev;
	/* attribute
	 * Create the corresponding attributes of the
	 * driver through the sysfs interface,
	 * which is used for Adb debugging
	 */
	ret = sysfs_create_group(&i2c->dev.kobj, &aw8680x_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev,
			"%s error creating sysfs attr files\n",
			__func__);
		goto err_attribute;
	}
	aw8680x_gpio_set_irq(aw8680x, i2c);
	aw8680x_file_init(aw8680x);
	/* Get the bin file in the /system/vendor/firmware/ directory */
	aw8680x_bin_init(aw8680x, AW8680X_BIN_INIT_BOOT_DELAY);
	aw8680x->polling_cycle = AW8680X_DILE_POLLING_DELAY;

	aw8680x->tp_workqueue = create_singlethread_workqueue("fts_wq");
	if (!aw8680x->tp_workqueue) {
		pr_info("create fts workqueue fail");
	}
	if (aw8680x->tp_workqueue) {
		INIT_WORK(&aw8680x->report_tp_work, aw8680x_report_tp_work);
	}
	aw8680x->ftp_enable = true;
	aw8680x->suspend_mode = false;
	/* into seep mode */
	aw8680x_drm_suspend(aw8680x);
	aw8680x->ftp_enable = false;

	/* add tp callback */
	aw8680x->fb_notif.notifier_call = aw8680x_fb_notifier_callback_tp;
	touch_event_register_notifier(&aw8680x->fb_notif);
	aw8680x_drm_register_notif(aw8680x);

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;
 err_attribute:
	sysfs_remove_group(&i2c->dev.kobj, &aw8680x_attribute_group);
 exit_input_register_device_failed:
	input_free_device(input_dev);
 exit_input_dev_alloc_failed:
 err_connect:
#ifdef CONFIG_TOUCH_AW8680X
	if (aw8680x->wake_mode == WAKE_MODE_SET)
		devm_gpio_free(&i2c->dev, aw8680x->wake_gpio);
#endif
 err_wake_register:
 err_irq_thread:
#ifdef CONFIG_TOUCH_AW8680X
	if (aw8680x->irq_mode == IRQ_MODE_SET)
		devm_gpio_free(&i2c->dev, aw8680x->irq_gpio);
#endif
 err_irq_register:
#ifdef CONFIG_TOUCH_AW8680X
	devm_gpio_free(&i2c->dev, aw8680x->reset_gpio);
#endif
 err_rst_register:
 err_dt:
	devm_kfree(&i2c->dev, aw8680x);
	return ret;
}

static int aw8680x_i2c_remove(struct i2c_client *i2c)
{
	struct aw8680x *aw8680x = i2c_get_clientdata(i2c);

	input_free_device(aw8680x->input);
	input_unregister_device(aw8680x->input);
	sysfs_remove_group(&i2c->dev.kobj, &aw8680x_attribute_group);
	devm_kfree(&i2c->dev, aw8680x);

	return 0;
}

/*****************************************************
 *
 * pm sleep
 *
 *****************************************************/
#ifdef CONFIG_PM_SLEEP
static int aw8680x_suspend(struct device *dev)
{
	pr_debug("enter aw8680x_suspend\n");
	return 0;
}

static int aw8680x_resume(struct device *dev)
{
	pr_debug("enter aw8680x_resume\n");
	return 0;
}

/* Sleep wake-up mechanism, in this function you can decide
 * what to do when the phone is in sleep wake-up state
 */
static SIMPLE_DEV_PM_OPS(aw8680x_pm_ops, aw8680x_suspend, aw8680x_resume);
#endif

static const struct i2c_device_id aw8680x_i2c_id[] = {
	{AW8680X_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw8680x_i2c_id);

/* Match device information in the device tree */
static const struct of_device_id aw8680x_dt_match[] = {
	{.compatible = "awinic,aw8680x"},
	{},
};

static struct i2c_driver aw8680x_i2c_driver = {
	.driver = {
		   .name = AW8680X_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw8680x_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw8680x_pm_ops,
#endif
		   },
	.probe = aw8680x_i2c_probe,
	.remove = aw8680x_i2c_remove,
	.id_table = aw8680x_i2c_id,
};

static int __init aw8680x_i2c_init(void)
{
	int ret = 0;

	pr_info("aw8680x driver version %s\n", AW8680X_DRIVER_VERSION);
	/* Register the device driver on the i2c bus */
	ret = i2c_add_driver(&aw8680x_i2c_driver);
	if (ret) {
		pr_err("fail to add aw8680x device into i2c\n");
		return ret;
	}
	return 0;
}

/* Entry function */
late_initcall(aw8680x_i2c_init);

static void __exit aw8680x_i2c_exit(void)
{
	i2c_del_driver(&aw8680x_i2c_driver);
}

module_exit(aw8680x_i2c_exit);

MODULE_DESCRIPTION("AW8680X Touch Key Driver");
MODULE_LICENSE("GPL v2");
