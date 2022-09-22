/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: focaltech_gestrue.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"
#include <linux/input/tp_common.h>

/******************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_GESTRUE_POINTS                      255
#define FTS_GESTRUE_POINTS_HEADER               8

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*
* header        -   byte0:gesture id
*                   byte1:pointnum
*                   byte2~7:reserved
* coordinate_x  -   All gesture point x coordinate
* coordinate_y  -   All gesture point y coordinate
* mode          -   1:enable gesture function(default)
*               -   0:disable
* active        -   1:enter into gesture(suspend)
*                   0:gesture disable or resume
*/
struct fts_gesture_st {
	u8 header[FTS_GESTRUE_POINTS_HEADER];
	u16 coordinate_x[FTS_GESTRUE_POINTS];
	u16 coordinate_y[FTS_GESTRUE_POINTS];
	u8 mode;		/*host driver enable gesture flag */
	u8 active;		/*gesture actutally work */
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct fts_gesture_st fts_gesture_data;
extern struct fts_ts_data *fts_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/* sysfs gesture node
 *   read example: cat  fts_gesture_mode        ---read gesture mode
 *   write example:echo 01 > fts_gesture_mode   ---write gesture mode to 01
 *
 */
static DEVICE_ATTR(fts_gesture_mode, S_IRUGO | S_IWUSR, fts_gesture_show, fts_gesture_store);
/*
 *   read example: cat fts_gesture_buf        ---read gesture buf
 */
static DEVICE_ATTR(fts_gesture_buf, S_IRUGO | S_IWUSR, fts_gesture_buf_show, fts_gesture_buf_store);
static struct attribute *fts_gesture_mode_attrs[] = {
	&dev_attr_fts_gesture_mode.attr,
	&dev_attr_fts_gesture_buf.attr,
	NULL,
};

static struct attribute_group fts_gesture_group = {
	.attrs = fts_gesture_mode_attrs,
};

static ssize_t double_tap_show(struct kobject *kobj,
                               struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fts_gesture_data.mode);
}

static ssize_t double_tap_store(struct kobject *kobj,
                                struct kobj_attribute *attr, const char *buf,
                                size_t count)
{
	int rc, val;

	rc = kstrtoint(buf, 10, &val);
	if (rc)
	return -EINVAL;

	fts_gesture_data.mode = !!val;
	return count;
}

static struct tp_common_ops double_tap_ops = {
	.show = double_tap_show,
	.store = double_tap_store,
};

/************************************************************************
* Name: fts_gesture_show
*  Brief:
*  Input: device, device attribute, char buf
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	u8 val;
	struct input_dev *input_dev = fts_data->input_dev;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&input_dev->mutex);
	fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &val);
	count = snprintf(buf, PAGE_SIZE, "Gesture Mode: %s\n", fts_gesture_data.mode ? "On" : "Off");
	count += snprintf(buf + count, PAGE_SIZE, "Reg(0xD0) = %d\n", val);
	mutex_unlock(&input_dev->mutex);

	return count;
}

/************************************************************************
* Name: fts_gesture_store
*  Brief:
*  Input: device, device attribute, char buf, char count
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_dev = fts_data->input_dev;
	mutex_lock(&input_dev->mutex);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_INFO("[GESTURE]enable gesture");
		fts_gesture_data.mode = ENABLE;
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_INFO("[GESTURE]disable gesture");
		fts_gesture_data.mode = DISABLE;
	}
	mutex_unlock(&input_dev->mutex);

	return count;
}

/************************************************************************
* Name: fts_gesture_buf_show
*  Brief:
*  Input: device, device attribute, char buf
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int i = 0;
	struct input_dev *input_dev = fts_data->input_dev;

	mutex_lock(&input_dev->mutex);
	count = snprintf(buf, PAGE_SIZE, "Gesture ID: 0x%x\n", fts_gesture_data.header[0]);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture PointNum: %d\n", fts_gesture_data.header[1]);
	count += snprintf(buf + count, PAGE_SIZE, "Gesture Point Buf:\n");
	for (i = 0; i < fts_gesture_data.header[1]; i++) {
		count +=
		    snprintf(buf + count, PAGE_SIZE, "%3d(%4d,%4d) ", i, fts_gesture_data.coordinate_x[i],
			     fts_gesture_data.coordinate_y[i]);
		if ((i + 1) % 4 == 0)
			count += snprintf(buf + count, PAGE_SIZE, "\n");
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");
	mutex_unlock(&input_dev->mutex);

	return count;
}

/************************************************************************
* Name: fts_gesture_buf_store
*  Brief:
*  Input: device, device attribute, char buf, char count
* Output:
* Return:
***********************************************************************/
static ssize_t fts_gesture_buf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

/*****************************************************************************
*   Name: fts_create_gesture_sysfs
*  Brief:
*  Input:
* Output:
* Return: 0-success or others-error
*****************************************************************************/
int fts_create_gesture_sysfs(struct i2c_client *client)
{
	int ret = 0;

	ret = sysfs_create_group(&client->dev.kobj, &fts_gesture_group);
	if (ret != 0) {
		FTS_ERROR("[GESTURE]fts_gesture_mode_group(sysfs) create failed!");
		sysfs_remove_group(&client->dev.kobj, &fts_gesture_group);
		return ret;
	}
	return 0;
}

/*****************************************************************************
*   Name: fts_gesture_recovery
*  Brief: recovery gesture state when reset or power on
*  Input:
* Output:
* Return:
*****************************************************************************/
void fts_gesture_recovery(struct i2c_client *client)
{
	if ((ENABLE == fts_gesture_data.mode) && (ENABLE == fts_gesture_data.active)) {
		FTS_INFO("enter fts_gesture_recovery");
		fts_i2c_write_reg(client, 0xD1, 0xff);
		fts_i2c_write_reg(client, 0xD2, 0xff);
		fts_i2c_write_reg(client, 0xD5, 0xff);
		fts_i2c_write_reg(client, 0xD6, 0xff);
		fts_i2c_write_reg(client, 0xD7, 0xff);
		fts_i2c_write_reg(client, 0xD8, 0xff);
		fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);

		fts_fod_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
	}
}

void fts_fod_recovery(struct i2c_client *client)
{
	FTS_FUNC_ENTER();
	if (fts_data->suspended) {
		FTS_INFO("%s, tp is in suspend mode, write 0xd0 to 1", __func__);
		fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
	}
	fts_fod_reg_write(client, FTS_REG_GESTURE_FOD_ON, true);
}

int fts_fod_reg_write(struct i2c_client *client, u8 mask, bool enable)
{
	int i;
	u8 state;
	u8 reg_value;

	for (i = 0; i < 5; i++) {
		fts_i2c_read_reg(client, FTS_REG_GESTURE_SUPPORT, &reg_value);
		if (enable)
			reg_value |= mask;
		else
			reg_value &= ~mask;
		fts_i2c_write_reg(client, FTS_REG_GESTURE_SUPPORT, reg_value);
		msleep(1);
		fts_i2c_read_reg(client, FTS_REG_GESTURE_SUPPORT, &state);
		if (state == reg_value)
			break;
	}

	if (i >= 5) {
		FTS_ERROR("[GESTURE]Write fod reg failed!\n");
		return -EIO;
	} else {
		FTS_ERROR("[GESTURE]Write fod reg success!\n");
		return 0;
	}
}

int fts_gesture_reg_write(struct i2c_client *client, u8 mask, bool enable)
{
	int i;
	u8 state;
	u8 reg_value;

	for (i = 0; i < 5; i++) {
		fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &reg_value);
		if (enable)
			reg_value |= mask;
		else
			reg_value &= ~mask;
		fts_i2c_write_reg(client, FTS_REG_GESTURE_EN, reg_value);
		msleep(1);
		fts_i2c_read_reg(client, FTS_REG_GESTURE_EN, &state);
		if (state == reg_value)
			break;
	}

	if (i >= 5) {
		FTS_ERROR("[GESTURE]Write gesture reg failed!\n");
		return -EIO;
	} else {
		FTS_ERROR("[GESTURE]Write gesture reg success!\n");
		return 0;
	}
}

/*****************************************************************************
*   Name: fts_gesture_suspend
*  Brief:
*  Input:
* Output:
* Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
int fts_gesture_suspend(struct i2c_client *client)
{
	int ret;

	FTS_INFO("gesture suspend...");
	/* gesture not enable, return immediately */
	if (fts_gesture_data.mode == DISABLE) {
		FTS_INFO("gesture is disabled");
		return -EINVAL;
	}
	ret = fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
	if (ret) {
		FTS_ERROR("[GESTURE]Enter into gesture(suspend) failed!\n");
		fts_gesture_data.active = DISABLE;
		return -EIO;
	}

	ret = fts_fod_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, true);
	if (ret) {
		FTS_ERROR("[GESTURE]Enter into gesture(suspend) failed!\n");
		fts_gesture_data.active = DISABLE;
		return -EIO;
	}

	fts_gesture_data.active = ENABLE;
	FTS_INFO("[GESTURE]Enter into gesture(suspend) successfully!");
	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
*   Name: fts_gesture_resume
*  Brief:
*  Input:
* Output:
* Return: return 0 if succuss, otherwise return error code
*****************************************************************************/
int fts_gesture_resume(struct i2c_client *client)
{
	int ret;

	FTS_INFO("gesture resume...");
	/* gesture not enable, return immediately */
	if (fts_gesture_data.mode == DISABLE) {
		FTS_DEBUG("gesture is disabled");
		return -EINVAL;
	}

	if (fts_gesture_data.active == DISABLE) {
		FTS_DEBUG("gesture in suspend is failed, no running fts_gesture_resume");
		return -EINVAL;
	}
	ret = fts_gesture_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, false);
	if (ret) {
		FTS_ERROR("[GESTURE]Resume from gesture failed!\n");
		return -EIO;
	}

	ret = fts_fod_reg_write(client, FTS_REG_GESTURE_DOUBLETAP_ON, false);
	if (ret) {
		FTS_ERROR("[GESTURE]resume from gesture(suspend) failed!\n");
		return -EIO;
	}

	fts_gesture_data.active = DISABLE;
	msleep(10);
	FTS_INFO("[GESTURE]resume from gesture successfully!");
	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
*   Name: fts_gesture_init
*  Brief:
*  Input:
* Output:
* Return:
*****************************************************************************/
int fts_gesture_init(struct fts_ts_data *ts_data)
{
	struct i2c_client *client = ts_data->client;
	struct input_dev *input_dev = ts_data->input_dev;
	int ret;

	FTS_FUNC_ENTER();
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	__set_bit(KEY_WAKEUP, input_dev->keybit);

	fts_create_gesture_sysfs(client);

	ret = tp_common_set_double_tap_ops(&double_tap_ops);
	if (ret < 0) {
		FTS_ERROR("%s: Failed to create double_tap node err=%d\n",
		          __func__, ret);
	}

	fts_gesture_data.mode = DISABLE;
	fts_gesture_data.active = DISABLE;

	FTS_FUNC_EXIT();
	return 0;
}

/************************************************************************
*   Name: fts_gesture_exit
*  Brief: call when driver removed
*  Input:
* Output:
* Return:
***********************************************************************/
int fts_gesture_exit(struct i2c_client *client)
{
	FTS_FUNC_ENTER();
	sysfs_remove_group(&client->dev.kobj, &fts_gesture_group);
	FTS_FUNC_EXIT();
	return 0;
}
