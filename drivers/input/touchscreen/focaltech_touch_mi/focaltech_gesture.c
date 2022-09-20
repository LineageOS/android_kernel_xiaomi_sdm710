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

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
/*****************************************************************************
* Static variables
*****************************************************************************/
extern struct fts_ts_data *fts_data;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static ssize_t double_tap_show(struct kobject *kobj,
                               struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fts_data->double_tap);
}

static ssize_t double_tap_store(struct kobject *kobj,
                                struct kobj_attribute *attr, const char *buf,
                                size_t count)
{
	int rc, val;

	rc = kstrtoint(buf, 10, &val);
	if (rc)
		return -EINVAL;

	fts_data->double_tap = !!val;

	return count;
}

static struct tp_common_ops double_tap_ops = {
	.show = double_tap_show,
	.store = double_tap_store,
};

/*****************************************************************************
*   Name: fts_gesture_recovery
*  Brief: recovery gesture state when reset or power on
*  Input:
* Output:
* Return:
*****************************************************************************/
void fts_gesture_recovery(struct i2c_client *client)
{
	if (fts_data->double_tap && fts_data->suspended) {
		fts_i2c_write_reg(client, 0xD1, 0xff);
		fts_i2c_write_reg(client, 0xD2, 0xff);
		fts_i2c_write_reg(client, 0xD5, 0xff);
		fts_i2c_write_reg(client, 0xD6, 0xff);
		fts_i2c_write_reg(client, 0xD7, 0xff);
		fts_i2c_write_reg(client, 0xD8, 0xff);
		fts_gesture_mode_set(client, true);

		fts_features_set(client, FTS_REG_FEATURES_DOUBLETAP, true);
	}
}

void fts_fod_recovery(struct i2c_client *client)
{
	FTS_FUNC_ENTER();
	if (fts_data->suspended) {
		FTS_INFO("%s, tp is in suspend mode, write 0xd0 to 1", __func__);
		fts_gesture_mode_set(client, true);
	}
	fts_features_set(client, FTS_REG_FEATURES_FOD, true);
}

int fts_features_set(struct i2c_client *client, u8 features_mask, bool enable)
{
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = fts_i2c_update_reg(client, FTS_REG_FEATURES,
					 features_mask, enable);
		if (ret != -EAGAIN)
			break;
		msleep(1);
	}

	if (ret < 0) {
		FTS_ERROR("Failed to update features register: %d", ret);
		ret = -EIO;
	}

	return ret;
}

int fts_gesture_mode_set(struct i2c_client *client, bool enable)
{
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = fts_i2c_update_reg(client, FTS_REG_GESTURE_MODE_EN,
					 FTS_REG_GESTURE_MODE_MASK, enable);
		if (ret != -EAGAIN)
			break;
		msleep(1);
	}

	if (ret < 0) {
                FTS_ERROR("Failed to set gesture mode: %d", ret);
		ret = -EIO;
        }

	return ret;
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

	if (!fts_data->double_tap) {
		FTS_INFO("Double-tap is disabled");
		return -EINVAL;
	}

	ret = fts_gesture_mode_set(client, true);
	if (ret) {
		FTS_ERROR("Failed to set gesture mode: %d", ret);
		return -EIO;
	}

	ret = fts_features_set(client, FTS_REG_FEATURES_DOUBLETAP, true);
	if (ret) {
		FTS_ERROR("Failed to enable double-tap feature");
		return -EIO;
	}

	FTS_INFO("Successfully entered gesture mode");

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

	if (!fts_data->double_tap) {
		FTS_INFO("Double-tap is disabled");
		return -EINVAL;
	}

	ret = fts_gesture_mode_set(client, false);
	if (ret) {
		FTS_ERROR("Failed to leave gesture mode: %d", ret);
		return -EIO;
	}

	ret = fts_features_set(client, FTS_REG_FEATURES_DOUBLETAP, false);
	if (ret) {
		FTS_ERROR("Failed to disable double-tap feature: %d", ret);
		return -EIO;
	}

	msleep(10);
	FTS_INFO("Successfully left gesture mode");

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
	struct input_dev *input_dev = ts_data->input_dev;
	int ret;

	FTS_FUNC_ENTER();
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	__set_bit(KEY_WAKEUP, input_dev->keybit);

	ret = tp_common_set_double_tap_ops(&double_tap_ops);
	if (ret < 0) {
		FTS_ERROR("%s: Failed to create double_tap node err=%d\n",
		          __func__, ret);
	}

	fts_data->double_tap = false;

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
	FTS_FUNC_EXIT();
	return 0;
}
