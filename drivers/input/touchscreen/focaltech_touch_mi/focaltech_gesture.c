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
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = fts_i2c_update_reg(client, FTS_REG_GESTURE_SUPPORT,
					 mask, enable);
		if (ret != -EAGAIN)
			break;
		msleep(1);
	}

	if (ret < 0) {
		FTS_ERROR("Failed to update fod reg\n");
		ret = -EIO;
	}

	return ret;
}

int fts_gesture_reg_write(struct i2c_client *client, u8 mask, bool enable)
{
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = fts_i2c_update_reg(client, FTS_REG_GESTURE_EN, mask,
					 enable);
		if (ret != -EAGAIN)
			break;
		msleep(1);
	}

	if (ret < 0) {
                FTS_ERROR("Failed to update gesture reg\n");
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
	FTS_FUNC_EXIT();
	return 0;
}
