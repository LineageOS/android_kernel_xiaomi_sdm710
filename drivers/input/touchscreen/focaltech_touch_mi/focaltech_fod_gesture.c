/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2022 The LineageOS Project
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

#include <linux/input/tp_common.h>
#include "focaltech_core.h"

#define FOD_EVENT_DOUBLE_TAP	0x24
#define FOD_EVENT_SINGLE_TAP	0x25
#define FOD_EVENT_FOD_PTR	0x26

struct fts_fod_event {
	__u8   point_id;
	__u8   type;
	__be16 area;
	__be16 x;
	__be16 y;
	__u8   released;
};

static inline bool fts_fod_is_changed(struct fts_ts_data *ts_data,
				      bool is_pressed)
{
	bool was_pressed = ts_data->finger_in_fod;

	if (is_pressed && ts_data->fod_finger_skip)
		return false;

	return was_pressed != is_pressed;
}

static int fts_fod_report_ptr_event(struct fts_ts_data *ts_data,
				    struct fts_fod_event *ev)
{
	struct input_dev *input_dev = ts_data->input_dev;
	bool pressed;
	int x, y, z;

	x = be16_to_cpu(ev->x);
	y = be16_to_cpu(ev->y);
	z = be16_to_cpu(ev->area);
	pressed = !ev->released;

	if (fts_fod_is_changed(ts_data, pressed)) {
		ts_data->finger_in_fod = pressed;
		ts_data->fod_x = x;
		ts_data->fod_y = y;
		tp_common_notify_fp_state();
		input_report_key(input_dev, BTN_INFO, pressed ? 1 : 0);
		input_sync(input_dev);
	}

	if (!pressed)
		ts_data->fod_finger_skip = false;

	if (!ts_data->suspended) {
		FTS_INFO("Not suspended. Report by normal touch report\n");
		return -EINVAL;
	}

	if (!pressed) {
		input_mt_slot(input_dev, ev->point_id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
		input_sync(input_dev);

		return 0;
	}

	if (ts_data->aod_status && !ts_data->fod_finger_skip) {
		input_mt_slot(input_dev, ev->point_id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
		input_report_key(input_dev, BTN_TOUCH, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, z);
		input_sync(input_dev);
	}

	return 0;
}

static int fts_fod_report(struct fts_ts_data *ts_data, struct fts_fod_event *ev)
{
	struct input_dev *input_dev = ts_data->input_dev;
	int ret = 0;

	switch (ev->type) {
	case FOD_EVENT_DOUBLE_TAP:
		FTS_INFO("Double-tap detected, wake-up system\n");
		input_report_key(input_dev, KEY_WAKEUP, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_WAKEUP, 0);
		input_sync(input_dev);
		break;
	case FOD_EVENT_SINGLE_TAP:
		FTS_INFO("FOD status report KEY_GOTO\n");
		input_report_key(input_dev, KEY_GOTO, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_GOTO, 0);
		input_sync(input_dev);
		break;
	case FOD_EVENT_FOD_PTR:
		ret = fts_fod_report_ptr_event(ts_data, ev);
		break;
	default:
		return ts_data->suspended ? 0 : -EINVAL;
	}

	FTS_INFO("FOD data: point_id=%u, type=%u, area=%u, x=%u, y=%u, rel=%u",
		 ev->point_id, ev->type, be16_to_cpu(ev->area),
		 be16_to_cpu(ev->x), be16_to_cpu(ev->y), ev->released);

	return ret;
}

int fts_fod_gesture_readdata(struct fts_ts_data *ts_data)
{
	struct fts_fod_event ev;
	int ret;
	u8 reg;

	reg = FTS_REG_FOD_OUTPUT_ADDRESS;
	ret = fts_i2c_read(ts_data->client, &reg, 1, (u8 *)&ev, sizeof(ev));
	if (ret < 0) {
		FTS_ERROR("failed to read fod data, ret: %d", ret);
		return ret;
	}

	mutex_lock(&ts_data->report_mutex);
	ret = fts_fod_report(ts_data, &ev);
	mutex_unlock(&ts_data->report_mutex);

	return ret;
}

static int fts_features_set(struct i2c_client *client, u8 features_mask,
			    bool enable)
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

static int fts_gesture_mode_set(struct i2c_client *client, bool enable)
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


void fts_fod_gesture_recovery(struct i2c_client *client)
{
	u8 features = FTS_REG_FEATURES_FOD;
	int ret;

	if (fts_data->double_tap)
		features |= FTS_REG_FEATURES_DOUBLETAP;

	ret = fts_features_set(client, features, true);
	if (ret < 0)
		FTS_ERROR("Failed to update features");


	if (fts_data->suspended) {
		fts_i2c_write_reg(client, 0xD1, 0xff);
		fts_i2c_write_reg(client, 0xD2, 0xff);
		fts_i2c_write_reg(client, 0xD5, 0xff);
		fts_i2c_write_reg(client, 0xD6, 0xff);
		fts_i2c_write_reg(client, 0xD7, 0xff);
		fts_i2c_write_reg(client, 0xD8, 0xff);

		ret = fts_gesture_mode_set(client, true);
		if (ret < 0)
			FTS_ERROR("Failed to restore gesture mode");
	}
}

/*
 * During irq disabled, FTS_REG_INT_ACK(0x3E) register will store three frames
 * touchevent data. This register need to be cleared in case some points lost
 * Tracking ID
 */
static int fts_ts_clear_buffer(struct i2c_client *client)
{
	int i, ret;
	u8 value;

	for (i = 0; i < 3; i++) {
		ret = fts_i2c_read_reg(client, FTS_REG_INT_ACK, &value);
		if (ret < 0) {
			FTS_ERROR("Failed to read INT ACK register");
			break;
		}
	}

	return ret;
}

int fts_fod_gesture_suspend(struct i2c_client *client)
{
	int ret;

	fts_data->fod_point_released = false;

	/* Enable/disable double-tap per request */
	ret = fts_features_set(client, FTS_REG_FEATURES_DOUBLETAP,
			       fts_data->double_tap);
	if (ret < 0)
		FTS_ERROR("Failed to %s double-tap feature",
			  fts_data->double_tap ? "enable" : "disable");

	/* Always enable FOD sensor */
	ret = fts_features_set(client, FTS_REG_FEATURES_FOD, true);
	if (ret < 0)
		FTS_ERROR("Failed to enable FOD sensor feature");

	/* Enter gesture mode */
	ret = fts_gesture_mode_set(client, true);
	if (ret < 0) {
		FTS_ERROR("Failed to enter gesture mode: %d", ret);
		return -EIO;
	}

	fts_ts_clear_buffer(client);

	FTS_INFO("Successfully entered gesture mode");

	return 0;
}

int fts_fod_gesture_resume(struct i2c_client *client)
{
	int ret;

	if (fts_data->finger_in_fod) {
		fts_data->finger_in_fod = false;
		fts_features_set(client, FTS_REG_FEATURES_FOD_NO_CAL, true);
	}

	/* Leave gesture mode */
	ret = fts_gesture_mode_set(client, false);
	if (ret < 0) {
		FTS_ERROR("Failed to leave gesture mode: %d", ret);
		return -EIO;
	}

	ret = fts_features_set(client, FTS_REG_FEATURES_DOUBLETAP, false);
	if (ret < 0) {
		FTS_ERROR("Failed to disable double-tap feature: %d", ret);
		return -EIO;
	}

	msleep(10);
	FTS_INFO("Successfully left gesture mode");

	return 0;
}

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

int fts_fod_gesture_init(struct fts_ts_data *ts_data)
{
	int ret;

	ret = tp_common_set_double_tap_ops(&double_tap_ops);
	if (ret < 0) {
		FTS_ERROR("%s: Failed to create double_tap node err=%d\n",
		          __func__, ret);
	}

	fts_data->double_tap = false;

	return 0;
}

int fts_fod_gesture_exit(struct i2c_client *client)
{
	return 0;
}
