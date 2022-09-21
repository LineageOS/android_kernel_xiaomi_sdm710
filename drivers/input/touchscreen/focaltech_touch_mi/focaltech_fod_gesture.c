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
		input_report_key(input_dev, BTN_TOOL_FINGER, 1);
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
