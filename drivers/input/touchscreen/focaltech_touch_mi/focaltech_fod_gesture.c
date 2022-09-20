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

int fts_fod_gesture_readdata(struct fts_ts_data *ts_data)
{
	struct fts_fod_event ev;
	int x, y, z;
	int ret;
	u8 reg;

	reg = FTS_REG_FOD_OUTPUT_ADDRESS;
	ret = fts_i2c_read(ts_data->client, &reg, 1, (u8 *)&ev, sizeof(ev));
	if (ret < 0) {
		FTS_ERROR("failed to read fod data, ret: %d", ret);
		return ret;
	}

	switch (ev.type) {
	case FOD_EVENT_DOUBLE_TAP:
		FTS_INFO("DoubleClick Gesture detected, Wakeup panel\n");
		input_report_key(ts_data->input_dev, KEY_WAKEUP, 1);
		input_sync(ts_data->input_dev);
		input_report_key(ts_data->input_dev, KEY_WAKEUP, 0);
		input_sync(ts_data->input_dev);
		break;
	case FOD_EVENT_SINGLE_TAP:
		FTS_INFO("FOD status report KEY_GOTO\n");
		input_report_key(ts_data->input_dev, KEY_GOTO, 1);
		input_sync(ts_data->input_dev);
		input_report_key(ts_data->input_dev, KEY_GOTO, 0);
		input_sync(ts_data->input_dev);
		break;
	case FOD_EVENT_FOD_PTR:
		x = be16_to_cpu(ev.x);
		y = be16_to_cpu(ev.y);
		z = be16_to_cpu(ev.area);
		if (!ev.released) {
			mutex_lock(&ts_data->report_mutex);
			if (!ts_data->fod_finger_skip && !ts_data->finger_in_fod) {
				input_report_key(ts_data->input_dev, BTN_INFO, 1);
				input_sync(ts_data->input_dev);
				FTS_INFO("Report 0x155 Down for FingerPrint\n");
				ts_data->finger_in_fod = true;
				ts_data->fod_x = x;
				ts_data->fod_y = y;
				tp_common_notify_fp_state();
			}
			if (!ts_data->suspended) {
				pr_info("FTS:touch is not in suspend state, report x,y value by touch nomal report\n");
				mutex_unlock(&ts_data->report_mutex);
				return -EINVAL;
			}
			if (!ts_data->aod_status) {
				FTS_INFO("Panel is suspended but not in AOD mode, do not report touch down event\n");
				mutex_unlock(&ts_data->report_mutex);
				return 0;
			}

			if (!ts_data->fod_finger_skip) {
				input_mt_slot(ts_data->input_dev, ev.point_id);
				input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 1);
				input_report_key(ts_data->input_dev, BTN_TOUCH, 1);
				input_report_key(ts_data->input_dev, BTN_TOOL_FINGER, 1);
				input_report_abs(ts_data->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(ts_data->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR, z);
				input_sync(ts_data->input_dev);
			}
			mutex_unlock(&ts_data->report_mutex);
		} else {
			if (ts_data->finger_in_fod) {
				input_report_key(ts_data->input_dev, BTN_INFO, 0);
				input_sync(ts_data->input_dev);
			}
			ts_data->finger_in_fod = false;
			ts_data->fod_finger_skip = false;
			ts_data->fod_x = 0;
			ts_data->fod_y = 0;
			tp_common_notify_fp_state();
			if (!ts_data->suspended) {
				pr_info("FTS:touch is not in suspend state, report x,y value by touch nomal report\n");
				return -EINVAL;
			}
			mutex_lock(&ts_data->report_mutex);
			input_mt_slot(ts_data->input_dev, ev.point_id);
			input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 0);
			input_report_key(ts_data->input_dev, BTN_TOUCH, 0);
			input_report_abs(ts_data->input_dev, ABS_MT_TRACKING_ID, -1);
			input_sync(ts_data->input_dev);
			mutex_unlock(&ts_data->report_mutex);
		}
		break;
	default:
		if (ts_data->suspended)
			return 0;
		else
			return -EINVAL;
		break;
	}

	FTS_INFO("FOD data: point_id=%u, type=%u, area=%u, x=%u, y=%u, rel=%u",
		 ev.point_id, ev.type, be16_to_cpu(ev.area), be16_to_cpu(ev.x),
		 be16_to_cpu(ev.y), ev.released);


	return 0;
}
