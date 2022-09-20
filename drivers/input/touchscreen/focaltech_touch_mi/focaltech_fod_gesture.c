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

int fts_fod_gesture_readdata(struct fts_ts_data *ts_data)
{
	u8 buf[10] = { 0 };
	int ret;
	int x, y, z;

	buf[0] = FTS_REG_FOD_OUTPUT_ADDRESS;
	ret = fts_i2c_read(ts_data->client, buf, 1, buf + 1, 9);
	if (ret < 0) {
		FTS_ERROR("read fod failed, ret:%d", ret);
		return ret;
	}

	/*
	 * buf[1]: point id
	 * buf[2]:event type， 0x24 is doubletap, 0x25 is single tap, 0x26 is fod pointer event
	 * buf[3]: touch area/fod sensor area
	 * buf[4]: touch area
	 * buf[5-8]: x,y position
	 * buf[9]:pointer up or down, 0 is down, 1 is up
	 * */
	switch (buf[2]) {
	case 0x24:
		FTS_INFO("DoubleClick Gesture detected, Wakeup panel\n");
		input_report_key(ts_data->input_dev, KEY_WAKEUP, 1);
		input_sync(ts_data->input_dev);
		input_report_key(ts_data->input_dev, KEY_WAKEUP, 0);
		input_sync(ts_data->input_dev);
		break;
	case 0x25:
		FTS_INFO("FOD status report KEY_GOTO\n");
		input_report_key(ts_data->input_dev, KEY_GOTO, 1);
		input_sync(ts_data->input_dev);
		input_report_key(ts_data->input_dev, KEY_GOTO, 0);
		input_sync(ts_data->input_dev);
		break;
	case 0x26:
		x = (buf[5] << 8) | buf[6];
		y = (buf[7] << 8) | buf[8];
		z = buf[4];
		pr_info("FTS:read fod data: 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x anxis_x: %d anxis_y: %d\n",
			buf[1], buf[2], buf[3], buf[4], buf[9], x, y);
		if (buf[9] == 0) {
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
				input_mt_slot(ts_data->input_dev, buf[1]);
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
			input_mt_slot(ts_data->input_dev, buf[1]);
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

	return 0;
}
