/*
 *
 * FocalTech fts TouchScreen driver.
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
* File Name: focaltech_common.h
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-16
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_COMMON_H__
#define __LINUX_FOCALTECH_COMMON_H__

#include "focaltech_config.h"

/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_VERSION                  "Focaltech V2.1 20171229"

#define BYTE_OFF_0(x)           (u8)((x) & 0xFF)
#define BYTE_OFF_8(x)           (u8)((x >> 8) & 0xFF)
#define BYTE_OFF_16(x)          (u8)((x >> 16) & 0xFF)
#define BYTE_OFF_24(x)          (u8)((x >> 24) & 0xFF)

#define FLAG_HID_BIT            10
#define FLAG_IDC_BIT            11

#define FTS_CHIP_IDC            (!!(FTS_CHIP_TYPE & BIT(FLAG_IDC_BIT)))
#define FTS_HID_SUPPORTTED      (!!(FTS_CHIP_TYPE & BIT(FLAG_HID_BIT)))

#define FTS_CHIP_TYPE_MAPPING {\
	{0x0D, 0x87, 0x19, 0x87, 0x19, 0x87, 0xA9, 0x87, 0xB9},\
	{0x81, 0x54, 0x52, 0x54, 0x52, 0x00, 0x00, 0x54, 0x5c},\
}

#define I2C_BUFFER_LENGTH_MAXINUM           256
#define FILE_NAME_LENGTH                    128
#define ENABLE                              1
#define DISABLE                             0
#define VALID                               1
#define INVALID                             0
#define FTS_CMD_START1                      0x55
#define FTS_CMD_START2                      0xAA
#define FTS_CMD_START_DELAY                 10
#define FTS_CMD_READ_ID                     0x90
#define FTS_CMD_READ_ID_LEN                 4
#define FTS_CMD_READ_ID_LEN_INCELL          1

/*register address*/
#define FTS_REG_INT_CNT                     0x8F
#define FTS_REG_CHIP_ID                     0xA3
#define FTS_REG_CHIP_ID2                    0x9F
#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_POWER_MODE_SLEEP_VALUE      0x03
#define FTS_REG_FW_VER                      0xA6
#define FTS_REG_VENDOR_ID                   0xA8
#define FTS_REG_LCD_BUSY_NUM                0xAB
#define FTS_REG_IDE_PARA_VER_ID             0xB5
#define FTS_REG_IDE_PARA_STATUS             0xB6
#define FTS_REG_CHARGER_MODE_EN             0x8B
#define FTS_REG_FEATURES                    0xCF /* Features */
#define FTS_REG_FEATURES_DOUBLETAP          0x01 /* Double-tap feature */
#define FTS_REG_FEATURES_FOD                0x02 /* FOD sensor feature */
#define FTS_REG_FEATURES_FOD_NO_CAL         0x10
#define FTS_REG_GESTURE_MODE_EN             0xD0 /* Gesture/normal mode */
#define FTS_REG_GESTURE_MODE_MASK           0x01
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define FTS_REG_FOD_OUTPUT_ADDRESS          0xE1
#define FTS_REG_MODULE_ID                   0xE3
#define FTS_REG_LIC_VER                     0xE4
#define FTS_REG_INT_ACK                     0x3E

#define BTN_INFO                            0x155

#define FTS_SYSFS_ECHO_ON(buf)      (buf[0] == '1')
#define FTS_SYSFS_ECHO_OFF(buf)     (buf[0] == '0')

#define kfree_safe(pbuf) do { \
    if (pbuf) { \
		kfree(pbuf); \
		pbuf = NULL; \
	} \
} while (0)

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct ft_chip_t {
	u64 type;
	u8 chip_idh;
	u8 chip_idl;
	u8 rom_idh;
	u8 rom_idl;
	u8 pb_idh;
	u8 pb_idl;
	u8 bl_idh;
	u8 bl_idl;
};

struct ts_ic_info {
	bool is_incell;
	bool hid_supported;
	struct ft_chip_t ids;
};

/*****************************************************************************
* DEBUG function define here
*****************************************************************************/
#if FTS_DEBUG_EN
#define FTS_DEBUG_LEVEL     1
#if (FTS_DEBUG_LEVEL == 2)
#define FTS_DEBUG(fmt, args...) printk("[FTS][%s]"fmt"\n", __func__, ##args)
#else
#define FTS_DEBUG(fmt, args...) printk("[FTS]"fmt"\n", ##args)
#endif
#define FTS_FUNC_ENTER() printk("[FTS]%s: Enter\n", __func__)
#define FTS_FUNC_EXIT()  printk("[FTS]%s: Exit(%d)\n", __func__, __LINE__)
#else /* #if FTS_DEBUG_EN */
#define FTS_DEBUG(fmt, args...)
#define FTS_FUNC_ENTER()
#define FTS_FUNC_EXIT()
#endif

#define FTS_INFO(fmt, args...) printk(KERN_INFO "[FTS][Info]"fmt"\n", ##args)
#define FTS_ERROR(fmt, args...) printk(KERN_ERR "[FTS][Error]"fmt"\n", ##args)

#endif /* __LINUX_FOCALTECH_COMMON_H__ */
