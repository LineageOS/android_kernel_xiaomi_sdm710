/*
 * ALSA SoC Texas Instruments TAS2562 High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2016 Texas Instruments, Inc.
 *
 * Author: saiprasad
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#ifdef CONFIG_TAS2562_REGMAP

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>

#include "tas2562.h"
#include "tas2562-codec.h"

static char pICN[] = {0x00, 0x03, 0x46, 0xdc};

static int tas2562_regmap_write(struct tas2562_priv *tas_priv,
	unsigned int reg, unsigned int value)
{
	int result = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if(tas_priv->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while(retry_count--)
	{
		result = regmap_write(tas_priv->regmap, reg,
			value);
		if (result >= 0)
			break;
		msleep(20);
	}
	if(retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_bulk_write(struct tas2562_priv *tas_priv,
	unsigned int reg, const unsigned char *buf, unsigned int len)
{
	int result = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if(tas_priv->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while(retry_count --)
	{
		result = regmap_bulk_write(tas_priv->regmap, reg,
			 buf, len);
		if (result >= 0)
			break;
		msleep(20);
	}
	if(retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_read(struct tas2562_priv *tas_priv,
	unsigned int reg, unsigned int *value)
{
	int result = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if(tas_priv->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while(retry_count --)
	{
		result = regmap_read(tas_priv->regmap, reg,
			value);
		if (result >= 0)
			break;
		msleep(20);
	}
	if(retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_bulk_read(struct tas2562_priv *tas_priv,
	unsigned int reg, unsigned char *buf, unsigned int len)
{
	int result = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if(tas_priv->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while(retry_count --)
	{
		result = regmap_bulk_read(tas_priv->regmap, reg,
			 buf, len);
		if (result >= 0)
			break;
		msleep(20);
	}
	if(retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_regmap_update_bits(struct tas2562_priv *tas_priv,
	unsigned int reg, unsigned int mask, unsigned int value)
{
	int result = 0;
	int retry_count = TAS2562_I2C_RETRY_COUNT;

	if(tas_priv->i2c_suspend)
		return ERROR_I2C_SUSPEND;

	while(retry_count--)
	{
		result = regmap_update_bits(tas_priv->regmap, reg,
			mask, value);
		if (result >= 0)
			break;
		msleep(20);
	}
	if(retry_count == -1)
		return ERROR_I2C_FAILED;
	else
		return 0;
}

static int tas2562_change_book_page(struct tas2562_priv *tas_priv,
	int book, int page)
{
	int result = 0;

	if ((tas_priv->cur_book == book)
		&& (tas_priv->cur_page == page))
		goto end;

	if (tas_priv->cur_book != book) {
		result = tas2562_regmap_write(tas_priv, TAS2562_BOOKCTL_PAGE, 0);
		if (result < 0) {
			dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, result);
			tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
			goto end;
		}
		tas_priv->cur_page = 0;
		result = tas2562_regmap_write(tas_priv, TAS2562_BOOKCTL_REG, book);
		if (result < 0) {
			dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, result);
			tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
			goto end;
		}
		tas_priv->cur_book = book;
	}

	if (tas_priv->cur_page != page) {
		result = tas2562_regmap_write(tas_priv, TAS2562_BOOKCTL_PAGE, page);
		if (result < 0) {
			dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
				__func__, __LINE__, result);
			tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
			goto end;
		}
		tas_priv->cur_page = page;
	}

end:
	return result;
}

int tas2562_read(struct tas2562_priv *tas_priv, unsigned int reg,
		 unsigned int *pvalue)
{
	int result = 0;

	mutex_lock(&tas_priv->dev_lock);

	result = tas2562_change_book_page(tas_priv,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (result < 0)
		goto end;

	result = tas2562_regmap_read(tas_priv, TAS2562_PAGE_REG(reg), pvalue);
	if (result < 0) {
		dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, result);
		tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
	}
	else
		dev_dbg(tas_priv->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
			TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg));

end:
	mutex_unlock(&tas_priv->dev_lock);
	return result;
}

int tas2562_write(struct tas2562_priv *tas_priv, unsigned int reg,
		  unsigned int value)
{
	int result = 0;

	mutex_lock(&tas_priv->dev_lock);

	result = tas2562_change_book_page(tas_priv,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (result < 0)
		goto end;

	result = tas2562_regmap_write(tas_priv, TAS2562_PAGE_REG(reg),
			value);
	if (result < 0) {
		dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, result);
		tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
	}
	else
		dev_dbg(tas_priv->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
			__func__, TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), value);

end:
	mutex_unlock(&tas_priv->dev_lock);
	return result;
}

int tas2562_bulk_write(struct tas2562_priv *tas_priv, unsigned int reg,
		       const unsigned char *buf, unsigned int len)
{
	int result = 0;

	mutex_lock(&tas_priv->dev_lock);

	result = tas2562_change_book_page(tas_priv,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (result < 0)
		goto end;

	result = tas2562_regmap_bulk_write(tas_priv,
		TAS2562_PAGE_REG(reg), buf, len);
	if (result < 0) {
		dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, result);
		tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
	}
	else
		dev_dbg(tas_priv->dev, "%s: BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
			__func__, TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), len);

end:
	mutex_unlock(&tas_priv->dev_lock);
	return result;
}

int tas2562_bulk_read(struct tas2562_priv *tas_priv, unsigned int reg,
		      unsigned char *buf, unsigned int len)
{
	int result = 0;

	mutex_lock(&tas_priv->dev_lock);

	result = tas2562_change_book_page(tas_priv,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (result < 0)
		goto end;

	result = tas2562_regmap_bulk_read(tas_priv,
	TAS2562_PAGE_REG(reg), buf, len);
	if (result < 0) {
		dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, result);
		tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
	}
	else
		dev_dbg(tas_priv->dev, "%s: BOOK:PAGE:REG %u:%u:%u, len: 0x%02x\n",
			__func__, TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), len);
end:
	mutex_unlock(&tas_priv->dev_lock);
	return result;
}

int tas2562_update_bits(struct tas2562_priv *tas_priv, unsigned int reg,
			unsigned int mask, unsigned int value)
{
	int result = 0;

	mutex_lock(&tas_priv->dev_lock);
	result = tas2562_change_book_page(tas_priv,
		TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg));
	if (result < 0)
		goto end;

	result = tas2562_regmap_update_bits(tas_priv,
	TAS2562_PAGE_REG(reg), mask, value);
	if (result < 0) {
		dev_err(tas_priv->dev, "%s, ERROR, L=%d, E=%d\n",
			__func__, __LINE__, result);
		tas_priv->err_code |= ERROR_DEVA_I2C_COMM;
	}
	else
		dev_dbg(tas_priv->dev, "%s: BOOK:PAGE:REG %u:%u:%u, mask: 0x%x, val=0x%x\n",
			__func__, TAS2562_BOOK_ID(reg), TAS2562_PAGE_ID(reg),
			TAS2562_PAGE_REG(reg), mask, value);
end:
	mutex_unlock(&tas_priv->dev_lock);
	return result;
}

static bool tas2562_volatile(struct device *dev, unsigned int reg)
{
	return true;
}

static bool tas2562_writeable(struct device *dev, unsigned int reg)
{
	return true;
}
static const struct regmap_config tas2562_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2562_writeable,
	.volatile_reg = tas2562_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 1 * 128,
};


void tas2562_hw_reset(struct tas2562_priv *tas_priv)
{
	if (gpio_is_valid(tas_priv->reset_gpio)) {
		gpio_direction_output(tas_priv->reset_gpio, 0);
		msleep(5);
		gpio_direction_output(tas_priv->reset_gpio, 1);
		msleep(2);
	}
	dev_err(tas_priv->dev, "gpio up !!\n");

	tas_priv->cur_book = -1;
	tas_priv->cur_page = -1;
	tas_priv->err_code = 0;
}

void tas2562_enable_irq(struct tas2562_priv *tas_priv, bool enable)
{
	if (enable) {
		if (tas_priv->irq_enabled)
			return;

		if (gpio_is_valid(tas_priv->irq_gpio)){
			enable_irq(tas_priv->irq);
			dev_info(tas_priv->dev, "%s, Enable irq\n", __func__);
		}
		tas_priv->irq_enabled = true;
	} else {
		if (gpio_is_valid(tas_priv->irq_gpio)){
			disable_irq_nosync(tas_priv->irq);
			dev_info(tas_priv->dev, "%s, Disable irq\n", __func__);
		}
		tas_priv->irq_enabled = false;
	}
}

static void irq_work_routine(struct work_struct *work)
{
	struct tas2562_priv *tas_priv =
		container_of(work, struct tas2562_priv, irq_work.work);
	unsigned int intr_status[5] = { 0 };
	int counter = 2;
	int result = 0;
	int irq_reg;

	dev_info(tas_priv->dev, "%s\n", __func__);
#ifdef CONFIG_TAS2562_CODEC
	mutex_lock(&tas_priv->codec_lock);
#endif
	tas2562_enable_irq(tas_priv, false);
	result = gpio_get_value(tas_priv->irq_gpio);
	dev_info(tas_priv->dev, "%s, irq GPIO state: %d\n", __func__, result);

	if (tas_priv->runtime_suspended) {
		dev_info(tas_priv->dev, "%s, Runtime Suspended\n", __func__);
		goto end;
	}

	if (tas_priv->power_state == TAS2562_POWER_SHUTDOWN) {
		dev_info(tas_priv->dev, "%s, device not powered\n", __func__);
		goto end;
	}

	dev_info(tas_priv->dev, "err_code: 0x%x\n", tas_priv->err_code);
	if(tas_priv->err_code & ERROR_DEVA_I2C_COMM)
		goto reload;

	result = tas2562_write(tas_priv, TAS2562_InterruptMaskReg0,
				TAS2562_InterruptMaskReg0_Disable);
	if (result < 0)
		goto reload;
	result = tas2562_write(tas_priv, TAS2562_InterruptMaskReg1,
				TAS2562_InterruptMaskReg1_Disable);
	if (result < 0)
		goto reload;

	tas2562_read(tas_priv, TAS2562_TDMConfigurationReg4, &irq_reg);
	if(irq_reg != 0x01) {
		dev_info(tas_priv->dev, "TX reg is: %s %d, %d\n", __func__, irq_reg, __LINE__);
	}
	result = tas2562_read(tas_priv, TAS2562_LatchedInterruptReg0, &intr_status[0]);
	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_LatchedInterruptReg1, &intr_status[1]);
	else
		goto reload;
	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_LatchedInterruptReg2, &intr_status[2]);
	else
		goto reload;
	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_LatchedInterruptReg3, &intr_status[3]);
	else
		goto reload;
	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_LatchedInterruptReg4, &intr_status[4]);
	else
		goto reload;

	dev_info(tas_priv->dev, "IRQ status : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
			intr_status[0], intr_status[1], intr_status[2], intr_status[3], intr_status[4]);

	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_VBATMSB, &intr_status[2]);
	else
		goto reload;
	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_VBATLSB, &intr_status[3]);
	else
		goto reload;
	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_TEMP, &intr_status[4]);
	else
		goto reload;
	dev_dbg(tas_priv->dev, "VBAT status : 0x%x, 0x%x, temperature: 0x%x\n",
			intr_status[2], intr_status[3], intr_status[4]);

	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_LimiterConfigurationReg0, &intr_status[2]);
	else
		goto reload;
	if (result >= 0)
		result = tas2562_read(tas_priv, TAS2562_LimiterConfigurationReg0, &intr_status[3]);
	else
		goto reload;
	dev_dbg(tas_priv->dev, " Thermal foldback : 0x%x, limiter status: 0x%x\n",
			intr_status[2], intr_status[3]);

	if (((intr_status[0] & 0x7) != 0) || ((intr_status[1] & 0x0f) != 0) ||
			(gpio_get_value(tas_priv->irq_gpio) == 0)) {
		/* in case of any IRQ, INT_OC, INT_OT, INT_OVLT, INT_UVLT, INT_BO */

		if (intr_status[0] & TAS2562_LatchedInterruptReg0_TDMClockErrorSticky_Mask) {
			tas_priv->err_code |= ERROR_DTMCLK_ERROR;
			dev_err(tas_priv->dev, "TDM clk error!\n");
		} else
			tas_priv->err_code &= ~ERROR_DTMCLK_ERROR;

		if (intr_status[0] & TAS2562_LatchedInterruptReg0_OCEFlagSticky_Interrupt) {
			tas_priv->err_code |= ERROR_OVER_CURRENT;
			dev_err(tas_priv->dev, "SPK over current!\n");
		} else
			tas_priv->err_code &= ~ERROR_OVER_CURRENT;

		if (intr_status[0] & TAS2562_LatchedInterruptReg0_OTEFlagSticky_Interrupt) {
			tas_priv->err_code |= ERROR_DIE_OVERTEMP;
			dev_err(tas_priv->dev, "die over temperature!\n");
		} else
			tas_priv->err_code &= ~ERROR_DIE_OVERTEMP;

		if (intr_status[1] & TAS2562_LatchedInterruptReg1_VBATOVLOSticky_Interrupt) {
			tas_priv->err_code |= ERROR_OVER_VOLTAGE;
			dev_err(tas_priv->dev, "SPK over voltage!\n");
		} else
			tas_priv->err_code &= ~ERROR_UNDER_VOLTAGE;

		if (intr_status[1] & TAS2562_LatchedInterruptReg1_VBATUVLOSticky_Interrupt) {
			tas_priv->err_code |= ERROR_UNDER_VOLTAGE;
			dev_err(tas_priv->dev, "SPK under voltage!\n");
		} else
			tas_priv->err_code &= ~ERROR_UNDER_VOLTAGE;

		if (intr_status[1] & TAS2562_LatchedInterruptReg1_BrownOutFlagSticky_Interrupt) {
			tas_priv->err_code |= ERROR_BROWNOUT;
			dev_err(tas_priv->dev, "brownout!\n");
		} else
			tas_priv->err_code &= ~ERROR_BROWNOUT;

		dev_err(tas_priv->dev, "before goto reload\n");
		goto reload;
	} else {
		counter = 2;

		while (counter > 0) {
			result = tas2562_read(tas_priv, TAS2562_PowerControl, &intr_status[0]);
			if (result < 0)
				goto reload;

			if ((intr_status[0] & TAS2562_PowerControl_OperationalMode10_Mask)
				!= TAS2562_PowerControl_OperationalMode10_Shutdown)
				break;

			tas2562_read(tas_priv, TAS2562_LatchedInterruptReg0, &irq_reg);
			dev_info(tas_priv->dev, "IRQ reg is: %s %d, %d\n", __func__, irq_reg, __LINE__);

			result = tas2562_update_bits(tas_priv, TAS2562_PowerControl,
				TAS2562_PowerControl_OperationalMode10_Mask,
				TAS2562_PowerControl_OperationalMode10_Active);
			if (result < 0)
				goto reload;

			dev_info(tas_priv->dev, "set ICN to -80dB\n");
			result = tas2562_bulk_write(tas_priv, TAS2562_ICN_REG, pICN, 4);
			if (result < 0)
				goto reload;

			tas2562_read(tas_priv, TAS2562_LatchedInterruptReg0, &irq_reg);
			dev_info(tas_priv->dev, "IRQ reg is: %s, %d, %d\n", __func__, irq_reg, __LINE__);

			counter--;
			if (counter > 0) {
				/* in case check pow status just after power on TAS2562 */
				dev_dbg(tas_priv->dev, "PowSts B: 0x%x, check again after 10ms\n",
					intr_status[0]);
				msleep(10);
			}
		}

		if ((intr_status[0] & TAS2562_PowerControl_OperationalMode10_Mask)
			== TAS2562_PowerControl_OperationalMode10_Shutdown) {
			dev_err(tas_priv->dev, "%s, Critical ERROR REG[0x%x] = 0x%x\n",
				__func__,
				TAS2562_PowerControl,
				intr_status[0]);
			tas_priv->err_code |= ERROR_CLASSD_PWR;
			goto reload;
		}
		tas_priv->err_code &= ~ERROR_CLASSD_PWR;
	}

	result = tas2562_write(tas_priv, TAS2562_InterruptMaskReg0, 0xf8);
	if (result < 0)
		goto reload;

	result = tas2562_write(tas_priv, TAS2562_InterruptMaskReg1, 0xb1);
	if (result < 0)
		goto reload;

	goto end;

reload:
	/* hardware reset and reload */
	tas2562_load_config(tas_priv);

end:
	tas2562_enable_irq(tas_priv, true);

#ifdef CONFIG_TAS2562_CODEC
	mutex_unlock(&tas_priv->codec_lock);
#endif
}

static void init_work_routine(struct work_struct *work)
{
	struct tas2562_priv *tas_priv =
		container_of(work, struct tas2562_priv, init_work.work);
	int result = 0;
	//int irq_reg;
	//dev_info(tas_priv->dev, "%s\n", __func__);
#ifdef CONFIG_TAS2562_CODEC
	mutex_lock(&tas_priv->codec_lock);
#endif

	tas2562_update_bits(tas_priv, TAS2562_PowerControl,
		TAS2562_PowerControl_OperationalMode10_Mask,
		TAS2562_PowerControl_OperationalMode10_Active);

	//dev_info(tas_priv->dev, "set ICN to -80dB\n");
	result = tas2562_bulk_write(tas_priv, TAS2562_ICN_REG, pICN, 4);

	result = gpio_get_value(tas_priv->irq_gpio);
	//dev_info(tas_priv->dev, "%s, irq GPIO state: %d\n", __func__, result);

#ifdef CONFIG_TAS2562_CODEC
	mutex_unlock(&tas_priv->codec_lock);
#endif
}


static irqreturn_t tas2562_irq_handler(int irq, void *dev_id)
{
	struct tas2562_priv *tas_priv = (struct tas2562_priv *)dev_id;
	/* get IRQ status after 100 ms */
	schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

int tas2562_runtime_suspend(struct tas2562_priv *tas_priv)
{
	dev_dbg(tas_priv->dev, "%s\n", __func__);

	tas_priv->runtime_suspended = true;

	if (delayed_work_pending(&tas_priv->irq_work)) {
		dev_dbg(tas_priv->dev, "cancel IRQ work\n");
		cancel_delayed_work_sync(&tas_priv->irq_work);
	}

	return 0;
}

int tas2562_runtime_resume(struct tas2562_priv *tas_priv)
{
	dev_dbg(tas_priv->dev, "%s\n", __func__);

	tas_priv->runtime_suspended = false;

	return 0;
}

static int tas2562_parse_dt(struct device *dev, struct tas2562_priv *tas_priv)
{
	struct device_node *np = dev->of_node;
	int rc = 0, ret = 0;

//	u32 debounceInfo[2] = { 0, 0 };
	rc = of_property_read_u32(np, "ti,asi-format", &tas_priv->asi_format);
	if (rc) {
		dev_err(tas_priv->dev, "Looking up %s property in node %s failed %d\n",
			"ti,asi-format", np->full_name, rc);
	} else {
		dev_dbg(tas_priv->dev, "ti,asi-format=%d",
			tas_priv->asi_format);
	}

	tas_priv->reset_gpio = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (!gpio_is_valid(tas_priv->reset_gpio)) {
		dev_err(tas_priv->dev, "Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, tas_priv->reset_gpio);
	} else {
		dev_dbg(tas_priv->dev, "ti,reset-gpio=%d",
			tas_priv->reset_gpio);
	}

	tas_priv->irq_gpio = of_get_named_gpio(np, "ti,irq-gpio", 0);
	if (!gpio_is_valid(tas_priv->irq_gpio)) {
		dev_err(tas_priv->dev, "Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio", np->full_name, tas_priv->irq_gpio);
	} else {
		dev_dbg(tas_priv->dev, "ti,irq-gpio=%d", tas_priv->irq_gpio);
	}

	return ret;
}

static int tas2562_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tas2562_priv *tas_priv;
	int result;

	dev_err(&client->dev, "Driver ID: %s\n", TAS2562_DRIVER_ID);
	dev_info(&client->dev, "%s enter\n", __func__);

	tas_priv = devm_kzalloc(&client->dev,
		sizeof(struct tas2562_priv), GFP_KERNEL);
	if (tas_priv == NULL) {
		dev_err(&client->dev, "failed to get i2c device\n");
		result = -ENOMEM;
		goto err;
	}

	tas_priv->dev = &client->dev;
	i2c_set_clientdata(client, tas_priv);
	dev_set_drvdata(&client->dev, tas_priv);

	tas_priv->regmap = devm_regmap_init_i2c(client, &tas2562_i2c_regmap);
	if (IS_ERR(tas_priv->regmap)) {
		result = PTR_ERR(tas_priv->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
					result);
		goto err;
	}

	if (client->dev.of_node)
		tas2562_parse_dt(&client->dev, tas_priv);

	if (gpio_is_valid(tas_priv->reset_gpio)) {
		result = gpio_request(tas_priv->reset_gpio, "TAS2562_RESET");
		if (result) {
			dev_err(tas_priv->dev, "%s: Failed to request gpio %d\n",
				__func__, tas_priv->reset_gpio);
			result = -EINVAL;
			goto err;
		}
		tas2562_hw_reset(tas_priv);
	}

	tas_priv->power_state = TAS2562_POWER_SHUTDOWN;

	mutex_init(&tas_priv->dev_lock);

	/* Reset the chip */
	result = tas2562_write(tas_priv, TAS2562_SoftwareReset, 0x01);
	if (result < 0) {
		dev_err(&client->dev, "I2c fail, %d\n", result);
		goto err;
	}

	if (gpio_is_valid(tas_priv->irq_gpio)) {
		result = gpio_request(tas_priv->irq_gpio, "TAS2562-IRQ");
		if (result < 0) {
			dev_err(tas_priv->dev, "%s: GPIO %d request error\n",
				__func__, tas_priv->irq_gpio);
			goto err;
		}
		tas2562_write(tas_priv, TAS2562_MiscConfigurationReg0, 0xcf);
		gpio_direction_input(tas_priv->irq_gpio);
		result = gpio_get_value(tas_priv->irq_gpio);
		dev_info(tas_priv->dev, "irq GPIO state: %d\n", result);

		tas_priv->irq = gpio_to_irq(tas_priv->irq_gpio);
		dev_dbg(tas_priv->dev, "irq = %d\n", tas_priv->irq);
		INIT_DELAYED_WORK(&tas_priv->irq_work, irq_work_routine);
		result = request_threaded_irq(tas_priv->irq, tas2562_irq_handler,
				NULL, IRQF_TRIGGER_FALLING,
				client->name, tas_priv);
		if (result < 0) {
			dev_err(tas_priv->dev,
				"request_irq failed, %d\n", result);
			goto err;
		}
		tas2562_enable_irq(tas_priv, true);
	}
	INIT_DELAYED_WORK(&tas_priv->init_work, init_work_routine);

#ifdef CONFIG_TAS2562_CODEC
	mutex_init(&tas_priv->codec_lock);
	result = tas2562_register_codec(tas_priv);
	if (result < 0) {
		dev_err(tas_priv->dev,
			"register codec failed, %d\n", result);
		goto err;
	}
#endif
err:
	return result;
}

static int tas2562_i2c_remove(struct i2c_client *client)
{
	struct tas2562_priv *tas_priv = i2c_get_clientdata(client);

	dev_info(tas_priv->dev, "%s\n", __func__);

#ifdef CONFIG_TAS2562_CODEC
	tas2562_deregister_codec(tas_priv);
	mutex_destroy(&tas_priv->codec_lock);
#endif

	if (gpio_is_valid(tas_priv->reset_gpio))
		gpio_free(tas_priv->reset_gpio);
	if (gpio_is_valid(tas_priv->irq_gpio))
		gpio_free(tas_priv->irq_gpio);

	return 0;
}


static const struct i2c_device_id tas2562_i2c_id[] = {
	{ "tas2562", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2562_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2562_of_match[] = {
	{ .compatible = "ti,tas2562" },
	{},
};
MODULE_DEVICE_TABLE(of, tas2562_of_match);
#endif


static struct i2c_driver tas2562_i2c_driver = {
	.driver = {
		.name   = "tas2562",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(tas2562_of_match),
#endif
	},
	.probe      = tas2562_i2c_probe,
	.remove     = tas2562_i2c_remove,
	.id_table   = tas2562_i2c_id,
};

module_i2c_driver(tas2562_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2562 I2C Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif
