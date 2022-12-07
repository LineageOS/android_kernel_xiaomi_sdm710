/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
**
** File:
**     tas2562-codec.c
**
** Description:
**     ALSA SoC driver for Texas Instruments TAS2562 High Performance 4W Smart
**     Amplifier
**
** =============================================================================
*/

#ifdef CONFIG_TAS2562_CODEC
#include <linux/module.h>
#include <linux/moduleparam.h>
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas2562.h"

#define TAS2562_MDELAY 0xFFFFFFFE
#define TAS2562_MSLEEP 0xFFFFFFFD

static char const *iv_enable_text[] = {"Off", "On"};
static int tas2562iv_enable = 1;
static int muted;
static const struct soc_enum tas2562_enum[] = {
    SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(iv_enable_text), iv_enable_text),
};
static int tas2562_set_fmt(struct tas2562_priv *tas_priv, unsigned int fmt);

static int tas2562_i2c_load_data(struct tas2562_priv *tas_priv, unsigned int *buf);
static int tas2562_mute_ctrl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
static int tas2562_mute_ctrl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
static int tas2562_load_init(struct tas2562_priv *tas_priv);
static unsigned int p_tas2562_classH_D_data[] = {
		/* reg address			size	values */
	TAS2562_ClassHHeadroom, 0x4, 0x09, 0x99, 0x99, 0x9a,
	TAS2562_ClassHHysteresis, 0x4, 0x0, 0x0, 0x0, 0x0,
	TAS2562_ClassHMtct, 0x4, 0xb, 0x0, 0x0, 0x0,
	TAS2562_VBatFilter, 0x1, 0x38,
	TAS2562_ClassHReleaseTimer, 0x1, 0x3c,
	TAS2562_BoostSlope, 0x1, 0x78,
	TAS2562_TestPageConfiguration, 0x1, 0xd,
	TAS2562_ClassDConfiguration3, 0x1, 0x8e,
	TAS2562_ClassDConfiguration2, 0x1, 0x49,
	TAS2562_ClassDConfiguration4, 0x1, 0x21,
	TAS2562_ClassDConfiguration1, 0x1, 0x80,
	TAS2562_EfficiencyConfiguration, 0x1, 0xc1,
	0xFFFFFFFF, 0xFFFFFFFF
};


static unsigned int tas2562_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int result = 0;
	unsigned int value = 0;

	result = tas2562_read(tas_priv, reg, &value);

	if (result < 0)
		dev_err(tas_priv->dev, "%s, ERROR, reg=0x%x, E=%d\n",
			__func__, reg, result);
	else
		dev_dbg(tas_priv->dev, "%s, reg: 0x%x, value: 0x%x\n",
				__func__, reg, value);

	if (result >= 0)
		return value;
	else
		return result;
}

static int tas2562_iv_enable(struct tas2562_priv *tas_priv, int enable)
{
	int result;

	if (enable) {
		pr_debug("%s: tas2562iv_enable \n", __func__);
		result = tas2562_update_bits(tas_priv, TAS2562_PowerControl,
		    TAS2562_PowerControl_ISNSPower_Mask |
		    TAS2562_PowerControl_VSNSPower_Mask,
		    TAS2562_PowerControl_VSNSPower_Active |
		    TAS2562_PowerControl_ISNSPower_Active);
	} else {
		pr_debug("%s: tas2562iv_disable \n", __func__);
		result = tas2562_update_bits(tas_priv, TAS2562_PowerControl,
			TAS2562_PowerControl_ISNSPower_Mask |
			TAS2562_PowerControl_VSNSPower_Mask,
			TAS2562_PowerControl_VSNSPower_PoweredDown |
			TAS2562_PowerControl_ISNSPower_PoweredDown);
	}
	tas2562iv_enable = enable;

	return result;
}

static int tas2562iv_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int iv_enable = 0, result = 0;

    if (codec == NULL) {
		pr_err("%s: codec is NULL \n",  __func__);
		return 0;
    }

    iv_enable = ucontrol->value.integer.value[0];

	result = tas2562_iv_enable(tas_priv, iv_enable);

	pr_debug("%s: tas2562iv_enable = %d\n", __func__, tas2562iv_enable);

	return result;
}

static int tas2562iv_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0] = tas2562iv_enable;
   return 0;
}

static const struct snd_kcontrol_new tas2562_controls[] = {
SOC_ENUM_EXT("TAS2562 IVSENSE ENABLE", tas2562_enum[0],
		    tas2562iv_get, tas2562iv_put),
};

static int tas2562_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	int result = 0;

	result = tas2562_write(tas_priv, reg, value);
	if (result < 0) {
		dev_err(tas_priv->dev, "%s, ERROR, reg=0x%x, E=%d\n",
			__func__, reg, result);
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(2));
	}
	else
		dev_dbg(tas_priv->dev, "%s, reg: 0x%x, 0x%x\n",
			__func__, reg, value);

	return result;

}
static int tas2562_i2c_load_data(struct tas2562_priv *tas_priv, unsigned int *data)
{
	unsigned int reg;
	unsigned int *ptr;
	unsigned char buf[128];
	unsigned int len = 0;
	unsigned int i = 0;
	unsigned int size = 0;
	int result = 0;
	do {
		reg = data[len];
		size = data[len + 1];
		ptr = &data[len + 2];
		if (reg == TAS2562_MSLEEP) {
			msleep(ptr[0]);
			dev_dbg(tas_priv->dev, "%s, msleep = %d\n",
				__func__, ptr[0]);
		} else if (reg == TAS2562_MDELAY) {
			mdelay(ptr[0]);
			dev_dbg(tas_priv->dev, "%s, mdelay = %d\n",
				__func__, ptr[0]);
		} else {
			if (reg != 0xFFFFFFFF) {
				if (size > 128) {
					dev_err(tas_priv->dev,
						"%s, Line=%d, invalid size, maximum is 128 bytes!\n",
						__func__, __LINE__);
					break;
				}
				if (size > 1) {
					for (i = 0; i < size; i++)
						buf[i] = (unsigned char)ptr[i];
					result = tas2562_bulk_write(tas_priv, reg, buf, size);
					if (result < 0)
						break;
				} else if (size == 1) {
					result = tas2562_write(tas_priv, reg, ptr[0]);
					if (result < 0)
						break;
				} else {
					dev_err(tas_priv->dev,
						"%s, Line=%d,invalid size, minimum is 1 bytes!\n",
						__func__, __LINE__);
				}
			}
		}
		len = len + 2 + data[len + 1];
	} while (reg != 0xFFFFFFFF);

	if(result < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(2));
	return result;
}
static int tas2562_codec_suspend(struct snd_soc_codec *codec)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&tas_priv->codec_lock);

	dev_dbg(tas_priv->dev, "%s\n", __func__);
	tas2562_runtime_suspend(tas_priv);

	mutex_unlock(&tas_priv->codec_lock);
	return ret;
}

static int tas2562_codec_resume(struct snd_soc_codec *codec)
{
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	mutex_lock(&tas_priv->codec_lock);

	dev_dbg(tas_priv->dev, "%s\n", __func__);
	tas2562_runtime_resume(tas_priv);

	mutex_unlock(&tas_priv->codec_lock);
	return ret;
}

static const struct snd_kcontrol_new tas2562_asi_controls[] = {
	SOC_DAPM_SINGLE("Left", TAS2562_TDMConfigurationReg2,
		4, 1, 0),
	SOC_DAPM_SINGLE("Right", TAS2562_TDMConfigurationReg2,
		4, 2, 0),
	SOC_DAPM_SINGLE("LeftRightDiv2", TAS2562_TDMConfigurationReg2,
		4, 3, 0),
};

static int tas2562_set_power_state(struct tas2562_priv *tas_priv, int state)
{
	int result = 0;
	/*unsigned int nValue;*/
	int irq_reg;

	if ((tas_priv->muted) && (state == TAS2562_POWER_ACTIVE))
		state = TAS2562_POWER_MUTE;
	dev_err(tas_priv->dev, "set power state: %d\n", state);

	switch (state) {
	case TAS2562_POWER_ACTIVE:
		result = tas2562_load_init(tas_priv);
		if (result < 0)
			return result;
        //if set format was not called by asoc, then set it default
		if(tas_priv->asi_format == 0)
			tas_priv->asi_format = SND_SOC_DAIFMT_CBS_CFS
				| SND_SOC_DAIFMT_IB_NF
				| SND_SOC_DAIFMT_I2S;

		result = tas2562_set_fmt(tas_priv, tas_priv->asi_format);
		if (result < 0)
			goto activer_end;

//Clear latched IRQ before power on

		result = tas2562_update_bits(tas_priv, TAS2562_InterruptConfiguration,
					TAS2562_InterruptConfiguration_LTCHINTClear_Mask,
					TAS2562_InterruptConfiguration_LTCHINTClear);
		if (result < 0)
			goto activer_end;

		result = tas2562_read(tas_priv, TAS2562_LatchedInterruptReg0, &irq_reg);
		if (result < 0)
			goto activer_end;
		dev_info(tas_priv->dev, "IRQ reg is: %s %d, %d\n", __func__, irq_reg, __LINE__);

activer_end:
		tas_priv->power_up = true;
		tas_priv->power_state = TAS2562_POWER_ACTIVE;
/* irq routine will handle the error, and power on */
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
		break;

	case TAS2562_POWER_MUTE:
		result = tas2562_update_bits(tas_priv, TAS2562_PowerControl,
			TAS2562_PowerControl_OperationalMode10_Mask |
			TAS2562_PowerControl_ISNSPower_Mask |
			TAS2562_PowerControl_VSNSPower_Mask,
			TAS2562_PowerControl_OperationalMode10_Mute |
			TAS2562_PowerControl_VSNSPower_Active |
			TAS2562_PowerControl_ISNSPower_Active);
			tas_priv->power_up = true;
			tas_priv->power_state = TAS2562_POWER_MUTE;
		break;

	case TAS2562_POWER_SHUTDOWN:
		result = tas2562_update_bits(tas_priv, TAS2562_PowerControl,
			TAS2562_PowerControl_OperationalMode10_Mask,
			TAS2562_PowerControl_OperationalMode10_Shutdown);
			tas_priv->power_up = false;
			tas_priv->power_state = TAS2562_POWER_SHUTDOWN;
		msleep(20);

		break;

	default:
		dev_err(tas_priv->dev, "wrong power state setting %d\n", state);

	}

	if(result < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
	return result;
}

static const struct snd_soc_dapm_widget tas2562_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("Voltage Sense", "ASI1 Capture",  1, TAS2562_PowerControl, 2, 1),
	SND_SOC_DAPM_AIF_OUT("Current Sense", "ASI1 Capture",  0, TAS2562_PowerControl, 3, 1),
	SND_SOC_DAPM_MIXER("ASI1 Sel",
		TAS2562_TDMConfigurationReg2, 4, 0,
		&tas2562_asi_controls[0],
		ARRAY_SIZE(tas2562_asi_controls)),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_SIGGEN("VMON"),
	SND_SOC_DAPM_SIGGEN("IMON")
};

static const struct snd_soc_dapm_route tas2562_audio_map[] = {
	{"ASI1 Sel", "Left", "ASI1"},
	{"ASI1 Sel", "Right", "ASI1"},
	{"ASI1 Sel", "LeftRightDiv2", "ASI1"},
	{"DAC", NULL, "ASI1 Sel"},
	{"OUT", NULL, "DAC"},
	/*{"VMON", NULL, "Voltage Sense"},
	{"IMON", NULL, "Current Sense"},*/
	{"Voltage Sense", NULL, "VMON"},
	{"Current Sense", NULL, "IMON"},
};


static int tas2562_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	dev_dbg(tas_priv->dev, "%s, %d \n", __func__, mute);

	mutex_lock(&tas_priv->codec_lock);
	if (mute) {
		tas2562_set_power_state(tas_priv, TAS2562_POWER_SHUTDOWN);
	} else {
		tas2562_set_power_state(tas_priv, TAS2562_POWER_ACTIVE);
	}
	mutex_unlock(&tas_priv->codec_lock);
	return 0;
}

static int tas2562_slot_config(struct snd_soc_codec *codec, struct tas2562_priv *tas_priv, int blr_clk_ratio)
{
	int ret = 0;
	if(tas_priv->slot_width == 16)
		ret = tas2562_update_bits(tas_priv,
			TAS2562_TDMConfigurationReg5, 0xff, 0x42);
	else
		ret = tas2562_update_bits(tas_priv,
			TAS2562_TDMConfigurationReg5, 0xff, 0x44);
	if(ret < 0)
		goto end;

	tas2562_update_bits(tas_priv,
			TAS2562_TDMConfigurationReg6, 0xff, 0x40);

end:
	return ret;
}

static int tas2562_set_slot(struct snd_soc_codec *codec, int slot_width)
{
	int ret = 0;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	switch (slot_width) {
	case 16:
	ret = tas2562_update_bits(tas_priv,
		TAS2562_TDMConfigurationReg2,
		TAS2562_TDMConfigurationReg2_RXSLEN10_Mask,
		TAS2562_TDMConfigurationReg2_RXSLEN10_16Bits);
	break;

	case 24:
	ret = tas2562_update_bits(tas_priv,
		TAS2562_TDMConfigurationReg2,
		TAS2562_TDMConfigurationReg2_RXSLEN10_Mask,
		TAS2562_TDMConfigurationReg2_RXSLEN10_24Bits);
	break;

	case 32:
	ret = tas2562_update_bits(tas_priv,
		TAS2562_TDMConfigurationReg2,
		TAS2562_TDMConfigurationReg2_RXSLEN10_Mask,
		TAS2562_TDMConfigurationReg2_RXSLEN10_32Bits);
	break;

	case 0:
	/* Do not change slot width */
	break;

	default:
		dev_err(tas_priv->dev, "slot width not supported");
		ret = -EINVAL;
	}

	if (ret >= 0)
		tas_priv->slot_width = slot_width;

	return ret;
}

static int tas2562_set_bitwidth(struct tas2562_priv *tas_priv, int bitwidth)
{
	int slot_width_tmp = 16;
	int ret = 0;
	dev_info(tas_priv->dev, "%s %d\n", __func__, bitwidth);

	switch (bitwidth) {
	case SNDRV_PCM_FORMAT_S16_LE:
		ret = tas2562_update_bits(tas_priv,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXWLEN32_Mask,
			TAS2562_TDMConfigurationReg2_RXWLEN32_16Bits);
			tas_priv->channel_width = 16;
				slot_width_tmp = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ret = tas2562_update_bits(tas_priv,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXWLEN32_Mask,
			TAS2562_TDMConfigurationReg2_RXWLEN32_24Bits);
			tas_priv->channel_width = 24;
				slot_width_tmp = 32;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		ret = tas2562_update_bits(tas_priv,
			TAS2562_TDMConfigurationReg2,
			TAS2562_TDMConfigurationReg2_RXWLEN32_Mask,
			TAS2562_TDMConfigurationReg2_RXWLEN32_32Bits);
			tas_priv->channel_width = 32;
				slot_width_tmp = 32;
		break;

	default:
		dev_info(tas_priv->dev, "Not supported params format\n");
	}

	/* If machine driver did not call set slot width */
	//if (tas_priv->slot_width == 0)
	if (ret < 0)
		goto end;
	ret = tas2562_set_slot(tas_priv->codec, slot_width_tmp);

end:
	dev_info(tas_priv->dev, "channel_width: %d,  slot_width_tmp: %d\n", tas_priv->channel_width, slot_width_tmp);
	tas_priv->pcm_format = bitwidth;

	return ret;
}

static int tas2562_set_samplerate(struct tas2562_priv *tas_priv, int samplerate)
{
	int ret = 0;
	switch (samplerate) {
	case 48000:
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
		if(ret < 0)
			goto end;
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_44_1_48kHz);
		if(ret < 0)
			goto end;
		break;
	case 44100:
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz);
		if(ret < 0)
			goto end;
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_44_1_48kHz);
		if(ret < 0)
			goto end;
		break;
	case 96000:
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
		if(ret < 0)
			goto end;
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_88_2_96kHz);
		if(ret < 0)
			goto end;
		break;
	case 88200:
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz);
		if(ret < 0)
			goto end;
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_88_2_96kHz);
		if(ret < 0)
			goto end;
		break;
	case 19200:
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_48KHz);
		if(ret < 0)
			goto end;
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_176_4_192kHz);
		if(ret < 0)
			goto end;
		break;
	case 17640:
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATERAMP_44_1KHz);
		if(ret < 0)
			goto end;
		ret = tas2562_update_bits(tas_priv,
				TAS2562_TDMConfigurationReg0,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_Mask,
				TAS2562_TDMConfigurationReg0_SAMPRATE31_176_4_192kHz);
		if(ret < 0)
			goto end;
		break;
	default:
			dev_info(tas_priv->dev, "%s, unsupported sample rate, %d\n", __func__, samplerate);

	}

end:
	tas_priv->sample_rate = samplerate;
	return ret;
}

static int tas2562_mute_ctrl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = tas_priv->muted;
	dev_dbg(tas_priv->dev, "tas2562_mute_ctrl_get = %d\n",
		tas_priv->muted);

	return 0;
}

static int tas2562_mute_ctrl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	muted = ucontrol->value.integer.value[0];

	dev_dbg(tas_priv->dev, "tas2562_mute_ctrl_put = %d\n", muted);

	tas_priv->muted = !!muted;

	return 0;
}

static int tas2562_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int blr_clk_ratio;
	int ret = 0;

	dev_dbg(tas_priv->dev, "%s, format: %d\n", __func__,
		params_format(params));

	mutex_lock(&tas_priv->codec_lock);

	ret = tas2562_set_bitwidth(tas_priv, params_format(params));
	if(ret < 0)
	{
		dev_info(tas_priv->dev, "set bitwidth failed, %d\n", ret);
		goto end;
	}

	blr_clk_ratio = params_channels(params) * tas_priv->channel_width;
	dev_info(tas_priv->dev, "blr_clk_ratio: %d\n", blr_clk_ratio);
	if(blr_clk_ratio != 0) {
		ret = tas2562_slot_config(tas_priv->codec, tas_priv, blr_clk_ratio);
		if(ret < 0)
			goto end;
	}

	dev_info(tas_priv->dev, "%s, sample rate: %d\n", __func__,
		params_rate(params));

	ret = tas2562_set_samplerate(tas_priv, params_rate(params));

end:
	mutex_unlock(&tas_priv->codec_lock);
	if(tas_priv->err_code & ERROR_DEVA_I2C_COMM)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
	return 0;
}

static int tas2562_set_fmt(struct tas2562_priv *tas_priv, unsigned int fmt)
{
	u8 tdm_rx_start_slot = 0, asi_cfg_1 = 0;
	int ret = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		asi_cfg_1 = 0x00;
		break;
	default:
		dev_err(tas_priv->dev, "ASI format master is not found\n");
		ret = -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		dev_info(tas_priv->dev, "INV format: NBNF\n");
		asi_cfg_1 |= TAS2562_TDMConfigurationReg1_RXEDGE_Rising;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dev_info(tas_priv->dev, "INV format: IBNF\n");
		asi_cfg_1 |= TAS2562_TDMConfigurationReg1_RXEDGE_Falling;
		break;
	default:
		dev_err(tas_priv->dev, "ASI format Inverse is not found\n");
		ret = -EINVAL;
	}

	ret = tas2562_update_bits(tas_priv, TAS2562_TDMConfigurationReg1,
		TAS2562_TDMConfigurationReg1_RXEDGE_Mask,
		asi_cfg_1);
	if(ret < 0)
		goto end;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case (SND_SOC_DAIFMT_I2S):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_DSP_A):
	case (SND_SOC_DAIFMT_DSP_B):
		tdm_rx_start_slot = 1;
		break;
	case (SND_SOC_DAIFMT_LEFT_J):
		tdm_rx_start_slot = 0;
		break;
	default:
		dev_err(tas_priv->dev, "DAI Format is not found, fmt=0x%x\n", fmt);
		ret = -EINVAL;
		break;
	}

	ret = tas2562_update_bits(tas_priv, TAS2562_TDMConfigurationReg1,
		TAS2562_TDMConfigurationReg1_RXOFFSET51_Mask,
		(tdm_rx_start_slot << TAS2562_TDMConfigurationReg1_RXOFFSET51_Shift));
	if(ret < 0)
		goto end;
	ret = tas2562_write(tas_priv, TAS2562_TDMConfigurationReg4, 0x01);
	if(ret < 0)
		goto end;

	tas_priv->asi_format = fmt;

end:
	return ret;
}

static int tas2562_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	dev_dbg(tas_priv->dev, "%s, format=0x%x\n", __func__, fmt);

	ret = tas2562_set_fmt(tas_priv, fmt);
	if(ret < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));
	return 0;
}

static int tas2562_set_dai_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	int ret = 0;
	struct snd_soc_codec *codec = dai->codec;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	dev_dbg(tas_priv->dev, "%s, tx_mask:%d, rx_mask:%d, slots:%d, slot_width:%d",
			__func__, tx_mask, rx_mask, slots, slot_width);

	ret = tas2562_set_slot(codec, slot_width);
	if(ret < 0)
		schedule_delayed_work(&tas_priv->irq_work, msecs_to_jiffies(10));

	return 0;
}

static struct snd_soc_dai_ops tas2562_dai_ops = {
	.digital_mute = tas2562_mute,
	.hw_params  = tas2562_hw_params,
	.set_fmt    = tas2562_set_dai_fmt,
	.set_tdm_slot = tas2562_set_dai_tdm_slot,
};

#define TAS2562_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define TAS2562_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 \
						SNDRV_PCM_RATE_88200 |\
						SNDRV_PCM_RATE_96000 |\
						SNDRV_PCM_RATE_176400 |\
						SNDRV_PCM_RATE_192000\
						)

static struct snd_soc_dai_driver tas2562_dai_driver[] = {
	{
		.name = "tas2562 ASI1",
		.id = 0,
		.playback = {
			.stream_name    = "ASI1 Playback",
			.channels_min   = 2,
			.channels_max   = 2,
			.rates      = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS2562_FORMATS,
		},
		.capture = {
			.stream_name    = "ASI1 Capture",
			.channels_min   = 0,
			.channels_max   = 2,
			.rates          = SNDRV_PCM_RATE_8000_192000,
			.formats    = TAS2562_FORMATS,
		},
		.ops = &tas2562_dai_ops,
		.symmetric_rates = 1,
	},
};

static int tas2562_load_init(struct tas2562_priv *tas_priv)
{
	int ret;

#ifdef TAS2558_CODEC
/* Max voltage to 9V */
	ret = tas2562_update_bits(tas_priv, TAS2562_BoostConfiguration2,
					TAS2562_BoostConfiguration2_BoostMaxVoltage_Mask,
					0x7);
	if(ret < 0)
		return ret;

        ret = tas2562_update_bits(tas_priv, TAS2562_PlaybackConfigurationReg0,
                                        TAS2562_PlaybackConfigurationReg0_AmplifierLevel51_Mask,
                                        0xd << 1);
        if(ret < 0)
                return ret;

#endif

	ret = tas2562_write(tas_priv, TAS2562_MiscConfigurationReg0, 0xcf);
	if(ret < 0)
		return ret;
	ret = tas2562_write(tas_priv, TAS2562_TDMConfigurationReg4, 0x01);
	if(ret < 0)
		return ret;
	ret = tas2562_write(tas_priv, TAS2562_ClockConfiguration, 0x0c);
	if(ret < 0)
		return ret;
	ret = tas2562_i2c_load_data(tas_priv, p_tas2562_classH_D_data);

	return ret;
}

static int tas2562_codec_probe(struct snd_soc_codec *codec)
{
	int ret;
	struct tas2562_priv *tas_priv = snd_soc_codec_get_drvdata(codec);

	ret = snd_soc_add_codec_controls(codec, tas2562_controls,
					 ARRAY_SIZE(tas2562_controls));
	if (ret < 0) {
		pr_err("%s: add_codec_controls failed, err %d\n",
			__func__, ret);
		return ret;
	}

	tas2562_load_init(tas_priv);
	tas2562_iv_enable(tas_priv, 1);
	tas_priv->codec = codec;

	tas2562_smartamp_add_controls(tas_priv);

	dev_err(tas_priv->dev, "%s\n", __func__);

	return 0;
}

static int tas2562_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static DECLARE_TLV_DB_SCALE(tas2562_digital_tlv, 1100, 50, 0);

static const struct snd_kcontrol_new tas2562_snd_controls[] = {
	SOC_SINGLE_TLV("Amp Output Level", TAS2562_PlaybackConfigurationReg0,
		0, 0x16, 0,
		tas2562_digital_tlv),
	SOC_SINGLE_EXT("SmartPA Mute", SND_SOC_NOPM, 0, 0x0001, 0,
			tas2562_mute_ctrl_get, tas2562_mute_ctrl_put),
};

static struct snd_soc_codec_driver soc_codec_driver_tas2562 = {
	.probe			= tas2562_codec_probe,
	.remove			= tas2562_codec_remove,
	.read			= tas2562_codec_read,
	.write			= tas2562_codec_write,
	.suspend		= tas2562_codec_suspend,
	.resume			= tas2562_codec_resume,
	.component_driver = {
	.controls		= tas2562_snd_controls,
	.num_controls		= ARRAY_SIZE(tas2562_snd_controls),
		.dapm_widgets		= tas2562_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(tas2562_dapm_widgets),
		.dapm_routes		= tas2562_audio_map,
		.num_dapm_routes	= ARRAY_SIZE(tas2562_audio_map),
	},
};

int tas2562_register_codec(struct tas2562_priv *tas_priv)
{
	int result = 0;

	dev_info(tas_priv->dev, "%s, enter\n", __func__);
	result = snd_soc_register_codec(tas_priv->dev,
		&soc_codec_driver_tas2562,
		tas2562_dai_driver, ARRAY_SIZE(tas2562_dai_driver));
	return result;
}

int tas2562_deregister_codec(struct tas2562_priv *tas_priv)
{
	snd_soc_unregister_codec(tas_priv->dev);

	return 0;
}

void tas2562_load_config(struct tas2562_priv *tas_priv)
{
	int ret = 0;

	tas2562_hw_reset(tas_priv);
	msleep(2);
	tas2562_write(tas_priv, TAS2562_SoftwareReset,
			TAS2562_SoftwareReset_SoftwareReset_Reset);
	msleep(3);

	ret = tas2562_slot_config(tas_priv->codec, tas_priv, 1);
	if(ret < 0) {
		goto end;
	}

	tas2562_load_init(tas_priv);
	tas2562_iv_enable(tas_priv, 1);

	ret = tas2562_set_slot(tas_priv->codec, tas_priv->slot_width);
	if (ret < 0)
		goto end;

	ret = tas2562_set_fmt(tas_priv, tas_priv->asi_format);
	if (ret < 0)
		goto end;

	ret = tas2562_set_bitwidth(tas_priv, tas_priv->pcm_format);
	if (ret < 0)
		goto end;

	ret = tas2562_set_samplerate(tas_priv, tas_priv->sample_rate);
	if (ret < 0)
		goto end;

	ret = tas2562_set_power_state(tas_priv, tas_priv->power_state);
	if (ret < 0)
		goto end;

end:
/* power up failed, restart later */
	if (ret < 0)
		schedule_delayed_work(&tas_priv->irq_work,
				msecs_to_jiffies(1000));
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2562 ALSA SOC Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
#endif /* CONFIG_TAS2562_CODEC */
