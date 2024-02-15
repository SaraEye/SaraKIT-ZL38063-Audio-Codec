// SPDX-License-Identifier: GPL-2.0-only
//
// Codec driver for Microsemi ZL38060 Connected Home Audio Processor.
//
// Copyright(c) 2020 Sven Van Asbroeck

// The ZL38060 is very flexible and configurable. This driver implements only a
// tiny subset of the chip's possible configurations:
//
// - DSP block bypassed: DAI        routed straight to DACs
//                       microphone routed straight to DAI
// - chip's internal clock is driven by a 12 MHz external crystal
// - chip's DAI connected to CPU is I2S, and bit + frame clock master
// - chip must be strapped for "host boot": in this mode, firmware will be
//   provided by this driver.
//
// increasing the capabilities of the codec, writing and reading any register
// adding volume controls 
// zl38063 SaraKIT.SaraAI.com project (Artur Majtczak)

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/module.h>
#include <linux/ihex.h>

#include <sound/pcm_params.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define DRV_NAME		"zl38060"

#define ZL38_RATES		(SNDRV_PCM_RATE_8000  |\
				SNDRV_PCM_RATE_16000 |\
				SNDRV_PCM_RATE_48000)
#define ZL38_FORMATS		SNDRV_PCM_FMTBIT_S16_LE

#define HBI_FIRMWARE_PAGE	0xFF
#define ZL38_MAX_RAW_XFER	0x100

#define REG_TDMA_CFG_CLK	0x0262
#define CFG_CLK_PCLK_SHIFT	4
#define CFG_CLK_PCLK_MASK	(0x7ff << CFG_CLK_PCLK_SHIFT)
#define CFG_CLK_PCLK(bits)	((bits - 1) << CFG_CLK_PCLK_SHIFT)
#define CFG_CLK_MASTER		BIT(15)
#define CFG_CLK_FSRATE_MASK	0x7
#define CFG_CLK_FSRATE_8KHZ	0x1
#define CFG_CLK_FSRATE_16KHZ	0x2
#define CFG_CLK_FSRATE_48KHZ	0x6

#define REG_CLK_CFG		0x0016
#define CLK_CFG_SOURCE_XTAL	BIT(15)

#define REG_CLK_STATUS		0x0014
#define CLK_STATUS_HWRST	BIT(0)

#define REG_PARAM_RESULT	0x0034
#define PARAM_RESULT_READY	0xD3D3

#define REG_PG255_BASE_HI	0x000C
#define REG_PG255_OFFS(addr)	((HBI_FIRMWARE_PAGE << 8) | (addr & 0xFF))
#define REG_FWR_EXEC		0x012C

#define REG_CMD			0x0032
#define REG_HW_REV		0x0020
#define REG_FW_PROD		0x0022
#define REG_FW_REV		0x0024

#define REG_SEMA_FLAGS		0x0006
#define SEMA_FLAGS_BOOT_CMD	BIT(0)
#define SEMA_FLAGS_APP_REBOOT	BIT(1)

#define REG_HW_REV		0x0020
#define REG_FW_PROD		0x0022
#define REG_FW_REV		0x0024
#define REG_GPIO_DIR		0x02DC
#define REG_GPIO_DAT		0x02DA

#define BOOTCMD_LOAD_COMPLETE	0x000D
#define BOOTCMD_FW_GO		0x0008

#define FIRMWARE_MAJOR		2
#define FIRMWARE_MINOR		2


/*------------------------------------------------------*/
/*TWOLF REGisters defintion */
#define ZL380xx_HOST_SW_FLAGS_REG                 0x0006
 #define ZL380xx_HOST_SW_FLAGS_HOST_CMD_SHIFT     0
 #define ZL380xx_HOST_SW_FLAGS_HOST_CMD           (1<<ZL380xx_HOST_SW_FLAGS_HOST_CMD_SHIFT)
 #define ZL380xx_HOST_SW_FLAGS_APP_REBOOT_SHIFT   1
 #define ZL380xx_HOST_SW_FLAGS_APP_REBOOT         (1<<ZL380xx_HOST_SW_FLAGS_APP_REBOOT_SHIFT)
 #define ZL380xx_HOST_SW_FLAGS_APP_HOST_CMD_SHIFT 2
 #define ZL380xx_HOST_SW_FLAGS_APP_HOST_CMD       (1<<ZL380xx_HOST_SW_FLAGS_APP_HOST_CMD_SHIFT)

#define ZL380xx_PAGE_255_CHKSUM_HI_REG            0x0008
#define ZL380xx_PAGE_255_CHKSUM_LO_REG            0x000A
#define ZL380xx_PAGE_255_BASE_HI_REG              0x000C
#define ZL380xx_PAGE_255_BASE_LO_REG              0x000E

#define ZL380xx_CLK_STATUS_REG                    0x014   /*Clock status register*/
 #define ZL380xx_CLK_STATUS_HWRST_SHIFT           0
 #define ZL380xx_CLK_STATUS_HWRST                 (1<<ZL380xx_CLK_STATUS_HWRST_SHIFT)
 #define ZL380xx_CLK_STATUS_RST_SHIFT             2
 #define ZL380xx_CLK_STATUS_RST                   (1<<ZL380xx_CLK_STATUS_RST_SHIFT)
 #define ZL380xx_CLK_STATUS_POR_SHIFT             3
 #define ZL380xx_CLK_STATUS_POR                   (1<<ZL380xx_CLK_STATUS_POR_SHIFT)
 
#define ZL380xx_FWR_COUNT_REG                     0x0026 /*Fwr on flash count register*/
#define ZL380xx_CUR_LOADED_FW_IMG_REG            0x0028
 #define ZL380xx_CUR_FW_APP_RUNNING               (1<< 15)
 #define ZL380xx_CUR_FW_IMG_NUM_SHIFT             0
 #define ZL380xx_CUR_FW_IMG_NUM_MASK              0xF
 
#define ZL380xx_SYSSTAT_REG                         0x066
#define ZL380xx_HOST_CMD_PARAM_RESULT_REG           0x0034 /*Host Command Param/Result register*/
#define HOST_FWR_EXEC_REG                           0x012C /*Fwr EXEC register*/
#define ZL380xx_HOST_CMD_REG                        0x0032 /*Host Command register*/
#define ZL380xx_CFG_REC_SIZE_REG                    0x01F0
#define ZL380xx_CFG_REC_CHKSUM_REG                  0x01F2
#define HOST_SW_FLAGS_CMD_NORST                     0x0004

#define HBI_CONFIG_IF_CMD           0xFD00
 #define HBI_CONFIG_ENDIAN_SHIFT     0
 #define HBI_CONFIG_IF_ENDIAN_LITTLE (1<<HBI_CONFIG_ENDIAN_SHIFT)
 #define HBI_CONFIG_INT_MODE_SHIFT    1
 #define HBI_CONFIG_INT_MODE_OD      (1<<HBI_CONFIG_INT_MODE_SHIFT)
 #define HBI_CONFIG_WAKE_SHIFT       7
 #define HBI_CONFIG_WAKE             (1<< HBI_CONFIG_WAKE_SHIFT)
 
#define ZL380xx_POWER_CFG_REG                        0x0206 
#define ZL380xx_POWER_I2C_SHIFT                      (1<<13)

/* I2S Port Config Regs */
#define ZL380xx_TDMA_CFG_REG                         0x0260
 #define ZL380xx_TDM_CFG_FS_ALIGN_SHIFT              0
 #define ZL380xx_TDM_CFG_FS_ALIGN_LEFT               (1<<ZL380xx_TDM_CFG_FS_ALIGN_SHIFT)
 #define ZL380xx_TDM_CFG_FS_POL_SHIFT                2
 #define ZL380xx_TDM_CFG_FS_POL_LOW                  (1<<ZL380xx_TDM_CFG_FS_POL_SHIFT)
 #define ZL380xx_TDM_CFG_ENCODE_SHIFT                15
 #define ZL380xx_TDM_CFG_ENCODE_I2S                  (1<<ZL380xx_TDM_CFG_ENCODE_SHIFT)
 #define ZL380xx_TDM_CFG_ENCODE_PCM                 ~(1<<ZL380xx_TDM_CFG_ENCODE_SHIFT)

#define ZL380xx_TDMA_CLK_CFG_REG                     0x262
 #define ZL380xx_TDM_CLK_FSRATE_SHIFT                 0
 #define ZL380xx_TDM_CLK_FSRATE_MASK                  0x0007
 #define ZL380xx_TDM_CLK_PCLKRATE_SHIFT               4
 #define ZL380xx_TDM_CLK_PCLKRATE_MASK                0x7FF0
 #define ZL380xx_TDM_CLK_MASTER_SHIFT                 15
 #define ZL380xx_TDM_CLK_MASTER_SHIFT                 15
 #define ZL380xx_TDM_CLK_MASTER                      (1<<ZL380xx_TDM_CLK_MASTER_SHIFT)
 #define ZL380xx_TDM_CLK_SLAVE                      ~(1<<ZL380xx_TDM_CLK_MASTER_SHIFT)

#define ZL380xx_TDMA_CHANNEL1_CFG_REG                0x268
#define ZL380xx_TDMA_CHANNEL2_CFG_REG                0x26A
#define ZL380xx_TDMB_CHANNEL1_CFG_REG                0x280
 #define ZL380xx_TDM_CHANNEL_CFG_ENCODE_SHIFT         8
#define ZL380xx_TDMB_CHANNEL2_CFG_REG                0x282

#define ZL380xx_TDMB_CFG_REG                         0x278
#define ZL380xx_TDMB_CLK_CFG_REG                     0x27A


#define ZL380xx_XROSS_PT_AUD_OUTPUT_PATH_EN_REG    0x0202
 #define ZL380xx_XROSS_PT_AUD_DAC1EN_SHIFT          0
 #define ZL380xx_XROSS_PT_AUD_DAC1EN                (1<<ZL380xx_XROSS_PT_AUD_DAC1EN_SHIFT)
 #define ZL380xx_XROSS_PT_AUD_DAC2EN_SHIFT          1
 #define ZL380xx_XROSS_PT_AUD_DAC2EN                (1<<ZL380xx_XROSS_PT_AUD_DAC2EN_SHIFT)
 #define ZL380xx_XROSS_PT_AUD_I2S1L_EN_SHIFT        2
 #define ZL380xx_XROSS_PT_AUD_I2S1L_EN              (1<<ZL380xx_XROSS_PT_AUD_I2S1L_EN_SHIFT)
 #define ZL380xx_XROSS_PT_AUD_I2S1R_EN_SHIFT        3
 #define ZL380xx_XROSS_PT_AUD_I2S1R_EN              (1<<ZL380xx_XROSS_PT_AUD_I2S1R_EN_SHIFT)
 #define ZL380xx_XROSS_PT_AUD_I2S2L_EN_SHIFT        6
 #define ZL380xx_XROSS_PT_AUD_I2S2L_EN              (1<<ZL380xx_XROSS_PT_AUD_I2S2L_EN_SHIFT)
 #define ZL380xx_XROSS_PT_AUD_I2S2R_EN_SHIFT        7
 #define ZL380xx_XROSS_PT_AUD_I2S2R_EN              (1<<ZL380xx_XROSS_PT_AUD_I2S2R_EN_SHIFT)
 #define ZL380xx_XROSS_PT_AUD_SIN_EN_SHIFT          10
 #define ZL380xx_XROSS_PT_AUD_SIN_EN                (1<<ZL380xx_XROSS_PT_AUD_SIN_EN_SHIFT)
 #define ZL380xx_XROSS_PT_AUD_RIN_EN_SHIFT          11
 #define ZL380xx_XROSS_PT_AUD_RIN_EN                (1<<ZL380xx_XROSS_PT_AUD_RIN_EN_SHIFT)

#define ZL380xx_CP_DAC1_SRC_REG                    0x210
 #define ZL380xx_CP_AUD_SRCA_SHIFT                  0
 #define ZL380xx_CP_AUD_SRCA_MASK                   (0xFF << ZL380xx_CP_AUD_SRCA_SHIFT)
 #define ZL380xx_CP_AUD_SRCB_SHIFT                  8
 #define ZL380xx_CP_AUD_SRCB_MASK                   (0xFF << ZL380xx_CP_AUD_SRCB_SHIFT)

#define ZL380xx_CP_DAC2_SRC_REG                    0x212
#define ZL380xx_CP_I2S1L_SRC_REG                   0x214
#define ZL380xx_CP_I2S1R_SRC_REG                   0x216
#define ZL380xx_CP_I2S2L_SRC_REG                   0x21C
#define ZL380xx_CP_I2S2R_SRC_REG                   0x21E
#define ZL380xx_CP_AECSIN_SRC_REG                  0x224
#define ZL380xx_CP_AECRIN_SRC_REG                  0x226
#define ZL380xx_CP_DAC1_GAIN_REG                   0x0238
#define ZL380xx_CP_DAC2_GAIN_REG                   0x023A
#define ZL380xx_CP_I2S1L_GAIN_REG                  0x023C
#define ZL380xx_CP_I2S1R_GAIN_REG                  0x023E
#define ZL380xx_CP_I2S2L_GAIN_REG                  0x0244
#define ZL380xx_CP_I2S2R_GAIN_REG                  0x0246

#define ZL380xx_CP_AEC_SIN_GAIN_REG                0x024C
#define ZL380xx_CP_AEC_RIN_GAIN_REG                0x024E
 #define ZL380xx_CP_GAINA_SHIFT                     0
 #define ZL380xx_CP_GAINA_MASK                      (0xFF << ZL380xx_CP_GAINA_SHIFT)
 #define ZL380xx_CP_GAINB_SHIFT                     8
 #define ZL380xx_CP_GAINB_MASK                      (0xFF << ZL380xx_CP_GAINB_SHIFT)



#define ZL380xx_MICEN_CFG_REG                     0x02B0
 #define ZL380xx_MICEN_MIC1_SHIFT                  0
 #define ZL380xx_MICEN_MIC1                        (1<<ZL380xx_MICEN_MIC1_SHIFT)
 #define ZL380xx_MICEN_MIC2_SHIFT                  1
 #define ZL380xx_MICEN_MIC2                        (1<<ZL380xx_MICEN_MIC2_SHIFT)
 #define ZL380xx_MICEN_MIC3_SHIFT                  2
 #define ZL380xx_MICEN_MIC3                        (1<<ZL380xx_MICEN_MIC3_SHIFT)

#define ZL380xx_DIG_MIC_GAIN_REG                  0x02B2

/* AEC module register bits */
#define ZL380xx_AEC_CTRL0_REG                       0x300
 #define ZL380xx_AEC_RST_SHIFT                       0
 #define ZL380xx_AEC_RST                             (1<<ZL380xx_AEC_RST_SHIFT)
 #define ZL380xx_AEC_MBYPASS_SHIFT                   1
 #define ZL380xx_AEC_MBYPASS                         (1<<ZL380xx_AEC_MBYPASS_SHIFT)
 #define ZL380xx_AEC_RIN_EQ_SHIFT                    2
 #define ZL380xx_AEC_RIN_EQ_DISABLE                  (1<<ZL380xx_AEC_RIN_EQ_SHIFT)
 #define ZL380xx_AEC_BYPASS_SHIFT                    4
 #define ZL380xx_AEC_BYPASS                          (1<<ZL380xx_AEC_BYPASS_SHIFT)
 #define ZL380xx_AEC_AUDENHBYPASS_SHIFT              5
 #define ZL380xx_AEC_AUDENHBYPASS                    (1<<ZL380xx_AEC_AUDENHBYPASS_SHIFT)
 #define ZL380xx_AEC_MUTE_ROUT_SHIFT                  7
 #define ZL380xx_AEC_MUTE_ROUT                       (1<<ZL380xx_AEC_MUTE_ROUT_SHIFT)
 #define ZL380xx_AEC_MUTE_SOUT_SHIFT                  8
 #define ZL380xx_AEC_MUTE_SOUT                       (1<<ZL380xx_AEC_MUTE_SOUT_SHIFT)
 #define ZL380xx_AEC_RIN_HPF_SHIFT                    9
 #define ZL380xx_AEC_RIN_HPF_DISABLE                 (1<<ZL380xx_AEC_RIN_HPF_SHIFT)
 #define ZL380xx_AEC_SIN_HPF_SHIFT                    10
 #define ZL380xx_AEC_SIN_HPF_DISABLE                 (1<<ZL380xx_AEC_SIN_HPF_SHIFT)
 #define ZL380xx_AEC_HD_SHIFT                         11
 #define ZL380xx_AEC_HD_DISABLE                      (1<<ZL380xx_AEC_HD_SHIFT)
 #define ZL380xx_AEC_AGCDIS_SHIFT                     12
 #define ZL380xx_AEC_AGC_DISABLE                     (1<<ZL380xx_AEC_AGCDIS_SHIFT) 
 #define ZL380xx_AEC_NBD_SHIFT                        13
 #define ZL380xx_AEC_NBD_DISABLE                     (1<<ZL380xx_AEC_NBD_SHIFT)
 #define ZL380xx_AEC_SWITCHED_ATTEN_SHIFT             14
 #define ZL380xx_AEC_SWITCHED_ATTEN_EN               (1<<ZL380xx_AEC_SWITCHED_ATTEN_SHIFT)

#define ZL380xx_ROUT_GAIN_CTRL_REG               0x030A
 #define ZL380xx_ROUT_GAIN_CTRL_UGAIN_SHIFT       0
 #define ZL380xx_ROUT_GAIN_CTRL_UGAIN_MASK        (0x7F << ZL380xx_ROUT_GAIN_CTRL_UGAIN_SHIFT)

#define ZL380xx_SOUT_GAIN_CTRL_REG               0x030C
 #define ZL380xx_SIN_GAIN_PAD_SHIFT               0
 #define ZL380xx_SIN_GAIN_PAD_MASK                (0xFF<<ZL380xx_SIN_GAIN_PAD_SHIFT)
 #define ZL380xx_SOUT_GAIN_PAD_SHIFT              8
 #define ZL380xx_SOUT_GAIN_PAD_MASK               (0xF << ZL380xx_SOUT_GAIN_PAD_SHIFT)

/* Automatic Gain Control Regs */
#define AUD_AGC_LVL_REG                           0x045B

#define AUD_SOUT_HI_SGNL_THRESHOLD_REG            0x05F1 /* automatically adjusts Sout gain */

#define AUD_ALC_CONTROL_REG                       0x05F3

#define AUD_USER_GAIN_CTRL_REG                    0x046B
 #define AUD_USER_ROUT_GAIN_SHIFT                  0
 #define AUD_USER_XROUT_GAIN_ADJUST_SHIFT          7

#define AUD_SEC_CHAN_USER_GAIN_CTRL_REG           0x046A
 #define AUD_SEC_CHAN_RX_PATH_GAIN_SHIFT           0
 #define AUD_SEC_CHAN_SEND_PATH_GAIN_SHIFT         4

#define ZL380xx_SND_LOC_DIR_REG 0x00A0

struct zl38_codec_priv {
	struct device *dev;
	struct regmap *regmap;
	bool is_stream_in_use[2];
	struct gpio_chip *gpio_chip;
	uint16_t readReg;
};

static int zl38_fw_issue_command(struct regmap *regmap, u16 cmd)
{
	unsigned int val;
	int err;

	err = regmap_read_poll_timeout(regmap, REG_SEMA_FLAGS, val,
				       !(val & SEMA_FLAGS_BOOT_CMD), 10000,
				       10000 * 100);
	if (err)
		return err;
	err = regmap_write(regmap, REG_CMD, cmd);
	if (err)
		return err;
	err = regmap_update_bits(regmap, REG_SEMA_FLAGS, SEMA_FLAGS_BOOT_CMD,
				 SEMA_FLAGS_BOOT_CMD);
	if (err)
		return err;

	return regmap_read_poll_timeout(regmap, REG_CMD, val, !val, 10000,
					10000 * 100);
}

static int zl38_fw_go(struct regmap *regmap)
{
	int err;

	err = zl38_fw_issue_command(regmap, BOOTCMD_LOAD_COMPLETE);
	if (err)
		return err;

	return zl38_fw_issue_command(regmap, BOOTCMD_FW_GO);
}

static int zl38_fw_enter_boot_mode(struct regmap *regmap)
{
	unsigned int val;
	int err;

	err = regmap_update_bits(regmap, REG_CLK_STATUS, CLK_STATUS_HWRST,
				 CLK_STATUS_HWRST);
	if (err)
		return err;

	return regmap_read_poll_timeout(regmap, REG_PARAM_RESULT, val,
					val == PARAM_RESULT_READY, 1000, 50000);
}

static int
zl38_fw_send_data(struct regmap *regmap, u32 addr, const void *data, u16 len)
{
	__be32 addr_base = cpu_to_be32(addr & ~0xFF);
	int err;

	err = regmap_raw_write(regmap, REG_PG255_BASE_HI, &addr_base,
			       sizeof(addr_base));
	if (err)
		return err;
	return regmap_raw_write(regmap, REG_PG255_OFFS(addr), data, len);
}

static int zl38_fw_send_xaddr(struct regmap *regmap, const void *data)
{
	/* execution address from ihex: 32-bit little endian.
	 * device register expects 32-bit big endian.
	 */
	u32 addr = le32_to_cpup(data);
	__be32 baddr = cpu_to_be32(addr);

	return regmap_raw_write(regmap, REG_FWR_EXEC, &baddr, sizeof(baddr));
}

static int zl38_load_firmware(struct device *dev, struct regmap *regmap)
{
	const struct ihex_binrec *rec;
	const struct firmware *fw;
	u32 addr;
	u16 len;
	int err;

	/* how to get this firmware:
	 * 1. request and download chip firmware from Microsemi
	 *    (provided by Microsemi in srec format)
	 * 2. convert downloaded firmware from srec to ihex. Simple tool:
	 *    https://gitlab.com/TheSven73/s3-to-irec
	 * 3. convert ihex to binary (.fw) using ihex2fw tool which is included
	 *    with the Linux kernel sources
	 */
	err = request_ihex_firmware(&fw, "zl38060.fw", dev);
	if (err)
		return err;
	err = zl38_fw_enter_boot_mode(regmap);
	if (err)
		goto out;
	rec = (const struct ihex_binrec *)fw->data;
	while (rec) {
		addr = be32_to_cpu(rec->addr);
		len = be16_to_cpu(rec->len);
		if (addr) {
			/* regular data ihex record */
			err = zl38_fw_send_data(regmap, addr, rec->data, len);
		} else if (len == 4) {
			/* execution address ihex record */
			err = zl38_fw_send_xaddr(regmap, rec->data);
		} else {
			err = -EINVAL;
		}
		if (err)
			goto out;
		/* next ! */
		rec = ihex_next_binrec(rec);
	}
	err = zl38_fw_go(regmap);

out:
	release_firmware(fw);
	return err;
}


static int zl38_software_reset(struct regmap *regmap)
{
	unsigned int val;
	int err;

	err = regmap_update_bits(regmap, REG_SEMA_FLAGS, SEMA_FLAGS_APP_REBOOT,
				 SEMA_FLAGS_APP_REBOOT);
	if (err)
		return err;

	/* wait for host bus interface to settle.
	 * Not sure if this is required: Microsemi's vendor driver does this,
	 * but the firmware manual does not mention it. Leave it in, there's
	 * little downside, apart from a slower reset.
	 */
	msleep(50);

	return regmap_read_poll_timeout(regmap, REG_SEMA_FLAGS, val,
					!(val & SEMA_FLAGS_APP_REBOOT), 10000,
					10000 * 100);
}

static int zl38_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	// struct zl38_codec_priv *priv = snd_soc_dai_get_drvdata(dai);
	// int err;
	// printk(KERN_DEBUG "zl38060 fmt: %x\n",fmt);
	// switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	// case SND_SOC_DAIFMT_I2S:
	// 	/* firmware default is normal i2s */
	// 	break;
	// default:
	// 	printk(KERN_DEBUG "zl38060 Err: %s:%i\n", __FILE__, __LINE__);
	// 	return -EINVAL;
	// }

	// switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	// case SND_SOC_DAIFMT_NB_NF:
	// 	/* firmware default is normal bitclock and frame */
	// 	break;
	// default:
	// 	printk(KERN_DEBUG "zl38060 Err: %s:%i\n", __FILE__, __LINE__);
	// 	return -EINVAL;
	// }

	// switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	// case SND_SOC_DAIFMT_CBP_CFP:
	// 	/* always 32 bits per frame (= 16 bits/channel, 2 channels) */
	// 	err = regmap_update_bits(priv->regmap, REG_TDMA_CFG_CLK,
	// 				 CFG_CLK_MASTER | CFG_CLK_PCLK_MASK,
	// 				 CFG_CLK_MASTER | CFG_CLK_PCLK(32));
	// 	if (err){
	// 		printk(KERN_DEBUG "zl38060 Err: %s:%i\n", __FILE__, __LINE__);
	// 		return err;
	// 	}
	// 	break;
	// default:
	// 	printk(KERN_DEBUG "zl38060 Err: %s:%i\n", __FILE__, __LINE__);
	// 	return -EINVAL;
	// }

	return 0;
}

static int zl38_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params,
			  struct snd_soc_dai *dai)
{
	struct zl38_codec_priv *priv = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int fsrate;
	int err;

	/* We cannot change hw_params while the dai is already in use - the
	 * software reset will corrupt the audio. However, this is not required,
	 * as the chip's TDM buses are fully symmetric, which mandates identical
	 * rates, channels, and samplebits for record and playback.
	 */
	if (priv->is_stream_in_use[!tx])
		goto skip_setup;

	switch (params_rate(params)) {
	case 8000:
		fsrate = CFG_CLK_FSRATE_8KHZ;
		break;
	case 16000:
		fsrate = CFG_CLK_FSRATE_16KHZ;
		break;
	case 48000:
		fsrate = CFG_CLK_FSRATE_48KHZ;
		break;
	default:
		return -EINVAL;
	}

	err = regmap_update_bits(priv->regmap, REG_TDMA_CFG_CLK,
				 CFG_CLK_FSRATE_MASK, fsrate);
	if (err)
		return err;

	/* chip requires a software reset to apply audio register changes */
	err = zl38_software_reset(priv->regmap);
	if (err)
		return err;

skip_setup:
	priv->is_stream_in_use[tx] = true;

	return 0;
}

static int zl38_hw_free(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct zl38_codec_priv *priv = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	priv->is_stream_in_use[tx] = false;

	return 0;
}

/* stereo bypass with no AEC */
static const struct reg_sequence cp_config_stereo_bypass[] = {
	/* interconnects must be programmed first */
	{ 0x0210, 0x0005 },	/* DAC1   in <= I2S1-L */
	{ 0x0212, 0x0006 },	/* DAC2   in <= I2S1-R */
	{ 0x0214, 0x0001 },	/* I2S1-L in <= MIC1   */
	{ 0x0216, 0x0001 },	/* I2S1-R in <= MIC1   */
	{ 0x0224, 0x0000 },	/* AEC-S  in <= n/a    */
	{ 0x0226, 0x0000 },	/* AEC-R  in <= n/a    */
	/* output enables must be programmed next */
	{ 0x0202, 0x000F },	/* enable I2S1 + DAC   */
};

static const struct snd_soc_dai_ops zl38_dai_ops = {
	//.mute_stream   = zl380xx_mute,
	.set_fmt = zl38_set_fmt,
	.hw_params = zl38_hw_params,
	.hw_free = zl38_hw_free,
};

static struct snd_soc_dai_driver zl38_dai = {
	.name = "zl38060-tdma",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ZL38_RATES,
		.formats = ZL38_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ZL38_RATES,
		.formats = ZL38_FORMATS,
	},
	.ops = &zl38_dai_ops,
	.symmetric_rate = 1,
	.symmetric_sample_bits = 1,
	.symmetric_channels = 1,
};

static const struct snd_soc_dapm_widget zl38_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("DAC1"),
	SND_SOC_DAPM_OUTPUT("DAC2"),

	SND_SOC_DAPM_INPUT("DMICL"),
};

static const struct snd_soc_dapm_route zl38_dapm_routes[] = {
	{ "DAC1",  NULL, "Playback" },
	{ "DAC2",  NULL, "Playback" },

	{ "Capture",  NULL, "DMICL" },
};

uint16_t swapBytes(uint16_t val) {
    return (val >> 8) | (val << 8);
}

static int zl380xx_master_control_read(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mixer_ctrl = (struct soc_mixer_control *) kcontrol->private_value;
    unsigned int reg = mixer_ctrl->reg;
    unsigned int shift = mixer_ctrl->shift;
    int ret;
    uint16_t val=0;

    ret = regmap_raw_read(priv->regmap, reg, &val, sizeof(val));
    val = swapBytes(val); 
    
    int8_t alsa_val = ((val >> shift) & 0x00FF); 
	
	// Convert values from 0-50 to -45 to 5
    alsa_val = alsa_val + 45;

    ucontrol->value.integer.value[0] = alsa_val; 

    return 0;  
}

static int zl380xx_master_control_write(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mixer_ctrl = (struct soc_mixer_control *) kcontrol->private_value;
    unsigned int reg = mixer_ctrl->reg;
    unsigned int shift = mixer_ctrl->shift;
    unsigned int val_new_user = ucontrol->value.integer.value[0];
    int val_new_reg;
    unsigned int val_old = 0;
    int status;

    // Convert values from 0-50 to -45 to 5
    val_new_reg = val_new_user - 45;

    status = regmap_raw_read(priv->regmap, reg, &val_old, sizeof(val_old));
    val_old = swapBytes(val_old);

    val_old &= ~(0xFF << shift);
    val_old |= ((unsigned int)(val_new_reg & 0xFF)) << shift;

    val_old = swapBytes(val_old);
    status = regmap_raw_write(priv->regmap, reg, &val_old, sizeof(val_old));

    if (status < 0)
        return status;

    return 0;
}

static int zl380xx_REG_control_read(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);

    int ret;
    uint16_t val=0;

    ret = regmap_raw_read(priv->regmap, priv->readReg, &val, sizeof(val));
    val = swapBytes(val); 
        
    ucontrol->value.integer.value[0] = val;

    return 0;  
}

static int zl380xx_REG_control_write(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);
    uint32_t val_new_reg = ucontrol->value.integer.value[0];
	
	uint16_t reg = val_new_reg >> 16;
	uint16_t val = val_new_reg;

//pr_info("reg11 0x%x", val_new_reg);
	
	if ((reg & 0x4000) != 0) { //the bit is set, so the next step will read this register
		priv->readReg=(reg & 0x0FFF);
		return 0;
	}

	if ((reg & 0x2000) == 0) {
		return 0;
	}
	
	reg=(reg & 0x0FFF);

    int status;

    val = swapBytes(val);
	
    status = regmap_raw_write(priv->regmap, reg, &val, sizeof(val));

    if (status < 0)
        return status;

    return 0;
}

static int zl380xx_PM_control_read(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int val1=0;
	unsigned int val2=0;
	uint32_t val=0;
	int ret;

	ret = regmap_raw_read(priv->regmap, 0x0E82, &val1, sizeof(val1));
	ret = regmap_raw_read(priv->regmap, 0x0E80, &val2, sizeof(val2));
	val1=swapBytes(val1);
	val2=swapBytes(val2);
	val=(val2 << 16) | val1;
	
	unsigned int valZero=0;
	ret = regmap_raw_write(priv->regmap, 0x0E82, &valZero, sizeof(valZero));

	
	ucontrol->value.integer.value[0] = val;
	
	return 0;	
}

static int zl380xx_control_read(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mixer_ctrl = (struct soc_mixer_control *) kcontrol->private_value;
	unsigned int reg = mixer_ctrl->reg;
    unsigned int shift = mixer_ctrl->shift;
    unsigned int max = mixer_ctrl->max;
    unsigned int invert = mixer_ctrl->invert;
	unsigned int val=0;
	int ret;

	ret = regmap_raw_read(priv->regmap, reg, &val, sizeof(val));
	val=swapBytes(val);
	
	val=((val >> shift) & 0x00FF);
	if (val>max) val=max;
	
	ucontrol->value.integer.value[0] = val;
    if (invert)
        ucontrol->value.integer.value[0] = max - ucontrol->value.integer.value[0];
	
	return 0;	
}

static int zl380xx_control_write(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct zl38_codec_priv *priv = snd_soc_component_get_drvdata(component);
	struct soc_mixer_control *mixer_ctrl = (struct soc_mixer_control *) kcontrol->private_value;
	unsigned int reg = mixer_ctrl->reg;
    unsigned int shift = mixer_ctrl->shift;
    unsigned int max = mixer_ctrl->max;
    unsigned int invert = mixer_ctrl->invert;
	unsigned int val_new = ucontrol->value.integer.value[0];
	unsigned int val_old = 0;
	int status;

    if (invert)
        val_new = max - val_new;

	status = regmap_raw_read(priv->regmap, reg, &val_old, sizeof(val_old));
	val_old=swapBytes(val_old);
	
    if (((val_old >> shift) & 0x00FF) == val_new) {
        return 0;
    }
	
    val_old &= ~(0xFF << shift);
    val_old |= val_new << shift;
	
	val_old=swapBytes(val_old);
	status = regmap_raw_write(priv->regmap, reg,	&val_old, sizeof(val_old));

	if (status < 0)
		return status;

	return 0;	
}

static const struct snd_kcontrol_new zl38_snd_controls[] = {
    SOC_SINGLE_EXT("MasterA", ZL380xx_CP_DAC1_GAIN_REG, 0, 50, 0, zl380xx_master_control_read, zl380xx_master_control_write),
	SOC_SINGLE_EXT("MasterB", ZL380xx_CP_DAC2_GAIN_REG, 0, 50, 0, zl380xx_master_control_read, zl380xx_master_control_write),
	SOC_SINGLE("DIG MIC", ZL380xx_DIG_MIC_GAIN_REG, 0, 0x7, 0),
	SOC_SINGLE_EXT("Power Meter", 0x0E00, 0, 0x18, 0, zl380xx_PM_control_read,zl380xx_control_write),
	SOC_SINGLE_EXT("Registers", 0xFF00, 0, 0x7FFFFFFF, 0, zl380xx_REG_control_read,zl380xx_REG_control_write),
};

static const struct snd_soc_component_driver zl38_component_dev = {
	.dapm_widgets = zl38_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(zl38_dapm_widgets),
	.dapm_routes = zl38_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(zl38_dapm_routes),
	.controls = zl38_snd_controls,
	.num_controls = ARRAY_SIZE(zl38_snd_controls),

	.endianness = 1,
};

static void chip_gpio_set(struct gpio_chip *c, unsigned int offset, int val)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);

	regmap_update_bits(regmap, REG_GPIO_DAT, mask, val ? mask : 0);
}

static int chip_gpio_get(struct gpio_chip *c, unsigned int offset)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);
	unsigned int val;
	int err;

	err = regmap_read(regmap, REG_GPIO_DAT, &val);
	if (err)
		return err;

	return !!(val & mask);
}

static int chip_direction_input(struct gpio_chip *c, unsigned int offset)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);

	return regmap_update_bits(regmap, REG_GPIO_DIR, mask, 0);
}

static int
chip_direction_output(struct gpio_chip *c, unsigned int offset, int val)
{
	struct regmap *regmap = gpiochip_get_data(c);
	unsigned int mask = BIT(offset);

	chip_gpio_set(c, offset, val);
	return regmap_update_bits(regmap, REG_GPIO_DIR, mask, mask);
}

static const struct gpio_chip template_chip = {
	.owner = THIS_MODULE,
	.label = DRV_NAME,

	.base = -1,
	.ngpio = 14,
	.direction_input = chip_direction_input,
	.direction_output = chip_direction_output,
	.get = chip_gpio_get,
	.set = chip_gpio_set,

	.can_sleep = true,
};

static int zl38_check_revision(struct device *dev, struct regmap *regmap)
{
	unsigned int hwrev, fwprod, fwrev;
	int fw_major, fw_minor, fw_micro;
	int err;

	err = regmap_read(regmap, REG_HW_REV, &hwrev);
	if (err)
		return err;
	err = regmap_read(regmap, REG_FW_PROD, &fwprod);
	if (err)
		return err;
	err = regmap_read(regmap, REG_FW_REV, &fwrev);
	if (err)
		return err;

	fw_major = (fwrev >> 12) & 0xF;
	fw_minor = (fwrev >>  8) & 0xF;
	fw_micro = fwrev & 0xFF;
	dev_info(dev, "hw rev 0x%x, fw product code %d, firmware rev %d.%d.%d",
		 hwrev & 0x1F, fwprod, fw_major, fw_minor, fw_micro);

	if (fw_major != FIRMWARE_MAJOR || fw_minor < FIRMWARE_MINOR) {
		dev_err(dev, "unsupported firmware. driver supports %d.%d",
			FIRMWARE_MAJOR, FIRMWARE_MINOR);
		return -EINVAL;
	}

	return 0;
}

static int zl38_bus_read(void *context,
			 const void *reg_buf, size_t reg_size,
			 void *val_buf, size_t val_size)
{
	struct spi_device *spi = context;
	const u8 *reg_buf8 = reg_buf;
	size_t len = 0;
	u8 offs, page;
	u8 txbuf[4];

	if (reg_size != 2 || val_size > ZL38_MAX_RAW_XFER)
		return -EINVAL;

	offs = reg_buf8[1] >> 1;
	page = reg_buf8[0];

	if (page) {
		txbuf[len++] = 0xFE;
		txbuf[len++] = page == HBI_FIRMWARE_PAGE ? 0xFF : page - 1;
		txbuf[len++] = offs;
		txbuf[len++] = val_size / 2 - 1;
	} else {
		txbuf[len++] = offs | 0x80;
		txbuf[len++] = val_size / 2 - 1;
	}

	return spi_write_then_read(spi, txbuf, len, val_buf, val_size);
}

static int zl38_bus_write(void *context, const void *data, size_t count)
{
	struct spi_device *spi = context;
	u8 buf[4 + ZL38_MAX_RAW_XFER];
	size_t val_len, len = 0;
	const u8 *data8 = data;
	u8 offs, page;

	if (count > (2 + ZL38_MAX_RAW_XFER) || count < 4)
		return -EINVAL;
	val_len = count - 2;
	offs = data8[1] >> 1;
	page = data8[0];

	if (page) {
		buf[len++] = 0xFE;
		buf[len++] = page == HBI_FIRMWARE_PAGE ? 0xFF : page - 1;
		buf[len++] = offs;
		buf[len++] = (val_len / 2 - 1) | 0x80;
	} else {
		buf[len++] = offs | 0x80;
		buf[len++] = (val_len / 2 - 1) | 0x80;
	}
	memcpy(buf + len, data8 + 2, val_len);
	len += val_len;

	return spi_write(spi, buf, len);
}

static const struct regmap_bus zl38_regmap_bus = {
	.read = zl38_bus_read,
	.write = zl38_bus_write,
	.max_raw_write = ZL38_MAX_RAW_XFER,
	.max_raw_read = ZL38_MAX_RAW_XFER,
};

static const struct regmap_config zl38_regmap_conf = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 2,
	.use_single_read = true,
	.use_single_write = true,
};

static int zl38_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct zl38_codec_priv *priv;
	struct gpio_desc *reset_gpio;
	int err;

	/* get the chip to a known state by putting it in reset */
	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return PTR_ERR(reset_gpio);
	if (reset_gpio) {
		/* datasheet: need > 10us for a digital + analog reset */
		usleep_range(15, 50);
		/* take the chip out of reset */
		gpiod_set_value_cansleep(reset_gpio, 0);
		/* datasheet: need > 3ms for digital section to become stable */
		usleep_range(3000, 10000);
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	dev_set_drvdata(dev, priv);
	priv->regmap = devm_regmap_init(dev, &zl38_regmap_bus, spi,
					&zl38_regmap_conf);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	//err = zl38_load_firmware(dev, priv->regmap);
	//if (err)
	//	return err;

	// err = zl38_check_revision(dev, priv->regmap);
	// if (err)
	// 	return err;

	priv->gpio_chip = devm_kmemdup(dev, &template_chip,
				       sizeof(template_chip), GFP_KERNEL);
	if (!priv->gpio_chip)
		return -ENOMEM;
	priv->gpio_chip->parent = dev;
	err = devm_gpiochip_add_data(dev, priv->gpio_chip, priv->regmap);
	if (err)
		return err;

	// /* setup the cross-point switch for stereo bypass */
	// err = regmap_multi_reg_write(priv->regmap, cp_config_stereo_bypass,
				     // ARRAY_SIZE(cp_config_stereo_bypass));
	// if (err)
		// return err;
	
	/* setup for 12MHz crystal connected to the chip */
	err = regmap_update_bits(priv->regmap, REG_CLK_CFG, CLK_CFG_SOURCE_XTAL,
				 CLK_CFG_SOURCE_XTAL);
	if (err)
		return err;

	return devm_snd_soc_register_component(dev, &zl38_component_dev,
					       &zl38_dai, 1);
}

static const struct of_device_id zl38_dt_ids[] = {
	{ .compatible = "mscc,zl38060", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, zl38_dt_ids);

static const struct spi_device_id zl38_spi_ids[] = {
	{ "zl38060", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, zl38_spi_ids);

static struct spi_driver zl38060_spi_driver = {
	.driver	= {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(zl38_dt_ids),
	},
	.probe = zl38_spi_probe,
	.id_table = zl38_spi_ids,
};
module_spi_driver(zl38060_spi_driver);

MODULE_DESCRIPTION("ASoC ZL38060 driver");
MODULE_AUTHOR("Sven Van Asbroeck <TheSven73@gmail.com>");
MODULE_LICENSE("GPL v2");
