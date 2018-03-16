/*
 * Driver for EPC660 ToF image sensor from Espros
 *
 * Copyright (C) 2016, Stefan Haun <stefan.haun@irisgmbh.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include <linux/module.h>

#include <media/soc_camera.h>
#include <media/drv-intf/soc_mediabus.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-ctrls.h>

#include <media/i2c/epc660_sequence.h>

#include <asm/uaccess.h>


/*
 * epc660 i2c address 0x20
 * The platform has to define struct i2c_board_info objects and link to them
 * from struct soc_camera_host_desc
 */

#define EPC660_MAX_WIDTH		320
#define EPC660_MAX_HEIGHT		240
#define EPC660_MIN_WIDTH		6
#define EPC660_MIN_HEIGHT		4


/* EPC660 has only one fixed colorspace per pixelcode */
struct epc660_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

/* Find a data format by a pixel code in an array */
static const struct epc660_datafmt *epc660_find_datafmt(
	u32 code, const struct epc660_datafmt *fmt,
	int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

static const struct epc660_datafmt epc660_monochrome_fmts[] = {
	/* Order important - see above */
	{MEDIA_BUS_FMT_Y12_1X12, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_EPC660_2X12, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_EPC660_3X12, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_EPC660_4X12, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_EPC660_5X12, V4L2_COLORSPACE_JPEG},
};

struct epc660 {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_rect rect;	/* Sensor window */
	struct v4l2_clk *clk;
	const struct epc660_datafmt *fmt;
	const struct epc660_datafmt *fmts;
	int num_fmts;
	u8  ic_version;
	u8  customer_id;
	u16 wafer_id;
	u16 chip_id;
	u8  part_type;
	u8  part_version;
};

static struct epc660 *to_epc660(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct epc660, subdev);
}

static u8 reg_read_byte(struct i2c_client *client, const u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static int reg_read(struct i2c_client *client, const u8 reg)
{
	return i2c_smbus_read_word_data(client, reg);
}

static int reg_write_byte(struct i2c_client *client, const u8 reg,
		     const u8 data)
{
	return i2c_smbus_write_byte_data(client, reg, data);
}

static int reg_write(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	return i2c_smbus_write_word_data(client, reg, data);
}

static int epc660_eeprom_read_byte(struct i2c_client *client,
								   u8 address, u8 *data)
{
	int ret;

	/* Write address to I2C */
	ret = reg_write_byte(client, EPC660_EEPROM_ADDRESS, address);
	if (ret < 0) {
		goto fail;
	}

	/* Read result from I2C */
	ret = reg_read_byte(client, EPC660_EEPROM_DATA);
	if (ret < 0) {
		goto fail;
	}

	*data = (u8)ret;

	return 0;

fail:
	printk(KERN_ERR "%s: Failed read EEPROM byte data from %02x!\n",
		   __func__, address);
	return -1;
}

static int epc660_eeprom_read_word(struct i2c_client *client,
								   u8 address, u16 *data)
{
	int ret;

	/* Write address to I2C */
	ret = reg_write_byte(client, EPC660_EEPROM_ADDRESS, address);
	if (ret < 0) {
		goto fail;
	}

	/* Read result from I2C */
	ret = reg_read(client, EPC660_EEPROM_DATA);
	if (ret < 0) {
		goto fail;
	}

	*data = (u16)ret;

	return 0;

fail:
	printk(KERN_ERR "%s: Failed read EEPROM word data from %02x!\n",
		   __func__, address);
	return -1;
}

/*
 * Send an I2C sequence to the imager.
 *
 * Return 0 on success, otherwise the i2c return code.
 */
static int epc660_send_i2c_sequence(struct i2c_client *client,
									const u8 *seq)
{
	int ret;
	int i;
	u8 len;

	i = 0;
	while ( (len = seq[i++]) != 0 ) {
		ret = i2c_master_send(client, seq+i, len);
		if (ret < 0) {
			printk(KERN_ERR
			      "Failed to send I2C sequence "
				  "with length 0x%02x at offset %04x\n",
		          len, i);
			goto fail;
		}
		i += len;

		udelay(100); // How long do we have to wait?
	}

	return 0;
fail:
	return ret;
}

static int epc660_device_init(struct i2c_client *client)
{
	int ret;
	/* Reset the imager */
	ret = reg_write_byte(client, 0x00, 0x06);
	if (ret < 0) {
		printk(KERN_ERR "Failed to reset the device!\n");
		goto fail;
	}
	udelay(350); // How long do we have to wait?

	printk(KERN_INFO "EPC660 I2C initialization ");
	ret = epc660_send_i2c_sequence(client, epc660_init_sequence);
	if (ret < 0) {
		goto fail;
	};
	// TODO check success
	printk(KERN_INFO " done.\n");


	printk(KERN_INFO "Programming EPC660 sequencer ");
	ret = epc660_send_i2c_sequence(client, epc660_003_Seq_Prog_8MHz_Default_8);
	if (ret < 0) {
		goto fail;
	};
	// TODO check success
	printk(KERN_INFO " done.\n");


	return 0;
fail:
	return ret;
}

static int epc660_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/*
	 * 8.7.15. Shutter_Control ("Datasheet_epc660-V1.03.pdf" page 99)
	 * 0xA4/0xA5 (because 16bit access here)
	 * 0xA4 Shutter-Ctrl => bit[1]=multi_frame_en(0/1), bit[0]=shutter_en(0/1);
	 * 0xA5 Power-Ctrl => bit[2:0] = 111b
	 */

	if (enable) {
		/* Enable power */
		if (reg_write_byte(client, EPC660_REG_POWER_CTRL, 0x07) < 0)
		  return -EIO;
		if (reg_write_byte(client, EPC660_REG_LED_DRIVER, 0xe0) < 0)
		  return -EIO;
		/* Switch to multi frame mode and enable shutter */
//		do not switch on the automatic shutter as the application shall have the possibilty to do that by hand
//		if (reg_write_byte(client, EPC660_REG_SHUTTER_CTRL, 0x03) < 0)
//		  return -EIO;
	} else {
		/* Switch off image acquisition */
		if (reg_write_byte(client, EPC660_REG_SHUTTER_CTRL, 0x00) < 0)
		  return -EIO;
		/* turn LEDs driver off */
		if (reg_write_byte(client, EPC660_REG_LED_DRIVER, 0x00) < 0)
		  return -EIO;
		/* Disable power */
		if (reg_write_byte(client, EPC660_REG_POWER_CTRL, 0x00) < 0)
		  return -EIO;
	}

	return 0;
}

static int epc660_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct epc660 *epc660 = to_epc660(client);

	mf->width	= epc660->rect.width;
	mf->height	= epc660->rect.height;
	mf->code	= epc660->fmt->code;
	mf->colorspace	= epc660->fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int epc660_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct epc660 *epc660 = to_epc660(client);
	const int centerX = (324+4)/2;
	const int centerY = (246+6)/2;
	const int bY = centerY-1;
	int lX;
	int rX;
	int uY;
	int walign = 4;
	int halign = 1;

	if (format->pad)
		return -EINVAL;

	v4l_bound_align_image(&mf->width, EPC660_MIN_WIDTH,
		EPC660_MAX_WIDTH, walign,
		&mf->height, EPC660_MIN_HEIGHT,
		EPC660_MAX_HEIGHT, halign, 0);
	epc660->fmt = epc660_find_datafmt(mf->code, epc660->fmts,
				   epc660->num_fmts);
	mf->colorspace	= epc660->fmt->colorspace;

	// set the ROI on the EPC
	lX = (centerX - mf->width / 2) & ~1; // the ROI has to start at an even offset
	rX = lX + mf->width - 1;
	uY = (centerY - mf->height / 2) & ~1; // the ROI has to start at an even offset

	lX = ((lX >> 8) & 0xff) | ((lX << 8) & 0xff00);
	rX = ((rX >> 8) & 0xff) | ((rX << 8) & 0xff00);
	reg_write(client, EPC660_REG_ROI_TL_X_HI, lX);
	reg_write(client, EPC660_REG_ROI_BR_X_HI, rX);
	reg_write_byte(client, EPC660_REG_ROI_TL_Y, uY);
	reg_write_byte(client, EPC660_REG_ROI_BR_Y, bY);

	epc660->rect.width  = mf->width;
	epc660->rect.height = mf->height;

	return 0;
}

static int epc660_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct epc660 *epc660 = to_epc660(client);

	if (code->pad || code->index >= epc660->num_fmts)
		return -EINVAL;

	code->code = epc660->fmts[code->index].code;
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int epc660_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff)
		return -EINVAL;

	reg->size = 2;
	reg->val = reg_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int epc660_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg > 0xff)
		return -EINVAL;

	if (reg_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif

#define EPC_660_IOCTL_CMD_SET_REGISTER 129
#define EPC_660_IOCTL_CMD_GET_REGISTER 130
struct epc_660_reg_params {
	u32 regNo;   /* the register (address) */
	u32 content; /* what to write into the register */
	u32 size;    /* the size of the register in bytes */
} ;
static long epc660_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void* arg)
{
	struct i2c_client *client;
	struct epc_660_reg_params params;
	client = v4l2_get_subdevdata(sd);
	switch (cmd) {
		case EPC_660_IOCTL_CMD_SET_REGISTER:
			if (copy_from_user(&params, (struct epc_660_reg_params *)arg, sizeof(params))) {
				return -EACCES;
			}
			if (params.regNo >= 0xfb
				|| params.regNo < 0
				|| params.size < 1
				|| params.size > 2) {
				return -EINVAL;
			}
			if (1 == params.size) {
				return reg_write_byte(client, params.regNo, params.content);
			} else if (2 == params.size) {
				return reg_write(client, params.regNo, params.content);
			}
			break;
		case EPC_660_IOCTL_CMD_GET_REGISTER:
			if (copy_from_user(&params, (struct epc_660_reg_params *)arg, sizeof(params))) {
				return -EACCES;
			}
			if (params.regNo >= 0xfb
				|| params.regNo < 0
				|| params.size < 1
				|| params.size > 2) {
				return -EINVAL;
			}
			if (1 == params.size) {
				params.content = reg_read_byte(client, params.regNo);
			} else if (2 == params.size) {
				params.content = reg_read(client, params.regNo);
			}
			if (copy_to_user((struct epc_660_reg_params *)arg, &params, sizeof(params))) {
				return -EACCES;
			}
			break;
		default:
			break;
	}
	return 0;
}


/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int epc660_video_probe(struct i2c_client *client)
{
	struct epc660 *epc660 = to_epc660(client);
	int ret;

	/* Read out the chip version register */
	ret = reg_read_byte(client, EPC660_REG_IC_VERSION);
	if (ret < 0) {
		goto ei2c;
	};
	epc660->ic_version = (u8)(ret & 0xff);

	// We need at least chip revision 3
	if (epc660->ic_version < 0x03) {
		ret = -ENODEV;
		dev_err(&client->dev,
			"\n"
			"\t\t************************************************\n"
			"\t\t*                                              *\n"
			"\t\t*  No viable EPC660 found, IC version is 0x%02x  *\n"
			"\t\t*                                              *\n"
			"\t\t************************************************\n",
		        epc660->ic_version);
		dev_err(&client->dev, "EPC660: Chip version must be at least 3.\n");
		goto ei2c;
	}

	ret = reg_read_byte(client, EPC660_REG_CUSTOMER_ID);
	if (ret < 0) {
		goto ei2c;
	};
	ret = 0;

	ret |= epc660_eeprom_read_byte(client, EPC660_REG_CUSTOMER_ID,
					&epc660->customer_id);
	ret |= epc660_eeprom_read_word(client, EPC660_REG_WAFER_ID_MSB,
					&epc660->wafer_id);
	ret |= epc660_eeprom_read_word(client, EPC660_REG_CHIP_ID_MSB,
					&epc660->chip_id);
	ret |= epc660_eeprom_read_byte(client, EPC660_REG_PART_TYPE,
					&epc660->part_type);
	ret |= epc660_eeprom_read_byte(client, EPC660_REG_PART_VERSION,
					&epc660->part_version);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read the manufacturer properties!\n");
		goto ei2c;
	};

	printk(KERN_INFO
	       "Found EPC660 with:\n"
			"\tic version %02x\n"
			"\tcustomer id %02x\n"
			"\twafer id %04x\n"
			"\tchip id %02x\n"
			"\tpart type %02x\n"
			"\tpart version:%02x\n",
		   (int)(epc660->ic_version),
		   (int)(epc660->customer_id),
		   (int)(epc660->wafer_id),
		   (int)(epc660->chip_id),
		   (int)(epc660->part_type),
		   (int)(epc660->part_version));

	ret = epc660_device_init(client);
	if (ret < 0)
		goto ei2c;

	/* Init the formats table */
	epc660->fmts = epc660_monochrome_fmts;
	epc660->num_fmts = ARRAY_SIZE(epc660_monochrome_fmts);
	epc660->fmt = &epc660->fmts[0];

	epc660->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

ei2c:
	return ret;
}
#if 0
static int epc660_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	//not implemented yet

	return 0;
}

static int epc660_s_mbus_config(struct v4l2_subdev *sd,
				 const struct v4l2_mbus_config *cfg)
{
	//not implemented yet

	return 0;
}
#endif

static struct v4l2_subdev_core_ops epc660_subdev_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= epc660_g_register,
	.s_register	= epc660_s_register,
#endif
	.ioctl      = epc660_ioctl,
};

static struct v4l2_subdev_video_ops epc660_subdev_video_ops = {
	.s_stream	= epc660_s_stream,
#if 0
	.g_mbus_config	= epc660_g_mbus_config,
	.s_mbus_config	= epc660_s_mbus_config,
#endif
};

static const struct v4l2_subdev_pad_ops epc660_subdev_pad_ops = {
	.enum_mbus_code = epc660_enum_mbus_code,
#if 0
	.get_selection	= epc660_get_selection,
	.set_selection	= epc660_set_selection,
#endif
	.get_fmt	= epc660_get_fmt,
	.set_fmt	= epc660_set_fmt,
};

static struct v4l2_subdev_ops epc660_subdev_ops = {
	.core	= &epc660_subdev_core_ops,
	.video	= &epc660_subdev_video_ops,
	.pad	= &epc660_subdev_pad_ops,
};

static int epc660_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct epc660 *epc660;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	epc660 = devm_kzalloc(&client->dev, sizeof(struct epc660), GFP_KERNEL);
	if (!epc660)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&epc660->subdev, client, &epc660_subdev_ops);
	v4l2_ctrl_handler_init(&epc660->hdl, 6);


	epc660->subdev.ctrl_handler = &epc660->hdl;
	if (epc660->hdl.error) {
		int err = epc660->hdl.error;

		dev_err(&client->dev, "control initialisation err %d\n", err);
		return err;
	}

	epc660->rect.width	= EPC660_MAX_WIDTH;
	epc660->rect.height	= EPC660_MAX_HEIGHT;

	epc660->clk = 0;

	ret = epc660_video_probe(client);
	if (ret)
		v4l2_ctrl_handler_free(&epc660->hdl);

	return ret;
}

static int epc660_remove(struct i2c_client *client)
{
	struct epc660 *epc660 = to_epc660(client);

	v4l2_device_unregister_subdev(&epc660->subdev);
	v4l2_ctrl_handler_free(&epc660->hdl);

	return 0;
}
static const struct i2c_device_id epc660_id[] = {
	{ "epc660", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, epc660_id);

static struct i2c_driver epc660_i2c_driver = {
	.driver = {
		.name = "epc660",
	},
	.probe		= epc660_probe,
	.remove		= epc660_remove,
	.id_table	= epc660_id,
};

module_i2c_driver(epc660_i2c_driver);

MODULE_DESCRIPTION("Espros EPC660 camera driver");
MODULE_AUTHOR("Stefan Haun <stefan.haun@irisgmbh.de>");
MODULE_LICENSE("GPL");
