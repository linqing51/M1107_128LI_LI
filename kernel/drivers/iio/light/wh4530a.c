/*
 * vcnl4000.c - Support for Vishay VCNL4000 combined ambient light and
 * proximity sensor
 *
 * Copyright 2012 Peter Meerwald <pmeerw@pmeerw.net>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for VCNL4000 (7-bit I2C slave address 0x13)
 *
 * TODO:
 *   allow to adjust IR current
 *   proximity threshold and event handling
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#define WH4530A_DRV_NAME "wh4530a"

/// wh4530a byte register
#define WH4530A_SYS_CTRL      0x00
#define WH4530A_INT_CTRL      0x01
#define WH4530A_INT_FLAG      0x02
#define WH4530A_WAIT_TIME     0x03
#define WH4530A_ALS_GAIN      0x04
#define WH4530A_ALS_TIME      0x05
#define WH4530A_PS_LED        0x06
#define WH4530A_PS_GAIN       0x07
#define WH4530A_PS_LED_PULSE  0x08
#define WH4530A_PS_TIME       0x09
#define WH4530A_PS_FILTER     0x0A
#define WH4530A_PERSISTENCE   0x0B
#define WH4530A_ALS_INT_SRC   0x16
#define WH4530A_ERR_FLAG      0x17

/// wh4530a word register
#define WH4530A_ALS_THR_L     0x0C
#define WH4530A_ALS_THR_H     0x0E
#define WH4530A_PS_THR_L      0x10
#define WH4530A_PS_THR_H      0x12
#define WH4530A_PS_OFFSET     0x14
#define WH4530A_PS_DATA       0x18
#define WH4530A_IR_DATA       0x1A
#define WH4530A_CHN0_DATA     0x1C
#define WH4530A_CHN1_DATA     0x1E

/// wh4530a als gain
#define WH4530A_ALS_GAIN_X1  0x00
#define WH4530A_ALS_GAIN_X4  0x01
#define WH4530A_ALS_GAIN_X8  0x02
#define WH4530A_ALS_GAIN_X32 0x03
#define WH4530A_ALS_GAIN_X96 0x04

/// wh4530a ps gain
#define WH4530A_PS_GAIN_X1   0x00
#define WH4530A_PS_GAIN_X2   0x01
#define WH4530A_PS_GAIN_X4   0x02
#define WH4530A_PS_GAIN_X8   0x03

/// wh4530a ps led current
#define WH4530A_LED_CURRENT_50MA  0x00
#define WH4530A_LED_CURRENT_100MA 0x40
#define WH4530A_LED_CURRENT_150MA 0x80
#define WH4530A_LED_CURRENT_200MA 0xC0

struct wh4530a_data {
	struct i2c_client *client;

  // dts config
	u8 wait_time;
	u8 als_int_src;
	u8 ps_int_mode;
	u8 ps_led_current;
	u8 ps_led_pulse_width;
	u8 ps_led_pulse_num;

	// config
	u8 als_gain;
	u8 als_time;
	u8 ps_gain;
	u8 ps_time;
	u16 ps_offset;

  // val
  u16 ps_data;
  u16 ir_data;
  u16 als_chn0_data;
  u16 als_chn1_data;
};

static int wh4530a_write_byte(struct wh4530a_data *wh4530a, u8 reg, u8 val)
{
  int err;
  unsigned char buf[2];

  buf[0] = reg;
  buf[1] = val;

	err = i2c_master_send(wh4530a->client, buf, sizeof(buf));
	if (err != sizeof(buf)) {
		dev_err(&wh4530a->client->dev,
				"%s: err=%d addr=%02x, data=%02x\n",
				__func__, err, buf[0], buf[1]);
		return -EIO;
	}

  return 0;
}

static int wh4530a_read_byte(struct wh4530a_data *wh4530a, u8 reg, u8 *val)
{
  int err;
  unsigned char buf[1];

  buf[0] = reg;

	err = i2c_master_send(wh4530a->client, buf, 1);
	if (err != sizeof(buf)) {
		dev_err(&wh4530a->client->dev,
				"%s: err=%d addr=%02x (send reg)\n",
				__func__, err, buf[0]);
		return -EIO;
	}

	err = i2c_master_recv(wh4530a->client, buf, 1);
	if (err != sizeof(buf)) {
		dev_err(&wh4530a->client->dev,
				"%s: err=%d addr=%02x (recv data)\n",
				__func__, err, buf[0]);
		return -EIO;
	}

  *val = buf[0];

  return 0;
}

static int wh4530a_write_word(struct wh4530a_data *wh4530a, u8 reg, u16 val)
{
  int ret;

  ret = wh4530a_write_byte(wh4530a, reg, val & 0xFF);
  if (ret < 0) {
    return ret;
  }

  ret = wh4530a_write_byte(wh4530a, reg + 1, (val >> 8) & 0xFF);
  if (ret < 0) {
    return ret;
  }

  return 0;
}

static int wh4530a_read_word(struct wh4530a_data *wh4530a, u8 reg, u16 *val)
{
  int ret;
  unsigned char buf[2];

  buf[0] = reg;

  ret = wh4530a_read_byte(wh4530a, reg, &buf[0]);
  if (ret < 0) {
    return ret;
  }

  ret = wh4530a_read_byte(wh4530a, reg, &buf[1]);
  if (ret < 0) {
    return ret;
  }

  *val = (buf[1] << 8) | buf[0];

  return 0;
}

static const struct iio_event_spec wh4530a_event[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
		                 BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
		                 BIT(IIO_EV_INFO_ENABLE),
	},
};


static const struct iio_chan_spec wh4530a_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		                      BIT(IIO_CHAN_INFO_SCALE) |
		                      BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.event_spec = wh4530a_event,
		.num_event_specs = ARRAY_SIZE(wh4530a_event),
	},
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		                      BIT(IIO_CHAN_INFO_SCALE) |
		                      BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
		                      BIT(IIO_CHAN_INFO_OFFSET),
		.event_spec = wh4530a_event,
		.num_event_specs = ARRAY_SIZE(wh4530a_event),
	}
};

static irqreturn_t wh4530a_event_handler(int irq, void *private)
{
	struct iio_dev *dev_info = private;
	struct wh4530a_data *wh4530a = iio_priv(dev_info);
	int ret;
	u8 data;

	ret = wh4530a_read_byte(wh4530a, WH4530A_INT_FLAG, &data);
	if (ret < 0) {
		return IRQ_NONE;
	}

	if (data & 0x40) {
		goto wh4530a_irq_handled;
	}

	if (data & 0x02) {
		iio_push_event(dev_info,
			       IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
						    0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns());
	}

	if (data & 0x01) {
		iio_push_event(dev_info,
			       IIO_UNMOD_EVENT_CODE(IIO_LIGHT,
						    0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns());
	}

	/* clear the interrupt and push the event */
wh4530a_irq_handled:
  wh4530a_write_byte(wh4530a, WH4530A_INT_FLAG, 0x00);
	return IRQ_HANDLED;
}

static int wh4530a_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	int ret = -EINVAL;
  u16 data;
	struct wh4530a_data *wh4530a = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = wh4530a_read_word(wh4530a, WH4530A_CHN0_DATA, &data);
			if (ret < 0)
				return ret;
			*val = data;
			ret = IIO_VAL_INT;
			break;
		case IIO_PROXIMITY:
			ret = wh4530a_read_word(wh4530a, WH4530A_PS_DATA, &data);
			if (ret < 0)
				return ret;
			*val = data;
			ret = IIO_VAL_INT;
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = wh4530a_read_word(wh4530a, WH4530A_ALS_TIME, &data);
			if (ret < 0)
				return ret;
			*val = data;
			ret = IIO_VAL_INT;
			break;
		case IIO_PROXIMITY:
			ret = wh4530a_read_word(wh4530a, WH4530A_PS_TIME, &data);
			if (ret < 0)
				return ret;
			*val = data;
			ret = IIO_VAL_INT;
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = wh4530a_read_word(wh4530a, WH4530A_ALS_GAIN, &data);
			if (ret < 0)
				return ret;
			*val = data;
			ret = IIO_VAL_INT;
			break;
		case IIO_PROXIMITY:
			ret = wh4530a_read_word(wh4530a, WH4530A_PS_GAIN, &data);
			if (ret < 0)
				return ret;
			*val = data;
			ret = IIO_VAL_INT;
			break;
		default:
			break;
		}
		break;
  case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_PROXIMITY:
			ret = wh4530a_read_word(wh4530a, WH4530A_PS_OFFSET, &data);
			if (ret < 0)
				return ret;
			*val = data;
			ret = IIO_VAL_INT;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return ret;
}

int wh4530a_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val,
				 int val2,
				 long mask)
{
	int ret = -EINVAL;
  u16 data = val;
	struct wh4530a_data *wh4530a = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_LIGHT:
			ret = wh4530a_write_word(wh4530a, WH4530A_ALS_TIME, data);
			if (ret < 0)
				return ret;
			break;
		case IIO_PROXIMITY:
			ret = wh4530a_write_word(wh4530a, WH4530A_PS_TIME, data & 0x0F);
			if (ret < 0)
				return ret;
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (chan->type) {
		case IIO_LIGHT:
      if (val > 0x05) data = 0x04;
			ret = wh4530a_write_word(wh4530a, WH4530A_ALS_GAIN, data);
			if (ret < 0)
				return ret;
			break;
		case IIO_PROXIMITY:
      if (val > 0x04) data = 0x03;
			ret = wh4530a_write_word(wh4530a, WH4530A_PS_GAIN, data);
			if (ret < 0)
				return ret;
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_PROXIMITY:
			ret = wh4530a_write_word(wh4530a, WH4530A_PS_OFFSET, data);
			if (ret < 0)
				return ret;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return ret;
}


int wh4530a_read_int_config(struct iio_dev *indio_dev,
			 const struct iio_chan_spec *chan,
			 enum iio_event_type type,
			 enum iio_event_direction dir)
{
	int ret = -EINVAL;
	u8 data;
	struct wh4530a_data *wh4530a = iio_priv(indio_dev);
	printk("%s:%d\n", __FUNCTION__, __LINE__);

	ret = wh4530a_read_byte(wh4530a, WH4530A_INT_CTRL, &data);
	if (ret < 0)
		return ret;

	switch (chan->type) {
	case IIO_LIGHT:
		ret = (data & 0x01) ? 1 : 0;
		break;
	case IIO_PROXIMITY:
		ret = (data & 0x02) ? 1 : 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

int wh4530a_write_int_config(struct iio_dev *indio_dev,
			  const struct iio_chan_spec *chan,
			  enum iio_event_type type,
			  enum iio_event_direction dir,
			  int state)
{
	int ret = -EINVAL;
	u8 data;
	struct wh4530a_data *wh4530a = iio_priv(indio_dev);
	printk("%s:%d\n", __FUNCTION__, __LINE__);

	ret = wh4530a_read_byte(wh4530a, WH4530A_INT_CTRL, &data);
	if (ret < 0)
		return ret;

	switch (chan->type) {
	case IIO_LIGHT:
		if (state == 0)
			data &= 0xFE;
		else
			data |= 0x01;
		break;
	case IIO_PROXIMITY:
		if (state == 0)
			data &= 0xFD;
		else
			data |= 0x02;
		break;
	default:
		return -EINVAL;
	}

	ret = wh4530a_write_byte(wh4530a, WH4530A_INT_CTRL, data);

	return ret;
}



int wh4530a_read_thresh(struct iio_dev *indio_dev,
		  const struct iio_chan_spec *chan,
		  enum iio_event_type type,
		  enum iio_event_direction dir,
		  enum iio_event_info info, int *val, int *val2)
{
	int ret = 0;
	unsigned int reg = 0;
	u16 data;
	struct wh4530a_data *wh4530a = iio_priv(indio_dev);

	printk("%s:%d\n", __FUNCTION__, __LINE__);

	switch (chan->type) {
	case IIO_LIGHT:
		if(type != IIO_EV_TYPE_THRESH)
			return -EINVAL;
		if(dir == IIO_EV_DIR_RISING)
			reg = WH4530A_ALS_THR_H;
		else if(dir == IIO_EV_DIR_FALLING)
			reg = WH4530A_ALS_THR_L;
		else
			return -EINVAL;
		break;
	case IIO_PROXIMITY:
		if(type != IIO_EV_TYPE_THRESH)
			return -EINVAL;
		if(dir == IIO_EV_DIR_RISING)
			reg = WH4530A_PS_THR_H;
		else if(dir == IIO_EV_DIR_FALLING)
			reg = WH4530A_PS_THR_L;
		else
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	ret = wh4530a_read_word(wh4530a, reg, &data);
	if (ret < 0)
		return ret;
	*val = data;

	return IIO_VAL_INT;
}


int wh4530a_write_thresh(struct iio_dev *indio_dev,
		   const struct iio_chan_spec *chan,
		   enum iio_event_type type,
		   enum iio_event_direction dir,
		   enum iio_event_info info, int val, int val2)
{
	int ret = 0;
	unsigned int reg = 0;
	u16 data = val;
	struct wh4530a_data *wh4530a = iio_priv(indio_dev);

	printk("%s:%d\n", __FUNCTION__, __LINE__);

	switch (chan->type) {
	case IIO_LIGHT:
		if(type != IIO_EV_TYPE_THRESH)
			return -EINVAL;
		if(dir == IIO_EV_DIR_RISING)
			reg = WH4530A_ALS_THR_H;
		else if(dir == IIO_EV_DIR_FALLING)
			reg = WH4530A_ALS_THR_L;
		else
			return -EINVAL;
	case IIO_PROXIMITY:
		if(type != IIO_EV_TYPE_THRESH)
			return -EINVAL;
		if(dir == IIO_EV_DIR_RISING)
			reg = WH4530A_PS_THR_H;
		else if(dir == IIO_EV_DIR_FALLING)
			reg = WH4530A_PS_THR_L;
		else
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	ret = wh4530a_write_word(wh4530a, reg, data);

	return ret;
}


static const struct iio_info wh4530a_info_not_irq = {
	.driver_module = THIS_MODULE,
	.read_raw = wh4530a_read_raw,
	.write_raw = wh4530a_write_raw,
};

static const struct iio_info wh4530a_info = {
	.driver_module = THIS_MODULE,
	.read_raw = wh4530a_read_raw,
	.write_raw = wh4530a_write_raw,
	.read_event_config = wh4530a_read_int_config,
	.write_event_config = wh4530a_write_int_config,
	.read_event_value = wh4530a_read_thresh,
	.write_event_value = wh4530a_write_thresh,
};

static int wh4530a_probe_dt(struct device *dev,
				struct wh4530a_data *data)
{
	struct device_node *np = dev->of_node;
	u8 prop;

	// default value
	data->wait_time = 0;
	data->als_int_src = 0;
  data->ps_led_current = WH4530A_LED_CURRENT_50MA;
  data->ps_led_pulse_width = 0;
  data->ps_led_pulse_num = 0;

	if (of_property_read_u8(np, "waittime", &prop)) {
		data->wait_time = prop;
	}

	if (of_property_read_u8(np, "als-int-src", &prop)) {
		data->als_int_src = (prop == 0x01) ? 0x01: 0x00;
	}

	if (of_property_read_u8(np, "ps-led-current", &prop)) {
		switch (prop) {
			case  50: data->ps_led_current = WH4530A_LED_CURRENT_50MA;  break;
			case 100: data->ps_led_current = WH4530A_LED_CURRENT_100MA; break;
			case 150: data->ps_led_current = WH4530A_LED_CURRENT_150MA; break;
			case 200: data->ps_led_current = WH4530A_LED_CURRENT_200MA; break;
		}
	}

	if (of_property_read_u8(np, "ps-pulse-width", &prop)) {
		data->ps_led_pulse_width = prop & 0x3F;
	}

	if (of_property_read_u8(np, "ps-pulse-num", &prop)) {
		data->ps_led_pulse_num = prop;
	}

	return 0;
}

static int wh4530a_als_init(struct wh4530a_data *wh4530a)
{
	int ret;

	ret = wh4530a_write_byte(wh4530a, WH4530A_ALS_INT_SRC, wh4530a->als_int_src);
	if (ret) {
		goto err_als_init;
	}

	return 0;

err_als_init:
	return -1;
}

static int wh4530a_ps_init(struct wh4530a_data *wh4530a)
{
	int ret;

	ret = wh4530a_write_byte(wh4530a, WH4530A_PS_LED,
		        wh4530a->ps_led_current | wh4530a->ps_led_pulse_width);
	if (ret) {
		goto err_ps_init;
	}

	ret = wh4530a_write_byte(wh4530a, WH4530A_PS_LED_PULSE,
		                       wh4530a->ps_led_pulse_num);
	if (ret) {
		goto err_ps_init;
	}

	ret = wh4530a_write_byte(wh4530a, WH4530A_PS_OFFSET,
		                       wh4530a->ps_offset & 0xFF);
	if (ret) {
		goto err_ps_init;
	}

	return 0;

err_ps_init:
	return -1;
}

static int wh4530a_init(struct wh4530a_data *wh4530a)
{
	int ret;
  u8 wait_flag = 0;

	ret = wh4530a_write_byte(wh4530a, WH4530A_SYS_CTRL, 0x80);
	if (ret) {
		goto err_init;
	}

  msleep(10);

	ret = wh4530a_write_byte(wh4530a, WH4530A_WAIT_TIME, wh4530a->wait_time);
	if (ret) {
		goto err_init;
	}

  ret = wh4530a_als_init(wh4530a);
  if (ret) {
    goto err_init;
  }

  ret = wh4530a_ps_init(wh4530a);
  if (ret) {
    goto err_init;
  }

	if (wh4530a->wait_time != 0) {
		wait_flag = 0x40;
	}

	ret = wh4530a_write_byte(wh4530a, WH4530A_INT_CTRL, 0x00);
	if (ret) {
		goto err_init;
	}

	ret = wh4530a_write_byte(wh4530a, WH4530A_SYS_CTRL, 0x03 | wait_flag);
	if (ret) {
		goto err_init;
	}

	return 0;

err_init:
  return -1;
}

static int wh4530a_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct wh4530a_data *wh4530a;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*wh4530a));
	if (!indio_dev)
		return -ENOMEM;

	wh4530a = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	wh4530a->client = client;

  wh4530a_probe_dt(&client->dev, wh4530a);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
					   NULL,
					   &wh4530a_event_handler,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "wh4530a_event",
					   indio_dev);
		if (ret) {
			dev_err(&client->dev, "irq request error %d\n", -ret);
			return ret;
		}
    indio_dev->info = &wh4530a_info;
  } else {
	  dev_info(&client->dev, "not irq\n");
    indio_dev->info = &wh4530a_info_not_irq;
  }

	ret = wh4530a_init(wh4530a);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to init WH4530A\n");
		return ret;
	}

	dev_info(&client->dev, "WH4530A Ambient light/proximity sensor, address 0x38\n");

	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = wh4530a_channels;
	indio_dev->num_channels = ARRAY_SIZE(wh4530a_channels);
	indio_dev->name = WH4530A_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id wh4530a_id[] = {
	{ "wh4530a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wh4530a_id);

static const struct of_device_id wh4530a_of_match[] = {
	{ .compatible = "wonhome,wh4530a" },
	{ }
};
MODULE_DEVICE_TABLE(of, sht4x_of_match);

static struct i2c_driver wh4530a_driver = {
	.driver = {
		.name   = WH4530A_DRV_NAME,
		.of_match_table = wh4530a_of_match,
	},
	.probe  = wh4530a_probe,
	.id_table = wh4530a_id,
};
module_i2c_driver(wh4530a_driver);

MODULE_AUTHOR("xinzhiting <xinzhiting@zlg.cn>");
MODULE_DESCRIPTION("Wonhome WH4530A proximity/ambient light sensor driver");
MODULE_LICENSE("GPL");
