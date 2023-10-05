// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright (c) Linumiz 2021
 *
 * sht4x.c - Linux hwmon driver for SHT4x Temperature and Humidity sensor
 *
 * Author: Navin Sankar Velliangiri <navin@linumiz.com>
 */

#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>

/*
 * Poll intervals (in milliseconds)
 */
#define SHT4X_MIN_POLL_INTERVAL	2000

/*
 * I2C command delays (in microseconds)
 */
#define SHT4X_MEAS_DELAY_HPM	8200	/* see t_MEAS,h in datasheet */
#define SHT4X_DELAY_EXTRA	10000

/*
 * Command Bytes
 */
#define SHT4X_CMD_MEASURE_HPM	0b11111101
#define SHT4X_CMD_RESET		0b10010100

#define SHT4X_CMD_LEN		1
#define SHT4X_CRC8_LEN		1
#define SHT4X_WORD_LEN		2
#define SHT4X_RESPONSE_LENGTH	6
#define SHT4X_CRC8_POLYNOMIAL	0x31
#define SHT4X_CRC8_INIT		0xff
#define SHT4X_MIN_TEMPERATURE	-45000
#define SHT4X_MAX_TEMPERATURE	125000
#define SHT4X_MIN_HUMIDITY	0
#define SHT4X_MAX_HUMIDITY	100000

static u8 sht4x_crc8_table[] = {
	  0,  49,  98,  83, 196, 245, 166, 151,
	185, 136, 219, 234, 125,  76,  31,  46,
	 67, 114,  33,  16, 135, 182, 229, 212,
	250, 203, 152, 169,  62,  15,  92, 109,
	134, 183, 228, 213,  66, 115,  32,  17,
	 63,  14,  93, 108, 251, 202, 153, 168,
	197, 244, 167, 150,   1,  48,  99,  82,
	124,  77,  30,  47, 184, 137, 218, 235,
	 61,  12,  95, 110, 249, 200, 155, 170,
	132, 181, 230, 215,  64, 113,  34,  19,
	126,  79,  28,  45, 186, 139, 216, 233,
	199, 246, 165, 148,   3,  50,  97,  80,
	187, 138, 217, 232, 127,  78,  29,  44,
	  2,  51,  96,  81, 198, 247, 164, 149,
	248, 201, 154, 171,  60,  13,  94, 111,
	 65, 112,  35,  18, 133, 180, 231, 214,
	122,  75,  24,  41, 190, 143, 220, 237,
	195, 242, 161, 144,   7,  54, 101,  84,
	 57,   8,  91, 106, 253, 204, 159, 174,
	128, 177, 226, 211,  68, 117,  38,  23,
	252, 205, 158, 175,  56,   9,  90, 107,
	 69, 116,  39,  22, 129, 176, 227, 210,
	191, 142, 221, 236, 123,  74,  25,  40,
	  6,  55, 100,  85, 194, 243, 160, 145,
	 71, 118,  37,  20, 131, 178, 225, 208,
	254, 207, 156, 173,  58,  11,  88, 105,
	  4,  53, 102,  87, 192, 241, 162, 147,
	189, 140, 223, 238, 121,  72,  27,  42,
	193, 240, 163, 146,   5,  52, 103,  86,
	120,  73,  26,  43, 188, 141, 222, 239,
	130, 179, 224, 209,  70, 119,  36,  21,
	 59,  10,  89, 104, 255, 206, 157, 172,
};

/**
 * struct sht4x_data - All the data required to operate an SHT4X chip
 * @client: the i2c client associated with the SHT4X
 * @lock: a mutex that is used to prevent parallel access to the i2c client
 * @update_interval: the minimum poll interval
 * @last_updated: the previous time that the SHT4X was polled
 * @temperature: the latest temperature value received from the SHT4X
 * @humidity: the latest humidity value received from the SHT4X
 */
struct sht4x_data {
	struct i2c_client	*client;
	struct mutex		lock;	/* atomic read data updates */
	bool			valid;	/* validity of fields below */
	long			update_interval;	/* in milli-seconds */
	long			last_updated;	/* in jiffies */
	s32			temperature;
	s32			humidity;
};

u8 sht4x_crc8(u8 *pdata, size_t nbytes)
{
  u8 crc = SHT4X_CRC8_INIT;
	/* loop over the buffer data */
	while (nbytes-- > 0)
		crc = sht4x_crc8_table[(crc ^ *pdata++) & 0xff];

	return crc;
}

/**
 * sht4x_read_values() - read and parse the raw data from the SHT4X
 * @sht4x_data: the struct sht4x_data to use for the lock
 * Return: 0 if successful, -ERRNO if not
 */
static int sht4x_read_values(struct sht4x_data *data)
{
	int ret = 0;
	u16 t_ticks, rh_ticks;
	unsigned long next_update;
	struct i2c_client *client = data->client;
	u8 crc;
	u8 cmd[SHT4X_CMD_LEN] = {SHT4X_CMD_MEASURE_HPM};
	u8 raw_data[SHT4X_RESPONSE_LENGTH] = {0};

	mutex_lock(&data->lock);
	next_update = data->last_updated +
		      msecs_to_jiffies(data->update_interval);

	if (data->valid && time_before_eq(jiffies, next_update))
		goto unlock;

	ret = i2c_master_send(client, cmd, SHT4X_CMD_LEN);
	if (ret < 0)
		goto unlock;

	usleep_range(SHT4X_MEAS_DELAY_HPM, SHT4X_MEAS_DELAY_HPM + SHT4X_DELAY_EXTRA);

	ret = i2c_master_recv(client, raw_data, SHT4X_RESPONSE_LENGTH);
	if (ret != SHT4X_RESPONSE_LENGTH) {
		if (ret >= 0)
			ret = -ENODATA;
		goto unlock;
	}

	t_ticks = raw_data[0] << 8 | raw_data[1];
	rh_ticks = raw_data[3] << 8 | raw_data[4];

  dev_err(&client->dev, "data[0]: 0x%02x, data[1]: 0x%02x, data[2]: 0x%02x, data[3]: 0x%02x, data[4]: 0x%02x, data[5]: 0x%02x\n",
            raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5]);

	crc = sht4x_crc8(&raw_data[0], SHT4X_WORD_LEN);
	if (crc != raw_data[2]) {
		dev_err(&client->dev, "data integrity check failed\n");
		ret = -EIO;
		goto unlock;
	}

//	crc = sht4x_crc8(&raw_data[3], SHT4X_WORD_LEN);
//	if (crc != raw_data[5]) {
//		dev_err(&client->dev, "data integrity check failed\n");
//		ret = -EIO;
//		goto unlock;
//	}

	data->temperature = ((21875 * (int32_t)t_ticks) >> 13) - 45000;
	data->humidity = ((15625 * (int32_t)rh_ticks) >> 13) - 6000;
	data->last_updated = jiffies;
	data->valid = true;
	ret = 0;

unlock:
	mutex_unlock(&data->lock);
	return ret;
}


ssize_t sht4x_store_interval(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct sht4x_data *sht4x = dev_get_drvdata(dev);
  int val;
  int ret;

  ret = kstrtoint(buf, count, &val);
  if (ret) {
    return -1;
  }

	sht4x->update_interval = clamp_val(val, SHT4X_MIN_POLL_INTERVAL, UINT_MAX);

	return 0;
}

/* sht4x_interval_read() - read the minimum poll interval in milliseconds */
static ssize_t sht4x_show_interval(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct sht4x_data *sht4x = dev_get_drvdata(dev);

	return sprintf(buf, "%ld\n", sht4x->update_interval);
}

/* sht4x_temperature1_read() - read the temperature in millidegrees */
static ssize_t sht4x_show_temperature(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct sht4x_data *sht4x = dev_get_drvdata(dev);
	int ret;

	ret = sht4x_read_values(sht4x);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", sht4x->temperature);
}

/* sht4x_humidity1_read() - read a relative humidity in millipercent */
static ssize_t sht4x_show_humidity(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct sht4x_data *sht4x = dev_get_drvdata(dev);
	int ret;

	ret = sht4x_read_values(sht4x);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", sht4x->humidity);
}


/* sysfs attributes */
static SENSOR_DEVICE_ATTR(interval, S_IRUGO | S_IWUSR, sht4x_show_interval,
	sht4x_store_interval, 0);
static SENSOR_DEVICE_ATTR(temp, S_IRUGO, sht4x_show_temperature,
	NULL, 0);
static SENSOR_DEVICE_ATTR(humidity, S_IRUGO, sht4x_show_humidity,
	NULL, 0);

static struct attribute *sht4x_attrs[] = {
	&sensor_dev_attr_interval.dev_attr.attr,
	&sensor_dev_attr_temp.dev_attr.attr,
	&sensor_dev_attr_humidity.dev_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(sht4x);

static int sht4x_probe(struct i2c_client *client,
		       const struct i2c_device_id *sht4x_id)
{
	struct device *device = &client->dev;
	struct device *hwmon_dev;
	struct sht4x_data *sht4x;
	u8 cmd[] = {SHT4X_CMD_RESET};
	int ret;

	/*
	 * we require full i2c support since the sht4x uses multi-byte read and
	 * writes as well as multi-byte commands which are not supported by
	 * the smbus protocol
	 */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	sht4x = devm_kzalloc(device, sizeof(*sht4x), GFP_KERNEL);
	if (!sht4x)
		return -ENOMEM;

	sht4x->update_interval = SHT4X_MIN_POLL_INTERVAL;
	sht4x->client = client;

	mutex_init(&sht4x->lock);

	ret = i2c_master_send(client, cmd, SHT4X_CMD_LEN);
	if (ret < 0)
		return ret;
	if (ret != SHT4X_CMD_LEN)
		return -EIO;

	hwmon_dev = devm_hwmon_device_register_with_groups(device, client->name,
							   sht4x, sht4x_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id sht4x_id[] = {
	{ "sht4x", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sht4x_id);

static const struct of_device_id sht4x_of_match[] = {
	{ .compatible = "sensirion,sht4x" },
	{ }
};
MODULE_DEVICE_TABLE(of, sht4x_of_match);

static struct i2c_driver sht4x_driver = {
	.driver = {
		.name = "sht4x",
		.of_match_table = sht4x_of_match,
	},
	.probe		= sht4x_probe,
	.id_table	= sht4x_id,
};

module_i2c_driver(sht4x_driver);

MODULE_AUTHOR("Navin Sankar Velliangiri <navin@linumiz.com>");
MODULE_DESCRIPTION("Sensirion SHT4x humidity and temperature sensor driver");
MODULE_LICENSE("GPL v2");
