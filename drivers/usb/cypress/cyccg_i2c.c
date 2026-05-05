/*
 * Cypress USB Type-C and PD Controller Driver I2C Interface
 *
 * Author: Dudley Du <dudl@cypress.com>
 *
 * Copyright (C) 2016 Cypress Semiconductor, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/of.h>
#include "cyccg_core.h"

#define I2C_RETRY_COUNT		5

static DEFINE_MUTEX(i2c_rw_buf_lock);
static u8 i2c_rw_buf[HPI_MAX_REG_RW_SIZE + HPI_MAX_REG_ADDR_SIZE];

/**
 * detect - Execute i2c BUS protocol operations of device detecting
 * &dev: Handle to the I2C Slave device
 *
 * This detects the exists of the device, return a negative errno code
 * if not found or failed, else zero on success.
 */
static int cyccg_i2c_detect(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_adapter *adapter = client->adapter;
	union i2c_smbus_data dummy;
	struct i2c_msg msg;
	int ret;

	ret = i2c_smbus_xfer(client->adapter, client->addr, 0,
			      I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &dummy);
	if (ret) {
		/*
		 * For CCG3 and CCG4 devices which following HPIv2 interface
		 * support read/write operation with two byte I2C address, so
		 * for the i2c_smbus_xfer() operation which only support one
		 * byte I2C address operation, if the I2C register
		 * map offset was not set to 0x0000 (DEVICE_MODE), the address
		 * detect operation will fail even the CCG3/CCG4 device
		 * has been attached to the bus.
		 * So to avoid this miss-detection issue, do the write operation
		 * with two byte data set as offset=0x0000 to detect the
		 * attached device.
		 * If write success, there must be an attached device on the
		 * bus. And for those HPIv1 CCGx devices, since the DEVICE_MODE
		 * register is always read only, so the write operation with the
		 * value of 0x00 has no side-effect on it. It's safe.
		 */
		memset(&dummy, 0, sizeof(dummy));
		msg.addr = client->addr;
		msg.flags = client->flags | I2C_M_TEN;
		msg.len = HPI_V2_REG_ADDR_SIZE;
		msg.buf = (u8 *)&dummy;
		ret = i2c_transfer(adapter, &msg, 1);
		if (ret < 1)
			ret = -ENODEV;
		else
			ret = 0;
	}

	return ret;
}

/**
 * read - Execute i2c bus protocol operations to read data
 * @dev: Handle to the I2C Slave device
 * @reg_addr_bytes: the register address bytes based on HPI version
 * @buf: Points to the buffer for return read data
 * @size: Bytes of the data should be read from the device
 * @addr: The start of the register address for reading
 *
 * This executes block read operation on the bus, return a negative
 * errno code on failure, else zero on success.
 */
static int cyccg_i2c_read(struct device *dev, size_t reg_addr_bytes,
			  void *buf, size_t size, u32 addr)
{
	struct i2c_client *client = to_i2c_client(dev);
	int retries = I2C_RETRY_COUNT;
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags & I2C_M_TEN,
			.len = reg_addr_bytes,
			.buf = i2c_rw_buf,
		},
		{
			.addr = client->addr,
			.flags = (client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = size,
			.buf = buf,
		},
	};

	mutex_lock(&i2c_rw_buf_lock);

	if (reg_addr_bytes == HPI_V1_REG_ADDR_SIZE)
		i2c_rw_buf[0] = (u8)addr;
	else
		put_unaligned_le16((u16)addr, i2c_rw_buf);

	while (retries--) {
		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs)) {
			ret = 0;
			break;
		}

		if (retries) {
			usleep_range(2000, 4000);
			continue;
		}

		ret = (ret < 0) ? ret : -EIO;
	}

	mutex_unlock(&i2c_rw_buf_lock);
	return ret;
}

/**
 * write - Execute i2c bus protocol operations to write data
 * &dev: Handle to the I2C Slave device
 * @reg_addr_bytes: the register address bytes based on HPI version
 * @buf: Points to the buffer contains the data need to be written
 * @size: Bytes of the data contained in the @buf memory buffer
 * @addr: The start of the register address for writing
 *
 * This executes block write operation on the bus, return a negative
 * errno code on failure, else zero on success.
 */
static int cyccg_i2c_write(struct device *dev, size_t reg_addr_bytes,
			   void *buf, size_t size, u32 addr)
{
	struct i2c_client *client = to_i2c_client(dev);
	int retries = I2C_RETRY_COUNT;
	int write_count;
	u8 *data;
	int ret;

	mutex_lock(&i2c_rw_buf_lock);

	if (reg_addr_bytes == HPI_V1_REG_ADDR_SIZE) {
		i2c_rw_buf[0] = (u8)addr;
		data = &i2c_rw_buf[HPI_V1_REG_ADDR_SIZE];
		write_count = size + HPI_V1_REG_ADDR_SIZE;
	} else {
		put_unaligned_le16((u16)addr, i2c_rw_buf);
		data = &i2c_rw_buf[HPI_V2_REG_ADDR_SIZE];
		write_count = size + HPI_V2_REG_ADDR_SIZE;
	}

	memcpy(data, buf, size);
	while (retries--) {
		ret = i2c_master_send(client, i2c_rw_buf, write_count);
		if (ret == write_count) {
			ret = 0;
			break;
		}

		if (retries) {
			usleep_range(2000, 4000);
			continue;
		}

		ret = (ret < 0) ? ret : -EIO;
	}

	mutex_unlock(&i2c_rw_buf_lock);
	return ret;
}

static struct ccg_bus_operations cyccg_i2c_ops = {
	.bustype = BUS_I2C,

	.detect = cyccg_i2c_detect,
	.read = cyccg_i2c_read,
	.write = cyccg_i2c_write,
};

static int cyccg_i2c_probe(struct i2c_client *client,
			   const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "%s: not a supported i2c adapter\n", __func__);
		return -EIO;
	}

	cyccg_i2c_ops.bus = client->adapter->nr;
	cyccg_i2c_ops.addr = client->addr;
	err = cyccg_probe(&client->dev, client->irq, &cyccg_i2c_ops);
	if (err) {
		dev_err(dev, "%s: error, cyccg_probe, %d\n", __func__, err);
		return err;
	}

	return 0;
}

static int cyccg_i2c_remove(struct i2c_client *client)
{
	return cyccg_remove(&client->dev);
}

static const struct i2c_device_id cyccg_i2c_id_table[] = {
	{ CYCCG_I2C_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, cyccg_i2c_id_table);

#ifdef CONFIG_OF
static const struct of_device_id cyccg_i2c_of_match[] = {
	{ .compatible = "cypress,cyccg_i2c" },
	{ },
};
#endif

static struct i2c_driver cyccg_i2c_driver = {
	.driver = {
		.name = CYCCG_I2C_NAME,
		.pm = &cyccg_pm_ops,
		.of_match_table = of_match_ptr(cyccg_i2c_of_match),
	},

	.probe = cyccg_i2c_probe,
	.remove = cyccg_i2c_remove,
	.id_table = cyccg_i2c_id_table,
};
module_i2c_driver(cyccg_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dudley Du <dudl@cypress.com>");
MODULE_DESCRIPTION("Cypress USB Type-C and PD Controller I2C Driver");