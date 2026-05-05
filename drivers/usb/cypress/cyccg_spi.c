/*
 * Cypress USB Type-C and PD Controller Driver SPI Interface
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
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/of.h>
#include "cyccg_core.h"

#define CCG_SPI_BITS_PER_WORD	8
#define SPI_RETRY_COUNT		5

/**
 * detect - Execute SPI BUS protocol operations of device detecting
 * &dev: Handle to the SPI Slave device
 *
 * This detects the exists of the device, return a negative errno code
 * if not found or failed, else zero on success.
 */
static int cyccg_spi_detect(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 dummy[HPI_MAX_REG_ADDR_SIZE];

	memset(dummy, 0, sizeof(dummy));
	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);

	xfer[0].tx_buf = dummy;
	xfer[0].len = 1;
	xfer[1].rx_buf = dummy;
	xfer[1].len = 1;

	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);

	return spi_sync(spi, &msg);
}

/**
 * read - Execute SPI bus protocol operations to read data
 * @dev: Handle to the SPI Slave device
 * @reg_addr_bytes: the register address bytes based on HPI version
 * @buf: Points to the buffer for return read data
 * @size: Bytes of the data should be read from the device
 * @addr: The start of the register address for reading
 *
 * This executes block read operation on the bus, return a negative
 * errno code on failure, else zero on success.
 */
static int cyccg_spi_read(struct device *dev, size_t reg_addr_bytes,
			  void *buf, size_t size, u32 addr)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 reg_addr[HPI_MAX_REG_ADDR_SIZE];
	int retries = SPI_RETRY_COUNT;
	int err;

	while (retries--) {
		if (reg_addr_bytes == HPI_V1_REG_ADDR_SIZE)
			reg_addr[0] = (u8)addr;
		else
			put_unaligned_le16((u16)addr, reg_addr);

		memset(xfer, 0, sizeof(xfer));
		spi_message_init(&msg);

		xfer[0].tx_buf = reg_addr;
		xfer[0].len = reg_addr_bytes;
		xfer[1].rx_buf = buf;
		xfer[1].len = size;

		spi_message_add_tail(&xfer[0], &msg);
		spi_message_add_tail(&xfer[1], &msg);

		err = spi_sync(spi, &msg);
		if (!err)
			break;

		if (retries) {
			usleep_range(2000, 4000);
			continue;
		}

		err = (err < 0) ? err : -EIO;
	}

	return err;
}

/**
 * write - Execute SPI bus protocol operations to write data
 * &dev: Handle to the SPI Slave device
 * @reg_addr_bytes: the register address bytes based on HPI version
 * @buf: Points to the buffer contains the data need to be written
 * @size: Bytes of the data contained in the @buf memory buffer
 * @addr: The start of the register address for writing
 *
 * This executes block write operation on the bus, return a negative
 * errno code on failure, else zero on success.
 */
static int cyccg_spi_write(struct device *dev, size_t reg_addr_bytes,
			   void *buf, size_t size, u32 addr)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 reg_addr[HPI_MAX_REG_ADDR_SIZE];
	int retries = SPI_RETRY_COUNT;
	int err;

	while (retries--) {
		if (reg_addr_bytes == HPI_V1_REG_ADDR_SIZE)
			reg_addr[0] = (u8)addr;
		else
			put_unaligned_le16((u16)addr, reg_addr);

		memset(xfer, 0, sizeof(xfer));
		spi_message_init(&msg);

		xfer[0].tx_buf = reg_addr;
		xfer[0].len = reg_addr_bytes;
		xfer[1].tx_buf = buf;
		xfer[1].len = size;

		spi_message_add_tail(&xfer[0], &msg);
		spi_message_add_tail(&xfer[1], &msg);

		err = spi_sync(spi, &msg);
		if (!err)
			break;

		if (retries) {
			usleep_range(2000, 4000);
			continue;
		}

		err = (err < 0) ? err : -EIO;
	}

	return err;
}

static struct ccg_bus_operations cyccg_spi_ops = {
	.bustype = BUS_SPI,

	.detect = cyccg_spi_detect,
	.read = cyccg_spi_read,
	.write = cyccg_spi_write,
};

static int cyccg_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int err;

	/*
	 * TODO (Customer):
	 * Set the SPI based on physical board/system desgin if different.
	 */
	spi->bits_per_word = CCG_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;

	err = spi_setup(spi);
	if (err) {
		dev_err(dev, "%s: spi_setup error, %d\n", __func__, err);
		return err;
	}

	cyccg_spi_ops.bus = (u16)spi->master->bus_num;
	cyccg_spi_ops.addr = (u32)spi->chip_select;
	err = cyccg_probe(dev, spi->irq, &cyccg_spi_ops);
	if (err) {
		dev_err(dev, "%s: cyccg_probe error, %d\n", __func__, err);
		return err;
	}

	return 0;
}

static int cyccg_spi_remove(struct spi_device *spi)
{
	return cyccg_remove(&spi->dev);
}

static const struct spi_device_id cyccg_spi_id_table[] = {
	{ CYCCG_SPI_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, cyccg_spi_id_table);

#ifdef CONFIG_OF
static const struct of_device_id cyccg_spi_of_match[] = {
	{ .compatible = "cypress,cyccg_spi" },
	{ },
};
#endif

static struct spi_driver cyccg_spi_driver = {
	.driver = {
		.name = CYCCG_SPI_NAME,
		.pm = &cyccg_pm_ops,
		.of_match_table = of_match_ptr(cyccg_spi_of_match),
	},

	.probe = cyccg_spi_probe,
	.remove = cyccg_spi_remove,
	.id_table = cyccg_spi_id_table,
};
module_spi_driver(cyccg_spi_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dudley Du <dudl@cypress.com>");
MODULE_DESCRIPTION("Cypress USB Type-C and PD Controller SPI Driver");
