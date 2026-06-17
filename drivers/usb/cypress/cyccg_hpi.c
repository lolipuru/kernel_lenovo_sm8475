/*
 * Cypress USB Type-C and PD Controller Driver
 *
 * Author: Dudley Du <dudl@cypress.com>
 *
 * Copyright (C) 2016 Cypress Semiconductor, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include "cyccg_core.h"
#include "linux/delay.h"
#include "linux/interrupt.h"

#define hpi_err(fmt, ...)	ccg_err(fmt, ##__VA_ARGS__)
#define hpi_warn(fmt, ...)	ccg_warn(fmt, ##__VA_ARGS__)
#define hpi_info(fmt, ...)	ccg_info(fmt, ##__VA_ARGS__)
#define hpi_dbg(fmt, ...)	ccg_dbg(fmt, ##__VA_ARGS__)
#define hpi_vdbg(fmt, ...)	ccg_vdbg(fmt, ##__VA_ARGS__)
#define hpi_dump(buf, size, fmt, ...)		\
	ccg_dump(fmt, buf, size, ##__VA_ARGS__)

#define hpi_port_err(fmt, port, ...)	port_err(fmt, port, ##__VA_ARGS__)
#define hpi_port_warn(fmt, port, ...)	port_warn(fmt, port, ##__VA_ARGS__)
#define hpi_port_info(fmt, port, ...)	port_info(fmt, port, ##__VA_ARGS__)
#define hpi_port_dbg(fmt, port, ...)	port_dbg(fmt, port, ##__VA_ARGS__)
#define hpi_port_vdbg(fmt, port, ...)	port_vdbg(fmt, port, ##__VA_ARGS__)
#define hpi_port_dump(fmt, port, buf, size, ...)	\
	port_dump(fmt, port, buf, size, ##__VA_ARGS__)

static inline void hpi_cmd_start_timer(struct hpi_device *hpidev,
				       unsigned long timeout);
static inline void hpi_cmd_stop_timer(struct hpi_device *hpidev);

static inline bool is_valid_reg_range(struct hpi_device *hpidev,
		u32 offset, size_t reg_size, void *buf, size_t rw_size)
{
	if (!reg_size || !buf || !rw_size || rw_size > reg_size ||
			(offset + reg_size) > hpidev->reg_map_size)
		return false;

	return true;
}

int hpi_device_read(struct hpi_device *hpidev,
		    u32 offset, size_t reg_size, void *buf, size_t buf_size)
{
	struct cyccg *cyccg = hpidev->cyccg;
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct ccg_bus_operations *bus_ops = cyccg->bus_ops;
	u32 addr = hpidev->reg_base_addr + offset;
	int err = -EIO;

	if (!is_valid_reg_range(hpidev, offset, reg_size, buf, buf_size)) {
		hpi_err("invalid input parameteres\n");
		return -EINVAL;
	}

	buf_size = min(reg_size, buf_size);
	if (likely(bus_ops && bus_ops->read))
		err = bus_ops->read(cyccg->dev, HPI_REG_ADDR_SIZE(hpi_ver),
					buf, buf_size, addr);

	return err;
}

int hpi_device_write(struct hpi_device *hpidev,
		     u32 offset, size_t reg_size, void *buf, size_t buf_size)
{
	struct cyccg *cyccg = hpidev->cyccg;
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct ccg_bus_operations *bus_ops = cyccg->bus_ops;
	u32 addr = hpidev->reg_base_addr + offset;
	int err = -EIO;

	if (!is_valid_reg_range(hpidev, offset, reg_size, buf, buf_size)) {
		hpi_err("invalid input parameteres\n");
		return -EINVAL;
	}

	buf_size = min(reg_size, buf_size);
	if (likely(bus_ops && bus_ops->write)) {
		cyccg_start_polling_timer(cyccg, false);
		err = bus_ops->write(cyccg->dev, HPI_REG_ADDR_SIZE(hpi_ver),
					buf, buf_size, addr);
	}

	return err;
}

/*
 * Directly Raw read/write for Device configuration and Status registers.
 */
static int _hpi_rw_data_memory(struct hpi_device *hpidev,
			       void *buf, size_t size,
			       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset;
	size_t reg_size;

	if (IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		if (rw_mode == HPI_READ) {
			reg_offset =
				HPI_REG_OFFSET_OF(DEV_READ_DATA_MEM, hpi_ver);
			reg_size = HPI_REG_SIZE_OF(DEV_READ_DATA_MEM, hpi_ver);
		} else {
			reg_offset =
				HPI_REG_OFFSET_OF(DEV_WRITE_DATA_MEM, hpi_ver);
			reg_size = HPI_REG_SIZE_OF(DEV_WRITE_DATA_MEM, hpi_ver);
		}
	} else {
		if (rw_mode == HPI_READ) {
			reg_offset =
				HPI_REG_OFFSET_OF(PD_READ_DATA_MEM, hpi_ver);
			reg_size = HPI_REG_SIZE_OF(PD_READ_DATA_MEM, hpi_ver);
		} else {
			reg_offset =
				HPI_REG_OFFSET_OF(PD_WRITE_DATA_MEM, hpi_ver);
			reg_size = HPI_REG_SIZE_OF(PD_WRITE_DATA_MEM, hpi_ver);
		}

	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_flash_memory(struct hpi_device *hpidev,
				void *buf, size_t size,
				enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(FLASH_RW, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(FLASH_RW, hpi_ver);

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_response_head(struct hpi_device *hpidev,
				 void *buf, size_t size,
				 enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset;
	size_t reg_size;

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for RESPONSE register\n");
		return -EINVAL;
	}

	if (IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		reg_offset = HPI_REG_OFFSET_OF(DEVICE_RESPONSE, hpi_ver);
		reg_size = HPI_REG_SIZE_OF(DEVICE_RESPONSE, hpi_ver);
	} else if (IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		reg_offset = HPI_REG_OFFSET_OF(PD_RESPONSE, hpi_ver);
		reg_size = HPI_REG_SIZE_OF(PD_RESPONSE, hpi_ver);
	} else {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}


static int _hpi_rw_device_mode(struct hpi_device *hpidev,
			       void *buf, size_t size, enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(DEVICE_MODE, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(DEVICE_MODE, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for DEVICE_MODE\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_boot_mode_reason(struct hpi_device *hpidev,
				    void *buf, size_t size,
				    enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(BOOT_MODE_REASON, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(BOOT_MODE_REASON, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for BOOT_MODE_REASON\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_silicon_id(struct hpi_device *hpidev,
			      void *buf, size_t size,
			      enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(READ_SILICON_ID, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(READ_SILICON_ID, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for READ_SILICON_ID\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_boot_loader_last_row(struct hpi_device *hpidev,
					void *buf, size_t size,
					enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(BOOT_LOADER_LAST_ROW, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(BOOT_LOADER_LAST_ROW, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for BOOT_LOADER_LAST_ROW\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_intr_reg(struct hpi_device *hpidev,
			    void *buf, size_t size,
			    enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(INTR_REG, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(INTR_REG, hpi_ver);

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_jump_to_boot(struct hpi_device *hpidev,
				void *buf, size_t size,
				enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(JUMP_TO_BOOT, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(JUMP_TO_BOOT, hpi_ver);

	if (rw_mode == HPI_READ) {
		hpi_err("invalid RW ops, WO for JUMP_TO_BOOT\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_reset(struct hpi_device *hpidev,
			 void *buf, size_t size,
			 enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(RESET, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(RESET, hpi_ver);

	if (rw_mode == HPI_READ) {
		hpi_err("invalid RW ops, WO for RESET\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_enter_flashing_mode(struct hpi_device *hpidev,
				       void *buf, size_t size,
				       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(ENTER_FLASHING_MODE, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(ENTER_FLASHING_MODE, hpi_ver);

	if (rw_mode == HPI_READ) {
		hpi_err("invalid RW ops, WO for ENTER_FLASHING_MODE\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_validate_fw(struct hpi_device *hpidev,
			       void *buf, size_t size, enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(VALIDATE_FW, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(VALIDATE_FW, hpi_ver);

	if (rw_mode == HPI_READ) {
		hpi_err("invalid RW ops, WO for VALIDATE_FW\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_flash_row(struct hpi_device *hpidev,
			     void *buf, size_t size, enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(FLASH_ROW_READ_WRITE, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(FLASH_ROW_READ_WRITE, hpi_ver);

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_all_version(struct hpi_device *hpidev,
			       void *buf, size_t size, enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(READ_ALL_VERSION, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(READ_ALL_VERSION, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for READ_ALL_VERSION\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_fw2_version(struct hpi_device *hpidev,
			       void *buf, size_t size,
			       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(FW2_VERSION, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(FW2_VERSION, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for FW2_VERSION\n");
		return -EINVAL;
	}

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_err("HPIv1 and CCG1/CCG2 not support FW2_VERSION reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_fw_binary_location(struct hpi_device *hpidev,
				      void *buf, size_t size,
				      enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(FW_BINARY_LOCATION, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(FW_BINARY_LOCATION, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for FW_BINARY_LOCATION\n");
		return -EINVAL;
	}

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_err("HPIv1 and CCG1/CCG2 not support FW2_VERSION reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_pdport_enable(struct hpi_device *hpidev,
				 void *buf, size_t size,
				 enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PDPORT_ENABLE, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(PDPORT_ENABLE, hpi_ver);

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_err("HPIv1 and CCG1/CCG2 not support FW2_VERSION reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_sleep_ctrl(struct hpi_device *hpidev,
			      void *buf, size_t size,
			      enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(SLEEP_CTRL, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(SLEEP_CTRL, hpi_ver);

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_err("HPIv1 and CCG1/CCG2 not support FW2_VERSION reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_battery_stat(struct hpi_device *hpidev,
				void *buf, size_t size,
				enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(BATTERY_STAT, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(BATTERY_STAT, hpi_ver);

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_err("HPIv1 and CCG1/CCG2 not support FW2_VERSION reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_app_priority(struct hpi_device *hpidev,
				void *buf, size_t size,
				enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(SET_APP_PRIORITY, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(SET_APP_PRIORITY, hpi_ver);

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_err("HPIv1 and CCG1/CCG2 not support FW2_VERSION reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_read_customer_info_reg(struct hpi_device *hpidev,
					  void *buf, size_t size,
					  enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(READ_CUSTOMER_INFO, hpi_ver);
	size_t reg_size = HPI_REG_SIZE_OF(READ_CUSTOMER_INFO, hpi_ver);

	if (rw_mode == HPI_READ) {
		hpi_err("invalid RW ops, WO for READ_CUSTOMER_INFO\n");
		return -EINVAL;
	}

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_err("CCG1/CCG2 not support READ_CUSTOMER_INFO reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_rw_customer_info(struct hpi_device *hpidev,
				 void *buf, size_t size,
				 enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_dbg("HPIv1 and CCG1/CCG2 not support FW2_VERSION reg\n");
		return -EINVAL;
	}

	return _hpi_rw_flash_memory(hpidev, buf, size, rw_mode);
}

/*
 * Directly Raw read/write for PD Policy/Status registers.
 */
static int _hpi_port_rw_data_memory(struct hpi_device *hpidev,
				    void *buf, size_t size,
				    enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset;
	size_t reg_size;

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ) {
		reg_offset = HPI_REG_OFFSET_OF(PD_READ_DATA_MEM, hpi_ver);
		reg_size = HPI_REG_SIZE_OF(PD_READ_DATA_MEM, hpi_ver);
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	}

	reg_offset = HPI_REG_OFFSET_OF(PD_WRITE_DATA_MEM, hpi_ver);
	reg_size = HPI_REG_SIZE_OF(PD_WRITE_DATA_MEM, hpi_ver);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_vdm_data(struct hpi_device *hpidev,
				    void *buf, size_t size,
				enum hpi_rw_mode rw_mode)
{
	if (size > VDM_MAX_MSG_SIZE || size % VDM_VDO_OBJ_SIZE) {
		hpi_err("invalid parameters, VDM data size=%zu\n", size);
		return -EINVAL;
	}

	return _hpi_port_rw_data_memory(hpidev, buf, size, rw_mode);
}

static int _hpi_port_rw_vdm_ctrl(struct hpi_device *hpidev,
				 void *buf, size_t size,
				 enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_VDM_CONTROL, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_VDM_CONTROL, hpi_ver);

	if (rw_mode == HPI_READ) {
		hpi_err("invalid RW ops, WO for VDM_CONTROL\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_effective_source_pdo_mask(struct hpi_device *hpidev,
						  void *buf, size_t size,
						  enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset =
		HPI_REG_OFFSET_OF(PORT_EFFECTIVE_SOURCE_PDO_MASK, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_EFFECTIVE_SOURCE_PDO_MASK, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for EFFECTIVE_SOURCE_PDO_MASK\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_effective_sink_pdo_mask(struct hpi_device *hpidev,
						void *buf, size_t size,
						enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset =
		HPI_REG_OFFSET_OF(PORT_EFFECTIVE_SINK_PDO_MASK, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_EFFECTIVE_SINK_PDO_MASK, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for EFFECTIVE_SINK_PDO_MASK\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_select_source_pdo(struct hpi_device *hpidev,
					  void *buf, size_t size,
					  enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_SELECT_SOURCE_PDO, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_SELECT_SOURCE_PDO, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_select_sink_pdo(struct hpi_device *hpidev,
				       void *buf, size_t size,
				       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_SELECT_SINK_PDO, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_SELECT_SINK_PDO, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_pd_control(struct hpi_device *hpidev,
				   void *buf, size_t size,
				   enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_PD_CONTROL, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_PD_CONTROL, hpi_ver);

	if (rw_mode == HPI_READ) {
		hpi_err("invalid RW ops, WO for PD_CONTROL\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid hpidev instance, dev_type: %d\n",
			hpidev->dev_type);
		return -EINVAL;
	}

	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_pd_status(struct hpi_device *hpidev,
				  void *buf, size_t size,
				  enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_PD_STATUS, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_PD_STATUS, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for PD_STATUS\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_type_c_status(struct hpi_device *hpidev,
				      void *buf, size_t size,
				      enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_TYPE_C_STATUS, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_TYPE_C_STATUS, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for TYPE_C_STATUS\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_current_pdo(struct hpi_device *hpidev,
				    void *buf, size_t size,
				    enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_CURRENT_PDO, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_CURRENT_PDO, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for CURRENT_PDO\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_current_rdo(struct hpi_device *hpidev,
				    void *buf, size_t size,
				    enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_CURRENT_RDO, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_CURRENT_RDO, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for CURRENT_RDO\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_current_cable_vdo(struct hpi_device *hpidev,
					  void *buf, size_t size,
					  enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_CURRENT_CABLE_VDO, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_CURRENT_CABLE_VDO, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for CURRENT_CABLE_VDO\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_ec_dp_hpd_ctrl(struct hpi_device *hpidev,
				       void *buf, size_t size,
				       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_EC_DP_HPD_CONTROL, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_EC_DP_HPD_CONTROL, hpi_ver);

	if (hpi_ver > HPI_VERSION_1 || ccg_ver > CCG2) {
		hpi_dbg("CCG3/CCG4 not support EC_DP_HPD_CONTROL reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_ec_dp_mux_ctrl(struct hpi_device *hpidev,
				       void *buf, size_t size,
				       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_EC_DP_MUX_CONTROL, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_EC_DP_MUX_CONTROL, hpi_ver);

	if (hpi_ver > HPI_VERSION_1 || ccg_ver > CCG2) {
		hpi_dbg("CCG3/CCG4 not support EC_DP_MUX_CONTROL reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_trigger_dp_mode(struct hpi_device *hpidev,
					void *buf, size_t size,
					enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_TRIGGER_DP_MODE, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_TRIGGER_DP_MODE, hpi_ver);

	if (hpi_ver > HPI_VERSION_1 || ccg_ver > CCG2) {
		hpi_dbg("CCG3/CCG4 not support TRIGGER_DP_MODE reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_dp_configure_mode(struct hpi_device *hpidev,
					  void *buf, size_t size,
					  enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_DP_CONFIGURE_MODE, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_DP_CONFIGURE_MODE, hpi_ver);

	if (hpi_ver > HPI_VERSION_1 || ccg_ver > CCG2) {
		hpi_dbg("CCG3/CCG4 not support DP_CONFIGURE_MODE reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_alt_mode_cmd(struct hpi_device *hpidev,
				     void *buf, size_t size,
				     enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_ALT_MODE_CMD, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_ALT_MODE_CMD, hpi_ver);

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_dbg("CCG1/CCG2 not support ALT_MODE_CMD reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_app_hw_cmd(struct hpi_device *hpidev,
				   void *buf, size_t size,
				   enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = hpidev->cyccg->ccg_info.ccg_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_APP_HW_CMD, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_APP_HW_CMD, hpi_ver);

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		hpi_dbg("CCG1/CCG2 not support APP_HW_CMD reg\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_event_mask(struct hpi_device *hpidev,
				   void *buf, size_t size,
				   enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_EVENT_MASK, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_EVENT_MASK, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_swap_response(struct hpi_device *hpidev,
				      void *buf, size_t size,
				      enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_SWAP_RESPONSE, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_SWAP_RESPONSE, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_active_ec_modes(struct hpi_device *hpidev,
					void *buf, size_t size,
					enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_ACTIVE_EC_MODES, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_ACTIVE_EC_MODES, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_vdm_ec_control(struct hpi_device *hpidev,
				       void *buf, size_t size,
				       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_VDM_EC_CONTROL, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_VDM_EC_CONTROL, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_cmd_timeout(struct hpi_device *hpidev,
				    void *buf, size_t size,
				    enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_CMD_TIMEOUT, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_CMD_TIMEOUT, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_port_intr_status(struct hpi_device *hpidev,
					 void *buf, size_t size,
					 enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_PORT_INTR_STATUS, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_PORT_INTR_STATUS, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_disable_billboard_reset(struct hpi_device *hpidev,
						void *buf, size_t size,
						enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset =
		HPI_REG_OFFSET_OF(PORT_DISABLE_BILLBOARD_RESET, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_DISABLE_BILLBOARD_RESET, hpi_ver);

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	if (rw_mode == HPI_READ)
		return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
	return hpi_device_write(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_billboard_altmode_status(struct hpi_device *hpidev,
						 void *buf, size_t size,
						 enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset =
		HPI_REG_OFFSET_OF(PORT_BILLBOARD_ALTMODE_STATUS, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_BILLBOARD_ALTMODE_STATUS, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for BILLBOARD_ALTMODE_STATUS\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_billboard_oper_model(struct hpi_device *hpidev,
					     void *buf, size_t size,
					     enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset = HPI_REG_OFFSET_OF(PORT_BILLBOARD_OPER_MODEL, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_BILLBOARD_OPER_MODEL, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for BILLBOARD_OPER_MODEL\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

static int _hpi_port_rw_external_power_control(struct hpi_device *hpidev,
					       void *buf, size_t size,
					       enum hpi_rw_mode rw_mode)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u32 reg_offset =
		HPI_REG_OFFSET_OF(PORT_EXTERNAL_POWER_CONTROL, hpi_ver);
	u32 reg_size = HPI_REG_SIZE_OF(PORT_EXTERNAL_POWER_CONTROL, hpi_ver);

	if (rw_mode == HPI_WRITE) {
		hpi_err("invalid RW ops, RO for EXTERNAL_POWER_CONTROL\n");
		return -EINVAL;
	}

	if (!IS_HPI_DEV_TYPE_PORT(hpidev->dev_type)) {
		hpi_err("invalid parameters, dev_type=%d, size=%zu\n",
			hpidev->dev_type, size);
		return -EINVAL;
	}

	return hpi_device_read(hpidev, reg_offset, reg_size, buf, size);
}

/*
 * HPI spec based command process mechanism APIs.
 */
static inline enum hpi_sync_mode hpi_cmd_get_sync_mode(
		struct hpi_device *hpidev)
{
	struct hpi_command *cmd = &hpidev->cmd;
	enum hpi_sync_mode sync_mode;

	spin_lock(&cmd->slock);
	sync_mode = cmd->sync_mode;
	spin_unlock(&cmd->slock);
	return sync_mode;
}

static inline enum hpi_cmd_flag hpi_cmd_get_flag(struct hpi_device *hpidev)
{
	struct hpi_command *cmd = &hpidev->cmd;
	enum hpi_cmd_flag flag;

	spin_lock(&cmd->slock);
	flag = cmd->flag;
	spin_unlock(&cmd->slock);
	return flag;
}

static inline enum hpi_cmd_state hpi_cmd_get_state(struct hpi_device *hpidev)
{
	struct hpi_command *cmd = &hpidev->cmd;
	enum hpi_cmd_state state;

	spin_lock(&cmd->slock);
	state = cmd->state;
	spin_unlock(&cmd->slock);
	return state;
}

static inline void hpi_cmd_set_state(struct hpi_device *hpidev,
			      enum hpi_cmd_state new_state)
{
	struct hpi_command *cmd = &hpidev->cmd;

	/*
	 * Do the check to avoid the later state was missed updated to old
	 * state. Such as, after the HPI_CMD_ISSUED is set, then after the done
	 * of write the command, the interrupt and threaded handler for the
	 * response data process may fast than the hpi_cmd_sync() API called.
	 * So the state will be updated to HPI_CMD_RESPONSE_RECEIVED in ahead of
	 * set the state to HPI_CMD_SYNC_MODE_CHECKED. It must be avoid.
	 * This phenomenon may happen on low perforance platform and the
	 * the irq threaded handler has much higher priority then normal system
	 * system thread.
	 */
	spin_lock(&cmd->slock);
	/* Should not set to an uninit command. */
	if (cmd->state > HPI_CMD_IDLE && cmd->state < new_state)
		cmd->state = new_state;
	spin_unlock(&cmd->slock);
}

void hpi_cmd_set_state_errcode(struct hpi_device *hpidev,
		enum hpi_cmd_state new_state, int errcode)
{
	struct hpi_command *cmd = &hpidev->cmd;

	spin_lock(&cmd->slock);
	if (cmd->state > HPI_CMD_IDLE) {
		if (cmd->state < new_state)
			cmd->state = new_state;
		cmd->errcode = errcode;
	}
	spin_unlock(&cmd->slock);
}

static inline hpi_cmd_filter_t hpi_cmd_get_filter_fn(struct hpi_device *hpidev)
{
	struct hpi_command *cmd = &hpidev->cmd;
	hpi_cmd_filter_t filter = NULL;

	spin_lock(&cmd->slock);
	filter = cmd->filter;
	spin_unlock(&cmd->slock);
	return filter;
}

static inline int hpi_cmd_get_errcode(struct hpi_device *hpidev)
{
	struct hpi_command *cmd = &hpidev->cmd;
	int errcode;

	spin_lock(&cmd->slock);
	errcode = cmd->errcode;
	spin_unlock(&cmd->slock);
	return errcode;
}

static inline uint hpi_cmd_get_cmd_id(struct hpi_device *hpidev)
{
	struct hpi_command *cmd = &hpidev->cmd;
	uint cmd_id;

	spin_lock(&cmd->slock);
	cmd_id = cmd->cmd_id;
	spin_unlock(&cmd->slock);
	return cmd_id;
}

void hpi_cmd_store_return_data(struct hpi_device *hpidev,
			       void *data, size_t size)
{
	struct hpi_command *cmd = &hpidev->cmd;
	struct hpi_msg *tmp_msg = &cmd->copy_msg;
	struct hpi_buffer *tmp_buf = &cmd->copy_buf;

	spin_lock(&cmd->slock);
	if (size && cmd->data && cmd->data_size && *(cmd->data_size)) {
		*(cmd->data_size) = min(*(cmd->data_size), size);
		memcpy(cmd->data, data, *(cmd->data_size));
	}

	if (cmd->sync_mode == HPI_ASYNC) {
		/*
		 * Copy to a tempory buffer which can be used later as the
		 * input data of async mode callback routine if exists.
		 */
		memset(tmp_msg, 0, sizeof(struct hpi_msg));
		tmp_msg->len = size;
		tmp_msg->data = tmp_buf->head;
	}
	spin_unlock(&cmd->slock);
}

void hpi_cmd_copy_return_data(struct hpi_device *hpidev,
				     struct hpi_msg *msg)
{
	struct hpi_command *cmd = &hpidev->cmd;
	struct hpi_buffer *copy_buf = &cmd->copy_buf;
	struct hpi_msg *copy_msg = &cmd->copy_msg;
	bool has_out_data = false;

	/*
	 * Logically, based on HPIv2 spec, the max copy bytes can be 504 bytes,
	 * because the max read data region size is 504 bytes for CCG3/CCG4
	 * PD events. But indeed, most message won't copy the return data max
	 * than 32 bytes, except the firmware flash row read operation.
	 * The max flash row read operation is 256 bytes, and it seldom used.
	 * So keep using spinlock here to protect it.
	 * If it really affect the system performance, could replaced it with
	 * a mutex lock to protect it for big data copy.
	 */
	spin_lock(&cmd->slock);
	if (msg->len && cmd->data && cmd->data_size && *(cmd->data_size)) {
		*(cmd->data_size) = min(*(cmd->data_size), msg->len);
		memcpy(cmd->data, msg->data, *(cmd->data_size));
		has_out_data = true;
	}

	if (cmd->sync_mode == HPI_ASYNC && !has_out_data) {
		/*
		 * Copy to a tempory buffer which can be used later as the
		 * input data of async mode callback routine if exists.
		 */
		if (copy_buf->head && copy_buf->size >= msg->len) {
			memcpy(copy_buf->head, msg->data, msg->len);
			copy_msg->code = msg->code;
			copy_msg->len = msg->len;
			copy_msg->data = copy_buf->head;
		}
	}
	spin_unlock(&cmd->slock);
}

/*
 * hpi_cmd_init_lock - prepare and initialize the states and flags for
 *	starting a new command process
 * @hpidev: The instance of the HPI device or HPI port device.
 * @sync_mode: The command process running in HPI_SYNC or HPI_ASYCN mode.
 * @flag: indicates the command resposne messages catalog, driver internal
 *	default message pocess mechanism working depending this flag.
 */
static inline void hpi_cmd_init_lock(struct hpi_device *hpidev,
		enum hpi_sync_mode sync_mode, enum hpi_cmd_flag flag)
{
	struct hpi_command *cmd = &hpidev->cmd;

	hpi_port_vdbg("<<<< enter, %s\n", hpidev,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");
	mutex_lock(&cmd->mlock);
	spin_lock(&cmd->slock);
	cmd->sync_mode = sync_mode;
	if (sync_mode == HPI_SYNC)
		init_completion(&cmd->done);
	cmd->flag = flag;
	cmd->errcode = 0;
	cmd->filter = NULL;
	cmd->cmd_id = 0;
	cmd->data = NULL;
	cmd->data_size = NULL;
	cmd->callback = NULL;
	cmd->param = NULL;
	cmd->state = HPI_CMD_INITED;
	spin_unlock(&cmd->slock);
}

static inline void hpi_cmd_set_filter_callback(struct hpi_device *hpidev,
	hpi_cmd_filter_t filter, void *filter_data, size_t *filter_data_size,
	hpi_cmd_cb_t callback, void *param)
{
	struct hpi_command *cmd = &hpidev->cmd;

	spin_lock(&cmd->slock);
	cmd->filter = filter;
	cmd->data = filter_data;
	cmd->data_size = filter_data_size;
	cmd->callback = callback;
	cmd->param = param;
	spin_unlock(&cmd->slock);
}

/*
 * hpi_cmd_init_lock - prepare and initialize the states and flags for
 *	starting a new command process
 * @hpidev: The instance of the HPI device or HPI port device.
 * @filter: The command specific process for the command response messages.
 * @cmd_id: indicates the command id when the filter supports process multiple
 *	commands' response in one filter routine. This command id can be used
 *	to distiongush which command the response message corresponding to.
 * @out_data: Point to the memory buffer used to retrieve the CCGx return data.
 * @out_data_size: Size in bytes of the @out_data buffer when input;
 *	When the command finished and returned, the @out_data_size will be
 *	updated to the real data size in bytes stored in the @out_data buffer.
 */
static inline void hpi_cmd_set_filter(struct hpi_device *hpidev,
		hpi_cmd_filter_t filter, uint cmd_id,
		void *out_data, size_t *out_data_size)
{
	struct hpi_command *cmd = &hpidev->cmd;

	spin_lock(&cmd->slock);
	cmd->filter = filter;
	cmd->cmd_id = cmd_id;
	cmd->data = out_data;
	cmd->data_size = out_data_size;
	spin_unlock(&cmd->slock);
}

/*
 * hpi_cmd_set_async_callback - Set the callback and parameter for async mode
 * @hpidev: The instance of the HPI device or HPI port device.
 * @callback: Point to the caller specified routine which will be called
 *	when the command was finished.
 * @param: The parameter in the callback routine.
 */
static inline void hpi_cmd_set_async_callback(struct hpi_device *hpidev,
					hpi_cmd_cb_t callback, void *param)
{
	struct hpi_command *cmd = &hpidev->cmd;

	spin_lock(&cmd->slock);
	cmd->callback = callback;
	cmd->param = param;
	spin_unlock(&cmd->slock);
}

void hpi_cmd_get_async_callback_param(struct hpi_device *hpidev,
		hpi_cmd_cb_t callback, void *param)
{
	struct hpi_command *cmd = &hpidev->cmd;

	spin_lock(&cmd->slock);
	if (callback)
		callback = cmd->callback;
	if (param)
		param = cmd->param;
	spin_unlock(&cmd->slock);
}

static inline int hpi_cmd_write_data(struct hpi_device *hpidev,
		hpi_reg_rw_t write_cmd_data_fn, void *buf, size_t size)
{
	/* No command data needs to be written before the command was sent. */
	if (!write_cmd_data_fn || !buf || !size)
		return 0;

	return write_cmd_data_fn(hpidev, buf, size, HPI_WRITE);
}

static inline int hpi_cmd_send_cmd(struct hpi_device *hpidev,
		hpi_reg_rw_t send_cmd_fn, void *buf, size_t size,
		enum hpi_rw_mode rw_mode)
{
	if (!send_cmd_fn || !buf || !size)
		return -EINVAL;

	hpi_cmd_set_state(hpidev, HPI_CMD_ISSUED);

	return send_cmd_fn(hpidev, buf, size, rw_mode);
}

static inline int hpi_cmd_sync(struct hpi_device *hpidev, unsigned long timeout)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	struct hpi_command *cmd = &hpidev->cmd;

	if (hpi_cmd_get_state(hpidev) < HPI_CMD_ISSUED) {
		hpi_port_err("invalid cmd->state=%d, cmd not issued yet\n",
			   hpidev, hpi_cmd_get_state(hpidev));
		return -EINVAL;
	}

	hpi_cmd_set_state(hpidev, HPI_CMD_SYNC_MODE_CHECKED);

	if (cmd->sync_mode == HPI_ASYNC) {
		/*
		 * timeout=0 and no filter_fu indicates user doesn't want to
		 * check the response message/data, only want to send out
		 * the command.
		 */
		if (timeout == 0 && !hpi_cmd_get_filter_fn(hpidev)) {
			hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
			hpi_port_vdbg("HPI_ASYNC, timeout=%lu, done\n",
				hpidev, timeout);
			goto out;
		} else {
			timeout = timeout ?: HPI_TIME_OF(DEFAULT, hpi_ver);
			hpi_cmd_start_timer(hpidev, timeout);
			hpi_port_vdbg("HPI_ASYNC, timeout=%lu ms, return\n",
				hpidev, timeout);
			return 0;
		}
	}

	/* Running in HPI_SYNC mode. */
	timeout = timeout ?: HPI_TIME_OF(DEFAULT, hpi_ver);
	timeout = wait_for_completion_timeout(&cmd->done,
					      msecs_to_jiffies(timeout));
	if (timeout == 0) {
		/*
		 * Double check the hpi_cmd->status, because after the command
		 * register was written and before the command issue thread
		 * start really executed the wait_for_completion_timeout()
		 * function. The command response interrupt may have come,
		 * so the interrupt thread will executed and the response
		 * message may be processed before the
		 * wait_for_completion_timeout() function, so it cannot capture
		 * complete(&cmd->done) event and cause the command timeout
		 * error. But indeed, it has been finished susccessfully.
		 * So double check here to cover this issue to get the correct
		 * command executed result in this situation.
		 * This issue is easily captured on low performance platform
		 * devices.
		 */
		if (hpi_cmd_get_state(hpidev) != HPI_CMD_COMPLETED) {
			hpi_port_vdbg("timeout error\n", hpidev);
			hpi_cmd_set_state_errcode(hpidev,
					HPI_CMD_COMPLETED, -ETIMEDOUT);
		}
	}

out:
	return hpi_cmd_get_errcode(hpidev);
}

static inline void hpi_cmd_deinit_unlock(struct hpi_device *hpidev)
{
	struct hpi_command *cmd = &hpidev->cmd;
	enum hpi_cmd_state state = HPI_CMD_IDLE;
	hpi_cmd_cb_t cb_fn = NULL;
	void *param = NULL;
	int errcode = 0;

	hpi_port_vdbg("state=%d\n", hpidev, hpi_cmd_get_state(hpidev));

	spin_lock(&cmd->slock);

	/* The command state has been deinited and released. */
	if (cmd->state == HPI_CMD_IDLE) {
		spin_unlock(&cmd->slock);
		hpi_port_vdbg(">>>> exit, HPI_CMD_IDLE\n", hpidev);
		return;
	}

	/*
	 * The commmand hasn't been init/called/sent in order, exit early.
	 * Should not happen in normal process.
	 */
	if (unlikely(cmd->state < HPI_CMD_SYNC_MODE_CHECKED)) {
		hpi_port_vdbg("failed to before hpi_cmd_sync\n", hpidev);
		goto deinit;
	}

	/*
	 * Using HPI_ASYNC mode, return directly let callback function or the
	 * timeout timer to do the left process.
	 */
	if (cmd->sync_mode == HPI_ASYNC) {
		/* HPI_ASYNC mode command has finished. */
		if (cmd->state <= HPI_CMD_SYNC_MODE_CHECKED) {
			spin_unlock(&cmd->slock);
			hpi_port_vdbg(">>>> exit, HPI_ASYNC sent\n", hpidev);
			return;
		}

		hpi_port_vdbg("HPI_ASYNC command finished\n", hpidev);
		goto deinit;
	}

deinit:	/* HPI_SYNC/HPI_ASYNC command has finished. */
	hpi_port_vdbg("deinit\n", hpidev);
	if (cmd->sync_mode == HPI_ASYNC) {
		state = cmd->state;
		cb_fn = cmd->callback;
		param = cmd->param;
		errcode = cmd->errcode;
		if (cmd->state != HPI_CMD_COMPLETED && !errcode) {
			hpi_port_warn("errcode not set, assigned to -EFAULT\n",
				    hpidev);
			errcode = -EFAULT;
		}
	}

	cmd->filter = NULL;
	cmd->cmd_id = 0;
	cmd->data = NULL;
	cmd->data_size = NULL;
	cmd->callback = NULL;
	cmd->param = NULL;
	cmd->errcode = 0;
	cmd->state = HPI_CMD_IDLE;
	spin_unlock(&cmd->slock);

	/* Stop the command timeout timer when the command finished. */
	if (cmd->sync_mode == HPI_ASYNC && state >= HPI_CMD_SYNC_MODE_CHECKED) {
		hpi_cmd_stop_timer(hpidev);
		hpi_port_vdbg("HPI_ASYNC timer stopped\n", hpidev);
	}

	spin_lock(&hpidev->slock);
	hpidev->is_swap_triggered_by_host = false;
	spin_unlock(&hpidev->slock);
	mutex_unlock(&cmd->mlock);

	/* The command has been finished, run the callback if existing. */
	if (cb_fn) {
		cb_fn(hpidev, param,
		      errcode, cmd->copy_msg.data, cmd->copy_msg.len);
		hpi_port_vdbg("the command callback function called\n", hpidev);
	}

	hpi_port_vdbg(">>>> exit\n", hpidev);
}

void hpi_cmd_timeout_function(struct work_struct *work)
{
	struct delayed_work *dwork =
			container_of(work, struct delayed_work, work);
	struct hpi_command *hpicmd =
			container_of(dwork, struct hpi_command, timer);
	struct hpi_device *hpidev =
			container_of(hpicmd, struct hpi_device, cmd);

	/* The command hasn't started the timer, should not happen. */
	if (unlikely(hpi_cmd_get_state(hpidev) < HPI_CMD_SYNC_MODE_CHECKED))
		return;

	if (hpi_cmd_get_state(hpidev) != HPI_CMD_COMPLETED) {
		hpi_port_dbg("error, HPI_ASYNC command timeout\n", hpidev);
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_TIMEOUT, -ETIMEDOUT);
	}

	hpi_cmd_deinit_unlock(hpidev);
}

static inline void hpi_cmd_start_timer(struct hpi_device *hpidev,
				unsigned long timeout)
{
	schedule_delayed_work(&hpidev->cmd.timer, msecs_to_jiffies(timeout));
}

static inline void hpi_cmd_stop_timer(struct hpi_device *hpidev)
{
	cancel_delayed_work(&hpidev->cmd.timer);
}



/*
 * hpi_device_default_command_response_filter - default command response message
 *	process. If not command response message, just bypass.
 * @hpidev: The instance of the HPI device or HPI port device.
 * @msg: Possible command response for a command.
 */
static inline enum hpi_msg_return
hpi_device_default_command_response_filter(
		struct hpi_device *hpidev, struct hpi_msg *msg)
{
	/* This response filter only process command response code message. */
	if (IS_HPI_EVENT_MSG(msg->code))
		return HPI_MSG_RETURN_NONE;

	if (hpi_cmd_get_state(hpidev) >= HPI_CMD_RESPONSE_RECEIVED ||
			hpi_cmd_get_state(hpidev) < HPI_CMD_ISSUED) {
		hpi_port_dbg("invalid cmd->state = %d\n", hpidev,
			hpi_cmd_get_state(hpidev));
		return HPI_MSG_RETURN_NONE;
	}

	if (msg->code == HPI_PD_RESP_SUCCESS ||
			 msg->code == HPI_PD_RESP_FLASH_DATA_AVAILABLE) {
		hpi_port_vdbg("command response code=0x%x success\n",
			hpidev, msg->code);
		/*
		 * If no next event/data message expected, then the command
		 * can be finished. Otherwise, continue for next message
		 * process.
		 */
		if (hpi_cmd_get_flag(hpidev) >= HPI_CMD_FLAG_RESP_EVENT_RAW)
			hpi_cmd_set_state_errcode(hpidev,
					HPI_CMD_RESPONSE_RECEIVED, 0);
		else
			hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
	} else {
		hpi_port_dbg("failed to send command data, error code=0x%x\n",
			hpidev, msg->code);
		/* Command data sent failed */
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, msg->code);
	}

	return HPI_MSG_RETURN_HANDLED;
}

/*
 * hpi_device_default_events_filter - default event message process for a
 *	command, itwas used to find any reset/error/recovery/abnormal events
 *	that have caused the command failed.
 * @hpidev: The instance of the HPI device or HPI port device.
 * @msg: Possible command response for a command.
 */
static inline enum hpi_msg_return
hpi_device_default_events_filter(struct hpi_device *hpidev, struct hpi_msg *msg)
{
	/* This filter only process event messages. */
	if (IS_HPI_RESPONSE_MSG(msg->code))
		return HPI_MSG_RETURN_NONE;

	/* cmd->state not match, not expected to process the msg. */
	if (hpi_cmd_get_state(hpidev) < HPI_CMD_RESPONSE_RECEIVED ||
			hpi_cmd_get_state(hpidev) >= HPI_CMD_TIMEOUT) {
		hpi_port_dbg("unexpected event received\n", hpidev);
		hpi_port_dbg("    cmd state=%d, resp code=0x%x\n", hpidev,
			hpi_cmd_get_state(hpidev), msg->code);

		/*
		 * Must wait the command response code to be received
		 * and process in advance.
		 */
		return HPI_MSG_RETURN_NONE;
	}

	/*
	 * CCG may encounter error, a reset/error/recovery/disconnect
	 * response event was received. So any in processing command
	 * should be terminated with an error code.
	 */
	switch (msg->code) {
	case HPI_PD_RESP_RESET_COMPLETE:
		hpi_port_dbg("HPI_PD_RESP_RESET_COMPLETE\n", hpidev);
		break;
	case HPI_PD_RESP_MESSAGE_QUEUE_OVERFLOW:
		hpi_port_dbg("HPI_PD_RESP_MESSAGE_QUEUE_OVERFLOW\n", hpidev);
		break;
	case HPI_PD_RESP_OVER_CURRENT_DETECTED:
		hpi_port_dbg("HPI_PD_RESP_OVER_CURRENT_DETECTED\n", hpidev);
		break;
	case HPI_PD_RESP_OVER_VOLTAGE_DETECTED:
		hpi_port_dbg("HPI_PD_RESP_OVER_VOLTAGE_DETECTED\n", hpidev);
		break;
	case HPI_PD_RESP_TYPE_C_DISCONNECTED:
		hpi_port_dbg("HPI_PD_RESP_TYPE_C_DISCONNECTED\n", hpidev);
		break;
	case HPI_PD_RESP_REJECT_MESSAGE:
		hpi_port_dbg("HPI_PD_RESP_REJECT_MESSAGE\n", hpidev);
		break;
	case HPI_PD_RESP_WAIT_MESSAGE:
		hpi_port_dbg("HPI_PD_RESP_WAIT_MESSAGE\n", hpidev);
		break;
	case HPI_PD_RESP_HARD_RESET:
		hpi_port_dbg("HPI_PD_RESP_HARD_RESET\n", hpidev);
		break;
	case HPI_PD_HARD_RESET_SENT:
		hpi_port_dbg("HPI_PD_HARD_RESET_SENT\n", hpidev);
		break;
	case HPI_PD_SOFT_RESET_SENT:
		hpi_port_dbg("HPI_PD_SOFT_RESET_SENT\n", hpidev);
		break;
	case HPI_PD_SOURCE_DISBALED_STATE_ENTERED:
		hpi_port_dbg("HPI_PD_SOURCE_DISBALED_STATE_ENTERED\n", hpidev);
		break;
	case HPI_PD_SENDER_RESPONSE_TIMER_TIMEOUT:
		hpi_port_dbg("HPI_PD_SENDER_RESPONSE_TIMER_TIMEOUT\n", hpidev);
		break;
	case HPI_PD_NO_VDM_RESPONSE_RECEIVED:
		hpi_port_dbg("HPI_PD_NO_VDM_RESPONSE_RECEIVED\n", hpidev);
		break;
	case HPI_PD_UNEXPECTED_VOLTAGE_VBUS:
		hpi_port_dbg("HPI_PD_UNEXPECTED_VOLTAGE_VBUS\n", hpidev);
		break;
	case HPI_PD_TYPE_C_ERROR_RECOVERY:
		hpi_port_dbg("HPI_PD_TYPE_C_ERROR_RECOVERY\n", hpidev);
		break;
	default:
		return HPI_MSG_RETURN_NONE;
	}

	/* The command failed with and unexcpeted error event received. */
	hpi_port_err("CCG device error event code received, code=0x%x\n",
		hpidev, msg->code);
	hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, msg->code);

	/*
	 * Do not mark it as handled, so it can be processed later
	 * in cyccg default event code process routine if needed.
	 */
	return HPI_MSG_RETURN_IGNORED;
}

/*
 * hpi_device_default_command_handler - entrance for message process by commands
 * @hpidev: The instance of the HPI device or HPI port device.
 * @msg: Possible command response or event message for a command.
 */
enum hpi_msg_return hpi_device_default_command_handler(
		struct hpi_device *hpidev, struct hpi_msg *msg)
{
	enum hpi_msg_return ret = HPI_MSG_RETURN_NONE;
	struct hpi_command *cmd = &hpidev->cmd;
	hpi_cmd_filter_t cmd_filter_fn;

	/* The command hasn't been ready/send or finished, so bypass. */
	if (hpi_cmd_get_state(hpidev) < HPI_CMD_ISSUED ||
			hpi_cmd_get_state(hpidev) >= HPI_CMD_TIMEOUT)
		return HPI_MSG_RETURN_NONE;

	/* Do default command response code message process if requied. */
	if (hpi_cmd_get_flag(hpidev) != HPI_CMD_FLAG_RAW)
		ret = hpi_device_default_command_response_filter(hpidev, msg);

	/*
	 * Do default command asynchronous event code message filter for
	 * unexpected reset/error/recovery event code messages if required.
	 */
	if (ret == HPI_MSG_RETURN_NONE &&
			hpi_cmd_get_flag(hpidev) > HPI_CMD_FLAG_RESP_EVENT_RAW)
		ret = hpi_device_default_events_filter(hpidev, msg);

	/* Finally call command specific filter function if it specified. */
	cmd_filter_fn = hpi_cmd_get_filter_fn(hpidev);
	if (ret == HPI_MSG_RETURN_NONE && cmd_filter_fn)
		ret = cmd_filter_fn(hpidev, msg);

	/*
	 * deinit and unlock the command when completed, so next command can
	 * be started immediately in the same thread through the callback
	 * in HPI_ASYNC mode.
	 */
	if (ret != HPI_MSG_RETURN_NONE &&
			hpi_cmd_get_state(hpidev) == HPI_CMD_COMPLETED) {
		if (hpi_cmd_get_sync_mode(hpidev) == HPI_SYNC) {
			hpi_port_vdbg("HPI_SYNC command complete\n", hpidev);
			complete(&cmd->done);
		} else {
			hpi_port_vdbg("HPI_ASYNC command complete\n", hpidev);
			hpi_cmd_deinit_unlock(hpidev);
		}
	}

	hpi_port_vdbg("return ret = %s\n", hpidev,
		(ret == HPI_MSG_RETURN_HANDLED) ? "HPI_MSG_RETURN_HANDLED" :
			(ret == HPI_MSG_RETURN_NONE) ? "HPI_MSG_RETURN_NONE" :
				"HPI_MSG_RETURN_NONE");
	return (ret == HPI_MSG_RETURN_IGNORED) ? HPI_MSG_RETURN_NONE : ret;
}

/* Routines to read HPI message head and data */
static inline void _hpi_message_head_parse(struct hpi_device *hpidev,
					   struct hpi_msg *msg,
					   struct hpi_buffer *hpi_buf)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;

	memset(msg, 0, sizeof(struct hpi_msg));
	if (IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type) ||
			hpi_ver == HPI_VERSION_1) {
		msg->code = hpi_buf->head[0];
		msg->len = hpi_buf->head[1];
		msg->data = &hpi_buf->head[sizeof(struct hpi_v1_msg)];
	} else {
		msg->code = hpi_buf->head[0];
		msg->len = get_unaligned_le16(&hpi_buf->head[2]);
		msg->data = &hpi_buf->head[sizeof(struct hpi_v2_msg)];
	}
}

static inline size_t hpi_msg_head_size(struct hpi_device *hpidev)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	size_t size;

	if (IS_HPI_DEV_TYPE_DEVICE(hpidev->dev_type))
		size = sizeof(struct hpi_v1_msg);
	else
		size = (hpi_ver == HPI_VERSION_1) ?
				(sizeof(struct hpi_v1_msg)) :
				(sizeof(struct hpi_v2_msg));
	return size;
}

int hpi_message_read(struct hpi_device *hpidev, struct hpi_msg *msg)
{
	struct hpi_buffer *hpi_buf = &hpidev->cmd.msg_buf;
	struct hpi_msg *hpi_msg = &hpidev->cmd.msg;
	int err;

	if (unlikely(hpi_buf->size < hpi_msg_head_size(hpidev))) {
		hpi_port_err("hpi_buf too small, size=%zu\n", hpidev,
			hpi_buf->size);
		return -EINVAL;
	}

	err = _hpi_rw_response_head(hpidev, hpi_buf->head,
			hpi_msg_head_size(hpidev), HPI_READ);
	if (err) {
		hpi_port_err("failed to read message resp code, %d\n",
			hpidev, err);
		return err;
	}

	_hpi_message_head_parse(hpidev, hpi_msg, hpi_buf);

	if (msg->code == HPI_PD_RESP_NO_RESPONSE || !msg->len)
		return 0;

	if (unlikely(hpi_buf->size <
			(hpi_msg_head_size(hpidev) + msg->len))) {
		hpi_port_err("msg->len=%zu too large, hpi_buf->size=%zu\n",
			hpidev, msg->len, hpi_buf->size);
		return -EINVAL;
	}

	err = _hpi_rw_data_memory(hpidev, msg->data, msg->len, HPI_READ);
	if (err) {
		hpi_port_err("failed to read message data, %d\n", hpidev, err);
		return err;
	}

	return 0;
}


/*
 * HPI interfaces
 */

int hpi_read_device_mode(struct cyccg *cyccg,
			 struct hpi_device_mode *device_mode)
{
	int err;

	err = _hpi_rw_device_mode(&cyccg->hpi_dev,
			device_mode, sizeof(struct hpi_device_mode), HPI_READ);
	if (!err)
		cyccg->ccg_state.running_mode =
			(enum ccg_fw_mode_type)device_mode->running_mode;

	return err;
}

int hpi_read_boot_mode_reason(struct cyccg *cyccg,
			      struct hpi_boot_mode_reason *boot_mode_reason)
{
	return _hpi_rw_boot_mode_reason(&cyccg->hpi_dev, boot_mode_reason,
			sizeof(struct hpi_boot_mode_reason), HPI_READ);
}

int hpi_read_silicon_id(struct cyccg *cyccg, u16 *silicon_id)
{
	int err;

	err = _hpi_rw_silicon_id(&cyccg->hpi_dev, silicon_id, sizeof(u16),
				 HPI_READ);
	if (!err)
		*silicon_id = get_unaligned_le16(silicon_id);
	return err;
}

int hpi_read_boot_loader_last_row(struct cyccg *cyccg, u16 *bl_last_row)
{
	int err;

	err = _hpi_rw_boot_loader_last_row(&cyccg->hpi_dev,
				bl_last_row, sizeof(u16), HPI_READ);
	if (!err)
		*bl_last_row = get_unaligned_le16(bl_last_row);

	return err;
}

int hpi_read_intr_reg(struct cyccg *cyccg, union hpi_intr_reg *intr_reg)
{
	intr_reg->val = 0;
	return _hpi_rw_intr_reg(&cyccg->hpi_dev, &intr_reg->val, sizeof(u8),
				HPI_READ);
}

int hpi_write_intr_reg(struct cyccg *cyccg, union hpi_intr_reg *intr_reg)
{
	return _hpi_rw_intr_reg(&cyccg->hpi_dev, &intr_reg->val, sizeof(u8),
				HPI_WRITE);
}

int hpi_clear_intr(struct hpi_device *hpidev)
{
	union hpi_intr_reg intr_reg;
	int err;

	if (!hpidev || hpidev->dev_type == HPI_DEV_TYPE_UNKNOWN)
		return 0;

	intr_reg.val = 0;
	if (hpidev->dev_type == HPI_DEV_TYPE_DEVICE ||
			hpidev->dev_type == HPI_DEV_TYPE_MIXED)
		intr_reg.dev_intr = 1;
	else
		intr_reg.val |= HPI_PORT_INTR_BIT_MASK(hpidev->idx);

	err = hpi_write_intr_reg(hpidev->cyccg, &intr_reg);
	if (err) {
		hpi_err("failed to write intr_reg, val=0x%02x, %d\n",
			intr_reg.val, err);
		return err;
	}

	hpi_port_vdbg("intr_reg bit cleared\n", hpidev);
	return 0;
}

int hpi_clear_all_intr(struct cyccg *cyccg)
{
	struct ccg_info *ccg_info = &cyccg->ccg_info;
	union hpi_intr_reg intr_reg;
	int err;

	if (ccg_info->hpi_ver == HPI_VERSION_1)
		intr_reg.val = 0x01;
	else
		intr_reg.val = ~(0xff << (ccg_info->num_port + 1));

	err = hpi_write_intr_reg(cyccg, &intr_reg);
	if (err) {
		hpi_err("failed to write intr_reg, val=0x%02x, %d\n",
			intr_reg.val, err);
		return err;
	}

	hpi_vdbg("success\n");
	return 0;
}

static enum hpi_msg_return _hpi_jump_to_boot_filter(struct hpi_device *hpidev,
					    struct hpi_msg *msg)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	struct hpi_device_mode device_mode;
	u8 signature = HPI_SIGNATURE_JUMP_TO_BOOT;	/* set to default. */
	int retries = 3;
	int err;

	hpi_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", msg->code, msg->len);
	if (IS_HPI_EVENT_MSG(msg->code) &&
			msg->code != HPI_PD_RESP_RESET_COMPLETE)
		return HPI_MSG_RETURN_NONE;

	signature = (u8)hpi_cmd_get_cmd_id(hpidev);
	hpi_dbg("signature = %s\n", signature == HPI_SIGNATURE_JUMP_TO_BOOT ?
		"J - Jump to Bootloader" : "A - Jump to Alternate FW");

	while (retries--) {
		err = hpi_read_device_mode(hpidev->cyccg, &device_mode);
		if (!err)
			break;
		msleep(HPI_TIME_OF(VALIDATE_FLASH, hpi_ver));
	}
	if (err) {
		hpi_vdbg("failed to read device mode, %d\n", err);
		if (IS_HPI_RESPONSE_MSG(msg->code)) {
			hpi_cmd_set_state_errcode(hpidev,
					HPI_CMD_COMPLETED, err);
		} else {
			/* CCG may in mode switching, so unable to access. */
			hpi_cmd_set_state_errcode(hpidev,
					HPI_CMD_PRE_EVENTS_RCVD, err);
		}
		return HPI_MSG_RETURN_HANDLED;
	}

	hpi_dbg("running_mode = %u\n", device_mode.running_mode);
	if (device_mode.running_mode == CCG_FW_MODE_TYPE_BOOTLAODER &&
			signature == HPI_SIGNATURE_JUMP_TO_BOOT) {
		hpi_dbg("already in boot mode\n");
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
	} else if (device_mode.running_mode != CCG_FW_MODE_TYPE_BOOTLAODER &&
			signature == HPI_SIGNATURE_JUMP_TO_ALT_FW) {
		hpi_dbg("already in app mode\n");
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
	} else {
		hpi_dbg("still in bootloader mode, continue waiting\n");
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_PRE_EVENTS_RCVD, 0);
	}

	return HPI_MSG_RETURN_HANDLED;
}

/*
 * When called it to jump to boot, must disable PD ports firstly, otherwise,
 * this command may failed to jump to boot.
 */
static int _hpi_jump_to_boot(struct cyccg *cyccg, u8 signature,
		      enum hpi_sync_mode sync_mode,
		      hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	struct hpi_device_mode device_mode;
	int err;

	hpi_vdbg("<<<< enter\n");
	hpi_dbg("%s, %s\n", sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		signature == HPI_SIGNATURE_JUMP_TO_BOOT ?
			"J - Jump to Bootloader" : "A - Jump to Alternate FW");
	if (signature != HPI_SIGNATURE_JUMP_TO_BOOT &&
			signature != HPI_SIGNATURE_JUMP_TO_ALT_FW) {
		hpi_err("invalid signature value=0x%02x\n", signature);
		return -EINVAL;
	}

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RAW);
	hpi_cmd_set_filter(hpidev, _hpi_jump_to_boot_filter, (uint)signature,
			   NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_jump_to_boot,
			       &signature, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write data to JUMP_TO_BOOT register, %d\n",
			err);
		goto err;
	}

	/* Start polling timer to support non-irq CCG device/FW App. */
	cyccg_start_polling_timer(cyccg, true);

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(JUMP_TO_BOOT, hpi_ver));
	if (err == -ETIMEDOUT) {
		if (hpi_read_device_mode(hpidev->cyccg, &device_mode))
			goto err;

		if ((device_mode.running_mode == CCG_FW_MODE_TYPE_BOOTLAODER &&
				signature == HPI_SIGNATURE_JUMP_TO_BOOT) ||
		    (device_mode.running_mode != CCG_FW_MODE_TYPE_BOOTLAODER &&
				signature == HPI_SIGNATURE_JUMP_TO_ALT_FW)) {
			hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
			err = hpi_cmd_get_errcode(hpidev);
		}
	}

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_jump_to_boot_sync(struct cyccg *cyccg)
{
	return _hpi_jump_to_boot(cyccg, HPI_SIGNATURE_JUMP_TO_BOOT, HPI_SYNC,
				 NULL, NULL);
}

int hpi_jump_to_boot_async(struct cyccg *cyccg,
			   hpi_cmd_cb_t callback, void *param)
{
	return _hpi_jump_to_boot(cyccg, HPI_SIGNATURE_JUMP_TO_BOOT, HPI_ASYNC,
				 callback, param);
}

int hpi_jump_to_alt_fw_sync(struct cyccg *cyccg)
{
	return _hpi_jump_to_boot(cyccg, HPI_SIGNATURE_JUMP_TO_ALT_FW, HPI_SYNC,
				 NULL, NULL);
}

int hpi_jump_to_alt_fw_async(struct cyccg *cyccg,
			     hpi_cmd_cb_t callback, void *param)
{
	return _hpi_jump_to_boot(cyccg, HPI_SIGNATURE_JUMP_TO_ALT_FW, HPI_ASYNC,
				 callback, param);
}

static enum hpi_msg_return _hpi_reset_filter(struct hpi_device *hpidev,
					  struct hpi_msg *msg)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	struct hpi_device_mode device_mode;
	enum hpi_reset_type reset_type = HPI_RESET_TYPE_DEVICE;
	int retries = 3;
	int err;

	hpi_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", msg->code, msg->len);
	if (IS_HPI_EVENT_MSG(msg->code) &&
			msg->code != HPI_PD_RESP_RESET_COMPLETE)
		return HPI_MSG_RETURN_NONE;

	reset_type = (enum hpi_reset_type)hpi_cmd_get_cmd_id(hpidev);
	hpi_dbg("reset_type = %s\n", reset_type == HPI_RESET_TYPE_DEVICE ?
			"Device Reset" : "I2C Reset");

	while (retries--) {
		err = hpi_read_device_mode(hpidev->cyccg, &device_mode);
		if (!err)
			break;
		msleep(HPI_TIME_OF(VALIDATE_FLASH, hpi_ver));
	}
	if (err) {
		hpi_err("failed to read device mode, %d\n", err);
		if (IS_HPI_RESPONSE_MSG(msg->code)) {
			hpi_cmd_set_state_errcode(hpidev,
					HPI_CMD_COMPLETED, err);
		} else {
			/* CCG may in mode switching, so unable to access. */
			hpi_cmd_set_state_errcode(hpidev,
					HPI_CMD_DATA_RECEIVED, err);
		}

		return HPI_MSG_RETURN_HANDLED;
	}

	if (IS_HPI_RESPONSE_MSG(msg->code)) {
		if (msg->code == HPI_PD_RESP_SUCCESS &&
				reset_type == HPI_RESET_TYPE_I2C) {
			/* I2C reset success. */
			hpi_dbg("I2C reset success\n");
			hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
		} else {
			/* Device/I2C reset failed. */
			hpi_err("%s reset failed, msg->code=0x%x\n",
				reset_type == HPI_RESET_TYPE_DEVICE ?
					"Device" : "I2C",
				msg->code);
			hpi_cmd_set_state_errcode(hpidev,
					HPI_CMD_COMPLETED, msg->code);
		}

		return HPI_MSG_RETURN_HANDLED;
	}

	hpi_dbg("running_mode = %u\n", device_mode.running_mode);
	if (device_mode.running_mode == CCG_FW_MODE_TYPE_BOOTLAODER) {
		hpi_dbg("in boot mode, waiting enter app mode\n");
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_PRE_EVENTS_RCVD, 0);
	} else {
		hpi_dbg("has entered app mode, reset completed\n");
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
	}

	return HPI_MSG_RETURN_HANDLED;
}

static int _hpi_reset(struct cyccg *cyccg, enum hpi_reset_type reset_type,
		      enum hpi_sync_mode sync_mode,
		      hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	struct hpi_device_mode device_mode;
	u8 reset_cmd[2];
	int err;

	hpi_vdbg("<<<< enter\n");
	hpi_dbg("%s, %s\n", sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		reset_type == HPI_RESET_TYPE_DEVICE ?
			"Device Reset" : "I2C Reset");
	err = hpi_read_device_mode(cyccg, &device_mode);
	if (err) {
		hpi_err("failed to read device_mode, %d\n", err);
		return err;
	}

	if (device_mode.running_mode != CCG_FW_MODE_TYPE_BOOTLAODER &&
			reset_type == HPI_RESET_TYPE_DEVICE) {
		/*
		 * Try to disable port firstly.
		 * If the port has been disabled, the port disable operation
		 * may have no response or have error code, so skip the return
		 * port disable operation to force continue the reset operation.
		 */
		err = hpi_disable_all_dpport(cyccg, sync_mode, callback, param);
		if (err)
			hpi_dbg("failed to disable all PD ports, %d\n", err);
	}

	reset_cmd[0] = HPI_SIGNATURE_RESET;
	reset_cmd[1] = reset_type == HPI_RESET_TYPE_DEVICE ? 0x01 : 0x00;

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RAW);
	hpi_cmd_set_filter(hpidev, _hpi_reset_filter, (uint)reset_type,
			   NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_reset,
			       reset_cmd, sizeof(reset_cmd), HPI_WRITE);
	if (err) {
		hpi_err("failed to write RESET register, %d\n", err);
		goto err;
	}

	cyccg_start_polling_timer(cyccg, true);

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(BOOT_INTO_FW, hpi_ver));
	if (err == -ETIMEDOUT) {
		if (hpi_read_device_mode(cyccg, &device_mode))
			goto err;

		if (device_mode.running_mode == CCG_FW_MODE_TYPE_BOOTLAODER) {
			hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);
			err = hpi_cmd_get_errcode(hpidev);
		}
	}

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_i2c_reset_sync(struct cyccg *cyccg)
{
	return _hpi_reset(cyccg, HPI_RESET_TYPE_I2C, HPI_SYNC, NULL, NULL);
}

int hpi_i2c_reset_async(struct cyccg *cyccg,
			hpi_cmd_cb_t callback, void *param)
{
	return _hpi_reset(cyccg, HPI_RESET_TYPE_I2C, HPI_SYNC, callback, param);
}

int hpi_device_reset(struct cyccg *cyccg, enum hpi_sync_mode sync_mode,
		     hpi_cmd_cb_t callback, void *param)
{
	return _hpi_reset(cyccg, HPI_RESET_TYPE_DEVICE, sync_mode,
			  callback, param);
}

int hpi_device_reset_sync(struct cyccg *cyccg)
{
	return _hpi_reset(cyccg, HPI_RESET_TYPE_DEVICE, HPI_SYNC, NULL, NULL);
}

int hpi_device_reset_async(struct cyccg *cyccg,
			   hpi_cmd_cb_t callback, void *param)
{
	return _hpi_reset(cyccg, HPI_RESET_TYPE_DEVICE, HPI_SYNC,
			  callback, param);
}

int hpi_enter_flashing_mode(struct cyccg *cyccg, enum hpi_sync_mode sync_mode,
			    hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	u8 signature = HPI_SIGNATURE_ENTER_FLASHING_MODE;
	int err;

	hpi_vdbg("<<<< enter, %s\n",
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");
	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(hpidev, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_enter_flashing_mode,
			       &signature, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write ENTER_FLASHING_MODE register, %d\n",
			err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_enter_flashing_mode_sync(struct cyccg *cyccg)
{
	return hpi_enter_flashing_mode(cyccg, HPI_SYNC, NULL, NULL);
}

int hpi_enter_flashing_mode_async(struct cyccg *cyccg,
				  hpi_cmd_cb_t callback, void *param)
{
	return hpi_enter_flashing_mode(cyccg, HPI_ASYNC, callback, param);
}

int hpi_validate_fw(struct cyccg *cyccg, enum ccg_fw_mode_type fw_mode,
		    enum hpi_sync_mode sync_mode,
		    hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	u8 fw_id = (u8)fw_mode;
	int err;

	hpi_vdbg("<<<< enter, %s, fw_mode=%d\n",
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC", (int)fw_mode);
	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(hpidev, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_validate_fw,
			       &fw_id, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write VALIDATE_FW register, %d\n",
			err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_validate_fw_sync(struct cyccg *cyccg, enum ccg_fw_mode_type fw_mode)
{
	return hpi_validate_fw(cyccg, fw_mode, HPI_SYNC, NULL, NULL);
}

int hpi_validate_fw_async(struct cyccg *cyccg, enum ccg_fw_mode_type fw_mode,
			  hpi_cmd_cb_t callback, void *param)
{
	return hpi_validate_fw(cyccg, fw_mode, HPI_ASYNC, callback, param);
}

static enum hpi_msg_return _hpi_flash_row_read_filter(
		struct hpi_device *hpidev, struct hpi_msg *msg)
{
	struct hpi_command *cmd = &hpidev->cmd;
	u16 flash_row_size = hpidev->cyccg->ccg_info.flash_row_size;
	struct hpi_buffer *tmp_buf = &cmd->copy_buf;
	int err;

	hpi_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", msg->code, msg->len);
	if (IS_HPI_EVENT_MSG(msg->code))
		return HPI_MSG_RETURN_NONE;

	if (msg->code != HPI_PD_RESP_SUCCESS) {
		hpi_err("failed to send read flash cmd\n");
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, msg->code);
		goto end;
	}

	/* Read the flash row data to the temperal copy buffer. */
	err = _hpi_rw_flash_memory(hpidev,
			tmp_buf->head, flash_row_size, HPI_READ);
	if (err) {
		hpi_err("failed to read flash row data, %d\n", err);
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, err);
		goto end;
	}

	hpi_cmd_store_return_data(hpidev, tmp_buf->head, flash_row_size);
	hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);

end:
	return HPI_MSG_RETURN_HANDLED;
}

int hpi_flash_row_read(struct cyccg *cyccg, u16 row_num,
		       void *buf, size_t *size,
		       enum hpi_sync_mode sync_mode,
		       hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	struct hpi_flash_row_rw_reg rw_reg;
	int err;

	hpi_vdbg("<<<< enter, %s, row_num=%u\n",
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC", row_num);
	if (!buf || !size)
		return -EINVAL;

	rw_reg.signature = HPI_SIGNATURE_FLASH_ROW_READ_WRITE;
	rw_reg.command = HPI_FLASH_ROW_READ;
	rw_reg.row_num = cpu_to_le16(row_num);

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RAW);
	hpi_cmd_set_filter(hpidev, _hpi_flash_row_read_filter, 0, buf, size);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_flash_row,
			       &rw_reg, sizeof(struct hpi_flash_row_rw_reg),
			       HPI_WRITE);
	if (err) {
		hpi_err("failed write FLASH_ROW_READ_WRITE reg, %d\n", err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_flash_row_read_sync(struct cyccg *cyccg, u16 row_num,
			    void *buf, size_t *size)
{
	return hpi_flash_row_read(cyccg, row_num, buf, size, HPI_SYNC,
				  NULL, NULL);
}

int hpi_flash_row_read_async(struct cyccg *cyccg, u16 row_num,
			     void *buf, size_t *size,
			     hpi_cmd_cb_t callback, void *param)
{
	return hpi_flash_row_read(cyccg, row_num, buf, size, HPI_ASYNC,
				  callback, param);
}

int hpi_flash_row_write(struct cyccg *cyccg, u16 row_num,
			void *buf, size_t size,
			enum hpi_sync_mode sync_mode,
			hpi_cmd_cb_t callback, void *param)
{
	union hpi_intr_reg intr_reg;
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	struct hpi_flash_row_rw_reg rw_reg;
	int retries = HPI_TIME_OF(FLASH_ROW_WRITE, hpi_ver);
	int err;

	hpi_vdbg("<<<< enter, %s, row_num=%u\n",
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC", row_num);
	if (!buf || size != cyccg->ccg_info.flash_row_size)
		return -EINVAL;

	/*
	 * In bootload firmware flashing mode, the message queue would not
	 * support, so must wait the previous flash write intr been cleared
	 * before the next write command. Otherwise, the intr bit of the next
	 * flash row write command may be cleared together with the prevous
	 * command clear operation, so the next command would encounter
	 * timeout error.
	 */
	do {
		err = hpi_read_intr_reg(cyccg, &intr_reg);
		if (err) {
			hpi_err("failed to read intr_reg, %d\n", err);
			return err;
		}

		usleep_range(1000, 2000);
		if (intr_reg.dev_intr)
			hpi_dbg("intr_reg not cleared, wait, retires=%d\n",
				retries);
	} while (intr_reg.dev_intr && --retries);

	rw_reg.signature = HPI_SIGNATURE_FLASH_ROW_READ_WRITE;
	rw_reg.command = HPI_FLASH_ROW_WRITE;
	rw_reg.row_num = cpu_to_le16(row_num);

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(hpidev, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_write_data(hpidev, _hpi_rw_flash_memory, buf, size);
	if (err) {
		hpi_err("failed to write flash row data\n");
		goto err;
	}

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_flash_row,
			       &rw_reg, sizeof(struct hpi_flash_row_rw_reg),
			       HPI_WRITE);
	if (err) {
		hpi_err("failed write FLASH_ROW_READ_WRITE reg, %d\n", err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(FLASH_ROW_WRITE, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_flash_row_write_sync(struct cyccg *cyccg, u16 row_num,
			     void *buf, size_t size)
{
	return hpi_flash_row_write(cyccg, row_num, buf, size, HPI_SYNC,
				  NULL, NULL);
}

int hpi_flash_row_write_async(struct cyccg *cyccg, u16 row_num,
			      void *buf, size_t size,
			      hpi_cmd_cb_t callback, void *param)
{
	return hpi_flash_row_write(cyccg, row_num, buf, size, HPI_ASYNC,
				   callback, param);
}

int hpi_read_all_version(struct cyccg *cyccg,
			 struct hpi_ccg_fw_version *all_version)
{
	struct hpi_image_version *img_ver = &all_version->btldr;

	return _hpi_rw_all_version(&cyccg->hpi_dev, img_ver,
				   2 * sizeof(struct hpi_image_version),
				   HPI_READ);
}

int hpi_read_fw2_version(struct cyccg *cyccg,
			 struct hpi_ccg_fw_version *all_version)
{
	struct hpi_image_version *fw2_ver = &all_version->fw2_app;

	return _hpi_rw_fw2_version(&cyccg->hpi_dev, fw2_ver,
				   sizeof(struct hpi_image_version),
				   HPI_READ);
}

int hpi_read_fw_binary_location(struct cyccg *cyccg,
				u16 *fw1_start, u16 *fw2_start)
{
	u8 buf[4];
	int err;

	err = _hpi_rw_fw_binary_location(&cyccg->hpi_dev,
					 buf, sizeof(buf), HPI_READ);
	if (!err) {
		*fw1_start = get_unaligned_le16(&buf[0]);
		*fw2_start = get_unaligned_le16(&buf[2]);
	}

	return err;
}

int hpi_pdport_enable(struct cyccg *cyccg, u8 port_mask,
		      enum hpi_sync_mode sync_mode,
		      hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	int err;

	if ((port_mask >> cyccg->ccg_info.num_port) & 0xff) {
		hpi_err("non-existent ports bit set, 0x%02x\n", port_mask);
		return -EINVAL;
	}

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(hpidev, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_pdport_enable,
			       &port_mask, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write DPORT_ENABLE register, %d\n", err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(PORT_DISABLE, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	return err;
}

int hpi_read_pdport_bitmask(struct cyccg *cyccg, u8 *port_mask)
{
	int err;

	err = _hpi_rw_pdport_enable(&cyccg->hpi_dev, port_mask, sizeof(u8),
				    HPI_READ);
	if (!err)
		*port_mask &= (~(0xff << cyccg->ccg_info.num_port));
	return err;
}

/* The value of @port_index is start from 0. */

int hpi_disable_dpport(struct cyccg *cyccg, int port_index,
		       enum hpi_sync_mode sync_mode,
		       hpi_cmd_cb_t callback, void *param)

{
	struct hpi_device *port = cyccg->ports[port_index];
	u8 port_mask;
	int err;

	if (port_index >= cyccg->ccg_info.num_port ||
			!port || port->dev_type <= HPI_DEV_TYPE_DEVICE)
		return -EINVAL;

	err = hpi_read_pdport_bitmask(cyccg, &port_mask);
	if (err)
		return err;

	port_mask &= ~(0x01 << port_index);
	err = hpi_pdport_enable(cyccg, port_mask, sync_mode, callback, param);
	if (!err) {
		spin_lock(&port->slock);
		port->enabled = false;
		spin_unlock(&port->slock);
	}

	return err;
}

int hpi_disable_dpport_sync(struct cyccg *cyccg, int port_index)
{
	return hpi_disable_dpport(cyccg, port_index, HPI_SYNC, NULL, NULL);
}

int hpi_disable_dpport_async(struct cyccg *cyccg, int port_index,
			     hpi_cmd_cb_t callback, void *param)
{
	return hpi_disable_dpport(cyccg, port_index, HPI_ASYNC,
				  callback, param);
}

/* The value of @port_index is start from 0. */
int hpi_enable_dpport(struct cyccg *cyccg, int port_index,
		      enum hpi_sync_mode sync_mode,
		      hpi_cmd_cb_t callback, void *param)
{
	struct hpi_device *port = cyccg->ports[port_index];
	u8 port_mask;
	int err;

	if (port_index >= cyccg->ccg_info.num_port ||
			!port || port->dev_type <= HPI_DEV_TYPE_DEVICE)
		return -EINVAL;

	err = hpi_read_pdport_bitmask(cyccg, &port_mask);
	if (err)
		return err;

	port_mask |= (0x01 << port_index);
	err = hpi_pdport_enable(cyccg, port_mask, sync_mode, callback, param);
	if (!err) {
		spin_lock(&port->slock);
		port->enabled = false;
		spin_unlock(&port->slock);
	}

	return err;

}

int hpi_enable_dpport_sync(struct cyccg *cyccg, int port_index)
{
	return hpi_enable_dpport(cyccg, port_index, HPI_ASYNC, NULL, NULL);
}

int hpi_enable_dpport_async(struct cyccg *cyccg, int port_index,
			    hpi_cmd_cb_t callback, void *param)
{
	return hpi_enable_dpport(cyccg, port_index, HPI_ASYNC, callback, param);
}

int hpi_sleep_ctrl(struct cyccg *cyccg, bool enable_deep_sleep,
		   enum hpi_sync_mode sync_mode,
		   hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	u8 sleep_ctrl = enable_deep_sleep ?
			HPI_DEEP_SLEEP_ENABLED : HPI_DEEP_SLEEP_DISABLED;
	int err;

	hpi_vdbg("<<<< enter, %s, enable_deep_sleep=%s\n",
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		enable_deep_sleep ? "true" : "false");

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(hpidev, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_sleep_ctrl,
			       &sleep_ctrl, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write SLEEP_CTRL register, %d\n", err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_sleep_ctrl_sync(struct cyccg *cyccg, bool enable_deep_sleep,
			hpi_cmd_cb_t callback, void *param)
{
	return hpi_sleep_ctrl(cyccg, enable_deep_sleep, HPI_SYNC, NULL, NULL);
}

int hpi_sleep_ctrl_async(struct cyccg *cyccg, bool enable_deep_sleep,
			 hpi_cmd_cb_t callback, void *param)
{
	return hpi_sleep_ctrl(cyccg, enable_deep_sleep, HPI_ASYNC,
			      callback, param);
}

int hpi_read_sleep_ctrl(struct cyccg *cyccg, bool *is_deep_sleep_enabled)
{
	u8 sleep_ctrl;
	int err;

	err = _hpi_rw_sleep_ctrl(&cyccg->hpi_dev, &sleep_ctrl, sizeof(u8),
				 HPI_READ);
	if (!err) {
		sleep_ctrl &= HPI_DEEP_SLEEP_MASK;
		*is_deep_sleep_enabled =
			sleep_ctrl == HPI_DEEP_SLEEP_ENABLED ? true : false;
	}

	return err;
}

int hpi_set_battery_state(struct cyccg *cyccg, bool enable_dead_battery_ops,
			  enum hpi_sync_mode sync_mode,
			  hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	u8 battery_state = enable_dead_battery_ops ?
			HPI_DEAD_BATTERY_ENABLED : HPI_DEAD_BATTERY_DISABLED;
	int err;

	hpi_vdbg("<<<< enter, %s, enable_dead_battery_ops=%s\n",
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		enable_dead_battery_ops ? "true" : "false");

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(hpidev, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_battery_stat,
			       &battery_state, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write BATTERY_STAT register, %d\n", err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

int hpi_set_battery_state_sync(struct cyccg *cyccg,
			       bool enable_dead_battery_ops)
{
	return hpi_set_battery_state(cyccg, enable_dead_battery_ops, HPI_SYNC,
			NULL, NULL);
}

int hpi_set_battery_state_async(struct cyccg *cyccg,
				bool enable_dead_battery_ops,
				enum hpi_sync_mode sync_mode,
				hpi_cmd_cb_t callback, void *param)
{
	return hpi_set_battery_state(cyccg, enable_dead_battery_ops, HPI_ASYNC,
			callback, param);
}

int hpi_read_battery_state(struct cyccg *cyccg,
			   bool *is_dead_battery_ops_enabled)
{
	u8 battery_state;
	int err;

	err = _hpi_rw_sleep_ctrl(&cyccg->hpi_dev, &battery_state, sizeof(u8),
				 HPI_READ);
	if (!err) {
		battery_state &= HPI_BATTERY_STAT_MASK;
		*is_dead_battery_ops_enabled =
			(battery_state == HPI_DEAD_BATTERY_ENABLED) ?
				true : false;
	}

	return err;
}

int hpi_set_app_priority(struct cyccg *cyccg,
			 enum hpi_app_priority app_priority,
			 enum hpi_sync_mode sync_mode,
			 hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	u8 value = (u8)app_priority;
	int err;

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(hpidev, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_app_priority,
			       &value, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write SET_APP_PRIORITY register, %d\n", err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	return err;
}

int hpi_set_app_priority_sync(struct cyccg *cyccg,
			 enum hpi_app_priority app_priority)
{
	return hpi_set_app_priority(cyccg, app_priority, HPI_SYNC, NULL, NULL);
}

int hpi_set_app_priority_async(struct cyccg *cyccg,
			 enum hpi_app_priority app_priority,
			 enum hpi_sync_mode sync_mode,
			 hpi_cmd_cb_t callback, void *param)
{
	return hpi_set_app_priority(cyccg, app_priority, HPI_SYNC,
				    callback, param);
}

static enum hpi_msg_return _hpi_read_customer_info_filter(
		struct hpi_device *hpidev, struct hpi_msg *msg)
{
	enum hpi_version hpi_ver = hpidev->cyccg->ccg_info.hpi_ver;
	u16 data_Size = HPI_REG_SIZE_OF(CUSTOMER_INFO_DATA, hpi_ver);
	struct hpi_buffer *tmp_buf = &hpidev->cmd.copy_buf;
	int err;

	hpi_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", msg->code, msg->len);
	if (IS_HPI_EVENT_MSG(msg->code))
		return HPI_MSG_RETURN_NONE;

	if (msg->code != HPI_PD_RESP_SUCCESS) {
		hpi_err("failed to send read customer info cmd\n");
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, msg->code);
		goto end;
	}

	hpi_cmd_set_state_errcode(hpidev, HPI_CMD_RESPONSE_RECEIVED, 0);

	/* Read the flash row data to the temperal copy buffer. */
	err = _hpi_rw_customer_info(hpidev, tmp_buf->head, data_Size, HPI_READ);
	if (err) {
		hpi_err("failed to read customer info data, %d\n", err);
		hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, err);
		goto end;
	}

	hpi_cmd_store_return_data(hpidev, tmp_buf->head, data_Size);
	hpi_cmd_set_state_errcode(hpidev, HPI_CMD_COMPLETED, 0);

end:
	return HPI_MSG_RETURN_HANDLED;
}

int hpi_read_customer_info(struct cyccg *cyccg, void *buf, size_t *size,
			   enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	struct hpi_device *hpidev = &cyccg->hpi_dev;
	u8 signature = HPI_SIGNATURE_READ_CUSTOMER_INFO;
	int err;

	hpi_vdbg("<<<< enter, %s\n",
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");
	if (sync_mode == HPI_SYNC && (!buf || !size)) {
		hpi_err("invalid input parameteres\n");
		return -EINVAL;
	}

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RAW);
	hpi_cmd_set_filter(hpidev, _hpi_read_customer_info_filter, 0,
				buf, size);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	err = hpi_cmd_send_cmd(hpidev, _hpi_rw_read_customer_info_reg,
			       &signature, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_err("failed to write READ_CUSTOMER_INFO register, %d\n",
			err);
		goto err;
	}

	err = hpi_cmd_sync(hpidev, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(hpidev);
	hpi_vdbg(">>>> exit, %d\n", err);
	return err;
}

void hpi_dump_pending_events(struct hpi_device *hpidev)
{
	struct hpi_command *hpi_cmd = &hpidev->cmd;
	struct hpi_msg *msg = &hpi_cmd->msg;
	char name[USBC_PDPORT_NAME_SIZE];
	int idle_count = 0;
	int err;

	if (!hpidev || hpidev->dev_type == HPI_DEV_TYPE_UNKNOWN)
		return;

	if (hpidev->dev_type == HPI_DEV_TYPE_PORT)
		strcpy(name, hpidev->pdport_name_addr->name);
	else
		strcpy(name, "CCG_DEVICE");

	hpi_cmd_init_lock(hpidev, HPI_SYNC, HPI_CMD_FLAG_RAW);
	disable_irq(hpidev->cyccg->irq);

	do {
		err = hpi_message_read(hpidev, msg);
		if (err) {
			hpi_port_err("failed to read hpi message, %d\n",
				hpidev, err);
			break;
		}

		hpi_port_dbg("events dump, msg->code=0x%x, len=%zu\n",
			hpidev, msg->code, msg->len);

		if (msg->code == HPI_PD_RESP_NO_RESPONSE) {
			if (idle_count++)
				break;
			continue;
		}

		idle_count = 0;
		hpi_clear_intr(hpidev);
	} while (true);

	enable_irq(hpidev->cyccg->irq);
	hpi_cmd_deinit_unlock(hpidev);
}

/*
 * HPI Port interfaces.
 */
int hpi_port_send_vdm_data_generic(struct hpi_device *port,
		enum vdm_sop_type vdm_mode, enum hpi_cmd_flag cmd_flag,
		void *vdm_data, size_t vdm_size,
		void *ret_vdm_data, size_t *ret_vdm_data_size,
		enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param,
		hpi_cmd_filter_t special_filter, unsigned long timeout)
{
	struct hpi_vdm_ctrl vdm_ctrl;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");
	if (!vdm_data || vdm_size < VDM_VDO_OBJ_SIZE ||
			vdm_size > VDM_MAX_MSG_SIZE ||
			(vdm_size % VDM_VDO_OBJ_SIZE) != 0) {
		hpi_port_err("invalid VDM command buffer=0x%p and size=%zu\n",
			port, vdm_data, vdm_size);
		return -EINVAL;
	}

	hpi_cmd_init_lock(port, sync_mode, cmd_flag);
	hpi_cmd_set_filter(port, special_filter, 0,
			   ret_vdm_data, ret_vdm_data_size);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_write_data(port, _hpi_port_rw_vdm_data,
				 vdm_data, vdm_size);
	if (err) {
		hpi_port_err("failed to write VDM data to data memory, %d\n",
			port, err);
		goto err;
	}

	if (ret_vdm_data && ret_vdm_data_size && *ret_vdm_data_size)
		memset(ret_vdm_data, 0, *ret_vdm_data_size);

	vdm_ctrl.vdm_mode = vdm_mode;
	vdm_ctrl.length = vdm_size;
	err = hpi_cmd_send_cmd(port, _hpi_port_rw_vdm_ctrl,
			       &vdm_ctrl, sizeof(vdm_ctrl), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write VDM_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, timeout);

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

static enum hpi_msg_return _hpi_port_vdm_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	hpi_port_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", port,
		msg->code, msg->len);
	if (msg->code != HPI_PD_RESP_VDM_RECEIVED ||
			hpi_cmd_get_state(port) != HPI_CMD_RESPONSE_RECEIVED)
		return HPI_MSG_RETURN_NONE;

	/* VDM data response received */
	hpi_cmd_copy_return_data(port, msg);
	hpi_port_dump("VDM response data received (%zu):\n", port,
		msg->data, msg->len, msg->len);

	hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
	return HPI_MSG_RETURN_HANDLED;
}

int hpi_port_send_vdm_data(struct hpi_device *port,
			   enum vdm_sop_type vdm_mode,
			   void *vdm_data, size_t vdm_size,
			   void *ret_vdm_data, size_t *ret_vdm_data_size,
			   enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;

	return hpi_port_send_vdm_data_generic(port,
			vdm_mode, HPI_CMD_FLAG_RESP_EVENT,
			vdm_data, vdm_size,
			ret_vdm_data, ret_vdm_data_size,
			sync_mode, callback, param,
			_hpi_port_vdm_filter, HPI_TIME_OF(DEFAULT, hpi_ver));
}

int hpi_port_send_vdm_data_sync(struct hpi_device *port,
				enum vdm_sop_type vdm_mode,
				void *vdm_data, size_t vdm_size,
				void *ret_vdm_data, size_t *ret_vdm_data_size)
{
	return hpi_port_send_vdm_data(port, vdm_mode, vdm_data, vdm_size,
				      ret_vdm_data, ret_vdm_data_size,
				      HPI_SYNC, NULL, NULL);
}

int hpi_port_send_vdm_data_async(struct hpi_device *port,
				 enum vdm_sop_type vdm_mode,
				 void *vdm_data, size_t vdm_size,
				 hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_send_vdm_data(port, vdm_mode, vdm_data, vdm_size,
				      NULL, NULL, HPI_ASYNC, callback, param);
}

int hpi_port_read_effective_source_pdo_mask(struct hpi_device *port, u8 *mask)
{
	return _hpi_port_rw_effective_source_pdo_mask(port, mask, sizeof(u8),
						      HPI_READ);
}

int hpi_port_read_effective_sink_pdo_mask(struct hpi_device *port, u8 *mask)
{
	return _hpi_port_rw_effective_sink_pdo_mask(port, mask, sizeof(u8),
						    HPI_READ);
}

/*
 * hpi_port_select_source_pdo - select which PDOs should be actived when PD
 *     contract established based on the default source PDOs existing in
 *     configuration table or based on the specificed updated source PDOs list.
 * @port: instance of the USB-C port in this driver
 * @mask: bit mask indicates which source PDOs will be actived by default
 * @pdo_list_buf: specifices the new source PDOs list data should be used by CCG
 * @pdo_list_buf_size: indicates the buffer size in bytes of @pdo_list_buf
 * @sync_mode: this function should called in sync or async mode.
 *
 * Note, for CCG1/CCG2, the @pdo_list_buf and @pdo_list_buf_size are not used,
 * they must be set to NULL and 0. For CCG3/CCG4, @pdo_list_buf and
 * @pdo_list_buf_size should be set to the specificed PDO list data and size
 * if it exists. Otherwise, they also must be set to NULL and 0.
 * For the PDO list data, it can be read out using hpi_port_read_source_pdo()
 * function, then modify and write it back to CCG.
 */
int hpi_port_select_source_pdo(struct hpi_device *port, u8 mask,
			       struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			       size_t pdo_list_buf_size,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s, mask=0x%02x\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC", mask);

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	/* Use used dynamic write Source PDOs instead of default if set. */
	if (pdo_list_buf_size >= sizeof(struct hpi_pd_src_sink_pdo_list) &&
			hpi_ver > HPI_VERSION_1 && pdo_list_buf &&
			pdo_list_buf->signature_pdo_data_type ==
				HPI_SIGNATURE_SELECT_SOURCE_PDO) {
		err = hpi_cmd_write_data(port, _hpi_port_rw_data_memory,
			pdo_list_buf, sizeof(struct hpi_pd_src_sink_pdo_list));
		if (err) {
			hpi_port_err("failed to write data memory reg, %d\n",
				port, err);
			goto err;
		}
	}

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_select_source_pdo,
			       &mask, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write SELECT_SOURCE_PDO reg, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_select_source_pdo_sync(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size)
{
	return hpi_port_select_source_pdo(port, mask,
			pdo_list_buf, pdo_list_buf_size,
			HPI_SYNC, NULL, NULL);
}

int hpi_port_select_source_pdo_async(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size,
			hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_select_source_pdo(port, mask,
			pdo_list_buf, pdo_list_buf_size,
			HPI_ASYNC, callback, param);
}

/*
 * hpi_port_select_source_pdo - select which PDOs should be actived when PD
 *     contract established based on the default sink PDOs existing in
 *     configuration table or based on the specificed updated sink PDOs list.
 * @port: instance of the USB-C port in this driver
 * @mask: bit mask indicates which sink PDOs will be actived by default
 * @pdo_list_buf: specifices the new sink PDOs list data should be used by CCG
 * @pdo_list_buf_size: indicates the buffer size in bytes of @pdo_list_buf
 * @sync_mode: this function should called in sync or async mode.
 *
 * Note, for CCG1/CCG2, the @pdo_list_buf and @pdo_list_buf_size are not used,
 * they must be set to NULL and 0. For CCG3/CCG4, @pdo_list_buf and
 * @pdo_list_buf_size should be set to the specificed PDO list data and size
 * if it exists. Otherwise, they also must be set to NULL and 0.
 * For the PDO list data, it can be read out using hpi_port_read_sink_pdo()
 * function, then modify and write it back to CCG.
 */
int hpi_port_select_sink_pdo(struct hpi_device *port, u8 mask,
			     struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			     size_t pdo_list_buf_size,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s, mask=0x%02x\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC", mask);

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	if (pdo_list_buf_size >= sizeof(struct hpi_pd_src_sink_pdo_list) &&
			hpi_ver > HPI_VERSION_1 && pdo_list_buf &&
			pdo_list_buf->signature_pdo_data_type ==
				HPI_SIGNATURE_SELECT_SINK_PDO) {
		err = hpi_cmd_write_data(port, _hpi_port_rw_data_memory,
			pdo_list_buf, sizeof(struct hpi_pd_src_sink_pdo_list));
		if (err) {
			hpi_port_err("failed to write data memory reg, %d\n",
				port, err);
			goto err;
		}
	}

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_select_sink_pdo,
			       &mask, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write SELECT_SINK_PDO reg, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_select_sink_pdo_sync(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size)
{
	return hpi_port_select_sink_pdo(port, mask,
			pdo_list_buf, pdo_list_buf_size,
			HPI_SYNC, NULL, NULL);
}

int hpi_port_select_sink_pdo_async(struct hpi_device *port, u8 mask,
			struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			size_t pdo_list_buf_size,
			hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_select_sink_pdo(port, mask,
			pdo_list_buf, pdo_list_buf_size,
			HPI_ASYNC, callback, param);
}

int hpi_port_read_pd_status(struct hpi_device *port,
			    struct hpi_pd_status *pd_status)
{
	return _hpi_port_rw_pd_status(port, pd_status,
			sizeof(struct hpi_pd_status), HPI_READ);
}

int hpi_port_read_type_c_status(struct hpi_device *port,
				struct hpi_type_c_status *type_c_status)
{
	return _hpi_port_rw_type_c_status(port, type_c_status,
			sizeof(struct hpi_type_c_status), HPI_READ);
}

__maybe_unused
static inline char *pd_ctrl_cmd_to_string(enum hpi_pd_ctrl_command pd_ctrl_cmd)
{
	switch (pd_ctrl_cmd) {
	case HPI_PD_CTRL_CMD_SET_TYPE_C_DEFAULT_PROFILE:
		return "HPI_PD_CTRL_CMD_SET_TYPE_C_DEFAULT_PROFILE";
	case HPI_PD_CTRL_CMD_SET_TYPE_C_1_5_A_PROFILE:
		return "HPI_PD_CTRL_CMD_SET_TYPE_C_1_5_A_PROFILE";
	case HPI_PD_CTRL_CMD_SET_TYPE_C_3_A_PROFILE:
		return "HPI_PD_CTRL_CMD_SET_TYPE_C_3_A_PROFILE";
	case HPI_PD_CTRL_CMD_TRIGGER_DATA_ROLE_SWAP:
		return "HPI_PD_CTRL_CMD_TRIGGER_DATA_ROLE_SWAP";
	case HPI_PD_CTRL_CMD_TRIGGER_POWER_ROLE_SWAP:
		return "HPI_PD_CTRL_CMD_TRIGGER_POWER_ROLE_SWAP";
	case HPI_PD_CTRL_CMD_SWITCH_ON_VCONN:
		return "HPI_PD_CTRL_CMD_SWITCH_ON_VCONN";
	case HPI_PD_CTRL_CMD_SWITCH_OFF_VCONN:
		return "HPI_PD_CTRL_CMD_SWITCH_OFF_VCONN";
	case HPI_PD_CTRL_CMD_TRIGGER_VCONN_ROLE_SWAP:
		return "HPI_PD_CTRL_CMD_TRIGGER_VCONN_ROLE_SWAP";
	case HPI_PD_CTRL_CMD_RETRIEVE_SOURCE_CAPABILITY:
		return "HPI_PD_CTRL_CMD_RETRIEVE_SOURCE_CAPABILITY";
	case HPI_PD_CTRL_CMD_RETRIEVE_SINK_CAPABILITY:
		return "HPI_PD_CTRL_CMD_RETRIEVE_SINK_CAPABILITY";
	case HPI_PD_CTRL_CMD_SEND_GOTOMIN_MESSAGE:
		return "HPI_PD_CTRL_CMD_SEND_GOTOMIN_MESSAGE";
	case HPI_PD_CTRL_CMD_SEND_HARD_RESET:
		return "HPI_PD_CTRL_CMD_SEND_HARD_RESET";
	case HPI_PD_CTRL_CMD_SEND_SOFT_RESET:
		return "HPI_PD_CTRL_CMD_SEND_SOFT_RESET";
	case HPI_PD_CTRL_CMD_SEND_CABLE_RESET:
		return "HPI_PD_CTRL_CMD_SEND_CABLE_RESET";
	case HPI_PD_CTRL_CMD_EC_INITIALIZATION_COMPLETE:
		return "HPI_PD_CTRL_CMD_EC_INITIALIZATION_COMPLETE";
	case HPI_PD_CTRL_CMD_PORT_DISABLE:
		return "HPI_PD_CTRL_CMD_PORT_DISABLE";
	case HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_PRIME:
		return "HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_PRIME";
	case HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_DPRIME:
		return "HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_DPRIME";
	case HPI_PD_CTRL_CMD_CHANGE_PD_PORT_PARAMETERS:
		return "HPI_PD_CTRL_CMD_CHANGE_PD_PORT_PARAMETERS";
	case HPI_PD_CTRL_CMD_ABORT_PENDING_PD_COMMAND:
		return "HPI_PD_CTRL_CMD_ABORT_PENDING_PD_COMMAND";
	case HPI_PD_CTRL_CMD_READ_SOURCE_PDOS:
		return "HPI_PD_CTRL_CMD_READ_SOURCE_PDOS";
	case HPI_PD_CTRL_CMD_READ_SINK_PDOS:
		return "HPI_PD_CTRL_CMD_READ_SINK_PDOS";
	default:
		break;
	}

	return "UNKNOWN_PD_CTRL_CMD";
}

int hpi_port_pd_control(struct hpi_device *port,
			enum hpi_pd_ctrl_command pd_ctrl_cmd,
			enum hpi_sync_mode sync_mode,
			hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 ctrl_cmd = (u8)pd_ctrl_cmd;
	int err;

	hpi_port_vdbg("<<<< enter, %s, pd_ctrl_cmd=%s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		pd_ctrl_cmd_to_string(pd_ctrl_cmd));

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_pd_control,
			       &ctrl_cmd, sizeof(ctrl_cmd), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_set_type_c_default_profile_sync(struct hpi_device *port)
{
	return hpi_port_pd_control(port,
			HPI_PD_CTRL_CMD_SET_TYPE_C_DEFAULT_PROFILE,
			HPI_SYNC, NULL, NULL);
}

int hpi_port_set_type_c_default_profile_async(struct hpi_device *port,
					      hpi_cmd_cb_t callback,
					      void *param)
{
	return hpi_port_pd_control(port,
			HPI_PD_CTRL_CMD_SET_TYPE_C_DEFAULT_PROFILE,
			HPI_ASYNC, callback, param);
}

int hpi_port_set_type_c_1_5_A_profile(struct hpi_device *port,
				      enum hpi_sync_mode sync_mode,
				      hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_pd_control(port,
		HPI_PD_CTRL_CMD_SET_TYPE_C_1_5_A_PROFILE, sync_mode,
		callback, param);
}

int hpi_port_set_type_c_3_A_profile(struct hpi_device *port,
				    enum hpi_sync_mode sync_mode,
				    hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_pd_control(port,
		HPI_PD_CTRL_CMD_SET_TYPE_C_3_A_PROFILE, sync_mode,
		callback, param);
}

static inline enum hpi_pd_ctrl_command _hpi_swap_type_to_pd_ctrl_cmd(
		struct hpi_swap_status *swap_status)
{
	enum hpi_swap_type swap_type =
			(enum hpi_swap_type)swap_status->swap_type;

	switch (swap_type) {
	case HPI_DR_SWAP:
		return HPI_PD_CTRL_CMD_TRIGGER_DATA_ROLE_SWAP;
	case HPI_PR_SWAP:
		return HPI_PD_CTRL_CMD_TRIGGER_POWER_ROLE_SWAP;
	case HPI_VCONN_SWAP:
		return HPI_PD_CTRL_CMD_TRIGGER_VCONN_ROLE_SWAP;
	default:
		/*
		 * Should not happen, just placehold which
		 * will cause mismatch error.
		 */
		return HPI_PD_CTRL_CMD_SET_TYPE_C_DEFAULT_PROFILE;
	}
}

static enum hpi_msg_return _hpi_port_swap_complete_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	enum hpi_pd_ctrl_command pd_ctrl_cmd;
	struct hpi_swap_status *swap_status;

	hpi_port_vdbg("<<<< enter\n", port);
	hpi_port_dbg("msg->code=0x%x, len=%zu\n", port, msg->code, msg->len);
	switch (msg->code) {
	case HPI_PD_RESP_ACCEPT_MESSAGE:
		hpi_port_dbg("Accept message received\n", port);
		hpi_cmd_set_state_errcode(port, HPI_CMD_PRE_EVENTS_RCVD, 0);
		break;

	case HPI_PD_RESP_SWAP_COMPLETE:
		hpi_port_dbg("Swap complete message received\n", port);
		if (msg->len < HPI_REG_SIZE_OF(PORT_SWAP_RESPONSE, hpi_ver)) {
			hpi_cmd_set_state_errcode(port,
					HPI_CMD_COMPLETED, -EINVAL);
			break;
		}

		pd_ctrl_cmd =
			(enum hpi_pd_ctrl_command)hpi_cmd_get_cmd_id(port);
		swap_status = (struct hpi_swap_status *)msg->data;
		hpi_port_dbg("pd_ctrl_cmd=0x%02x\n", port, pd_ctrl_cmd);
		hpi_port_dbg("swap_pd_cmd=0x%02x, swap_resp_code=0x%02x\n",
			port, _hpi_swap_type_to_pd_ctrl_cmd(swap_status),
			swap_status->swap_resp_code);
		if (_hpi_swap_type_to_pd_ctrl_cmd(swap_status) == pd_ctrl_cmd) {
			if (swap_status->swap_resp_code == SWAP_RESP_ACCEPT) {
				hpi_cmd_set_state_errcode(port,
						HPI_CMD_COMPLETED, 0);
			} else {
				hpi_cmd_set_state_errcode(port,
						HPI_CMD_COMPLETED, -EFAULT);
			}
		} else {
			/*
			 * When do PR_SWAP command, the data role will also be
			 * swapped in some platforms, so there will be two
			 * SWAP_COMPLETE events.
			 * The first is the SWAP_COMPLETE of data role swap,
			 * the second is the SWAP_COMPLETE of power role swap.
			 * If the SWAP type not match with the trigger reason,
			 * skip it and continue wait for the correct
			 * SWAP_COMPLETE event.
			 */
			hpi_port_dbg("SWAP Type not match SWAP reason, skip\n",
				port);
			hpi_cmd_set_state_errcode(port,
					HPI_CMD_PRE_EVENTS_RCVD, 0);
		}

		break;
	default:
		return HPI_MSG_RETURN_NONE;
	}

	return HPI_MSG_RETURN_HANDLED;
}

static int _hpi_port_role_swap(struct hpi_device *port,
			enum hpi_pd_ctrl_command pd_ctrl_swap_cmd,
			enum hpi_sync_mode sync_mode,
			hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 swap_cmd = (u8)pd_ctrl_swap_cmd;
	int err;

	hpi_port_vdbg("<<<< enter\n", port);
	hpi_port_dbg("%s, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		pd_ctrl_cmd_to_string(pd_ctrl_swap_cmd));

	/* Data Role swap not supported when in Alternate Mode. */
	if (pd_ctrl_swap_cmd == HPI_PD_CTRL_CMD_TRIGGER_DATA_ROLE_SWAP) {
		spin_lock(&port->slock);
		if (port->is_in_alt_mode) {
			spin_unlock(&port->slock);
			hpi_port_err("port working in alternate mode, -EBUSY\n",
				port);
			return -EBUSY;
		}
		spin_unlock(&port->slock);
	}

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_EVENT);
	hpi_cmd_set_filter(port, _hpi_port_swap_complete_filter,
			   (uint)pd_ctrl_swap_cmd, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	/* Will be auto reset after the command completed. */
	spin_lock(&port->slock);
	port->is_swap_triggered_by_host = true;
	spin_unlock(&port->slock);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_pd_control,
			       &swap_cmd, sizeof(swap_cmd), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_data_role_swap_sync(struct hpi_device *port)
{
	return _hpi_port_role_swap(port,
			HPI_PD_CTRL_CMD_TRIGGER_DATA_ROLE_SWAP, HPI_SYNC,
			NULL, NULL);
}

int hpi_port_data_role_swap_async(struct hpi_device *port,
				  hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_role_swap(port,
			HPI_PD_CTRL_CMD_TRIGGER_DATA_ROLE_SWAP, HPI_ASYNC,
			callback, param);
}

int hpi_port_power_role_swap_sync(struct hpi_device *port)
{
	return _hpi_port_role_swap(port,
			HPI_PD_CTRL_CMD_TRIGGER_POWER_ROLE_SWAP, HPI_SYNC,
			NULL, NULL);
}

int hpi_port_power_role_swap_async(struct hpi_device *port,
				   hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_role_swap(port,
			HPI_PD_CTRL_CMD_TRIGGER_POWER_ROLE_SWAP, HPI_ASYNC,
			callback, param);
}

int hpi_port_vconn_role_swap_sync(struct hpi_device *port)
{
	return _hpi_port_role_swap(port,
			HPI_PD_CTRL_CMD_TRIGGER_VCONN_ROLE_SWAP, HPI_SYNC,
			NULL, NULL);
}

int hpi_port_vconn_role_swap_async(struct hpi_device *port,
				   hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_role_swap(port,
			HPI_PD_CTRL_CMD_TRIGGER_VCONN_ROLE_SWAP, HPI_ASYNC,
			callback, param);
}

int hpi_port_switch_vconn_sync(struct hpi_device *port, bool sourcing_on)
{
	enum hpi_pd_ctrl_command pd_ctrl_cmd =
		sourcing_on ? HPI_PD_CTRL_CMD_SWITCH_ON_VCONN :
			HPI_PD_CTRL_CMD_SWITCH_OFF_VCONN;

	return hpi_port_pd_control(port, pd_ctrl_cmd, HPI_SYNC, NULL, NULL);
}

int hpi_port_switch_vconn_async(struct hpi_device *port, bool sourcing_on,
				hpi_cmd_cb_t callback, void *param)
{
	enum hpi_pd_ctrl_command pd_ctrl_cmd =
		sourcing_on ? HPI_PD_CTRL_CMD_SWITCH_ON_VCONN :
			HPI_PD_CTRL_CMD_SWITCH_OFF_VCONN;

	return hpi_port_pd_control(port, pd_ctrl_cmd, HPI_ASYNC,
				   callback, param);
}

static enum hpi_msg_return _hpi_port_retrieve_capabilities_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	enum hpi_pd_ctrl_command pd_ctrl_cmd;

	hpi_port_dump("<<<< enter, msg->code=0x%x, len=%zu\n", port,
		msg->data, msg->len, msg->code, msg->len);

	pd_ctrl_cmd = (enum hpi_pd_ctrl_command)hpi_cmd_get_cmd_id(port);
	hpi_port_vdbg("pd_ctrl_cmd = 0x%x\n", port, pd_ctrl_cmd);
	if (pd_ctrl_cmd == HPI_PD_CTRL_CMD_RETRIEVE_SOURCE_CAPABILITY &&
			msg->code == HPI_PD_RESP_SRC_CAP_RCVD) {
		/* Source capability received. */
		hpi_cmd_copy_return_data(port, msg);
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		hpi_port_vdbg("source capabilities received\n", port);
	} else if (pd_ctrl_cmd == HPI_PD_CTRL_CMD_RETRIEVE_SINK_CAPABILITY &&
			msg->code == HPI_PD_RESP_SINK_CAP_RCVD) {
		/* Sink capability received. */
		hpi_cmd_copy_return_data(port, msg);
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		hpi_port_vdbg("sink capabilities received\n", port);
	} else {
		return HPI_MSG_RETURN_NONE;
	}

	return HPI_MSG_RETURN_HANDLED;
}

static int _hpi_port_retrieve_partner_capabilities(
		struct hpi_device *port,
		enum hpi_pd_ctrl_command pd_ctrl_retrieve_cap_cmd,
		void *buf, size_t *size, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 cap_cmd = (u8)pd_ctrl_retrieve_cap_cmd;
	int err;

	hpi_port_vdbg("<<<< enter, %s, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		pd_ctrl_cmd_to_string(pd_ctrl_retrieve_cap_cmd));

	if (sync_mode == HPI_SYNC && (!buf && !size &&
			*size < sizeof(struct hpi_capabilities_message))) {
		hpi_port_err("invalid input parameters\n", port);
		return -EINVAL;
	}

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_EVENT);
	hpi_cmd_set_filter(port, _hpi_port_retrieve_capabilities_filter,
			   (uint)cap_cmd, buf, size);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_pd_control,
			       &cap_cmd, sizeof(cap_cmd), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_get_partner_source_capabilities_sync(
		struct hpi_device *port, void *buf, size_t *size)
{
	return _hpi_port_retrieve_partner_capabilities(port,
			HPI_PD_CTRL_CMD_RETRIEVE_SOURCE_CAPABILITY,
			buf, size, HPI_SYNC, NULL, NULL);
}

int hpi_port_get_partner_source_capabilities_async(
		struct hpi_device *port, hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_retrieve_partner_capabilities(port,
			HPI_PD_CTRL_CMD_RETRIEVE_SOURCE_CAPABILITY,
			NULL, NULL, HPI_ASYNC, callback, param);
}

int hpi_port_get_partner_sink_capabilities_sync(
		struct hpi_device *port, void *buf, size_t *size)
{
	return _hpi_port_retrieve_partner_capabilities(port,
			HPI_PD_CTRL_CMD_RETRIEVE_SINK_CAPABILITY,
			buf, size, HPI_SYNC, NULL, NULL);
}

int hpi_port_get_partner_sink_capabilities_async(
		struct hpi_device *port, hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_retrieve_partner_capabilities(port,
			HPI_PD_CTRL_CMD_RETRIEVE_SINK_CAPABILITY,
			NULL, NULL, HPI_ASYNC, callback, param);
}

int hpi_port_send_gotomin_sync(struct hpi_device *port)
{
	return hpi_port_pd_control(port,
			HPI_PD_CTRL_CMD_SEND_GOTOMIN_MESSAGE, HPI_SYNC,
			NULL, NULL);
}

int hpi_port_send_gotomin_async(struct hpi_device *port,
				hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_pd_control(port,
			HPI_PD_CTRL_CMD_SEND_GOTOMIN_MESSAGE, HPI_ASYNC,
			callback, param);
}

static enum hpi_msg_return _hpi_port_send_reset_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	enum hpi_msg_return ret = HPI_MSG_RETURN_HANDLED;
	enum hpi_pd_ctrl_command pd_cmd;

	hpi_port_vdbg("<<<< enter\n", port);
	hpi_port_dbg("msg->code=0x%x, len=%zu\n", port, msg->code, msg->len);

	pd_cmd = (enum hpi_pd_ctrl_command)hpi_cmd_get_cmd_id(port);
	switch (msg->code) {
	case HPI_PD_HARD_RESET_SENT:
		hpi_port_dbg("HPI_PD_HARD_RESET_SENT, pd_cmd=%d\n",
			port, pd_cmd);
		if (pd_cmd == HPI_PD_CTRL_CMD_SEND_HARD_RESET) {
			/* Success done.*/
			hpi_port_dbg("hard reset sent success\n", port);
			hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		} else if (pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET) {
			/* Failed to do soft reset, hard reset was applied. */
			hpi_port_dbg("Soft reset failed, hard reset used\n",
				port);
			hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		}

		break;
	case HPI_PD_SOFT_RESET_SENT:
		hpi_port_dbg("HPI_PD_SOFT_RESET_SENT, pd_cmd=%d\n",
			port, pd_cmd);
		if (pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET ||
		    pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_PRIME ||
		    pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_DPRIME) {
			/*
			 * Soft reset send success,
			 * now wait for port partner / EMCA ACCEPT response.
			 */
			hpi_cmd_set_state_errcode(port,
					HPI_CMD_PRE_EVENTS_RCVD, 0);
			hpi_port_dbg("soft reset sent success, pd_cmd=%d\n",
				port, pd_cmd);
		}

		break;
	case HPI_PD_CABLE_RESET_SENT:
		hpi_port_dbg("HPI_PD_CABLE_RESET_SENT, pd_cmd=%d\n",
			port, pd_cmd);
		if (pd_cmd == HPI_PD_CTRL_CMD_SEND_CABLE_RESET) {
			/* Cable reset send success, done. */
			hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		}

		break;
	case HPI_PD_RESP_ACCEPT_MESSAGE:
		hpi_port_dbg("HPI_PD_RESP_ACCEPT_MESSAGE, pd_cmd=%d\n",
			port, pd_cmd);
		if ((pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET ||
		     pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_PRIME ||
		     pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_DPRIME) &&
		    (hpi_cmd_get_state(port) == HPI_CMD_PRE_EVENTS_RCVD)) {
			/* Soft reset successfully accepted and done. */
			hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
		}

		break;
	case HPI_PD_SENDER_RESPONSE_TIMER_TIMEOUT:
		hpi_port_dbg(
			"HPI_PD_SENDER_RESPONSE_TIMER_TIMEOUT, pd_cmd=%d\n",
			port, pd_cmd);
		if (pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET ||
		    pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_PRIME ||
		    pd_cmd == HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_DPRIME) {
			/* Port partner has no response, timeout. done */
			hpi_cmd_set_state_errcode(port,
					HPI_CMD_TIMEOUT, msg->code);
		}

		break;
	default:
		return HPI_MSG_RETURN_NONE;
	}

	hpi_port_vdbg(">>>> exit\n", port);
	return ret;
}


static int _hpi_port_send_hard_reset(struct hpi_device *port,
			enum hpi_pd_ctrl_command pd_ctrl_send_reset_cmd,
			enum hpi_sync_mode sync_mode,
			hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 send_reset_cmd = (u8)pd_ctrl_send_reset_cmd;
	size_t send_reset_cmd_size = sizeof(u8);
	int err;

	hpi_port_vdbg("<<<< enter, %s, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		pd_ctrl_cmd_to_string(pd_ctrl_send_reset_cmd));

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_EVENT_RAW);
	hpi_cmd_set_filter(port, _hpi_port_send_reset_filter,
			   (uint)pd_ctrl_send_reset_cmd, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_pd_control,
			       &send_reset_cmd, send_reset_cmd_size, HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_send_hard_reset(struct hpi_device *port,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_send_hard_reset(port,
			HPI_PD_CTRL_CMD_SEND_HARD_RESET, sync_mode,
			callback, param);
}

int hpi_port_send_soft_reset(struct hpi_device *port,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_send_hard_reset(port,
			HPI_PD_CTRL_CMD_SEND_SOFT_RESET, sync_mode,
			callback, param);
}

int hpi_port_send_cable_reset(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	struct hpi_pd_status pd_status;
	int err;

	/*
	 * CCG must be DFP and exlicit PD contract should exit.
	 * CCG must be the supplier of VCONN, check PD_STATUS register.
	 * VCONN must be turned on, check PD_STATUS register.
	 */
	err = hpi_port_read_pd_status(port, &pd_status);
	if (err)
		return err;

	if (pd_status.current_data_role != PD_PORT_DATA_ROLE_DFP ||
			!pd_status.contract_established ||
			!pd_status.vconn_supplier ||
			!pd_status.vconn_sourcing) {
		hpi_port_err("must be DFP, VCONN supplier and VCONN is ON\n",
			port);
		return -EINVAL;
	}

	return _hpi_port_send_hard_reset(port,
			HPI_PD_CTRL_CMD_SEND_CABLE_RESET, sync_mode,
			callback, param);
}

int hpi_port_send_soft_reset_sop_prime(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_send_hard_reset(port,
			HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_PRIME, sync_mode,
			callback, param);
}

int hpi_port_send_soft_reset_sop_dprime(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_send_hard_reset(port,
			HPI_PD_CTRL_CMD_SEND_SOFT_RESET_SOP_DPRIME, sync_mode,
			callback, param);
}

static enum hpi_msg_return _hpi_port_initialization_complete_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	if (IS_HPI_EVENT_MSG(msg->code))
		return HPI_MSG_RETURN_NONE;

	if (msg->code == HPI_PD_RESP_SUCCESS) {
		/* Initialization complete applied successfully. */
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
	} else if (msg->code == HPI_PD_RESP_PD_COMMAND_FAILED) {
		/*
		 * CCG has done the initialization internally by itself after
		 * 100ms timeout.
		 */
		hpi_port_dbg("CCG has auto-inited ater 100ms\n", port);
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);
	} else {
		/* Encounter error with error response. */
		hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, msg->code);
	}

	return HPI_MSG_RETURN_HANDLED;
}

int hpi_port_ec_initialization_complete(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 pd_cmd = (u8)HPI_PD_CTRL_CMD_EC_INITIALIZATION_COMPLETE;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RAW);
	hpi_cmd_set_filter(port, _hpi_port_initialization_complete_filter, 0,
			   NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_pd_control,
			       &pd_cmd, sizeof(pd_cmd), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(INIT_COMPLETE, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_ec_initialization_complete_sync(struct hpi_device *port)
{
	return hpi_port_ec_initialization_complete(port, HPI_SYNC, NULL, NULL);
}

int hpi_port_ec_initialization_complete_async(
		struct hpi_device *port, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_ec_initialization_complete(port, HPI_ASYNC,
						   callback, param);
}

int hpi_port_disable(struct hpi_device *port, enum hpi_sync_mode sync_mode,
		     hpi_cmd_cb_t callback, void *param)
{
	int err;

	err = hpi_port_pd_control(port, HPI_PD_CTRL_CMD_PORT_DISABLE,
				  sync_mode, callback, param);
	if (!err) {
		spin_lock(&port->slock);
		port->enabled = false;
		spin_unlock(&port->slock);
	}

	return err;
}

int hpi_port_disable_sync(struct hpi_device *port)
{
	return hpi_port_disable(port, HPI_SYNC, NULL, NULL);
}

int hpi_port_disable_async(struct hpi_device *port,
			   hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_disable(port, HPI_ASYNC, callback, param);
}

int hpi_port_enable(struct hpi_device *port, enum hpi_sync_mode sync_mode,
		    hpi_cmd_cb_t callback, void *param)
{
	struct cyccg *cyccg = port->cyccg;
	enum hpi_version hpi_ver = cyccg->ccg_info.hpi_ver;
	enum ccg_version ccg_ver = cyccg->ccg_info.ccg_ver;
	int port_num = cyccg->ccg_info.num_port;
	int port_index;
	int err;

	if (hpi_ver < HPI_VERSION_2 || ccg_ver < CCG3) {
		err = hpi_device_reset(cyccg, sync_mode, callback, param);
		goto out;
	}

	for (port_index = 0; port_index < port_num; port_index++) {
		if (cyccg->ports[port_index] == port)
			break;
	}

	err = hpi_enable_dpport(cyccg, port_index, sync_mode, callback, param);
out:
	if (!err) {
		spin_lock(&port->slock);
		port->enabled = true;
		spin_unlock(&port->slock);
	}
	return err;
}

int hpi_port_enable_sync(struct hpi_device *port)
{
	return hpi_port_enable(port, HPI_SYNC, NULL, NULL);
}

int hpi_port_enable_async(struct hpi_device *port,
		    hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_enable(port, HPI_ASYNC, callback, param);
}

int hpi_disable_all_dpport(struct cyccg *cyccg, enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param)
{
	struct hpi_device *port;
	int i;
	int err;

	if (cyccg->ccg_info.hpi_ver == HPI_VERSION_1) {
		err = hpi_port_disable(&cyccg->hpi_dev,
				       sync_mode, callback, param);
		goto out;
	}

	err = hpi_pdport_enable(cyccg, 0, sync_mode, callback, param);
out:
	if (!err) {
		for (i = 0; i < cyccg->ccg_info.num_port; i++) {
			port = cyccg->ports[i];
			if (!port || port->dev_type <= HPI_DEV_TYPE_DEVICE)
				continue;

			spin_lock(&port->slock);
			port->enabled = false;
			spin_unlock(&port->slock);
		}
	}

	return err;
}

int hpi_disable_all_dpport_sync(struct cyccg *cyccg)
{
	return hpi_disable_all_dpport(cyccg, HPI_SYNC, NULL, NULL);
}

int hpi_disable_all_dpport_async(struct cyccg *cyccg,
				 enum hpi_sync_mode sync_mode,
				 hpi_cmd_cb_t callback, void *param)
{
	return hpi_disable_all_dpport(cyccg, HPI_ASYNC, callback, param);
}

int hpi_port_change_pd_port_parameters(struct hpi_device *port,
				       struct hpi_pd_port_change_config *config,
				       enum hpi_sync_mode sync_mode,
				       hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 cmd = HPI_PD_CTRL_CMD_CHANGE_PD_PORT_PARAMETERS;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	err = hpi_port_disable(port, sync_mode, callback, param);
	if (err) {
		hpi_port_err("failed to disable port for change config, %d\n",
			port, err);
		return err;
	}

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_write_data(port, _hpi_port_rw_data_memory,
		config, sizeof(struct hpi_pd_port_change_config));
	if (err) {
		hpi_port_err("failed to write new PD port paarmeters, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_pd_control,
			       &cmd, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_abort_pending_pd_command(struct hpi_device *port,
				      enum hpi_sync_mode sync_mode,
				      hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_pd_control(port,
			HPI_PD_CTRL_CMD_ABORT_PENDING_PD_COMMAND, sync_mode,
			callback, param);
}

int hpi_port_ovp_ocp_otp_triggered(struct hpi_device *port,
				   enum hpi_sync_mode sync_mode,
				   hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_pd_control(port,
			HPI_PD_CTRL_CMD_OVP_OCP_OTP_TRIGGERED, sync_mode,
			callback, param);
}

static enum hpi_msg_return _hpi_port_read_src_sink_pdo_list_filter(
		struct hpi_device *port, struct hpi_msg *msg)
{
	hpi_port_vdbg("<<<< enter, msg->code=0x%x, len=%zu\n", port,
		msg->code, msg->len);
	if (msg->code != HPI_PD_RESP_READ_PDO_DATA ||
			hpi_cmd_get_state(port) != HPI_CMD_RESPONSE_RECEIVED)
		return HPI_MSG_RETURN_NONE;

	/* Read Source/Sink PDO data response received */
	hpi_cmd_copy_return_data(port, msg);
	hpi_port_dump("Read PDO data response data (%zu):\n", port,
		msg->data, msg->len, msg->len);

	hpi_cmd_set_state_errcode(port, HPI_CMD_COMPLETED, 0);

	return HPI_MSG_RETURN_HANDLED;
}

static int _hpi_port_read_pdo_list(struct hpi_device *port,
		enum hpi_pd_ctrl_command pd_ctrl_read_pdo_list_cmd,
		struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
		size_t *pdo_list_buf_size,
		enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 pd_cmd = (u8)pd_ctrl_read_pdo_list_cmd;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	if (sync_mode == HPI_SYNC && (!pdo_list_buf || !pdo_list_buf_size ||
		*pdo_list_buf_size < sizeof(struct hpi_pd_src_sink_pdo_list))) {
		hpi_port_err("invalid input parameters\n", port);
		return -EINVAL;
	}

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_EVENT);
	hpi_cmd_set_filter(port, _hpi_port_read_src_sink_pdo_list_filter, 0,
			   pdo_list_buf, pdo_list_buf_size);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_pd_control,
			       &pd_cmd, sizeof(pd_cmd), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_source_pdo(struct hpi_device *port,
			     struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			     size_t *pdo_list_buf_size,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_read_pdo_list(port,
			HPI_PD_CTRL_CMD_READ_SOURCE_PDOS,
			pdo_list_buf, pdo_list_buf_size, sync_mode,
			callback, param);
}

int hpi_port_read_sink_pdo(struct hpi_device *port,
			   struct hpi_pd_src_sink_pdo_list *pdo_list_buf,
			   size_t *pdo_list_buf_size,
			   enum hpi_sync_mode sync_mode,
			   hpi_cmd_cb_t callback, void *param)
{
	return _hpi_port_read_pdo_list(port,
			HPI_PD_CTRL_CMD_READ_SINK_PDOS,
			pdo_list_buf, pdo_list_buf_size, sync_mode,
			callback, param);
}

int hpi_port_read_current_pdo(struct hpi_device *port, u32 *pdo)
{
	int err = _hpi_port_rw_current_pdo(port, pdo, sizeof(u32), HPI_READ);

	if (!err)
		*pdo = get_unaligned_le32(pdo);
	return err;
}

int hpi_port_read_current_rdo(struct hpi_device *port, u32 *rdo)
{
	int err = _hpi_port_rw_current_rdo(port, rdo, sizeof(u32), HPI_READ);

	if (!err)
		*rdo = get_unaligned_le32(rdo);
	return err;
}

int hpi_port_read_current_cable_vdo(struct hpi_device *port, u32 *vdo)
{
	int err = _hpi_port_rw_current_cable_vdo(port,
						 vdo, sizeof(u32), HPI_READ);
	if (!err)
		*vdo = get_unaligned_le32(vdo);
	return err;
}

int hpi_port_read_ec_dp_hpd_control(struct hpi_device *port,
				    struct hpi_hpd_control *hpd_ctrl)
{
	return _hpi_port_rw_ec_dp_hpd_ctrl(port,
			hpd_ctrl, sizeof(struct hpi_hpd_control), HPI_READ);
}

int hpi_port_ec_dp_hpd_control(struct hpi_device *port,
			       struct hpi_hpd_control *hpd_ctrl,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_ec_dp_hpd_ctrl,
			       hpd_ctrl, sizeof(struct hpi_hpd_control),
			       HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write EC_DP_HPD_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_ec_dp_mux_control(struct hpi_device *port,
				    enum hpi_dp_mux_config *dp_mux_config)
{
	u8 dp_mux_ctrl;
	int err;

	err = _hpi_port_rw_ec_dp_mux_ctrl(port,
			&dp_mux_ctrl, sizeof(dp_mux_ctrl), HPI_READ);
	if (!err)
		*dp_mux_config = (enum hpi_dp_mux_config)dp_mux_ctrl;
	return err;
}

int hpi_port_ec_dp_mux_control(struct hpi_device *port,
			       enum hpi_dp_mux_config *dp_mux_config,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 dp_mux_ctrl = *(u8 *)&dp_mux_config;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_ec_dp_mux_ctrl,
			       &dp_mux_ctrl, sizeof(dp_mux_ctrl), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write EC_DP_MUX_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_trigger_dp_mode(struct hpi_device *port,
				  struct hpi_trigger_dp_mode *trigger_mode)
{
	return _hpi_port_rw_trigger_dp_mode(port, trigger_mode,
			sizeof(struct hpi_trigger_dp_mode), HPI_READ);
}

int hpi_port_trigger_dp_mode(struct hpi_device *port,
			     struct hpi_trigger_dp_mode *trigger_mode,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_trigger_dp_mode,
			       trigger_mode, sizeof(struct hpi_trigger_dp_mode),
			       HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write TRIGGER_DP_MODE register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_dp_source_configure(struct hpi_device *port,
			   enum hpi_dp_source_config_mode *config)
{
	u8 value;
	int err;

	err = _hpi_port_rw_dp_configure_mode(port,
					     &value, sizeof(u8), HPI_READ);
	*config = (enum hpi_dp_source_config_mode)value;
	return err;
}

int hpi_port_read_dp_sink_configure(struct hpi_device *port,
			   struct hpi_dp_sink_config_mode *config)
{
	return _hpi_port_rw_dp_configure_mode(port, config,
			sizeof(struct hpi_dp_sink_config_mode), HPI_READ);
}

int hpi_port_read_dp_configure_mode(struct hpi_device *port, void *mode)
{
	enum hpi_dp_mode dp_mode = port->cyccg->ccg_info.dp_mode;

	if (dp_mode == DP_UNKNOWN)
		return -EINVAL;

	if (DP_SOURCE)
		return hpi_port_read_dp_source_configure(port, mode);
	return hpi_port_read_dp_sink_configure(port, mode);
}

int hpi_port_dp_source_configure(struct hpi_device *port,
				 enum hpi_dp_source_config_mode *config,
				 enum hpi_sync_mode sync_mode,
				 hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 dp_src_config_mode = *(u8 *)config;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_dp_configure_mode,
			       &dp_src_config_mode, sizeof(dp_src_config_mode),
			       HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write DP_CONFIGURE_MODE register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_dp_sink_configure(struct hpi_device *port,
			       struct hpi_dp_sink_config_mode *config,
			       u32 *new_status_vdo,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	if (config->set_status_vdo || config->set_status_vdo_pdo) {
		err = hpi_cmd_write_data(port, _hpi_port_rw_data_memory,
					 new_status_vdo, sizeof(u32));
		if (err) {
			hpi_port_err("failed to write data memory reg, %d\n",
				port, err);
			goto err;
		}
	}

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_dp_configure_mode,
			       config, sizeof(struct hpi_dp_sink_config_mode),
			       HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write DP_CONFIGURE_MODE reg, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_write_dp_config(struct hpi_device *port,
			     void *config, void *new_status_vdo,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param)
{
	enum hpi_dp_mode dp_mode = port->cyccg->ccg_info.dp_mode;

	if (dp_mode == DP_UNKNOWN)
		return -EINVAL;

	if (dp_mode == DP_SOURCE)
		return hpi_port_dp_source_configure(port, config, sync_mode,
						    callback, param);
	return hpi_port_dp_sink_configure(port, config, new_status_vdo,
					  sync_mode, callback, param);
}

int hpi_port_read_alt_mode_cmd(struct hpi_device *port,
			       struct hpi_alt_mode_cmd *alt_mode_cmd)
{
	return _hpi_port_rw_alt_mode_cmd(port, alt_mode_cmd,
			sizeof(struct hpi_alt_mode_cmd), HPI_READ);
}

int hpi_port_alt_mode_cmd(struct hpi_device *port,
			  struct hpi_alt_mode_cmd *alt_mode_cmd,
			  enum hpi_sync_mode sync_mode,
			  hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_alt_mode_cmd,
			       alt_mode_cmd, sizeof(struct hpi_alt_mode_cmd),
			       HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write ALT_MODE_CMD register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_app_hw_cmd(struct hpi_device *port,
			       struct hpi_app_hw_cmd *app_hw_cmd)
{
	return _hpi_port_rw_alt_mode_cmd(port, app_hw_cmd,
			sizeof(struct hpi_app_hw_cmd), HPI_READ);
}

int hpi_port_app_hw_cmd(struct hpi_device *port,
			  struct hpi_app_hw_cmd *app_hw_cmd,
			  enum hpi_sync_mode sync_mode,
			  hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_app_hw_cmd,
			       app_hw_cmd, sizeof(struct hpi_app_hw_cmd),
			       HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write ALT_MODE_CMD register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_event_mask(struct hpi_device *port, u32 *event_mask)
{
	int err;

	err = _hpi_port_rw_event_mask(port, event_mask, sizeof(u32), HPI_READ);
	if (!err)
		*event_mask = get_unaligned_le32(event_mask);
	return err;
}

int hpi_port_write_event_mask(struct hpi_device *port, u32 event_mask,
			      enum hpi_sync_mode sync_mode,
			      hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	__le32 mask = cpu_to_le32(event_mask);
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_event_mask,
			       &mask, sizeof(u32), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write EVENT_MASK register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_write_event_mask_sync(struct hpi_device *port, u32 event_mask)
{
	return hpi_port_write_event_mask(port, event_mask, HPI_SYNC,
					 NULL, NULL);
}

int hpi_port_write_event_mask_async(struct hpi_device *port, u32 event_mask,
				    hpi_cmd_cb_t callback, void *param)
{
	return hpi_port_write_event_mask(port, event_mask, HPI_ASYNC,
					 callback, param);
}

int hpi_port_read_swap_response(struct hpi_device *port,
				struct hpi_swap_response_reg *swap_resp_reg)
{
	return _hpi_port_rw_swap_response(port, swap_resp_reg,
					  sizeof(struct hpi_swap_response_reg),
					  HPI_READ);
}

int hpi_port_swap_response_update(struct hpi_device *port,
				  struct hpi_swap_response_reg *swap_resp_reg,
				  enum hpi_sync_mode sync_mode,
				  hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s, swap_resp_reg=0x%02x\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		*(u8 *)swap_resp_reg);

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_swap_response,
		swap_resp_reg, sizeof(struct hpi_swap_response_reg), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write SWAP_RESPONSE register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_active_ec_modes(struct hpi_device *port,
				  bool *is_ec_has_active_alternate_modes)
{
	u8 status;
	int err;

	err = _hpi_port_rw_active_ec_modes(port, &status, sizeof(u8), HPI_READ);
	if (!err)
		*is_ec_has_active_alternate_modes =
				IS_EC_HAS_ACTIVE_ALT_MODES(status);
	return err;
}

int hpi_port_active_ec_modes(struct hpi_device *port,
			     bool is_ec_has_active_alternate_modes,
			     enum hpi_sync_mode sync_mode,
			     hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	u8 status = (u8)is_ec_has_active_alternate_modes;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_active_ec_modes,
			       &status, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write ACTIVE_EC_MODES register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_vdm_ec_control(struct hpi_device *port,
				 struct hpi_vdm_ec_control *vdm_ec_ctrl)
{
	return _hpi_port_rw_vdm_ec_control(port, vdm_ec_ctrl,
				sizeof(struct hpi_vdm_ec_control), HPI_READ);
}

int hpi_port_write_vdm_ec_control(struct hpi_device *port,
				  struct hpi_vdm_ec_control *vdm_ec_ctrl,
				  enum hpi_sync_mode sync_mode,
				  hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s, ec_ctrl_enabled=%s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC",
		vdm_ec_ctrl->ec_ctrl_enabled ? "true" : "false");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_vdm_ec_control,
		vdm_ec_ctrl, sizeof(struct hpi_vdm_ec_control), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write VDM_EC_CONTROL register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_cmd_timeout(struct hpi_device *port, u8 *cmd_timeout)
{
	return _hpi_port_rw_cmd_timeout(port, cmd_timeout,
					sizeof(u8), HPI_READ);
}

int hpi_port_write_cmd_timeout(struct hpi_device *port, u8 cmd_timeout,
			       enum hpi_sync_mode sync_mode,
			       hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s, cmd_timeout=0x%02x\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC", cmd_timeout);

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_cmd_timeout,
			       &cmd_timeout, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write CMD_TIMEOUT register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_get_port_intr_status(struct hpi_device *port, u32 *intr_status)
{
	int err;

	err = _hpi_port_rw_port_intr_status(port, intr_status,
					    sizeof(u32), HPI_READ);
	if (!err)
		*intr_status = get_unaligned_le32(intr_status);
	return err;
}

int hpi_port_set_port_intr_status(struct hpi_device *port, u32 intr_status,
				  enum hpi_sync_mode sync_mode,
				  hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	__le32 status = cpu_to_le32(intr_status);
	int err;

	hpi_port_vdbg("<<<< enter, %s, intr_status=0x%08x\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC", intr_status);

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_port_intr_status,
			       &status, sizeof(u32), HPI_WRITE);
	if (err) {
		hpi_port_err("failed to write PORT_INTR_STATUS register, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

int hpi_port_read_disable_billboard_reset(struct hpi_device *port,
					  u8 *signature)
{
	return _hpi_port_rw_disable_billboard_reset(port, signature,
						    sizeof(u8), HPI_READ);
}

int hpi_port_read_billboard_reset_status(struct hpi_device *port,
					 bool *is_disabled)
{
	u8 signature;
	int err;

	err = hpi_port_read_disable_billboard_reset(port, &signature);
	if (!err) {
		if (signature == HPI_SIGNATURE_BILLBOARD_RESET_DISABLED)
			*is_disabled = true;
		else
			*is_disabled = false;
	}

	return err;
}

static int _hpi_port_disable_billboard_reset(struct hpi_device *port,
		u8 signature, enum hpi_sync_mode sync_mode,
		hpi_cmd_cb_t callback, void *param)
{
	enum hpi_version hpi_ver = port->cyccg->ccg_info.hpi_ver;
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", port,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(port, sync_mode, HPI_CMD_FLAG_RESP_ONLY);
	hpi_cmd_set_filter(port, NULL, 0, NULL, NULL);
	hpi_cmd_set_async_callback(port, callback, param);

	err = hpi_cmd_send_cmd(port, _hpi_port_rw_disable_billboard_reset,
			       &signature, sizeof(u8), HPI_WRITE);
	if (err) {
		hpi_port_err("fail to write DISBALE_BILLBOARD_RESET reg, %d\n",
			port, err);
		goto err;
	}

	err = hpi_cmd_sync(port, HPI_TIME_OF(DEFAULT, hpi_ver));

err:
	hpi_cmd_deinit_unlock(port);
	hpi_port_vdbg(">>>> exit, %d\n", port, err);
	return err;
}

/* hpi_port_disable_billboard_reset - inform CCGx to not initiate a billboard
 *	reset
 * @port: instance of the USB-C port in this driver.
 * @disabled: set to true indicates CCGx not reset; otherwise, set to false.
 * @sync_mode: this function should called in sync or async mode.
 * @callback: The routine called after the command was finished, only valid
 *	when running in asycn mode that @sync_mode set to HPI_ASYNC.
 *	When @sync_mode=HPI_SYNC, set @callback=NULL.
 * @param: A custom parameter input to @callback routine, only valid
 *	in async mode as as @callback. In sync mode, set to NULL.
 */
int hpi_port_disable_billboard_reset(struct hpi_device *port, bool disabled,
				     enum hpi_sync_mode sync_mode,
				     hpi_cmd_cb_t callback, void *param)
{
	u8 signature = disabled ? HPI_SIGNATURE_BILLBOARD_RESET_DISABLED :
				(u8)~HPI_SIGNATURE_BILLBOARD_RESET_DISABLED;

	return _hpi_port_disable_billboard_reset(port, signature, sync_mode,
						 callback, param);
}

int hpi_port_read_billboard_altmode_status(struct hpi_device *port,
		struct hpi_billboard_altmode_status *status)
{
	return _hpi_port_rw_billboard_altmode_status(port, status,
			sizeof(struct hpi_billboard_altmode_status), HPI_READ);
}

int hpi_port_read_billboard_oper_model(struct hpi_device *port,
		struct hpi_billboard_oper_model *model_status)
{
	return _hpi_port_rw_billboard_oper_model(port, model_status,
			sizeof(struct hpi_billboard_oper_model), HPI_READ);
}

int hpi_port_read_external_power_control(struct hpi_device *port,
		struct hpi_external_power_control *status)
{
	return _hpi_port_rw_external_power_control(port, status,
			sizeof(struct hpi_external_power_control), HPI_READ);
}

int hpi_device_event_monitor(struct hpi_device *hpidev,
		enum hpi_sync_mode sync_mode,
		hpi_cmd_filter_t event_monitor_filter, unsigned long timeout,
		hpi_cmd_cb_t callback, void *param)
{
	int err;

	hpi_port_vdbg("<<<< enter, %s\n", hpidev,
		sync_mode == HPI_SYNC ? "HPI_SYNC" : "HPI_ASYNC");

	hpi_cmd_init_lock(hpidev, sync_mode, HPI_CMD_FLAG_RAW);
	hpi_cmd_set_filter(hpidev, event_monitor_filter, 0, NULL, NULL);
	hpi_cmd_set_async_callback(hpidev, callback, param);

	hpi_cmd_set_state(hpidev, HPI_CMD_ISSUED);

	err = hpi_cmd_sync(hpidev, timeout);

	hpi_cmd_deinit_unlock(hpidev);
	hpi_port_vdbg(">>>> exit, %d\n", hpidev, err);
	return err;
}

__maybe_unused
char *hpi_vdm_sop_type_to_string(enum vdm_sop_type sop_type)
{
	switch (sop_type) {
	case VDM_SOP_TYPE_SOP:
		return "SOP";
	case VDM_SOP_TYPE_SOP_PRIME:
		return "SOP_PRIME";
	case VDM_SOP_TYPE_SOP_DPRIME:
		return "SOP_DPRIME";
	default:
		break;
	}

	return "Invalid/Unknown SOP Type";
}

__maybe_unused
char *ccg_fw_mode_type_to_string(enum ccg_fw_mode_type mode_type,
				 enum ccg_version ccg_ver)
{
	switch (mode_type) {
	case CCG_FW_MODE_TYPE_BOOTLAODER:
		return "Bootloader";
	case CCG_FW_MODE_TYPE_FW_IMAGE1:
		if (ccg_ver <= CCG2)
			return "Application";
		return "FW Image-1";
	case CCG_FW_MODE_TYPE_FW_IMAGE2:
		return "FW Image-2";
	default:
		break;
	}

	return "Invalid/Unknown";
}

__maybe_unused
char *hpi_attached_dev_type_to_string(u8 attached_dev_type)
{
	enum hpi_type_c_attached_device_type dev_type =
		(enum hpi_type_c_attached_device_type)attached_dev_type;

	switch (dev_type) {
	case ATTACHED_DEV_TYPE_NOTHING:
		return "Nothing";
	case ATTACHED_DEV_TYPE_SINK:
		return "Sink";
	case ATTACHED_DEV_TYPE_SOURCE:
		return "Source";
	case ATTACHED_DEV_TYPE_DEBUG_ACCESSORY:
		return "Debug_Accessory";
	case ATTACHED_DEV_TYPE_AUDIO_ACCESSORY:
		return "Audio_Accessory";
	case ATTACHED_DEV_TYPE_POWERED_ACCESSORY:
		return "Powered_Accessory";
	case ATTACHED_DEV_TYPE_UNSUPPORTED_ACCESSORY:
		return "Unsupported_Accessory";
	default:
		break;
	}

	return "Unknown Attached Device Type";
}

__maybe_unused
char *hpi_type_c_current_level_to_string(u8 current_level)
{
	switch (current_level) {
	case 0x00:
		return "Default";
	case 0x01:
		return "1.5A";
	case 0x02:
		return "3A";
	default:
		break;
	}

	return "Unknown Current Level";
}

int hpi_port_get_port_status(struct hpi_device *port,
			     struct usbc_pdport_status *port_status,
			     bool with_pdo_rdo_cable_vdo)
{
	struct hpi_type_c_status type_c_status;
	struct hpi_pd_status pd_status;
	int err;

	memset(port_status, 0, sizeof(struct usbc_pdport_status));

	err = hpi_port_read_type_c_status(port, &type_c_status);
	if (err) {
		hpi_port_err("failed to read type-c status reg, %d\n",
			port, err);
		return err;
	}
	port_status->type_c_connected = type_c_status.type_c_connected;
	port_status->cc_polarity = type_c_status.cc_polarity;
	port_status->attached_device_type = type_c_status.attached_device_type;
	port_status->ra_detected = type_c_status.ra_detected;
	port_status->current_level = type_c_status.current_level;

	err = hpi_port_read_pd_status(port, &pd_status);
	if (err) {
		hpi_port_err("failed to read PD status reg, %d\n",
			port, err);
		return err;
	}
	port_status->default_data_role = pd_status.default_data_role;
	port_status->default_data_role_in_drp =
			pd_status.default_data_role_in_drp;
	port_status->default_power_role = pd_status.default_power_role;
	port_status->default_power_role_in_dual =
			pd_status.default_power_role_in_dual;
	port_status->current_data_role = pd_status.current_data_role;
	port_status->current_power_role = pd_status.current_power_role;
	port_status->contract_established = pd_status.contract_established;
	port_status->emca_present = pd_status.emca_present;
	port_status->vconn_supplier = pd_status.vconn_supplier;
	port_status->vconn_sourcing = pd_status.vconn_sourcing;

	if (!with_pdo_rdo_cable_vdo)
		return 0;

	err = hpi_port_read_current_pdo(port, &port_status->current_pdo);
	if (err) {
		hpi_port_err("failed to read current PDO reg, %d\n", port, err);
		return err;
	}

	err = hpi_port_read_current_rdo(port, &port_status->current_rdo);
	if (err) {
		hpi_port_err("failed to read current RDO reg, %d\n", port, err);
		return err;
	}

	err = hpi_port_read_current_cable_vdo(port,
			&port_status->current_cable_vdo);
	if (err) {
		hpi_port_err("failed to read current Cable VDO reg, %d\n",
			port, err);
		return err;
	}

	return 0;
}

__maybe_unused
unsigned long get_bits_value(void *bits_data, int bit_offset, int bit_num)
{
#define U8_BIT sizeof(u8)
	u8 *bit_array = bits_data;
	int byte_offset = bit_offset / U8_BIT;
	int shift = bit_offset % U8_BIT;
	unsigned long data = *(unsigned long *)&bit_array[byte_offset];
	unsigned long mask = 0;
	int i;

	for (i = 0; i < bit_num; i++) {
		mask <<= 1;
		mask |= 1;
	}

	data >>= shift;
	return (unsigned long)(data & mask);
}

__maybe_unused
char *value_to_binary_string(unsigned long val, char *buf, size_t size)
{
	unsigned char *arr = (unsigned char *)&val;
	int bits = sizeof(unsigned long) * 8;
	int bit;
	int i;
	int count;

	count = 0;
	buf[0] = '\0';
	for (i = bits - 1; i >= 0; i--) {
		bit = get_bits_value(arr, i, 1);
		if (bit == 0 && !count) {
			continue;
		} else {
			strcat(buf, bit ? "1" : "0");
			count++;
		}
	}

	if (!count)
		sprintf(buf, "0");
	else
		buf[count] = '\0';

	return buf;
}

__maybe_unused
void hex_dump(int flag, const void *hex_data, size_t size)
{
#define BYTES_PRE_ROW 16
	const u8 *buf = hex_data;
	int i, j, rows;	int offset;
	char one_byte_str[4];
	char row_byte_str[BYTES_PRE_ROW * 4];

	if (!flag)
		return;

	/* Output a empty line if no data. */
	if (!buf || !size)
		return;

	rows = size / BYTES_PRE_ROW;
	for (i = 0; i <= rows; i++) {
		offset = BYTES_PRE_ROW * i;
		if (offset >= size) {
			/*
			 * avoid output duplicate empty line when total bytes
			 * is properly multiple of BYTES_PRE_ROW.
			 */
			goto out;
		}

		memset(row_byte_str, 0, sizeof(row_byte_str));
		for (j = 0; j < BYTES_PRE_ROW; j++) {
			offset = BYTES_PRE_ROW * i + j;
			if (offset >= size) {
				/* end of the buffer bytes output. */
				pr_err("%s\n", row_byte_str);
				goto out;
			}

			/* output bytes in same row. */
			sprintf(one_byte_str, "%02x ", buf[offset]);
			strcat(row_byte_str, one_byte_str);
		}

		/* 16 bytes output, turned to new line. */
		pr_err("%s\n", row_byte_str);
	}

out:
	return;
}

__maybe_unused
void hpi_pdo_dump(u32 pdo_data, bool is_source)
{
	union power_data_object *pdo = (union power_data_object *)&pdo_data;
	enum power_supply_type power_type;
	u32 val;

	hpi_info("Current (%s) PDO data (0x%08x):\n",
		is_source ? "Source" : "Sink", pdo_data);
	power_type =
		(enum power_supply_type)pdo->source_battery.power_supply_type;
	if (power_type == POWER_TYPE_FIXED_SUPPLY && is_source) {
		hpi_info("    Fixed supply (Vmin = Vmax)\n");
		hpi_info("    Dual-Role Power: %s\n",
			pdo->source_fixed_supply.dual_role_power ?
				"Yes" : "No");
		hpi_info("    USB Suspend Supported: %s\n",
			pdo->source_fixed_supply.usb_suspend_supported ?
				"Yes" : "No");
		hpi_info("    Externally Powered: %s\n",
			pdo->source_fixed_supply.externally_powered ?
				"Yes" : "No");
		hpi_info("    USB Communications Capable: %s\n",
			pdo->source_fixed_supply.usb_communications_capable ?
				"Yes" : "No");
		hpi_info("    Dual-Role Data: %s\n",
			pdo->source_fixed_supply.dual_role_data ? "Yes" : "No");
		hpi_info("    Unchunked Extended Messages Supported: %s\n",
		pdo->source_fixed_supply.unchunked_extended_messages_supported ?
				"Yes" : "No");
		hpi_info("    Peak Current: %u\n",
			pdo->source_fixed_supply.peak_current);
		hpi_info("    Voltage in 50mV units: %u\n",
			pdo->source_fixed_supply.voltage_in_50mV_units);
		hpi_info("    Maximum Current in 10mA units: %u\n",
			pdo->source_fixed_supply.maximum_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_BATTERY && is_source) {
		hpi_info("    Battery\n");
		hpi_info("    Maximum Voltage in 50mV units: %u\n",
			pdo->source_battery.maximum_voltage_in_50mV_units);
		hpi_info("    Minimum Voltage in 50mV units: %u\n",
			pdo->source_battery.minimum_voltage_in_50mV_units);
		hpi_info("    Maximum Allowable Power in 250mV units: %u\n",
		pdo->source_battery.maximum_allowable_power_in_250mW_units);
	} else if (power_type == POWER_TYPE_VARIABLE_SUPPLY && is_source) {
		hpi_info("    Variable Supply (non-Battery)\n");
		hpi_info("    Maximum Voltage in 50mV units: %u\n",
		pdo->source_variable_supply.maximum_voltage_in_50mV_units);
		hpi_info("    Minimum Voltage in 50mV units: %u\n",
		pdo->source_variable_supply.minimum_voltage_in_50mV_units);
		hpi_info("    Maximum Current in 10mA units: %u\n",
		pdo->source_variable_supply.maximum_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_FIXED_SUPPLY && !is_source) {
		hpi_info("    Fixed supply\n");
		hpi_info("    Dual-Role Power: %s\n",
			pdo->sink_fixed_supply.dual_role_power ? "Yes" : "No");
		hpi_info("    Higher Capability: %s\n",
			pdo->sink_fixed_supply.higher_capability ?
				"Yes" : "No");
		hpi_info("    Externally Powered: %s\n",
			pdo->sink_fixed_supply.externally_powered ?
				"Yes" : "No");
		hpi_info("    USB Communications Capable: %s\n",
			pdo->sink_fixed_supply.usb_communications_capable ?
				"Yes" : "No");
		hpi_info("    Dual-Role Data: %s\n",
			pdo->sink_fixed_supply.dual_role_data ? "Yes" : "No");
		val = pdo->sink_fixed_supply.fast_role_swap_required_current;
		hpi_info("    Fast Role Swap required USB Type-C Current: %s\n",
			(val == 0) ? "Fast Swap not supported (default)" :
				((val == 1) ? "Default USB Power" :
				((val == 2) ? "1.5A @ 5V" : "3.0A @ 5V")));
		hpi_info("    Voltage in 50mV units: %u\n",
			pdo->sink_fixed_supply.voltage_in_50mV_units);
		hpi_info("    Operational Current in 10mA units: %u\n",
		pdo->sink_fixed_supply.operational_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_BATTERY && !is_source) {
		hpi_info("    Battery\n");
		hpi_info("    Maximum Voltage in 50mV units: %u\n",
			pdo->sink_battery.maximum_voltage_in_50mV_units);
		hpi_info("    Minimum Voltage in 50mV units: %u\n",
			pdo->sink_battery.minimum_voltage_in_50mV_units);
		hpi_info("    Operational Power in 250mW units: %u\n",
			pdo->sink_battery.operational_current_in_10mA_units);
	} else if (power_type == POWER_TYPE_VARIABLE_SUPPLY && !is_source) {
		hpi_info("    Variable Supply (non-Battery)\n");
		hpi_info("    Maximum Voltage in 50mV units: %u\n",
		pdo->sink_variable_supply.maximum_voltage_in_50mV_units);
		hpi_info("    Minimum Voltage in 50mV units: %u\n",
		pdo->sink_variable_supply.minimum_voltage_in_50mV_units);
		hpi_info("    Operational Current in 10mA units: %u\n",
		pdo->sink_variable_supply.operational_power_in_250mW_units);
	} else {
		hpi_info("    Reserved\n");
	}

}

__maybe_unused
void hpi_capabilities_message_dump(struct hpi_device *port,
				      struct hpi_msg *msg)
{
	struct hpi_capabilities_message *cap_msg =
			(struct hpi_capabilities_message *)msg->data;
	union vdm_msg_header header;
	bool is_source;
	int i;

	header.msg_header = get_unaligned_le16(&cap_msg->msg_header.msg_header);
	is_source = header.port_power_role_cable_plug ? "Source" : "Sink";

	hpi_info("Capabilities Messages of %s\n", port->pdport_name_addr->name);
	hpi_info("Message header:\n");
	hpi_info("Message type: %u\n", header.msg_type);
	if (cap_msg->sop_type == VDM_SOP_TYPE_SOP) {
		hpi_info("Port data role: %s\n",
			header.port_data_role ? "DFP" : "UFP");
		hpi_info("Port power role: %s\n",
			header.port_power_role_cable_plug ? "Source" : "Sink");
	} else {
		hpi_info("The message from: %s\n",
			header.port_power_role_cable_plug ?
					"Cable Plug" : "DFP/UFP");
	}
	hpi_info("PD Spec rev: %d\n", (int)header.spec_rev + 1);
	hpi_info("message ID: %u\n", header.message_id);
	hpi_info("Number of Data Objects: %u\n", header.data_objs);
	hpi_info("Extended: %s\n", header.extended ?
				 "Extended Message" : "Control Message");

	for (i = 0; i < header.data_objs; i++)
		hpi_pdo_dump(get_unaligned_le32(&cap_msg->pdo[i]), is_source);
}

__maybe_unused
void hpi_ccg_fw_version_dump(struct hpi_ccg_fw_version *ccg_fw_vers)
{
	struct hpi_image_version *fw_ver;

	fw_ver = &ccg_fw_vers->btldr;
	hpi_info("Bootloader Version: %u.%u.%u.%u\n",
		fw_ver->base.major, fw_ver->base.minor,
		fw_ver->base.patch_ver, fw_ver->base.build_number);

	fw_ver = &ccg_fw_vers->fw1_app;
	hpi_info("Image-1 FW App Version: %u.%u,%u,%c%c\n",
		   fw_ver->app.major, fw_ver->app.minor,
		   fw_ver->app.external_circuit_ver,
		   fw_ver->app.name[1], fw_ver->app.name[0]);

	fw_ver = &ccg_fw_vers->fw2_app;
	hpi_info("Image-2 FW App Version: %u.%u,%u,%c%c\n",
		   fw_ver->app.major, fw_ver->app.minor,
		   fw_ver->app.external_circuit_ver,
		   fw_ver->app.name[1], fw_ver->app.name[0]);
}